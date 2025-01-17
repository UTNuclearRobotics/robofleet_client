#! /usr/bin/env python3
"""
Utilities for generating Flatbuffers schemas for ROS message types. 

Schema formats documentation:
    http://wiki.ros.org/msg and 
    https://google.github.io/flatbuffers/flatbuffers_guide_writing_schema.html
"""

from __future__ import print_function, division
from argparse import ArgumentParser
from typing import List
from rosidl_adapter.parser import MessageSpecification, Field
from lib2to3.pgen2.token import EQUAL
import os
import re
import sys
from typing import Set, Optional

# from rosbridge_library.internal.ros_loader import get_message_class
from .msg_util import get_msg_spec, get_srv_spec

# Flatbuffers types considered "scalar"
scalar_types = {
    "bool",
    "byte",
    "int8",
    "int16",
    "int32",
    "int64",
    "uint8",
    "uint16",
    "uint32",
    "uint64",
    "float32",
    "float64",
}

# types that are already defined in Flatbuffers
primitive_types = scalar_types | {
    "string"
}

support_types = {
  # generated by gen_base_schema()
    "MsgMetadata"
}

# all types that can be used in Flatbuffers at startup
# all of the Flatbuffers primitives directly correspond to ROS types
base_defined_types = primitive_types | support_types

def gen_metadata_item(base_ns):
    """ Generate a table field containing a MsgMetadata. """ 
    yield "  __metadata:{}.MsgMetadata;".format(base_ns)

def gen_base_schema(base_ns):
    """ Generate supporting definitions """
    yield "// *** begin supporting definitions ***"
    # Namespace everything
    yield "namespace {};".format(base_ns)

    # Metadata table for all messages, to support RoboFleet
    yield "table MsgMetadata {"
    yield "  type:string;"
    yield "  topic:string;"
    yield "}"

    # All generated messages can be read as MsgWithMetadata to access metadata
    yield "table MsgWithMetadata {"
    for x in gen_metadata_item(base_ns):
        yield x
    yield "}"

    yield "table RobofleetSubscription {"
    for x in gen_metadata_item(base_ns):
      yield x
    yield "  topic_regex:string;"
    yield "  action:uint8;"
    yield "}"
    yield "// *** end supporting definitions ***"

# direct ROS to Flatbuffers type remappings
type_mapping = {
    "char":"uint8",
    "wstring":"[ushort]",
    "byte":"uint8"           # bytes are signed in FB and unsigned in ROS
}

def type_remap(ros_type_name):
    """ apply direct translations from ROS type names to flatbuffers type names """
    if ros_type_name in type_mapping:
        return type_mapping[ros_type_name]
    return ros_type_name

class Type:
    """
    Represents a data type. Constructed from a ROS type string.
    Produces Flatbuffers type strings.
    """
    def __init__(self, ros_type, base_ns):

        self.base_ns = base_ns
        
        match = re.match(r"^(?P<type>(?:(?P<ns>[\w\/]+)\/)?(?P<name>\w+))(?P<array>\[(?P<size>\d*)\])?$", ros_type)
        if match is None:
            raise RuntimeError("Invalid ROS type: {}".format(ros_type))

        # the fully-qualified ROS type, including array syntax if any
        self.ros_type_raw = ros_type

        # the fully-qualified ROS type, without any array syntax
        self.ros_type = match.group("type")
        self.package = self.ros_type.split('/')[0]

        if match.group("ns") is not None:
            self.namespace = match.group("ns").replace("/", ".")
        else:
            self.namespace = None

        # the unqualified name of this type
        self.name = match.group("name")

        # is this a vector OR array type?
        self.is_array = match.group("array") is not None

        # size will be None if the type is a vector rather than an array
        self.array_size = match.group("size")
    
    def full_namespace(self):
        """ Namespace including base namespace """
        if self.namespace is None:
            return self.base_ns
        return "{}.{}.msg".format(self.base_ns, self.namespace)
    
    def fbs_type_name(self):
        """ Fully qualified Flatbuffers type name with remapping """
        if self.namespace is not None:
            n = "{}.{}.{}".format(self.base_ns, self.namespace, self.name)
        else: 
            n = self.name
        return type_remap(n)
    
    def fbs_type(self):
        """ Fully qualified Flatbuffers type (including array syntax) """
        full_name = self.fbs_type_name()
        if self.is_array:
            return "[{}]".format(full_name)
        return full_name

    def is_defined(self, defined_types):
        """ Determine whether this type has been defined for Flatbuffers """
        return self.fbs_type_name() in defined_types
    
    def mark_defined(self, defined_types):
        """ Add this type to defined types set """
        defined_types.add(self.fbs_type_name())

def gen_table(msg_type: MessageSpecification,
              fields: List[Field],
              base_ns: str) -> str:
    """ Generate a table for a Flatbuffers type and its fields. """

    yield "table {} {{".format(msg_type.name)
    for x in gen_metadata_item(base_ns):
        yield x
    for field in fields:
        if field.type.type in type_mapping:
            type = type_mapping[field.type.type]
        else:
            type = field.type.type

        attrs = ""
        # mark all non-scalar fields "required" (since they are in ROS)
        if type not in scalar_types:
            attrs = attrs + " (required)"
        
        ns = ''
        if not field.type.is_primitive_type():
            ns = base_ns + '.' + field.type.pkg_name + '.msg.'

        # add square brackets to indicate an array
        full_type = ns + type
        if field.type.is_array:
            full_type = f'[{full_type}]'

        yield "  {}:{}{};".format(field.name, full_type, attrs)
    yield "}"

def gen_constants_enums(msg_type, spec) -> str:
    """
    Generate enums to represent ROS message constants.

    Enums are Flatbuffers' closest approximation of constants. We generate one
    enum per constant, with a single field "value". Only integer types are 
    supported by Flatbuffers.
    """

    if len(spec.constants) == 0:
        return

    yield "namespace {}.{}Constants;".format(msg_type.full_namespace(), msg_type.name)
    
    # https://google.github.io/flatbuffers/flatbuffers_guide_writing_schema.html
    allowed_enum_types = {"int8", "int16", "int32", "int64", "uint8", "uint16", "uint32", "uint64"}
    
    for c in spec.constants:
        if c.type in allowed_enum_types:
            # name is converted to lower case to avoid all-uppercase camelCase in, e.g. JS code
            yield "enum {} : {} {{ value = {} }}".format(c.name.lower(), c.type, c.val)
        else:
            print("Warning: skipped non-integral constant {}.{}".format(msg_type.fbs_type(), c.name), file=sys.stderr)

def gen_constants_table(msg_type, base_ns, spec) -> Optional[str]:
    """
    Generate a table with default values to represent ROS message constants.
    Using generated Flatbuffers code, this table should be constructed as a 
    singleton to read constants values.

    The table is named MsgTypeConstants, where MsgType is the name of the given
    message type.
    
    The advantage of this method over enums is that it supports any ROS 
    constant type.
    """ 

    if len(spec.constants) == 0:
        return

    yield "namespace {};".format(msg_type.full_namespace())
    yield "table {}Constants {{".format(msg_type.name)
    for c in spec.constants:
        t = Type(c.type, base_ns)
        if t.fbs_type() in primitive_types:
            # name is converted to lower case to avoid all-uppercase camelCase in, e.g. JS code
            yield "  {}:{} = {};".format(c.name.lower(), t.fbs_type(), c.val_text)
        else:
            raise RuntimeError("Invalid non-primitive constant")
    yield "}"

def gen_msg(msg_type: Type,
            defined_types: Set[str],
            base_ns: str,
            gen_enums: bool,
            gen_constants: bool):
    """ 
    Generate .fbs definitions for a ROS message type, including dependencies. 
    """
    # no need to generate already-defined type
    if msg_type.is_defined(defined_types):
        raise RuntimeError('Type ' + msg_type.ros_type + ' is already defined.')
    msg_type.mark_defined(defined_types)
    
    # look for a message with this name    
    try:
      msg_spec = get_msg_spec(msg_type.package, msg_type.name)
      for x in process_type(msg_type, msg_spec, base_ns, gen_enums, gen_constants):
        yield x
      return
    except LookupError:
      pass

    # look for a service with this name
    try:
      srv_spec = get_srv_spec(msg_type.package, msg_type.name)
      request_type = Type(msg_type.ros_type_raw + '_Request', base_ns)
      response_type = Type(msg_type.ros_type_raw + '_Response', base_ns)
      for x in process_type(request_type, srv_spec.request, base_ns, gen_enums, gen_constants):
        yield x
      for x in process_type(response_type, srv_spec.response, base_ns, gen_enums, gen_constants):
        yield x
      return
    except LookupError:
      pass

    raise RuntimeError("ROS Message or Service type {} not found. Ensure any necessary packages are on your path.".format(msg_type.ros_type))

def process_type(msg_type, spec, base_ns, gen_enums, gen_constants):
  types = [Type(field.type.type, base_ns) for field in spec.fields]

  # generate constants definitions
  if gen_enums:
      for x in gen_constants_enums(msg_type, spec):
          yield x
  if gen_constants:
      for x in gen_constants_table(msg_type, base_ns, spec):
          yield x

  # generate type definition
  yield "namespace {};".format(msg_type.full_namespace())
  for x in gen_table(msg_type, spec.fields, base_ns):
      yield x

def generate_base_schema(base_ns):
  for x in gen_base_schema(base_ns):
        yield x

def generate_schema(msg_type_names, base_ns, gen_enums, gen_constants, depended_packages=set()):
    """ Generate Flatbuffers .fbs schema for several ROS message types, including dependencies. """
    defined_types = set(base_defined_types)
    yield "// Generated by msg2fbs; do not edit."
    # add dependency on the base schema, which lives in the robofleet_client package
    yield 'include "base_schema.fbs";'
    for package in depended_packages:
        yield 'include "' + package.name + '.fbs";'
    for msg_type_name in msg_type_names:
        for x in gen_msg(Type(msg_type_name, base_ns), defined_types, base_ns, gen_enums, gen_constants):
            yield x

if __name__ == "__main__":
    ap = ArgumentParser("msg2fbs.py", description=__doc__)
    ap.add_argument("message_type", nargs="+",
        help="ROS names specifying which messages to generate (e.g. std_msgs/String)")
    ap.add_argument("--output-file", "-o", nargs="?", default=None,
        help="Specify an output file. Otherwise, the schema is written to stdout.")
    ap.add_argument("--base-namespace", "-n", nargs="?", default="fb", type=str,
        help="Base namespace for Flatbuffers types, to avoid collisions with ROS types.")
    ap.add_argument("--gen-enums", "-e", action="store_true",
        help="Generate (un)signed integer-type ROS message constants as enums")
    ap.add_argument("--gen-constants", "-c", action="store_true",
        help="Generate ROS message constants as tables with default values")
    ns = ap.parse_args() 
    
    if ns.output_file is None:
        output_file = sys.stdout
    else:
        output_file = open(ns.output_file, "w+")
    
    lines = generate_schema(ns.message_type, base_ns=ns.base_namespace, gen_enums=ns.gen_enums, gen_constants=ns.gen_constants)
    output_file.writelines(line + os.linesep for line in lines)
    output_file.close()
