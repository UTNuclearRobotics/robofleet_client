#!/usr/bin/env python3
######################################################################
#      Title     : generate_plugin_pkg.py
#      Author    : Blake Anderson
#      Copyright : Copyright The University of Texas at Austin, 2022.
#                  All rights reserved.
######################################################################

import argparse
import glob
import robofleet_client.generate.msg2fbs.msg2fbs as msg2fbs
import robofleet_client.generate.msg2fbs.msg_util as msg_util
import pathlib
import os
import re
from ament_index_python.packages import get_package_prefix, get_package_share_path
from rosidl_runtime_py import get_message_interfaces, get_service_interfaces
from rosidl_adapter.parser import MessageSpecification, ServiceSpecification
import shutil
import subprocess
import sys

class MsgSpecHashable(MessageSpecification):
  def __init__(self, spec):
    #pkg_name = parts = spec.base_type.split('/')[0]
    super().__init__(spec.base_type.pkg_name, spec.msg_name, spec.fields, spec.constants)
  
  def __hash__(self):
    return hash(self.base_type)

class SrvSpecHashable(ServiceSpecification):
  def __init__(self, spec):
    super().__init__(spec.pkg_name, spec.service_name, spec.request, spec.response)
  
  def __hash__(self):
    return hash(self.pkg_name + '/' + self.service_name)    

class PackageData:
  """
  Holds the name and message data for a package.
  """
  def __init__(self, msg_pkg):
    self.name = msg_pkg
    self.plugin_pkg_name = msg_pkg + '_robofleet'
    self.messages = set()
    self.services = set()
    self.all_messages = set()

  def __hash__(self):
    """ So we can use this as a dictionary key. """
    return hash(self.name)

  def __eq__(self, other):
    """ Equality with peer instances and with strings. """
    if not other:
      return False
    if isinstance(other, PackageData):
      return self.name == other.name
    if isinstance(other, str):
      return self.name == other

    return False

  def __str__(self):
    output = 'PACKAGE: ' + self.name

    if self.messages:
      output += '\n\tMessages'
      for msg in self.messages:
        output += '\n\t\t' + str(msg.base_type)
    if self.services:
      output += '\n\tServices'
      for srv in self.services:
        output += '\n\t\t' + str(srv.base_type)

    return output


def parse_args(args):
  """ Parses the command line args. """
  default_output_path = pathlib.Path(get_package_share_path('robofleet_client'), 'generate/output')

  parser = argparse.ArgumentParser(description="Generates plugin packages which expose ROS msgs and services to robofleet_client.")
  parser.add_argument('-o', '--out', default=default_output_path,
                      help="Specify the output directory. Defaults to 'robofleet_client/scripts/generate/output'")
  parser.add_argument('-w', '--overwrite', action='store_true',
                      help='If the requested plugin packages already exist in the output location, they will be deleted and recreated.')
  parser.add_argument('-i', '--leave-schema', action='store_true',
                      help='Intermediate flatbuffer schema files will not be deleted during cleanup.')
  parser.add_argument('packages', nargs='+',
                      help='The msg or srv packages that we want to generate plugins for.'
                           ' The program will automatically include dependencies of the listed packages.')

  return parser.parse_args(args)


def check_product_existence(packages, output_dir, overwrite):
  """
  Returns False if any of the requested package directories already
  exist in the output directory, AND we are not allowed to overwrite them.
 """
  for package in packages:
    # does the generated package exist in the output directory?
    if pathlib.Path(output_dir, package.plugin_pkg_name).exists():
      # if yes, we must be in overwrite mode to proceed
      if overwrite:
        # delete the package
        shutil.rmtree(pathlib.Path(output_dir, package.plugin_pkg_name))
      else:
        # exit with error message
        print('ERROR: Requested package {} already exists. '
              'Either delete the package or use the --overwrite flag.'.format(package.plugin_pkg_name))
        return False
  
  return True



def get_msg_and_srv_data(package, msg_depends_graph, package_depends_graph):
  """
  Looks up the messages and services of a package. Fills out the dependency
  graphs for the package and for its messages.
  """

  # add the package to the dependency graph
  package_depends_graph[package] = set()

  # get the names of all the messages in the package
  try:
    message_names = get_message_interfaces([package.name])
  except LookupError as err:
    print('ERROR: Package {} not found.').format(err.args[0])
    print(err)
    return 1
  
  # get the names of all the services in the package
  try:
    service_names = get_service_interfaces([package.name])
  except LookupError as err:
    print('ERROR: Package {} not found.').format(err.args[0])
    print(err)
    return 1
  
  if not message_names and not service_names:
    print('WARNING: No messages or services found for package' + package.name)
  
  # get the definitions of each message
  if package.name in message_names.keys():
    package.messages = set(MsgSpecHashable(msg_util.get_msg_spec(package.name, message_name))
                          for message_name in message_names[package.name])

  # get the definitions of each service
  if package.name in service_names.keys():
    package.services = set(SrvSpecHashable(msg_util.get_srv_spec(package.name, service_name))
                          for service_name in service_names[package.name])

  # service consists of a Request message and a Response message
  # combine all the regular messages and service messages together
  package.all_messages = package.messages.union(set(MsgSpecHashable(x.request) for x in package.services),
                                        set(MsgSpecHashable(x.response) for x in package.services))

  # get all the dependencies of the package
  for message in package.messages.union(set(MsgSpecHashable(x.request) for x in package.services),
                                        set(MsgSpecHashable(x.response) for x in package.services)):
    msg_depends_graph[message.base_type] = set()

    # iterate over the fields of each message
    for field in message.fields:
      # check if this field's type is a primitive
      if not field.type.is_primitive_type():

        # get the package this field's type is from
        dep_package_name = field.type.pkg_name
        msg_depends_graph[message.base_type].add(field.type)

        # don't depend on self
        if dep_package_name != package.name:
          new_package = PackageData(dep_package_name)
          package_depends_graph[package].add(new_package)

          # if we have not seen this dependency package before, process it recursively
          if new_package not in package_depends_graph.keys():
            print('Adding dependency {} of {}'.format(dep_package_name, package.name))
            get_msg_and_srv_data(new_package, msg_depends_graph, package_depends_graph)



def generate_directories(pkg_name: str, output_path: pathlib.Path):
  """
  Creates the directory tree for the plugin package, in the 'output' directory.
  """

  print('Generating ' + pkg_name)
  
  if not output_path.is_dir():
    output_path.mkdir(parents=True)

  package_dir = pathlib.Path(output_path, pkg_name)

  try:
    pathlib.Path(package_dir, 'include', pkg_name, 'msg').mkdir(parents=True)
    pathlib.Path(package_dir, 'include', pkg_name, 'srv').mkdir(parents=True)
    pathlib.Path(package_dir, 'src/msg').mkdir(parents=True)
    pathlib.Path(package_dir, 'src/srv').mkdir(parents=True)
  except OSError as err:
    print(err.strerror)
    return False

  return True


def generate_cmakelists(package, depends, output_path, templates_path):
  """
  Builds the plugin package's CMakelists.txt from a template.
  """
  cmakelists_template_path = pathlib.Path(templates_path, 
                                          'template_CMakeLists')
  cmakelists_out_path = pathlib.Path(output_path,
                                     package.plugin_pkg_name,
                                     'CMakeLists.txt')

  # read in the template
  try:
    with open(cmakelists_template_path, 'r') as file:
      filedata = file.read()
  except IOError as err:
    print('ERROR: Failed to read CMakeLists template at ' + err.filename)
    print(err.strerror)
    return False

  # Build find_package({dependencies})
  find_dependencies_str = '\n'.join(['find_package({}_robofleet)'.format(x.name) for x in depends])
  dependencies_str = '\n'.join(['\t{}_robofleet'.format(x.name) for x in depends])

  # Source files for the library target
  source_files_list = '\n'.join(['\tsrc/msg/{}.cpp'.format(message.msg_name)
                       for message in package.messages] +
                       ['\tsrc/srv/{}.cpp'.format(service.srv_name)
                       for service in  package.services])

  # replace the target strings
  filedata = filedata.format(msg_package=package.name,
                             sources_list=source_files_list,
                             find_dependencies=find_dependencies_str,
                             dependencies=dependencies_str)

  # write the filled out CMakelists
  try:
    with open(cmakelists_out_path, 'w') as file:
      file.write(filedata)
  except OSError as err:
    print('ERROR: Failed to write CMakeLists.txt. ' + err.strerror)
    return False

  return True



def generate_package_xml(package, depends, output_path, templates_path):
  """
  Builds the plugin package's Package.xml from a template.
  """
  xml_template_path = pathlib.Path(templates_path, 'template_packagexml')
  xml_out_path = pathlib.Path(output_path, package.plugin_pkg_name, 'package.xml')

  # read in the template
  try:
    with open(xml_template_path, 'r') as file :
      filedata = file.read()
  except IOError:
    print('ERROR: Failed to read package.xml template.')
    return False

  # Build dependencies
  dependencies = '\n'.join(['\t<depend>{}_robofleet</depend>'.format(x.name)
                   for x in depends])

  # replace the target strings
  filedata = filedata.format(msg_package=package.name,
                             dependencies=dependencies)

  # write the filled out package.xml
  try:
    with open(xml_out_path, 'w') as file:
      file.write(filedata)
  except OSError as err:
    print('ERROR: Failed to write package.xml. ' + err.strerror)
    return False

  return True



def generate_msg_headers(message, msg_depends_graph, output_path, templates_path):
  """
  Generates the header file for a message's handler classes
  """
  header_template_path = pathlib.Path(templates_path, 'template_msg_header')
  header_out_path = pathlib.Path(output_path,
                                 message.base_type.pkg_name + '_robofleet',
                                 'include',
                                 message.base_type.pkg_name + '_robofleet',
                                 'msg',
                                 message.msg_name + '.hpp')

  # read in the template
  try:
    with open(header_template_path, 'r') as file :
      filedata = file.read()
  except IOError:
    print('ERROR: Failed to read msg header template.')
    return False

  # includes for the message types used in this message's fields
  dependencies = '\n'.join(['#include <{}_robofleet/{}.hpp>'.format(depend.pkg_name, depend.type)\
                           for depend in msg_depends_graph[message.base_type]])
  
  # replace the target strings
  filedata = filedata.format(msg_package=message.base_type.pkg_name,
                             msg_name_lowercase=message.msg_name.lower(),
                             msg_name=message.msg_name,
                             dependencies=dependencies)

  # write the filled out class header
  try:
    with open(header_out_path, 'w') as file:
      file.write(filedata)
  except OSError as err:
    print('ERROR: Failed to write msg header {}. {}'.format(err.filename, err.strerror))
    return False

  return True



def generate_msg_impl(message, output_path, templates_path):
  """
  Generates the implementation (.cpp) file for a message's handler classes
  """
  impl_template_path = pathlib.Path(templates_path, 'template_msg_impl')
  impl_out_path = pathlib.Path(output_path,
                               message.base_type.pkg_name + '_robofleet',
                               'src/msg',
                               message.msg_name + '.cpp')

  # read in the template
  try:
    with open(impl_template_path, 'r') as file :
      filedata = file.read()
  except IOError:
    print('ERROR: Failed to read msg implementation template.')
    return False

  # this is the code for assignments from flatbuffer fields to ROS fields
  msg_decode_assignments = ''

  # code for assignments from ROS fields to flatbuffer fields
  msg_encode_assignments = ''

  for field in message.fields:
    field_name_decode = 'src->' + field.name + '()'
    field_name_encode = 'msg.' + field.name

    # handle variables that themselves require a call to a conversion function
    # RosTime and RosDuration require special handling
    # also have to handle arrays and primitives
    if not field.type.is_primitive_type() or field.type.type in ['string']\
          or (not field.type.is_primitive_type() and field.is_array):

      # a modifier string to add to the end of the conversion function name
      p = ''

      # construct function template parameters to help the compiler find the right overloads.
      # TODO: remove line - type_with_c_ns = field.type.replace('/','::')
      if field.type.type == 'string' and field.type.is_array:
        p = '<std::string,flatbuffers::String>'
      
      # decide if we should be calling the primitive set of conversion templates,
      # or the set for compound types. Only compound types require the
      # template parameters constructed above
      if field.type.pkg_name is None and field.type.is_array and field.type.type != 'string':
        p = 'Primitive'
      field_name_encode = '::RostoFb{}(fbb, {})'.format(p, field_name_encode)

      if field.type.array_size is None:
        field_name_decode = '::FbtoRos{}({})'.format(p, field_name_decode)
      else:
        # ROS uses boost::array to represent fixed-length vector message fields
        # We need to help the compiler find the right template overload
        # in this case.

        # We also need to replace some ROS base types with ones that C++ understands
        replacements = {'float32': 'float',
                        'float64': 'double',
                        'int8': 'int8_t',
                        'uint8': 'uint8_t'}
        t = field.base_type
        if field.base_type in replacements:
          t = replacements[field.base_type]

        # here we add text for the explicit template specification for arrays.
        # the C++ compiler needs this to distinguish them.
        field_name_decode = '::FbtoRos{}<{}, {}>({})'.format(p, t,
                                                           field.array_len,
                                                           field_name_decode)

    # text to assign fields from fb objects to ROS messages
    msg_decode_assignments += ('\n\t\tmsg.{}={};'.format(field.name, field_name_decode))
    # text to assign fields from ROS messages to fb objects
    msg_encode_assignments += (',\n\t\t\t\t' + field_name_encode)
  

  filedata = filedata.format(msg_package=message.base_type.pkg_name,
                             msg_name=message.msg_name,
                             msg_decode_assignments=msg_decode_assignments,
                             msg_encode_assignments=msg_encode_assignments)

  # write the filled out class impl
  try:
    with open(impl_out_path, 'w') as file:
      file.write(filedata)
  except OSError as err:
    print('ERROR: Failed to write msg implementation. ' + err.strerror)
    return False

  return True



def generate_srv_headers(service, msg_depends_graph, output_path, templates_path):
  """
  Generates the header file for a service's handler classes
  """
  header_template_path = os.path.join(templates_path, 'template_srv_header')
  header_out_path = os.path.join(output_path,
                                 service.package + '_robofleet',
                                 'include',
                                 service.package + '_robofleet',
                                 'srv',
                                 service.short_name + '.hpp')

  request = service.request
  response = service.response

  # read in the template
  try:
    with open(header_template_path, 'r') as file :
      filedata = file.read()
  except IOError:
    print('ERROR: Failed to read srv header template.')
    return False

  # includes for the service types used in this service's fields
  dependencies = '\n'.join(['#include <{}_robofleet/{}.hpp>'.format(*depend.split('/'))\
                           for depend in msg_depends_graph[request.base_type.type] | msg_depends_graph[response.base_type.type]])

  # replace the target strings
  filedata = filedata.format(srv_package=request.package,
                             srv_type=service.srv_name,
                             srv_type_lowercase=service.msg_name.lower(),
                             request_type=service.request.msg_name,
                             response_type=service.response.msg_name,
                             dependencies=dependencies)

  # write the filled out class header
  try:
    with open(header_out_path, 'w') as file:
      file.write(filedata)
  except OSError as err:
    print('ERROR: Failed to write srv header. ' + err.strerror)
    return False

  return True



def generate_srv_impl(service, output_path, templates_path):
  """
  Generates the implementation (.cpp) file for a service's handler classes
  """
  impl_template_path = os.path.join(templates_path, 'template_srv_impl')
  impl_out_path = os.path.join(output_path,
                               service.package + '_robofleet',
                               'src/srv',
                               service.short_name + '.cpp')

  request = service.request
  response = service.response

  # read in the template
  try:
    with open(impl_template_path, 'r') as file :
      filedata = file.read()
  except IOError:
    print('ERROR: Failed to read srv implementation template.')
    return False

  # this is the code for assignments from flatbuffer fields to ROS fields
  request_decode_assignments = ''

  # code for assignments from ROS fields to flatbuffer fields
  request_encode_assignments = ''

  for field in request.parsed_fields():

    # We need the call to lower() because some ROS services don't follow the
    # convention of all lower case.
    field_name_decode = 'src->' + field.name.lower() + '()'
    field_name_encode = 'msg.' + field.name

    # handle variables that themselves require a call to a conversion function
    # RosTime and RosDuration require special handling
    # also have to handle arrays and primitives
    if not field.is_builtin or field.base_type in ['string', 'time', 'duration']\
          or (field.is_builtin and field.is_array):

      # a modifier string to add to the end of the conversion function name
      p = ''

      # construct function template parameters to help the compiler find the right overloads.
      type_with_c_ns = field.base_type.replace('/','::')
      if field.base_type == 'string' and field.is_array:
        p = '<std::string,flatbuffers::String>'
      elif field.base_type == 'time' and field.is_array:
        p = '<ros::Time,fb::RosTime'
      elif field.base_type == 'duration' and field.is_array:
        p = '<rclcpp::Duration,fb::RosDuration'
      
      # decide if we should be calling the primitive set of conversion templates,
      # or the set for compound types. Only compound types require the
      # template parameters constructed above
      if field.is_builtin and field.is_array and field.base_type != 'string':
        p = 'Primitive'
      field_name_encode = '::RostoFb{}(fbb, {})'.format(p, field_name_encode)

      if field.array_len is None:
        field_name_decode = '::FbtoRos{}({})'.format(p, field_name_decode)
      else:
        # ROS uses boost::array to represent fixed-length vector fields
        # We need to help the compiler find the right template overload
        # in this case.

        # We also need to replace some ROS base types with ones that C++ understands
        replacements = {'float32': 'float',
                        'float64': 'double',
                        'int8': 'int8_t',
                        'uint8': 'uint8_t'}
        t = field.base_type
        if field.base_type in replacements:
          t = replacements[field.base_type]

        # here we add text for the explicit template specification for arrays.
        # the C++ compiler needs this to distinguish them.
        field_name_decode = '::FbtoRos{}<{}, {}>({})'.format(p, t,
                                                           field.array_len,
                                                           field_name_decode)

    # text to assign fields from fb objects to ROS services
    request_decode_assignments += ('\n\t\tmsg.{}={};'.format(field.name, field_name_decode))
    # text to assign fields from ROS services to fb objects
    request_encode_assignments += (',\n\t\t\t\t' + field_name_encode)
  
  ''' Build the encode and decode functions for the response type '''

  # this is the code for assignments from flatbuffer fields to ROS fields
  response_decode_assignments = ''

  # code for assignments from ROS fields to flatbuffer fields
  response_encode_assignments = ''
  
  for field in response.parsed_fields():

    # We need the call to lower() because some ROS services don't follow the
    # convention of all lower case.
    field_name_decode = 'src->' + field.name.lower() + '()'
    field_name_encode = 'msg.' + field.name

    # handle variables that themselves require a call to a conversion function
    # RosTime and RosDuration require special handling
    # also have to handle arrays and primitives
    if not field.is_builtin or field.base_type in ['string', 'time', 'duration']\
          or (field.is_builtin and field.is_array):

      # a modifier string to add to the end of the conversion function name
      p = ''

      # construct function template parameters to help the compiler find the right overloads.
      type_with_c_ns = field.base_type.replace('/','::')
      if field.type.type == 'string' and field.type.is_array:
        p = '<std::string,flatbuffers::String>'
      
      # decide if we should be calling the primitive set of conversion templates,
      # or the set for compound types. Only compound types require the
      # template parameters constructed above
      if field.is_builtin and field.type.is_array and field.type.type != 'string':
        p = 'Primitive'
      field_name_encode = '::RostoFb{}(fbb, {})'.format(p, field_name_encode)

      if field.array_size is None:
        field_name_decode = '::FbtoRos{}({})'.format(p, field_name_decode)
      else:
        # ROS uses boost::array to represent fixed-length vector fields
        # We need to help the compiler find the right template overload
        # in this case.

        # We also need to replace some ROS base types with ones that C++ understands
        replacements = {'float32': 'float',
                        'float64': 'double',
                        'int8': 'int8_t',
                        'uint8': 'uint8_t'}
        t = field.base_type
        if field.base_type in replacements:
          t = replacements[field.base_type]

        # here we add text for the explicit template specification for arrays.
        # the C++ compiler needs this to distinguish them.
        field_name_decode = '::FbtoRos{}<{}, {}>({})'.format(p, t,
                                                           field.array_len,
                                                           field_name_decode)

    # text to assign fields from fb objects to ROS services
    response_decode_assignments += ('\n\t\tmsg.{}={};'.format(field.name, field_name_decode))
    # text to assign fields from ROS services to fb objects
    response_encode_assignments += (',\n\t\t\t\t' + field_name_encode)

  filedata = filedata.format(srv_package=service.package,
                             srv_type=service.srv_name,
                             request_type=service.request.msg_name,
                             response_type=service.response.msg_name,
                             response_decode_assignments=response_decode_assignments,
                             response_encode_assignments=response_encode_assignments,
                             request_decode_assignments=request_decode_assignments,
                             request_encode_assignments=request_encode_assignments)

  # write the filled out class impl
  try:
    with open(impl_out_path, 'w') as file:
      file.write(filedata)
  except OSError as err:
    print('ERROR: Failed to write srv implementation. ' + err.strerror)
    return False

  return True


def generate_plugin_manifest(package, output_path, templates_path):
  """
  Build the plugin descriptions file which exposes the
  message handlers to pluginlib
  """
  msg_template_path = pathlib.Path(templates_path, 
                                        'template_msg_plugin_description')
  srv_template_path = pathlib.Path(templates_path, 
                                        'template_srv_plugin_description')
  manifest_out_path = pathlib.Path(output_path,
                                   package.plugin_pkg_name,
                                   'plugin_description.xml')

  # read in the template
  try:
    with open(msg_template_path, 'r') as msg_file :
      msgdata = msg_file.read()
  except IOError:
    print('ERROR: Failed to read plugin_description.xml template.')
    return False

  try:
    with open(srv_template_path, 'r') as srv_file :
      srvdata = srv_file.read()
  except IOError:
    print('ERROR: Failed to read plugin_description.xml template.')
    return False

  # build the contents from template
  output = '<library path="lib/lib{msg_package}_robofleet">\n'
  for message in package.messages:
    output += msgdata.format(msg_package=package.name,
                              msg_name=message.base_type.type,
                              msg_full_name=message.msg_name)
  for service in package.services:
    output += srvdata.format(msg_package=package.name,
                              srv_name=service.base_type.type,
                              srv_full_name=service.msg_name)
  output += '</library>'
  output = output.format(msg_package=package.name)

  # write the filled out plugin_description.xml
  try:
    with open(manifest_out_path, 'w') as file:
      file.write(output)
  except OSError as err:
    print('ERROR: Failed to write plugin_description.xml. ' + err.strerror)
    return False

  return True



def generate_base_schema_file(msg2fbs_dir, includes_destination):
  """
  Creates and compiles the schema file for the base definitions (MetaData, etc).
  Also copies the compiled file to robofleet_client's public includes location,
  where the plugin packages can see it.

  This uses the msg2fbs subpackage to generate schema definition (.fbs) files.
  """

  # generate the schema data with the base definitions
  schema_data = msg2fbs.generate_base_schema(base_ns='fb')

  # save the schema definitions file
  schema_filename = "base_schema.fbs"
  try:
    path = pathlib.Path(msg2fbs_dir, schema_filename)
    output_file = open(path, "w+")
  except OSError as err:
    print('ERROR: Failed to write base definitions schema file to ' + err.filename)
    return False
  
  output_file.writelines(line + os.linesep for line in schema_data)
  output_file.close()

  # compile the schema by calling the flatc compiler
  # we actually call a make file which calls flatc
  print('Compiling flatbuffer schema for base definitions')
  try:
    make_file_path = pathlib.Path(msg2fbs_dir, schema_filename)
    # TODO: Add a timeout in Python3
    compile_prcs = subprocess.check_call(['make', 'all', 'SCHEMA_FILE=base_schema', '--directory='+str(msg2fbs_dir)],
                                         stdout=subprocess.PIPE)
  except subprocess.CalledProcessError:
    print('ERROR: Failed to compile the flatbuffer schema for base definitions')
    return False
  
  # copy the compiled file to robofleet_client's public includes location
  try:
    shutil.copyfile(pathlib.Path(msg2fbs_dir, 'base_schema_generated.h'),
                    pathlib.Path(includes_destination, 'base_schema_generated.h'))
  except PermissionError:
    print("Permission denied to copy the base schema file to the robofleet_client includes.")
    return False

  return True

def generate_flatbuffer_schema(package,
                               dependency_graph, 
                               generated_packages, 
                               output_dir, 
                               msg2fbs_dir):
  """
  Generates schema definition (.fbs) files and then also compiles them
  into C++ headers.

  This function calls itself recursively such that we build dependencies of
  a package before building the package itself.
  """
  # generate .fbs files for all the packages, from the bottom of the dependency graph
  for depend in dependency_graph[package]:
    if depend not in generated_packages:
      # we need to find the object in the keys, since that has all the message data for the package
      for key in dependency_graph.keys():
        if key.name == depend.name:
          # /!\ recursion /!\
          generate_flatbuffer_schema(key, dependency_graph, generated_packages, output_dir, msg2fbs_dir)
  
  if package in generated_packages:
    return True
    
  generated_packages.add(package)
  schema_data = msg2fbs.generate_schema([str(message.base_type) for message in package.messages | package.services],
                                        depended_packages=dependency_graph[package],
                                        base_ns='fb',
                                        gen_enums=False,
                                        gen_constants=False)

  # save the file
  schema_filename = package.name + ".fbs"
  try:
    path = pathlib.Path(msg2fbs_dir, schema_filename)
    output_file = open(path, "w+")
  except OSError:
    print('ERROR: Failed to write flatbuffer schema file.')
    return False
  
  output_file.writelines(line + os.linesep for line in schema_data)
  output_file.close()

  # compile the definitions file
  compile_flatbuffer_schema(package, msg2fbs_dir)

  return True


def compile_flatbuffer_schema(package, msg2fbs_dir):
  """
  Compiles a .fbs definitions into C++ headers.
  We do this by calling make using a makefile in the msg2fbs directory.
  """
  schema_filename = package.name + ".fbs"

  # compile the schema by calling the flatc compiler
  print('Compiling flatbuffer schema for ' + package.plugin_pkg_name)
  try:
    make_file_path = pathlib.Path(msg2fbs_dir, schema_filename)
    
    compile_prcs = subprocess.check_call(['make', 'all', 'SCHEMA_FILE='+package.name, '--directory='+str(msg2fbs_dir)],
                                         stdout=subprocess.PIPE, timeout=60.0)
  except subprocess.CalledProcessError:
    print('ERROR: Failed to compile the flatbuffer schema for ' + package.name)
    return TimeoutError
  except subprocess.CalledProcessError:
    print('ERROR: Compillation of flatbuffer schema timed out for ' + package.name)
    return False

  return True


def modify_and_install_schema(packages, output_path, msg2fbs_dir):
  '''
  Modifies a compiled schema file so that its include paths will 
  correctly point to the other plugin packages.

  e.g. 'sensor_msgs_generated.h' becomes 'sensor_msgs_robofleet/sensor_msgs_generated.h'
  '''

  # modify the include paths in the compiled file so it will 
  for package in packages:
    # open this package's schema file
    filename = package.name+'_generated.h'
    try:
      path = os.path.join(msg2fbs_dir, filename)
      with open(path, 'r') as file :
        lines = file.readlines()
      file.close()
    except IOError:
      print('ERROR: Failed to read compiled schema during modification step.')
      return False

    # modify the contents
    modified_data = ''
    for line in lines:
      # regex magic to extract the package name from the include statement
      match = re.match('#include "(?P<name>\w+)_generated.h"', line)
      
      # prepend the package name onto the include path.
      # the base_schema.h file is a special case since it
      # resides in robofleet_client
      if match:
        pkg_name = match.group('name')
        if pkg_name == 'base_schema':
          line = line.replace(pkg_name, 'robofleet_client/'+pkg_name)
        else:
          line = line.replace(pkg_name, pkg_name+'_robofleet/'+pkg_name)

      modified_data += line

    # write the modified file to its package location
    schema_out_path = os.path.join(output_path,
                                   package.plugin_pkg_name,
                                   'include',
                                   package.plugin_pkg_name,
                                   filename)
    try:
      with open(schema_out_path, 'w') as file:
        file.write(modified_data)
    except OSError:
      print('ERROR: Failed to write schema file to package ' + package.plugin_pkg_name)
      return False
  
  return True



def generate_plugin_packages(packages,
                             msg_depends_graph,
                             output_path: pathlib.Path,
                             templates_path: pathlib.Path,
                             msg2fbs_dir: pathlib.Path):

  for package,depends in packages.items():
    if not generate_directories(package.plugin_pkg_name, output_path):
      return False

    if not generate_cmakelists(package, depends, output_path, templates_path):
      return False

    if not generate_package_xml(package, depends, output_path, templates_path):
      return False

    for message in package.messages:
      if not generate_msg_headers(message, msg_depends_graph, output_path, templates_path):
        return False

      if not generate_msg_impl(message, output_path, templates_path):
        return False

    for service in package.services:
      if not generate_srv_headers(service, msg_depends_graph, output_path, templates_path):
        return False
        
      if not generate_srv_impl(service, output_path, templates_path):
        return False

    if not generate_plugin_manifest(package, output_path, templates_path):
      return False

  """
  We now must generate the schema code for the types used by flatbuffers
  to encode and transmit the data to the server.

  1. Generate base_schema.fbs which contains the definitions for metadata and
     other base types.
  2. Generate <package_name>.fbs files with the definitions for each
     package's messages. This must be done from the bottom of the
     dependency graph.
  3. Compile each of the definition files into C++ headers: <package_name_generated.h
     Again, this must happen in dependency order.
  4. Modify the #include statements in the C++ headers so they can find each other
     in their eventual locations in the plugin packages.
  5. Place the C++ headers in their respective plugin packages.
  6. Clean up the definition files, if the user doesn't specify otherwise.
  """
  includes_path = pathlib.Path(get_package_prefix('robofleet_client'),
                              'include',
                              'robofleet_client')

  if not generate_base_schema_file(msg2fbs_dir, includes_path):
    return False

  generated_packages = set()
  for package in packages:
    if not generate_flatbuffer_schema(package,
                                      packages,
                                      generated_packages,
                                      output_path,
                                      msg2fbs_dir):
      return False

  print('Finished compiling flatbuffer schema.')

  if not modify_and_install_schema(packages, output_path, msg2fbs_dir):
    return False

  return True

def cleanup(msg2fbs_path):
  # delete .fbs files
  for file in msg2fbs_path.glob('*.fbs'):
    try:
      file.unlink()
    except OSError as err:
      print('Unable to delete file ' + file)
      print(err)

  # delete generated header files
  for file in msg2fbs_path.glob('*_generated.h'):
    try:
      file.unlink()
    except OSError as err:
      print('Unable to delete file ' + file)
      print(err)

def main(args):
  my_args = sys.argv

  # parse arguments
  parsed_args = parse_args(my_args[1:])
  num_pkgs = len(parsed_args.packages)
  
  # get input and output locations
  my_share_directory = get_package_share_path('robofleet_client')

  templates_dir = pathlib.Path(my_share_directory, 'generate/templates')
  msg2fbs_dir = pathlib.Path(my_share_directory, 'generate/msg2fbs')
  output_dir = pathlib.Path(parsed_args.out)

  input_package_set = set(parsed_args.packages)

  # check for duplicates in the requested packages
  if len(input_package_set) != len(parsed_args.packages):
    raise argparse.ArgumentError('The provided package list contains duplicates.')
    return 1

  requested_packages = set(PackageData(msg_package) for msg_package in input_package_set)
  
  # get msgs, services, and actions for listed packages
  # after this loop, package_depends_graph will contain the full set of dependency relations
  package_depends_graph = dict()
  msg_depends_graph = dict()
  for package in requested_packages:
    get_msg_and_srv_data(package, msg_depends_graph, package_depends_graph)
  
  # print found messages and services
  print('---------------------------')
  print('Found Messages and Services')
  for package in package_depends_graph.keys():
    print(package)

  # check if any of the listed packages already exist, and delete them if allowed
  if not check_product_existence(package_depends_graph.keys(), output_dir, parsed_args.overwrite):
    return 1

  print('---------------------------')
  print(pathlib.Path('Generating output in ', output_dir))

  # create the output
  if not generate_plugin_packages(package_depends_graph,
                                  msg_depends_graph,
                                  output_dir,
                                  templates_dir,
                                  msg2fbs_dir):
    return 2

  print('---------------------------')
  print('Package Generation Complete!')

  if parsed_args.leave_schema:
    print('Leaving intermediate flatbuffer schema files.')
  else:
    cleanup(msg2fbs_dir)
    print('Cleanup complete.')

  return 0

if __name__ == '__main__':
  result = main(sys.argv)
  exit(result)