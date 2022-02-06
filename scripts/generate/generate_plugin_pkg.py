#!/usr/bin/env python
######################################################################
#      Title     : generate_plugin_pkg.py
#      Author    : Blake Anderson
#      Copyright : Copyright The University of Texas at Austin, 2022.
#                  All rights reserved.
######################################################################

import argparse
import itertools
import msg2fbs.msg2fbs as msg2fbs
import msg2fbs.msg_util as msg_util
import os.path
import rosmsg
import rospkg.rospack
import shutil
import subprocess
import sys

class PackageData:
  def __init__(self, msg_pkg):
    self.msg_pkg_name = msg_pkg
    self.plugin_pkg_name = msg_pkg + '_robofleet'
    self.messages = []
    self.services = []

  def __str__(self):
    output = 'PACKAGE: ' + self.msg_pkg_name
    for msg in self.messages:
      output = output + '\n\t' + msg.full_name 
    for srv in self.services:
      output = output + '\n\t' + srv.full_name

    return output




def parse_args(args):
  parser = argparse.ArgumentParser("Generates plugin packages which expose ROS msgs and services to robofleet_client.")
  parser.add_argument('-o', '--OVERWRITE', action='store_true',
                      help='If the requested plugin packages already exist, they will be deleted and recreated.')
  parser.add_argument('packages', nargs='+',
                      help='The msg or srv packages that we want to generate plugins for.')

  return parser.parse_args(args)




def check_product_existence(packages, output_dir, overwrite):
  for package in packages:
    if os.path.isdir(os.path.join(output_dir,package.plugin_pkg_name)):
      # if they do, we must be in overwrite mode to proceed
      if overwrite:
        # delete the package
        shutil.rmtree(output_dir + '/' + package.plugin_pkg_name)
      else:
        # exit with error message
        print('ERROR: Requested package ' + package.plugin_pkg_name + ' already exists. '
              'Either delete the package or use the --OVERWRITE flag.')
        return False
  return True



def get_msg_and_srv_data(package, traversed_packages):
  
  # remember that we processed this package
  traversed_packages.add(package)
  
  # get the names of all the messages in the package
  try:
    message_names = set(rosmsg.list_msgs(package.msg_pkg_name))
  except rospkg.common.ResourceNotFound as err:
    print('ERROR: Package ' + err.args[0] + ' not found.')
    print(err)
    return 1
  
  # get the names of all the services in the package
  service_names = set(rosmsg.list_srvs(package.msg_pkg_name))
  
  print('Found package ' + package.msg_pkg_name)
  if not message_names and not service_names:
    print('WARNING: No messages or services found for package' + package.msg_pkg_name)

  # the search paths from which to pull msg data
  search_path = msg_util.get_msg_search_path()

  # get the definitions of each message
  package.messages = [msg_util.get_msg_spec(name, search_path) for name in message_names]

  # get the definitions of each service
  package.services = [msg_util.get_srv_spec(name, search_path) for name in service_names]

  # split the service definitions into Request and Response messages
  for service in package.services:
    # split the contents at the delimiter
    split_contents = service.contents.split("---")

    # create messages
    req = MsgData(package.msg_pkg_name, service.name + 'Request', split_contents[0])
    res = MsgData(package.msg_pkg_name, service.name + 'Response', split_contents[1])

    package.messages.append(req)
    package.messages.append(res)

  # clear the services now that we've generated messages from them
  package.services = []

  # traverse the dependencies
  for message in package.messages:
    for field in message.parsed_fields():
      if '/' in field.base_type:
        # get the part in front of the '/'
        dependency_name = field.base_type.split('/')[0]

        # this conditional prevents duplication
        if dependency_name not in set(x.msg_pkg_name for x in traversed_packages):
          print('Adding dependency ' + dependency_name + ' of ' + package.msg_pkg_name)
          get_msg_and_srv_data(PackageData(dependency_name), traversed_packages)



def generate_directories(name, output_path):
  print('Generating ' + name)
  
  if not os.path.exists(output_path):
    os.mkdir(output_path)

  package_dir = os.path.join(output_path, name)

  try:
    os.mkdir(package_dir)
    os.makedirs(os.path.join(package_dir, 'include', name))
    os.mkdir(os.path.join(package_dir, 'src'))
  except OSError as err:
    print(err)
    return False

  return True


def generate_cmakelists(package, output_path, templates_path):
  cmakelists_template_path = os.path.join(templates_path, 'template_CMakeLists')
  cmakelists_out_path = os.path.join(output_path, package.plugin_pkg_name, 'CMakeLists.txt')

  # read in the template
  try:
    with open(cmakelists_template_path, 'r') as file :
      filedata = file.read()
  except IOError:
    print('ERROR: Failed to read CMakeLists template.')
    return False

  source_files_list = ''
  for message in package.messages:
    source_files_list = source_files_list + "  src/" + message.short_name + ".cpp\n"

  # replace the target strings
  filedata = filedata.format(msg_package=package.msg_pkg_name,
                             sources_list=source_files_list)

  # write the filled out CMakelists
  try:
    with open(cmakelists_out_path, 'w') as file:
      file.write(filedata)
  except OSError:
    print('ERROR: Failed to write CMakeLists.txt.')
    return False

  return True



def generate_package_xml(package, output_path, templates_path):
  xml_template_path = os.path.join(templates_path, 'template_packagexml')
  xml_out_path = os.path.join(output_path, package.plugin_pkg_name, 'package.xml')

  # read in the template
  try:
    with open(xml_template_path, 'r') as file :
      filedata = file.read()
  except IOError:
    print('ERROR: Failed to read package.xml template.')
    return False

  # replace the target strings
  filedata = filedata.format(msg_package=package.msg_pkg_name)

  # write the filled out package.xml
  try:
    with open(xml_out_path, 'w') as file:
      file.write(filedata)
  except OSError:
    print('ERROR: Failed to write package.xml.')
    return False

  return True



def generate_msg_headers(message, output_path, templates_path):
  header_template_path = os.path.join(templates_path, 'template_msg_header')
  header_out_path = os.path.join(output_path,
                                 message.package + '_robofleet',
                                 'include',
                                 message.package + '_robofleet',
                                 message.short_name + '.h')

  # read in the template
  try:
    with open(header_template_path, 'r') as file :
      filedata = file.read()
  except IOError:
    print('ERROR: Failed to read msg header template.')
    return False

  # replace the target strings
  filedata = filedata.format(msg_package=message.package,
                             msg_name=message.short_name)

  # write the filled out class header
  try:
    with open(header_out_path, 'w') as file:
      file.write(filedata)
  except OSError:
    print('ERROR: Failed to write msg header.')
    return False

  return True



def generate_msg_impl(message, output_path, templates_path):
  impl_template_path = os.path.join(templates_path, 'template_msg_impl')
  impl_out_path = os.path.join(output_path,
                                 message.package + '_robofleet',
                                 'src',
                                 message.short_name + '.cpp')

  # read in the template
  try:
    with open(impl_template_path, 'r') as file :
      filedata = file.read()
  except IOError:
    print('ERROR: Failed to read msg implementation template.')
    return False

  # replace the target strings
  # TODO: Corrie magic for the encode and decode functions
  msg_decode_assignments = ''
  msg_encode_assignments = ''
  for field_type,field_name in itertools.izip(message.types, message.names):

    # handle c-string variables
    field_name_decode = field_name
    field_name_encode = field_name
    if field_type == 'string':
      field_name_decode = field_name_decode + '->str'
      field_name_encode = field_name_encode + '.c_str()'

    msg_decode_assignments = msg_decode_assignments + 'msg.' +\
                                                    field_name +\
                                                    '=root->' +\
                                                    field_name_decode +\
                                                    '();\n'

    msg_encode_assignments = msg_encode_assignments + ',\nmsg.' + field_name_encode

  filedata = filedata.format(msg_package=message.package,
                             msg_name=message.short_name,
                             msg_decode_assignments=msg_decode_assignments,
                             msg_encode_assignments=msg_encode_assignments)

  # write the filled out class impl
  try:
    with open(impl_out_path, 'w') as file:
      file.write(filedata)
  except OSError:
    print('ERROR: Failed to write msg implementation.')
    return False

  return True



def generate_plugin_manifest(package, output_path, templates_path):
  manifest_template_path = os.path.join(templates_path, 'template_plugin_description')
  manifest_out_path = os.path.join(output_path, package.plugin_pkg_name, 'plugin_description.xml')

  # read in the template
  try:
    with open(manifest_template_path, 'r') as file :
      filedata = file.read()
  except IOError:
    print('ERROR: Failed to read plugin_description.xml template.')
    return False

  output = '<library path="lib/lib{msg_package}_robofleet">\n'
  for message in package.messages:
    output = output + filedata.format(msg_package=package.msg_pkg_name,
                                      msg_name=message.short_name,
                                      msg_full_name=message.full_name)
  output = output + '</library>'
  output = output.format(msg_package=package.msg_pkg_name)

  # write the filled out plugin_description.xml
  try:
    with open(manifest_out_path, 'w') as file:
      file.write(output)
  except OSError:
    print('ERROR: Failed to write plugin_description.xml.')
    return False

  return True

def generate_flatbuffer_schema(package, output_dir, msg2fbs_dir):
  # generate the schema tables file
  schema = msg2fbs.generate_schema([message.full_name for message in package.messages],
                                   base_ns='fb',
                                   gen_enums=True,
                                   gen_constants=True)
  schema_filename = "schema.fbs"
  try:
    path = os.path.join(msg2fbs_dir, schema_filename)
    output_file = open(path, "w+")
  except OSError:
    print('ERROR: Failed to write flatbuffer schema file.')
    return False
  
  output_file.writelines(line + os.linesep for line in schema)
  output_file.close()

  # compile the schema by calling the flatc compiler
  print('Compiling flatbuffer schema for ' + package.plugin_pkg_name)
  try:
    make_file_path = os.path.join(msg2fbs_dir, schema_filename)
    # TODO: Add a timeout in Python3
    compile_prcs = subprocess.check_call(['make', 'all', '--directory='+msg2fbs_dir],
                                         stdout=subprocess.PIPE)
  except subprocess.CalledProcessError:
    print('ERROR: Failed to compile the flatbuffer schema.')
    return False
  
  print('Finished compiling flatbuffer schema.')


  # Install the schema into the plugin package's includes
  install_path = os.path.join(output_dir,
                              package.plugin_pkg_name,
                              'include',
                              package.plugin_pkg_name)
  shutil.copyfile(os.path.join(msg2fbs_dir, 'schema_generated.h'),
                  os.path.join(install_path, 'schema_generated.h'))

  return True




def generate_plugin_package(package, output_path, templates_path, msg2fbs_dir):
  if not generate_directories(package.plugin_pkg_name, output_path):
    return False

  if not generate_cmakelists(package, output_path, templates_path):
    return False

  if not generate_package_xml(package, output_path, templates_path):
    return False

  for message in package.messages:
    if not generate_msg_headers(message, output_path, templates_path):
      return False

  for message in package.messages:
    if not generate_msg_impl(message, output_path, templates_path):
      return False

  if not generate_plugin_manifest(package, output_path, templates_path):
    return False

  if not generate_flatbuffer_schema(package, output_path, msg2fbs_dir):
    return False

  return True



def main(args):
  # parse arguments
  parsed_args = parse_args(args[1:])
  num_pkgs = len(parsed_args.packages)
  
  # get input and output locations
  my_package_path = rospkg.RosPack().get_path('robofleet_client')
  templates_dir = my_package_path + '/scripts/generate/templates'
  msg2fbs_dir = my_package_path + '/scripts/generate/msg2fbs'
  output_dir = my_package_path + '/scripts/generate/output'

  input_package_set = set(parsed_args.packages)

  # check for duplicates in the requested packages
  if len(input_package_set) != len(parsed_args.packages):
    raise argparse.ArgumentError('The provided package list contains duplicates.')
    return 1

  # <msg_package>_robofleet
  package_data = set(PackageData(msg_package) for msg_package in input_package_set)
  
  # get msgs, services, and actions for listed packages
  # after this loop, full_package_set will contain the full package dependency set
  full_package_set = set()
  for package in package_data:
    get_msg_and_srv_data(package, full_package_set)
  
  # print found messages and services
  print('---------------------------')
  print('Found Messages and Services')
  for package in full_package_set:
    print(package)

  # check if any of the listed packages already exist, and delete them if allowed
  if not check_product_existence(full_package_set, output_dir, parsed_args.OVERWRITE):
    return 1

  print('---------------------------')
  print('Generating output in ' + output_dir)

  # create the output
  for package in full_package_set:
    if not generate_plugin_package(package, output_dir, templates_dir, msg2fbs_dir):
      return 2

  print('---------------------------')
  print('Generation Complete!')

  return 0

if __name__ == '__main__':
  result = main(sys.argv)
  exit(result)