#!/usr/bin/env python
######################################################################
#      Title     : generate_plugin_pkg.py
#      Author    : Blake Anderson
#      Copyright : Copyright The University of Texas at Austin, 2022.
#                  All rights reserved.
######################################################################

import argparse
import os.path

from cv2 import split
import rosmsg
import rospkg.rospack
import shutil
import sys

class MsgData:
  def __init__(self, package, name, contents):
    self.package = package
    self.name = name
    self.contents = contents

  def full_name(self):
    return self.package + '/' + self.name

  def __str__(self):
    return self.full_name() + '\n' + self.contents

class PackageData:
  def __init__(self, msg_pkg):
    self.msg_pkg_name = msg_pkg
    self.plugin_pkg_name = msg_pkg + '_robofleet'
    self.messages = []
    self.services = []

  def __str__(self):
    output = self.msg_pkg_name
    for msg in self.messages:
      output = output + '\n\t' + msg.full_name()
    for srv in self.services:
      output = output + '\n\t' + srv.full_name()

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




def get_msg_and_srv_data(package):
  try:
    package.messages = [MsgData(package.msg_pkg_name, name.split('/')[1], '') for name in rosmsg.list_msgs(package.msg_pkg_name)]
  except rospkg.common.ResourceNotFound as err:
    print('ERROR: Package ' + err.args[0] + ' not found.')
    print(err)
    return 1
  
  print('Found package ' + package.msg_pkg_name)
  
  package.services = [MsgData(package.msg_pkg_name, name.split('/')[1], '') for name in rosmsg.list_srvs(package.msg_pkg_name)]
  
  if not package.messages and not package.services:
    print('WARNING: No messages or services found for package package.msg_pkg_name.')

  # get the definitions of each message
  for message in package.messages:
    message.contents = rosmsg.get_msg_text(message.full_name(), True)

  # get the definitions of each service
  for service in package.services:
    service.contents = rosmsg.get_srv_text(service.full_name(), True)

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



def generate_directories(name, output_path):
  print('Generating ' + name)

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

  # replace the target strings
  # TODO: In python3, we can use str.format() for this
  filedata = filedata.replace('{template_msg_package}', package.msg_pkg_name)

  sources_list = ''
  for message in package.messages:
    sources_list = sources_list + "  src/" + message.name + ".cpp\n"
  filedata = filedata.replace('{template_sources_list}', sources_list)

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
  # TODO: In python3, we can use str.format() for this
  filedata = filedata.replace('{template_msg_package}', package.msg_pkg_name)

  # write the filled out package.xml
  try:
    with open(xml_out_path, 'w') as file:
      file.write(filedata)
  except OSError:
    print('ERROR: Failed to write package.xml.')
    return False

  return True



def generate_class_headers(message, output_path, templates_path):
  header_template_path = os.path.join(templates_path, 'template_class_header')
  header_out_path = os.path.join(output_path,
                                 message.package + '_robofleet',
                                 'include',
                                 message.package + '_robofleet',
                                 message.name + '.h')

  # read in the template
  try:
    with open(header_template_path, 'r') as file :
      filedata = file.read()
  except IOError:
    print('ERROR: Failed to read class header template.')
    return False

  # replace the target strings
  # TODO: In python3, we can use str.format() for this
  filedata = filedata.replace('{template_msg_package}', message.package)
  filedata = filedata.replace('{template_msg_name}', message.name)

  # write the filled out class header
  try:
    with open(header_out_path, 'w') as file:
      file.write(filedata)
  except OSError:
    print('ERROR: Failed to write class header.')
    return False

  return True



def generate_class_impl(message, output_path, templates_path):
  impl_template_path = os.path.join(templates_path, 'template_class_impl')
  impl_out_path = os.path.join(output_path,
                                 message.package + '_robofleet',
                                 'src',
                                 message.name + '.cpp')

  # read in the template
  try:
    with open(impl_template_path, 'r') as file :
      filedata = file.read()
  except IOError:
    print('ERROR: Failed to read class implementation template.')
    return False

  # replace the target strings
  # TODO: In python3, we can use str.format() for this
  # TODO: Corrie magic for the encode and decode functions
  filedata = filedata.replace('{template_msg_package}', message.package)
  filedata = filedata.replace('{template_msg_name}', message.name)

  # write the filled out class impl
  try:
    with open(impl_out_path, 'w') as file:
      file.write(filedata)
  except OSError:
    print('ERROR: Failed to write class implementation.')
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

  output = '<library path="lib/lib{template_msg_package}_robofleet">\n'
  for message in package.messages:
    output = output + filedata.replace('{msg_name}', message.name)
  output = output + '</library>'

  # replace the target strings
  # TODO: In python3, we can use str.format() for this
  output = output.replace('{template_msg_package}', package.msg_pkg_name)

  # write the filled out plugin_description.xml
  try:
    with open(manifest_out_path, 'w') as file:
      file.write(output)
  except OSError:
    print('ERROR: Failed to write plugin_description.xml.')
    return False

  return True



def generate_plugin_package(package, output_path, templates_path):
  if not generate_directories(package.plugin_pkg_name, output_path):
    return False

  if not generate_cmakelists(package, output_path, templates_path):
    return False

  if not generate_package_xml(package, output_path, templates_path):
    return False

  for message in package.messages:
    if not generate_class_headers(message, output_path, templates_path):
      return False

  for message in package.messages:
    if not generate_class_impl(message, output_path, templates_path):
      return False

  if not generate_plugin_manifest(package, output_path, templates_path):
    return False

  return True



def main(args):
  # parse arguments
  parsed_args = parse_args(args[1:])
  num_pkgs = len(parsed_args.packages)
  
  # get input and output locations
  my_package_path = rospkg.RosPack().get_path('robofleet_client')
  templates_dir = my_package_path + '/scripts/generate/templates'
  output_dir = my_package_path + '/scripts/generate/output'

  # TODO: Automatically add dependent packages of the ones the user requested
  package_list = parsed_args.packages

  # <msg_package>_robofleet
  package_data = [PackageData(msg_package) for msg_package in package_list]
  
  # get msgs, services, and actions for listed packages
  for package in package_data:
    get_msg_and_srv_data(package)
  
  # print found messages and services
  print('---------------------------')
  print('Found Messages and Services')
  for package in package_data:
    print(package)

  # check if any of the listed packages already exist, and delete them if allowed
  if not check_product_existence(package_data, output_dir, parsed_args.OVERWRITE):
    return 1

  print('Generating output in ' + output_dir)

  # create the output
  for package in package_data:
    if not generate_plugin_package(package, output_dir, templates_dir):
      return 2

  return 0

if __name__ == '__main__':
  result = main(sys.argv)
  exit(result)