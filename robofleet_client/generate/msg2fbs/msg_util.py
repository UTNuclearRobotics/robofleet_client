from rosidl_runtime_py import get_interface_path
from rosidl_adapter.parser import \
    parse_message_file, parse_service_file, parse_action_file, InvalidResourceName

def get_msg_spec(package_name, msg_name):
    if msg_name.startswith('msg/'):
        full_name = package_name + '/' + msg_name + '.msg'
    else:
        full_name = package_name + '/msg/' + msg_name + '.msg'
    return parse_message_file(package_name, get_interface_path(full_name))

def get_srv_spec(package_name, srv_name):
    if srv_name.startswith('srv/'):
        full_name = package_name + '/' + srv_name + '.srv'
    else:
        full_name = package_name + '/srv/' + srv_name + '.srv'
    return parse_service_file(package_name, get_interface_path(full_name))

def get_action_spec(package_name, action_name):
    if action_name.startswith('action/'):
        full_name = package_name + '/' + action_name + '.action'
    else:
        full_name = package_name + '/action/' + action_name + '.action'
    return parse_action_file(package_name, get_interface_path(full_name))