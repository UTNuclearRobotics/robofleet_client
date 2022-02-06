import os
import rosmsg
import rospkg
import genmsg

def get_msg_search_path():
    """ Get ROS search path for messages """
    # see rosmsg.get_msg_text for reference
    rospack = rospkg.RosPack()
    search_path = {}
    for p in rospack.list():
        package_paths = rosmsg._get_package_paths(p, rospack)
        search_path[p] = [os.path.join(d, 'msg') for d in package_paths]
    return search_path

def get_msg_spec(msg_type_name, search_path=get_msg_search_path()):
    """ Get ROS MsgSpec for given message type """
    context = genmsg.MsgContext.create_default()
    search_path = get_msg_search_path()
    return genmsg.msg_loader.load_msg_by_type(context, msg_type_name, search_path)

def get_srv_spec(srv_type_name, search_path=get_msg_search_path()):
    """ Get ROS MsgSpec for given message type """
    context = genmsg.MsgContext.create_default()
    search_path = get_msg_search_path()
    return genmsg.msg_loader.load_srv_by_type(context, srv_type_name, search_path)