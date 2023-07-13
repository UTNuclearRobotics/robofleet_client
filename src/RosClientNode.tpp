template<class Handler>
bool RosClientNode::getHandler(const TopicParams& params,
                               const std::string handler_type,
                               boost::shared_ptr<Handler>& out_handler)
{
  typedef boost::shared_ptr<Handler> HandlerPtr;
  HandlerPtr msg_handler(nullptr);
  try {
    const std::string base_class = "robofleet_client::RBF" + handler_type;

    typedef pluginlib::ClassLoader<Handler> ClassLoader;
    ClassLoader loader("robofleet_client",
                        base_class);
    
    try {
      const std::string plugin_package = params.message_package + "_robofleet";
      const std::string msg_class = plugin_package + "::" + params.message_type + handler_type;
      msg_handler = HandlerPtr(loader.createUnmanagedInstance(msg_class));
    } catch(const pluginlib::LibraryLoadException& e) {
      ROS_ERROR("%s", e.what());
      return false;
    } catch (const pluginlib::CreateClassException& e) {
      ROS_ERROR("%s", e.what());
      return false;
    }
  } catch (const pluginlib::ClassLoaderException& e) {
    ROS_ERROR("%s", e.what());
    return false;
  }

  if (msg_handler == nullptr) {
    return false;
  }
  out_handler = msg_handler;

  return true;
}