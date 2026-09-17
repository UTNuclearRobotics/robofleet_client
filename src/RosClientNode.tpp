template<class Handler>
bool RosClientNode::getHandler(const TopicParams& params,
                               const std::string handler_type,
                               std::shared_ptr<Handler>& out_handler,
                               const std::string ns)
{
  typedef std::shared_ptr<Handler> HandlerPtr;
  typedef pluginlib::ClassLoader<Handler> ClassLoader;
  typedef LoaderWrapper<Handler> Wrapper;

  const std::string base_class = "robofleet_client::" + ns + handler_type;
  const std::string cache_key = base_class;

  std::lock_guard<std::mutex> lock(loader_cache_mutex_);

  // Retrieve or create cached loader
  std::shared_ptr<ClassLoader> loader;
  auto it = loader_cache_.find(cache_key);

  if (it == loader_cache_.end()) {
    // Create and cache a new loader
    try {
      auto wrapper = std::make_shared<Wrapper>("robofleet_client", base_class);
      loader_cache_[cache_key] = wrapper;
      loader = wrapper->loader;
    } catch (const pluginlib::ClassLoaderException& e) {
      RCLCPP_ERROR(this->get_logger(), "Failed to create class loader: %s", e.what());
      return false;
    }
  } else {
    // Retrieve cached loader
    auto wrapper = std::dynamic_pointer_cast<Wrapper>(it->second);
    if (!wrapper) {
      RCLCPP_ERROR(this->get_logger(), "Cache corruption: wrong loader type for %s", cache_key.c_str());
      return false;
    }
    loader = wrapper->loader;
  }

  try {
    const std::string plugin_package = params.message_package + "_robofleet";
    const std::string msg_class = plugin_package + "::" + params.message_type + handler_type;

    HandlerPtr msg_handler = HandlerPtr(loader->createUnmanagedInstance(msg_class));

    if (msg_handler == nullptr) {
      RCLCPP_ERROR(this->get_logger(), "Failed to instantiate handler: %s", msg_class.c_str());
      return false;
    }

    out_handler = msg_handler;
    return true;
  } catch(const pluginlib::LibraryLoadException& e) {
    RCLCPP_ERROR(this->get_logger(), "Library load error for %s::%s: %s",
                 params.message_package.c_str(), params.message_type.c_str(), e.what());
    return false;
  } catch (const pluginlib::CreateClassException& e) {
    RCLCPP_ERROR(this->get_logger(), "Create class error for %s::%s: %s",
                 params.message_package.c_str(), params.message_type.c_str(), e.what());
    return false;
  }
}