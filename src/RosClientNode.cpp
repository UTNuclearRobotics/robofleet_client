#include "RosClientNode.hpp"

#include <flatbuffers/flatbuffers.h>
#include <pluginlib/class_loader.h>
#include <schema_generated.h>

#include <yaml-cpp/yaml.h>

RosClientNode::RosClientNode(Verbosity verbosity, MessageScheduler& scheduler) :
  verbosity_(verbosity),
  scheduler_(&scheduler)
{}

bool RosClientNode::configure(const YAML::Node& root)
{
  subs_.clear();
  pubs_.clear();

  // get the subscribers and publishers
  YAML::Node subscribers_list;
  YAML::Node publishers_list;
  try {
    subscribers_list = root["subscribers"];
    publishers_list = root["publishers"];
  } catch (const YAML::InvalidNode& e) {
    ROS_ERROR("%s", e.what());
    return false;
  }

  // generate the subscribe handlers
  for (const YAML::Node& subscriber : subscribers_list) {
    TopicParams topic_params;
    if (!readTopicParams(subscriber, topic_params, false)) {
      ROS_ERROR("Invalid subscriber.");
      return false;
    }

    robofleet_client::ROSSubscribeHandlerPtr handler;
    if (!getSubscribeHandler(topic_params, handler)) {
      ROS_ERROR("Failed to generate subscribe handler.");
      return false;
    }
    
    if (!handler->initialize(nh_,
                              scheduler_,
                              topic_params.from,
                              topic_params.priority,
                              topic_params.rate_limit,
                              topic_params.no_drop)) {
      ROS_ERROR("Failed to inizialize handler for topic %s.",
                topic_params.from.c_str());
      return false;
    }

    subs_[topic_params.from] = handler;

    if (verbosity_ >= Verbosity::CFG_ONLY) {
      ROS_INFO("Subscribing Local Messages: %s [%s]->%s", 
                topic_params.from.c_str(),
                topic_params.message_type.c_str(),
                topic_params.to.c_str());
    }
  }

  // generate the publish handlers
  for (const YAML::Node& publisher : publishers_list) {
    TopicParams topic_params;
    if (!readTopicParams(publisher, topic_params, true)) {
      ROS_ERROR("Invalid publisher.");
      return false;
    }

    robofleet_client::ROSPublishHandlerPtr handler;
    if (!getPublishHandler(topic_params, handler)) {
      ROS_ERROR("Failed to generate publish handler.");
      return false;
    }

    handler->initialize(nh_, topic_params.to, topic_params.latched);
    pubs_[topic_params.from] = handler;

    if (verbosity_ >= Verbosity::CFG_ONLY) {
      ROS_INFO("Publishing Local Messages: %s [%s]->%s", 
                topic_params.from.c_str(),
                topic_params.message_type.c_str(),
                topic_params.to.c_str());
    }
  }

  if (subs_.empty() && pubs_.empty()) {
    ROS_WARN("Configuration complete with no subscribers or publishers found.");
  }

  return true;
}

bool RosClientNode::readTopicParams(const YAML::Node& node,
                                    TopicParams& out_params,
                                    const bool publisher)
{
  TopicParams params;

  try {
    params.from = node["from"].as<TopicString>();
    params.to = node["to"].as<TopicString>();
    
    if (node["rate_limit_hz"]) {
      params.rate_limit = node["rate_limit_hz"].as<double>();
    }
    else {
      params.rate_limit = std::numeric_limits<double>::max();
    }

    if (node["priority"]) {
      params.priority = node["priority"].as<double>();
    }
    else {
      params.priority = 0.0;
    }

    if (publisher && node["latched"]) {
      params.latched = node["latched"].as<double>();
    }
    else {
      params.latched = false;
    }

  } catch (const YAML::InvalidNode& e) {
    ROS_ERROR("%s", e.what());
    return false;
  }

  const MsgTypeString type = node["type"].as<MsgTypeString>();
  const MsgTypeString::size_type pos = type.find('/');
  if (pos != MsgTypeString::npos && pos+1 < type.size()) {
    params.message_package = type.substr(0, pos);
    params.message_type = type.substr(pos+1);
  }
  else {
    ROS_ERROR("Invalid string %s for param type.", type.c_str());
    return false;
  }

  out_params = params;
  return true;
}

bool RosClientNode::getSubscribeHandler(
  const TopicParams& params,
  robofleet_client::ROSSubscribeHandlerPtr& out_handler)
{
  robofleet_client::ROSSubscribeHandlerPtr msg_handler(nullptr);
  try {
    const std::string plugin_package = params.message_package + "_robofleet";
    const std::string base_class = "robofleet_client::ROSSubscribeHandler";

    typedef pluginlib::ClassLoader<robofleet_client::ROSSubscribeHandler> ClassLoader;
    ClassLoader loader("robofleet_client",
                        base_class);

    try {
      const std::string msg_class = plugin_package + "::" + params.message_type + "SubscribeHandler";
      msg_handler = robofleet_client::ROSSubscribeHandlerPtr(loader.createUnmanagedInstance(msg_class));
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

bool RosClientNode::getPublishHandler(
  const TopicParams& params,
  robofleet_client::ROSPublishHandlerPtr& out_handler)
{
  robofleet_client::ROSPublishHandlerPtr msg_handler(nullptr);
  try {
    const std::string plugin_package = params.message_package + "_robofleet";
    const std::string base_class = "robofleet_client::ROSPublishHandler";

    typedef pluginlib::ClassLoader<robofleet_client::ROSPublishHandler> ClassLoader;
    ClassLoader loader("robofleet_client",
                        base_class);

    try {
      const std::string msg_class = plugin_package + "::" + params.message_type + "PublishHandler";
      msg_handler = robofleet_client::ROSPublishHandlerPtr(loader.createUnmanagedInstance(msg_class));
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


void RosClientNode::decode_net_message(const QByteArray& data) {
  // extract metadata
  const fb::MsgWithMetadata* msg =
      flatbuffers::GetRoot<fb::MsgWithMetadata>(data.data());
  const MsgTypeString msg_type = msg->__metadata()->type()->str();
  const TopicString topic = msg->__metadata()->topic()->str();

  // try to publish
  if (pubs_.count(topic) == 0) {
    ROS_WARN_ONCE("Ignoring message of unregistered topic %s.", topic.c_str());
    return;
  }

  if (verbosity_ == Verbosity::ALL) {
    ROS_INFO("Received message of type %s on topic %s.", msg_type.c_str(), topic.c_str());
  }

  pubs_[topic]->publish(data);
}