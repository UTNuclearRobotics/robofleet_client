#include "RosClientNode.hpp"

#include <yaml-cpp/yaml.h>

RosClientNode::RosClientNode(Verbosity verbosity, MessageScheduler& scheduler) :
  verbosity_(verbosity),
  scheduler_(&scheduler)
{}

bool RosClientNode::configure(const YAML::Node& root)
{
  // get the subscribers and publishers
  YAML::Node subscribers_list;
  YAML::Node publishers_list;
  YAML::Node incoming_srv_list;
  YAML::Node outgoing_srv_list;
  try {
    subscribers_list = root["subscribers"];
    publishers_list = root["publishers"];
    incoming_srv_list = root["incoming_services"];
    outgoing_srv_list = root["outgoing_services"];
  } catch (const YAML::InvalidNode& e) {
    ROS_ERROR("%s", e.what());
    return false;
  }

  if (!configureTopics(publishers_list, subscribers_list)) {
    return false;
  }

  if (!configureServices(incoming_srv_list, outgoing_srv_list)) {
    return false;
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
  return getHandler(params, "SubscribeHandler", out_handler);
}

bool RosClientNode::getPublishHandler(
  const TopicParams& params,
  robofleet_client::ROSPublishHandlerPtr& out_handler)
{
  return getHandler(params, "PublishHandler", out_handler);
}


bool RosClientNode::getSrvInHandler(
  const TopicParams& params,
  robofleet_client::ROSSrvInHandlerPtr& out_handler)
{
  return getHandler(params, "SrvInHandler", out_handler);
}

bool RosClientNode::getSrvOutHandler(
  const TopicParams& params,
  robofleet_client::ROSSrvOutHandlerPtr& out_handler)
{
  return getHandler(params, "SrvOutHandler", out_handler);
}

bool RosClientNode::configureTopics(const YAML::Node& publishers_list,
                                    const YAML::Node& subscribers_list)
{
  subs_.clear();
  pubs_.clear();

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

bool RosClientNode::configureServices(const YAML::Node& incoming_list,
                                      const YAML::Node& outgoing_list)
{
  incoming_srvs_.clear();
  outgoing_srvs_.clear();

  // generate the SrvIn handlers
  for (const YAML::Node& service : incoming_list) {
    TopicParams topic_params;
    if (!readTopicParams(service, topic_params, false)) {
      ROS_ERROR("Invalid incoming service.");
      return false;
    }

    robofleet_client::ROSSrvInHandlerPtr handler;
    if (!getSrvInHandler(topic_params, handler)) {
      ROS_ERROR("Failed to generate service handler.");
      return false;
    }
    
    if (!handler->initialize(nh_,
                             scheduler_,
                             topic_params.from)) {
      ROS_ERROR("Failed to initialize handler for service %s.",
                topic_params.from.c_str());
      return false;
    }

    incoming_srvs_[topic_params.from] = handler;

    if (verbosity_ >= Verbosity::CFG_ONLY) {
      ROS_INFO("Connecting to incoming service: %s [%s]->%s", 
                topic_params.from.c_str(),
                topic_params.message_type.c_str(),
                topic_params.to.c_str());
    }
  }

  // generate the SrvOut handlers
  for (const YAML::Node& service : outgoing_list) {
    TopicParams topic_params;
    if (!readTopicParams(service, topic_params, false)) {
      ROS_ERROR("Invalid outgoing service.");
      return false;
    }

    robofleet_client::ROSSrvOutHandlerPtr handler;
    if (!getSrvOutHandler(topic_params, handler)) {
      ROS_ERROR("Failed to generate service handler.");
      return false;
    }
    
    handler->initialize(nh_, topic_params.from);

    outgoing_srvs_[topic_params.from] = handler;

    if (verbosity_ >= Verbosity::CFG_ONLY) {
      ROS_INFO("Connecting to incoming service: %s [%s]->%s", 
                topic_params.from.c_str(),
                topic_params.message_type.c_str(),
                topic_params.to.c_str());
    }
  }

  return true;
}