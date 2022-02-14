#include "RosClientNode.hpp"
#include "WsServer.hpp"

#include <yaml-cpp/yaml.h>

RosClientNode::RosClientNode(Verbosity verbosity, MessageScheduler& scheduler) :
  verbosity_(verbosity),
  scheduler_(&scheduler),
  server_(nullptr)
{}

RosClientNode::RosClientNode(Verbosity verbosity, WsServer& server) :
  verbosity_(verbosity),
  scheduler_(nullptr),
  server_(&server)
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
  } catch (const YAML::InvalidNode& e) {}
  try {
    publishers_list = root["publishers"];
  } catch (const YAML::InvalidNode& e) {}
  try {
    incoming_srv_list = root["incoming_services"];
  } catch (const YAML::InvalidNode& e) {}
  try {
    outgoing_srv_list = root["outgoing_services"];
  } catch (const YAML::InvalidNode& e) {}

  if (subscribers_list.IsNull() && publishers_list.IsNull() &&
      incoming_srv_list.IsNull() && outgoing_srv_list.IsNull()) {
    ROS_ERROR("Cannot configure robofleet client. No topics found in config file.");
    return false;
  }

  if (!configureTopics(publishers_list, subscribers_list)) {
    return false;
  }

  if (!configureServices(incoming_srv_list, outgoing_srv_list)) {
    return false;
  }

  if (subs_.empty() && pubs_.empty() &&
      incoming_srvs_.empty() && outgoing_srvs_.empty()) {
    ROS_ERROR("Cannot configure robofleet client. No topics found in config file.");
    return false;
  }

  return true;
}

void RosClientNode::routeMessageToHandlers(const QByteArray& data) const
{
  // decode the metadata of the message
  const fb::MsgWithMetadata* msg =
      flatbuffers::GetRoot<fb::MsgWithMetadata>(data.data());
  const std::string& topic = msg->__metadata()->topic()->str();

  // look for a publisher on this topic
  {
    const HandlerMap<robofleet_client::ROSPublishHandlerPtr>::const_iterator it =
        pubs_.find(topic);

    if (it != pubs_.end()) {
      it->second->publish(data);

      if (verbosity_ >= Verbosity::ALL) {
        ROS_INFO("Publishing message on topic %s", topic.c_str());
      }

      return;
    }
  }

  // look for an incoming service on this topic
  // in this case, the data is the response to the original client
  {
    const HandlerMap<robofleet_client::ROSSrvInHandlerPtr>::const_iterator it =
        incoming_srvs_.find(topic);
    
    if (it != incoming_srvs_.end()) {
      it->second->returnResponse(data);

      if (verbosity_ >= Verbosity::ALL) {
        ROS_INFO("Publishing service response on topic %s", topic.c_str());
      }

      return;
    }
  }

  // look for an outgoing service on this topic
  // in this case, the data is the request from the original client
  {
    const HandlerMap<robofleet_client::ROSSrvOutHandlerPtr>::const_iterator it =
        outgoing_srvs_.find(topic);
    
    if (it != outgoing_srvs_.end()) {
      it->second->sendRequest(data);

      if (verbosity_ >= Verbosity::ALL) {
        ROS_INFO("Publishing service request on topic %s", topic.c_str());
      }

      return;
    }
  }

  // we didn't find any handler
  ROS_WARN_ONCE("Received message for unregistered topic %s", topic.c_str());
}

bool RosClientNode::readTopicParams(const YAML::Node& node,
                                    TopicParams& out_params,
                                    const bool publisher,
                                    const bool service)
{
  TopicParams params;

  try {
    params.client_topic = node["client_topic"].as<TopicString>();
    params.rbf_topic = node["rbf_topic"].as<TopicString>();

    if (service) {
      if (node["timeout"]) {
        params.timeout = ros::Duration(node["timeout"].as<double>());

        if (params.timeout.toSec() < 0.0) {
          ROS_ERROR("Invalid negative value for timeout on topic %s.", params.client_topic.c_str());
          return false;
        }
      }
      else {
        params.timeout = ros::Duration(0.0);
      }
    } else {
      if (publisher) {
        if (node["latched"]) {
          params.latched = node["latched"].as<bool>();
        }
        else {
          params.latched = false;
        }
      }
      else {
        if (node["rate_limit"]) {
          params.rate_limit = node["rate_limit"].as<double>();

          if (params.rate_limit < 0.0) {
            ROS_ERROR("Invalid negative value for rate_limit on topic %s.", params.client_topic.c_str());
            return false;
          }
        }
        else {
          params.rate_limit = std::numeric_limits<double>::max();
        }

        if (node["priority"]) {
          params.priority = node["priority"].as<double>();

          if (params.priority < 0) {
            ROS_ERROR("Invalid negative value for priority on topic %s.", params.client_topic.c_str());
            return false;
          }
        }
        else {
          params.priority = 1.0;
        }

        if (node["no_drop"]) {
          params.no_drop = node["no_drop"].as<bool>();
        }
        else {
          params.no_drop = false;
        }
      }
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
    if (!readTopicParams(subscriber, topic_params, false, false)) {
      ROS_ERROR("Invalid subscriber.");
      return false;
    }

    robofleet_client::ROSSubscribeHandlerPtr handler;
    if (!getSubscribeHandler(topic_params, handler)) {
      ROS_ERROR("Failed to generate subscribe handler.");
      return false;
    }
    
    if (scheduler_ != nullptr) {
      handler->initialize(nh_,
                          *scheduler_,
                          topic_params.client_topic,
                          topic_params.rbf_topic,
                          topic_params.priority,
                          topic_params.rate_limit,
                          topic_params.no_drop);
    }
    else if (server_ != nullptr) {
      handler->initialize(nh_,
                          *server_,
                          topic_params.client_topic,
                          topic_params.rbf_topic);
    }
    else {
      ROS_ERROR("Neither a message scheduler nor a websocket server "
                "were provided to the client node. Exiting.");
      return false;
    }

    subs_[topic_params.rbf_topic] = handler;

    if (verbosity_ >= Verbosity::CFG_ONLY) {
      ROS_INFO("Subscribing to incoming topic: %s [%s]->%s", 
                topic_params.client_topic.c_str(),
                topic_params.message_type.c_str(),
                topic_params.rbf_topic.c_str());
    }
  }

  // generate the publish handlers
  for (const YAML::Node& publisher : publishers_list) {
    TopicParams topic_params;
    if (!readTopicParams(publisher, topic_params, true, false)) {
      ROS_ERROR("Invalid publisher.");
      return false;
    }

    robofleet_client::ROSPublishHandlerPtr handler;
    if (!getPublishHandler(topic_params, handler)) {
      ROS_ERROR("Failed to generate publish handler.");
      return false;
    }

    handler->initialize(nh_, topic_params.client_topic, topic_params.latched);
    pubs_[topic_params.rbf_topic] = handler;

    if (verbosity_ >= Verbosity::CFG_ONLY) {
      ROS_INFO("Advertising outgoing topic: %s [%s]->%s", 
                topic_params.rbf_topic.c_str(),
                topic_params.message_type.c_str(),
                topic_params.client_topic.c_str());
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
    if (!readTopicParams(service, topic_params, false, true)) {
      ROS_ERROR("Invalid incoming service.");
      return false;
    }

    robofleet_client::ROSSrvInHandlerPtr handler;
    if (!getSrvInHandler(topic_params, handler)) {
      ROS_ERROR("Failed to generate service handler.");
      return false;
    }
    
    if (scheduler_ != nullptr) {
      handler->initialize(nh_,
                          *scheduler_,
                          topic_params.client_topic,
                          topic_params.rbf_topic+"Responses",
                          topic_params.timeout);
    }
    else if (server_ != nullptr) {
      handler->initialize(nh_,
                          *server_,
                          topic_params.client_topic,
                          topic_params.rbf_topic+"Responses",
                          topic_params.timeout);
    }
    else {
      ROS_ERROR("Neither a message scheduler nor a websocket server "
                "were provided to the client node. Exiting.");
      return false;
    }

    incoming_srvs_[topic_params.rbf_topic+"Responses"] = handler;

    if (verbosity_ >= Verbosity::CFG_ONLY) {
      ROS_INFO("Connecting to incoming service: %s [%s]->%s", 
                topic_params.client_topic.c_str(),
                topic_params.message_type.c_str(),
                topic_params.rbf_topic.c_str());
    }
  }

  // generate the SrvOut handlers
  for (const YAML::Node& service : outgoing_list) {
    TopicParams topic_params;
    if (!readTopicParams(service, topic_params, false, true)) {
      ROS_ERROR("Invalid outgoing service.");
      return false;
    }

    robofleet_client::ROSSrvOutHandlerPtr handler;
    if (!getSrvOutHandler(topic_params, handler)) {
      ROS_ERROR("Failed to generate service handler.");
      return false;
    }
    
    if (scheduler_ != nullptr) {
      handler->initialize(nh_,
                          *scheduler_,
                          topic_params.client_topic,
                          topic_params.rbf_topic+"Requests");
    }
    else if (server_ != nullptr) {
      handler->initialize(nh_,
                          *server_,
                          topic_params.client_topic,
                          topic_params.rbf_topic+"Requests");
    }
    else {
      ROS_ERROR("Neither a message scheduler nor a websocket server "
                "were provided to the client node. Exiting.");
      return false;
    }

    outgoing_srvs_[topic_params.rbf_topic+"Requests"] = handler;

    if (verbosity_ >= Verbosity::CFG_ONLY) {
      ROS_INFO("Advertising outgoing service: %s [%s]->%s", 
                topic_params.rbf_topic.c_str(),
                topic_params.message_type.c_str(),
                topic_params.client_topic.c_str());
    }
  }

  return true;
}