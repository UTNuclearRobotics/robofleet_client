#include "RosClientNode.hpp"
#include "MessageScheduler.hpp"
#include "WsServer.hpp"

#include <robofleet_client/base_schema_generated.h>
#include <robofleet_client_msgs/msg/robofleet_subscription.hpp>

#include <yaml-cpp/yaml.h>

RosClientNode::RosClientNode(Verbosity verbosity, MessageScheduler& scheduler) :
  Node("robofleet_client"),
  verbosity_(verbosity),
  scheduler_(&scheduler),
  server_(nullptr)
{}

RosClientNode::RosClientNode(Verbosity verbosity, WsServer& server) :
  Node("robofleet_client"),
  verbosity_(verbosity),
  scheduler_(nullptr),
  server_(&server)
{}

bool RosClientNode::configure(const YAML::Node& root)
{
  // get the subscribers and publishers
  YAML::Node publishers_list;
  YAML::Node subscribers_list;
  YAML::Node incoming_srv_list;
  YAML::Node outgoing_srv_list;
  YAML::Node incoming_action_list;
  YAML::Node outgoing_action_list;
  try {
    publishers_list = root["publishers"];
  } catch (const YAML::InvalidNode& e) {}
  try {
    subscribers_list = root["subscribers"];
  } catch (const YAML::InvalidNode& e) {}
  try {
    incoming_srv_list = root["incoming_services"];
  } catch (const YAML::InvalidNode& e) {}
  try {
    outgoing_srv_list = root["outgoing_services"];
  } catch (const YAML::InvalidNode& e) {}
  try {
    incoming_action_list = root["incoming_actions"];
  } catch (const YAML::InvalidNode& e) {}
  try {
    outgoing_action_list = root["outgoing_actions"];
  } catch (const YAML::InvalidNode& e) {}

  if (publishers_list.IsNull() && subscribers_list.IsNull() &&
      incoming_srv_list.IsNull() && outgoing_srv_list.IsNull() &&
      incoming_action_list.IsNull() && outgoing_action_list.IsNull()) {
    RCLCPP_ERROR(this->get_logger(), "Cannot configure robofleet client. No topics found in config file.");
    return false;
  }

  subs_.clear();
  pubs_.clear();
  incoming_srvs_.clear();
  outgoing_srvs_.clear();

  if (!configureTopics(subscribers_list, publishers_list)) {
    return false;
  }

  if (!configureServices(incoming_srv_list, outgoing_srv_list)) {
    return false;
  }

  if (!configureActions(incoming_action_list, outgoing_action_list)) {
    return false;
  }

  if (subs_.empty() && pubs_.empty() &&
      incoming_srvs_.empty() && outgoing_srvs_.empty()) {
    RCLCPP_ERROR(this->get_logger(), "Cannot configure robofleet client. No topics found in config file.");
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
    const HandlerMap<robofleet_client::RBFSubscribeHandlerPtr>::const_iterator it =
        subs_.find(topic);

    if (it != subs_.end()) {
      it->second->publishROS(data);

      if (verbosity_ >= Verbosity::ALL) {
        RCLCPP_INFO(this->get_logger(), "Subscribing to robofleet and publishing ROS message on ROS topic %s", topic.c_str());
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
        RCLCPP_INFO(this->get_logger(), "Publishing service response on topic %s", topic.c_str());
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
      it->second->send_request_function(data);

      if (verbosity_ >= Verbosity::ALL) {
        RCLCPP_INFO(this->get_logger(), "Publishing service request on topic %s", topic.c_str());
      }

      return;
    }
  }

  // we didn't find any handler
  RCLCPP_WARN_ONCE(this->get_logger(), "Received message for unregistered topic %s", topic.c_str());
}

bool RosClientNode::readTopicParams(const YAML::Node& node,
                                    TopicParams& out_params,
                                    const bool subscriber,
                                    const bool service)
{
  TopicParams params;

  try {
    params.client_topic = node["client_topic"].as<TopicString>();
  } catch (const YAML::InvalidNode& e) {
    RCLCPP_ERROR(this->get_logger(), "%s", e.what());
    RCLCPP_ERROR(this->get_logger(), "Missing YAML tag 'client_topic'.");
    return false;
  }
  try {
    params.rbf_topic = node["rbf_topic"].as<TopicString>();
  } catch (const YAML::InvalidNode& e) {
    RCLCPP_ERROR(this->get_logger(), "%s", e.what());
    RCLCPP_ERROR(this->get_logger(), "Missing YAML tag 'rbf_topic'.");
    return false;
  }

  if (service) {
    if (node["timeout"]) {
      params.timeout = rclcpp::Duration::from_seconds(node["timeout"].as<double>());

      if (params.timeout.seconds() < 0.0) {
        RCLCPP_ERROR(this->get_logger(), "Invalid negative value for timeout on topic %s.", params.client_topic.c_str());
        return false;
      }
    }
    else {
      params.timeout = rclcpp::Duration::from_seconds(0.0);
    }
  } else {
    if (subscriber) {
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
          RCLCPP_ERROR(this->get_logger(), "Invalid negative value for rate_limit on topic %s.", params.client_topic.c_str());
          return false;
        }
      }
      else {
        params.rate_limit = 0.0;
      }

      if (node["priority"]) {
        params.priority = node["priority"].as<double>();

        if (params.priority < 0) {
          RCLCPP_ERROR(this->get_logger(), "Invalid negative value for priority on topic %s.", params.client_topic.c_str());
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

      if (node["queue_size"]) {
        params.queue_size = node["queue_size"].as<int>();

        if (params.queue_size < 1) {
          RCLCPP_ERROR(this->get_logger(), "Invalid non-positive value for queue_size on topic %s.", params.client_topic.c_str());
          return false;
        }
      }
      else {
        params.queue_size = 1;
      }
    }
  }

  const MsgTypeString type = node["type"].as<MsgTypeString>();
  const MsgTypeString::size_type pos = type.find('/');
  if (pos != MsgTypeString::npos && pos+1 < type.size()) {
    params.message_package = type.substr(0, pos);
    params.message_type = type.substr(pos+1);
  }
  else {
    RCLCPP_ERROR(this->get_logger(), "Invalid string %s for param type.", type.c_str());
    return false;
  }

  out_params = params;
  return true;
}


bool RosClientNode::getPublishHandler(
  const TopicParams& params,
  robofleet_client::RBFPublishHandlerPtr& out_handler)
{
  return getHandler(params, "PublishHandler", out_handler,"RBF");
}

bool RosClientNode::getSubscribeHandler(
  const TopicParams& params,
  robofleet_client::RBFSubscribeHandlerPtr& out_handler)
{
  return getHandler(params, "SubscribeHandler", out_handler,"RBF");
}


bool RosClientNode::getSrvInHandler(
  const TopicParams& params,
  robofleet_client::ROSSrvInHandlerPtr& out_handler)
{
  return getHandler(params, "SrvInHandler", out_handler,"ROS");
}

bool RosClientNode::getSrvOutHandler(
  const TopicParams& params,
  robofleet_client::ROSSrvOutHandlerPtr& out_handler)
{
  return getHandler(params, "SrvOutHandler", out_handler,"ROS");
}

bool RosClientNode::configureTopics(const YAML::Node& subscribers_list,
                                    const YAML::Node& publishers_list)
{
  // generate the ros subscribe handlers
  for (const YAML::Node& publisher : publishers_list) {
    TopicParams topic_params;
    if (!readTopicParams(publisher, topic_params, false, false)) {
      RCLCPP_ERROR(this->get_logger(), "Invalid publisher.");
      return false;
    }

    robofleet_client::RBFPublishHandlerPtr handler;
    if (!getPublishHandler(topic_params, handler)) {
      RCLCPP_ERROR(this->get_logger(), "Failed to generate publish handler.");
      return false;
    }
    
    if (scheduler_ != nullptr) {
      handler->initialize(shared_from_this(),
                          *scheduler_,
                          topic_params.client_topic,
                          topic_params.rbf_topic,
                          topic_params.priority,
                          topic_params.rate_limit,
                          topic_params.no_drop,
                          topic_params.queue_size);
    }
    else if (server_ != nullptr) {
      handler->initialize(shared_from_this(),
                          *server_,
                          topic_params.client_topic,
                          topic_params.rbf_topic);
    }
    else {
      RCLCPP_ERROR(this->get_logger(), "Neither a message scheduler nor a websocket server "
                "were provided to the client node. Exiting.");
      return false;
    }

    pubs_[topic_params.rbf_topic] = handler;

    if (verbosity_ >= Verbosity::CFG_ONLY) {
      RCLCPP_INFO(this->get_logger(), "Publishing to robofleet from ROS topic: %s->%s [%s]", 
                topic_params.client_topic.c_str(),
                topic_params.rbf_topic.c_str(),
                topic_params.message_type.c_str());
    }
  }

  // generate the publish handlers
  for (const YAML::Node& subscriber : subscribers_list) {
    TopicParams topic_params;
    if (!readTopicParams(subscriber, topic_params, true, false)) {
      RCLCPP_ERROR(this->get_logger(), "Invalid subscriber.");
      return false;
    }

    robofleet_client::RBFSubscribeHandlerPtr handler;
    if (!getSubscribeHandler(topic_params, handler)) {
      RCLCPP_ERROR(this->get_logger(), "Failed to generate subscribe handler.");
      return false;
    }

    handler->initialize(shared_from_this(), topic_params.client_topic, topic_params.latched);
    subs_[topic_params.rbf_topic] = handler;

    if (verbosity_ >= Verbosity::CFG_ONLY) {
      RCLCPP_INFO(this->get_logger(), "Subscribing to robofleet and outputting to ROS topic: %s->%s [%s]", 
                topic_params.rbf_topic.c_str(),
                topic_params.client_topic.c_str(),
                topic_params.message_type.c_str());
    }
  }

  if (subs_.empty() && pubs_.empty()) {
    RCLCPP_WARN(this->get_logger(), "Configuration complete with no subscribers or publishers found.");
  }

  return true;
}

bool RosClientNode::configureServices(const YAML::Node& incoming_list,
                                      const YAML::Node& outgoing_list)
{

  // generate the SrvIn handlers
  for (const YAML::Node& service : incoming_list) {
    TopicParams topic_params;
    if (!readTopicParams(service, topic_params, false, true)) {
      RCLCPP_ERROR(this->get_logger(), "Invalid incoming service.");
      return false;
    }

    robofleet_client::ROSSrvInHandlerPtr handler;
    if (!getSrvInHandler(topic_params, handler)) {
      RCLCPP_ERROR(this->get_logger(), "Failed to generate service handler.");
      return false;
    }
    
    if (scheduler_ != nullptr) {
      handler->initialize(shared_from_this(),
                          *scheduler_,
                          topic_params.client_topic,
                          topic_params.rbf_topic+"Responses",
                          topic_params.timeout);
    }
    else if (server_ != nullptr) {
      handler->initialize(shared_from_this(),
                          *server_,
                          topic_params.client_topic,
                          topic_params.rbf_topic+"Responses",
                          topic_params.timeout);
    }
    else {
      RCLCPP_ERROR(this->get_logger(), "Neither a message scheduler nor a websocket server "
                "were provided to the client node. Exiting.");
      return false;
    }

    incoming_srvs_[topic_params.rbf_topic+"Responses"] = handler;

    if (verbosity_ >= Verbosity::CFG_ONLY) {
      RCLCPP_INFO(this->get_logger(), "Connecting to ROS-to-Robofleet service: %s [%s]->%s", 
                topic_params.client_topic.c_str(),
                topic_params.message_type.c_str(),
                topic_params.rbf_topic.c_str());
    }
  }

  // generate the SrvOut handlers
  for (const YAML::Node& service : outgoing_list) {
    TopicParams topic_params;
    if (!readTopicParams(service, topic_params, false, true)) {
      RCLCPP_ERROR(this->get_logger(), "Invalid outgoing service.");
      return false;
    }

    robofleet_client::ROSSrvOutHandlerPtr handler;
    if (!getSrvOutHandler(topic_params, handler)) {
      RCLCPP_ERROR(this->get_logger(), "Failed to generate service handler.");
      return false;
    }
    
    if (scheduler_ != nullptr) {
      handler->initialize(shared_from_this(),
                          *scheduler_,
                          topic_params.client_topic,
                          topic_params.rbf_topic+"Requests");
    }
    else if (server_ != nullptr) {
      handler->initialize(shared_from_this(),
                          *server_,
                          topic_params.client_topic,
                          topic_params.rbf_topic+"Requests");
    }
    else {
      RCLCPP_ERROR(this->get_logger(), "Neither a message scheduler nor a websocket server "
                "were provided to the client node. Exiting.");
      return false;
    }

    outgoing_srvs_[topic_params.rbf_topic+"Requests"] = handler;

    if (verbosity_ >= Verbosity::CFG_ONLY) {
      RCLCPP_INFO(this->get_logger(), "Advertising Robofleet-to-ROS service: %s [%s]->%s", 
                topic_params.rbf_topic.c_str(),
                topic_params.message_type.c_str(),
                topic_params.client_topic.c_str());
    }
  }

  return true;
}

bool RosClientNode::configureActions(const YAML::Node& incoming_list,
                                     const YAML::Node& outgoing_list)
{
  struct ActionParams {
    ActionParams(TopicParams params)
    {
      params.latched = false;
      params.no_drop = true;
      params.queue_size = 1;

      goal =
      feedback =
      result =
      status =
      cancel = params;

      goal.client_topic     += "/goal";
      goal.rbf_topic        += "/goal";
      feedback.client_topic += "/feedback";
      feedback.rbf_topic    += "/feedback";
      result.client_topic   += "/result";
      result.rbf_topic      += "/result";
      status.client_topic   += "/status";
      status.rbf_topic      += "/status";
      cancel.client_topic   += "/cancel";
      cancel.rbf_topic      += "/cancel";

      goal.message_type     += "ActionGoal";
      feedback.message_type += "ActionFeedback";
      result.message_type   += "ActionResult";

      status.message_package =
      cancel.message_package = "actionlib_msgs";
      status.message_type    = "GoalStatusArray";
      cancel.message_type    = "GoalID";
    }

    TopicParams goal;
    TopicParams feedback;
    TopicParams result;
    TopicParams status;
    TopicParams cancel;
  };

  // generate the handlers for incoming topics
  for (const YAML::Node& action : incoming_list) {
    TopicParams topic_params;
    if (!readTopicParams(action, topic_params, false, false)) {
      RCLCPP_ERROR(this->get_logger(), "Invalid action.");
      return false;
    }

    const ActionParams action_params(topic_params);

    struct ActionHandlers {
      robofleet_client::RBFPublishHandlerPtr    goal;
      robofleet_client::RBFSubscribeHandlerPtr  feedback;
      robofleet_client::RBFSubscribeHandlerPtr  result;
      robofleet_client::RBFSubscribeHandlerPtr  status;
      robofleet_client::RBFPublishHandlerPtr    cancel;
    };

    ActionHandlers handlers;

    /**
     * For some reason, we need to get the handlers for actionlib_msgs_robofleet first.
     * Otherwise it fails to instantiate the handlers.
     * Feels like this is some sort of bug in pluginlib.
     */
    if (!getSubscribeHandler(action_params.status, handlers.status)) {
      RCLCPP_ERROR(this->get_logger(), "Failed to generate publish handler.");
      return false;
    }

    if (!getPublishHandler(action_params.cancel, handlers.cancel)) {
      RCLCPP_ERROR(this->get_logger(), "Failed to generate subscribe handler.");
      return false;
    }

    if (!getPublishHandler(action_params.goal, handlers.goal)) {
      RCLCPP_ERROR(this->get_logger(), "Failed to generate subscribe handler.");
      return false;
    }

    if (!getSubscribeHandler(action_params.feedback, handlers.feedback)) {
      RCLCPP_ERROR(this->get_logger(), "Failed to generate publish handler.");
      return false;
    }

    if (!getSubscribeHandler(action_params.result, handlers.result)) {
      RCLCPP_ERROR(this->get_logger(), "Failed to generate publish handler.");
      return false;
    }

    handlers.feedback->initialize(shared_from_this(),
                                  action_params.feedback.client_topic,
                                  action_params.feedback.latched);

    handlers.result->initialize(shared_from_this(),
                                action_params.result.client_topic,
                                action_params.result.latched);
    
    handlers.status->initialize(shared_from_this(),
                                action_params.status.client_topic,
                                action_params.status.latched);
    
    if (scheduler_ != nullptr) {    
      handlers.goal->initialize(shared_from_this(),
                                *scheduler_,
                                action_params.goal.client_topic,
                                action_params.goal.rbf_topic,
                                action_params.goal.priority,
                                action_params.goal.rate_limit,
                                action_params.goal.no_drop,
                                action_params.goal.queue_size);

      handlers.cancel->initialize(shared_from_this(),
                                  *scheduler_,
                                  action_params.cancel.client_topic,
                                  action_params.cancel.rbf_topic,
                                  action_params.cancel.priority,
                                  action_params.cancel.rate_limit,
                                  action_params.cancel.no_drop,
                                  action_params.goal.queue_size);
    }
    else if (server_ != nullptr) {
      handlers.goal->initialize(shared_from_this(),
                                *server_,
                                action_params.goal.client_topic,
                                action_params.goal.rbf_topic);

      handlers.cancel->initialize(shared_from_this(),
                                  *server_,
                                  action_params.cancel.client_topic,
                                  action_params.cancel.rbf_topic);
    }
    else {
      RCLCPP_ERROR(this->get_logger(), "Neither a message scheduler nor a websocket server "
                "were provided to the client node. Exiting.");
      return false;
    }

    pubs_[action_params.goal.rbf_topic] = handlers.goal;
    subs_[action_params.feedback.rbf_topic] = handlers.feedback;
    subs_[action_params.result.rbf_topic] = handlers.result;
    subs_[action_params.status.rbf_topic] = handlers.status;
    pubs_[action_params.cancel.rbf_topic] = handlers.cancel;

    if (verbosity_ >= Verbosity::CFG_ONLY) {
      RCLCPP_INFO(this->get_logger(), "Advertising ROS-to-Robofleet action: %s [%s]->%s", 
                (topic_params.client_topic + "/*").c_str(),
                topic_params.message_type.c_str(),
                (topic_params.rbf_topic + "/*").c_str());
    }
  }

    // generate the handlers for outgoing topics
  for (const YAML::Node& action : outgoing_list) {
    TopicParams topic_params;
    if (!readTopicParams(action, topic_params, false, false)) {
      RCLCPP_ERROR(this->get_logger(), "Invalid action.");
      return false;
    }

    const ActionParams action_params(topic_params);

    struct ActionHandlers {
      robofleet_client::RBFSubscribeHandlerPtr  goal;
      robofleet_client::RBFPublishHandlerPtr    feedback;
      robofleet_client::RBFPublishHandlerPtr    result;
      robofleet_client::RBFPublishHandlerPtr    status;
      robofleet_client::RBFSubscribeHandlerPtr  cancel;
    };

    ActionHandlers handlers;

    /**
     * For some reason, we need to get the handlers for actionlib_msgs_robofleet first.
     * Otherwise it fails to instantiate the handlers.
     * Feels like this is some sort of bug in pluginlib.
     */
    if (!getPublishHandler(action_params.status, handlers.status)) {
      RCLCPP_ERROR(this->get_logger(), "Failed to generate publish handler.");
      return false;
    }

    if (!getSubscribeHandler(action_params.cancel, handlers.cancel)) {
      RCLCPP_ERROR(this->get_logger(), "Failed to generate subscribe handler.");
      return false;
    }

    if (!getSubscribeHandler(action_params.goal, handlers.goal)) {
      RCLCPP_ERROR(this->get_logger(), "Failed to generate subscribe handler.");
      return false;
    }

    if (!getPublishHandler(action_params.feedback, handlers.feedback)) {
      RCLCPP_ERROR(this->get_logger(), "Failed to generate publish handler.");
      return false;
    }

    if (!getPublishHandler(action_params.result, handlers.result)) {
      RCLCPP_ERROR(this->get_logger(), "Failed to generate publish handler.");
      return false;
    }

    handlers.goal->initialize(shared_from_this(),
                                  action_params.goal.client_topic,
                                  action_params.goal.latched);

    handlers.cancel->initialize(shared_from_this(),
                                action_params.cancel.client_topic,
                                action_params.cancel.latched);
    
    if (scheduler_ != nullptr) {    
      handlers.feedback->initialize(shared_from_this(),
                                    *scheduler_,
                                    action_params.feedback.client_topic,
                                    action_params.feedback.rbf_topic,
                                    action_params.feedback.priority,
                                    action_params.feedback.rate_limit,
                                    action_params.feedback.no_drop,
                                    action_params.goal.queue_size);

      handlers.result->initialize(shared_from_this(),
                                  *scheduler_,
                                  action_params.result.client_topic,
                                  action_params.result.rbf_topic,
                                  action_params.result.priority,
                                  action_params.result.rate_limit,
                                  action_params.result.no_drop,
                                  action_params.goal.queue_size);

      handlers.status->initialize(shared_from_this(),
                                  *scheduler_,
                                  action_params.status.client_topic,
                                  action_params.status.rbf_topic,
                                  action_params.status.priority,
                                  action_params.status.rate_limit,
                                  action_params.status.no_drop,
                                  action_params.goal.queue_size);
    }
    else if (server_ != nullptr) {
      handlers.feedback->initialize(shared_from_this(),
                                    *server_,
                                    action_params.feedback.client_topic,
                                    action_params.feedback.rbf_topic);

      handlers.result->initialize(shared_from_this(),
                                  *server_,
                                  action_params.result.client_topic,
                                  action_params.result.rbf_topic);

      handlers.status->initialize(shared_from_this(),
                                  *server_,
                                  action_params.status.client_topic,
                                  action_params.status.rbf_topic);
    }
    else {
      RCLCPP_ERROR(this->get_logger(), "Neither a message scheduler nor a websocket server "
                "were provided to the client node. Exiting.");
      return false;
    }

    subs_[action_params.goal.rbf_topic] = handlers.goal;
    pubs_[action_params.feedback.rbf_topic] = handlers.feedback;
    pubs_[action_params.result.rbf_topic] = handlers.result;
    pubs_[action_params.status.rbf_topic] = handlers.status;
    subs_[action_params.cancel.rbf_topic] = handlers.cancel;

    if (verbosity_ >= Verbosity::CFG_ONLY) {
      RCLCPP_INFO(this->get_logger(), "Advertising Robofleet-to-ROS action: %s [%s]->%s", 
                (topic_params.rbf_topic + "/*").c_str(),
                topic_params.message_type.c_str(),
                (topic_params.client_topic + "/*").c_str());
    }
  }

  return true;
}

void RosClientNode::sendSubscriptionMsg()
{
  for (const HandlerMap<robofleet_client::RBFSubscribeHandlerPtr>::value_type& pair : subs_) {
    flatbuffers::FlatBufferBuilder fbb;

    const flatbuffers::Offset<fb::MsgMetadata> metadata =
      fb::CreateMsgMetadataDirect(fbb, "amrl_msgs/RobofleetSubscription", "/subscriptions");

    const flatbuffers::uoffset_t root_offset = fb::CreateRobofleetSubscriptionDirect(
      fbb,
      metadata,
      pair.first.c_str(),
      robofleet_client_msgs::msg::RobofleetSubscription::ACTION_SUBSCRIBE).o;

    fbb.Finish(flatbuffers::Offset<void>(root_offset));
    const QByteArray data{reinterpret_cast<const char*>(fbb.GetBufferPointer()),
                          static_cast<int>(fbb.GetSize())};

    if (scheduler_ != nullptr) {
      scheduler_->enqueue(QString("/subscriptions"),
                          data,
                          0.0,
                          std::numeric_limits<double>::max(),
                          true,
                          subs_.size());
    }
    else {
      server_->broadcast_message(data, nullptr);
    }
  }
}