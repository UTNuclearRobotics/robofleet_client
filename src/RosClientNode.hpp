#pragma once

#include <QObject>
#include <unordered_map>

#include <pluginlib/class_loader.h>

#include "robofleet_client/ROSMsgHandlers.hpp"
#include "robofleet_client/ROSSrvHandlers.hpp"

namespace YAML {
  class Node;
}

class WsServer;

class RosClientNode : public QObject {
  Q_OBJECT

public:
  enum class Verbosity {MINIMAL, CFG_ONLY, ALL};

  // constructor for client mode
  RosClientNode(Verbosity verbosity, MessageScheduler& scheduler);

  // constructor for direct mode
  RosClientNode(Verbosity verbosity, WsServer& server);

  bool configure(const YAML::Node& root);

public Q_SLOTS:
  void sendSubscriptionMsg();

  void routeMessageToHandlers(const QByteArray& data) const;
  
private:
  typedef std::string TopicString;
  typedef std::string MsgTypeString;

  struct TopicParams {
    // used for all topic types
    std::string message_package;
    MsgTypeString message_type;
    TopicString client_topic;
    TopicString rbf_topic;

    // only used for message subscriber topics
    double priority;
    double rate_limit;
    bool no_drop;

    // only used for message publisher topics
    bool latched;

    // only used for service topics
    // set to zero if we don't want to time out
    ros::Duration timeout;
  };

  const Verbosity verbosity_;
  MessageScheduler* const scheduler_;
  WsServer* const server_;
  ros::NodeHandle nh_;

  // handlers are mapped according to their Robofleet topic name
  template<class Handler>
  using HandlerMap = std::unordered_map<TopicString, Handler>;

  HandlerMap<robofleet_client::ROSSubscribeHandlerPtr> subs_;
  HandlerMap<robofleet_client::ROSPublishHandlerPtr> pubs_;
  HandlerMap<robofleet_client::ROSSrvInHandlerPtr> incoming_srvs_;
  HandlerMap<robofleet_client::ROSSrvOutHandlerPtr> outgoing_srvs_;

  bool readTopicParams(const YAML::Node& node,
                       TopicParams& out_params,
                       const bool publisher,
                       const bool service);

  template<class Handler>
  bool getHandler(const TopicParams& params,
                  const std::string handler_type,
                  boost::shared_ptr<Handler>& out_handler);

  bool getSubscribeHandler(const TopicParams& params,
                           robofleet_client::ROSSubscribeHandlerPtr& out_handler);

  bool getPublishHandler(const TopicParams& params,
                         robofleet_client::ROSPublishHandlerPtr& out_handler);

  bool getSrvInHandler(const TopicParams& params,
                         robofleet_client::ROSSrvInHandlerPtr& out_handler);

  bool getSrvOutHandler(const TopicParams& params,
                          robofleet_client::ROSSrvOutHandlerPtr& out_handler);

  bool configureTopics(const YAML::Node& publishers_list,
                       const YAML::Node& subscribers_list);

  bool configureServices(const YAML::Node& incoming_list,
                         const YAML::Node& outgoing_list);

  bool configureActions(const YAML::Node& incoming_list,
                        const YAML::Node& outgoing_list);
};

// template definitions
#include "RosClientNode.tpp"