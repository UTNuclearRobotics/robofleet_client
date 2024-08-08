#pragma once

#include <QObject>
#include <unordered_map>

#include <pluginlib/class_loader.hpp>

#include "robofleet_client/ROSMsgHandlers.hpp"
#include "robofleet_client/ROSSrvHandlers.hpp"

namespace YAML {
  class Node;
}

class RosClientNode;
class WsServer;

class RosClientNode : public QObject, public rclcpp::Node {
  Q_OBJECT

public:
  enum class Verbosity {MINIMAL, CFG_ONLY, ALL};

  // constructor for client mode
  RosClientNode(Verbosity verbosity, MessageScheduler& scheduler);

  // constructor for direct mode
  RosClientNode(Verbosity verbosity, WsServer& server);

  // sets up the client topics as instructed by the configuration file
  bool configure(const YAML::Node& root);

public Q_SLOTS:
  // sends the subscriptinos info to the Robofleet server
  void sendSubscriptionMsg();

  /**
   * @brief Finds the handler to send received data to
   * @param data Raw data from the websocket
   */
  void routeMessageToHandlers(const QByteArray& data) const;
  
private:
  typedef std::string TopicString;
  typedef std::string MsgTypeString;

  // holds the configuration params for a data topic
  struct TopicParams {
    TopicParams():
      timeout(0,0)
    {}

    // used for all topic types
    std::string message_package;
    MsgTypeString message_type;
    TopicString client_topic;
    TopicString rbf_topic;

    // only used for message subscriber topics
    double priority;
    double rate_limit;
    bool no_drop;
    int queue_size;

    // only used for message publisher topics
    bool latched;

    // only used for service topics
    // set to zero if we don't want to time out
    rclcpp::Duration timeout;
  };

  const Verbosity verbosity_;

  // used in normal mode
  MessageScheduler* const scheduler_;

  // used in direct mode
  WsServer* const server_;
  
  // handlers are mapped according to their Robofleet topic name
  template<class Handler>
  using HandlerMap = std::unordered_map<TopicString, Handler>;

  // holds all of our topic handlers
  HandlerMap<robofleet_client::RBFPublishHandlerPtr> pubs_;
  HandlerMap<robofleet_client::RBFSubscribeHandlerPtr> subs_;
  HandlerMap<robofleet_client::ROSSrvInHandlerPtr> incoming_srvs_;
  HandlerMap<robofleet_client::ROSSrvOutHandlerPtr> outgoing_srvs_;

  // parses the configuration file
  bool readTopicParams(const YAML::Node& node,
                       TopicParams& out_params,
                       const bool publisher,
                       const bool service);

  // instantiates handlers for a given data type
  // getHandler is invoked through getSubscribeHandler, etc
  template<class Handler>
  bool getHandler(const TopicParams& params,
                  const std::string handler_type,
                  std::shared_ptr<Handler>& out_handler,
                  const std::string ns);

  bool getPublishHandler(const TopicParams& params,
                           robofleet_client::RBFPublishHandlerPtr& out_handler);

  bool getSubscribeHandler(const TopicParams& params,
                         robofleet_client::RBFSubscribeHandlerPtr& out_handler);

  bool getSrvInHandler(const TopicParams& params,
                         robofleet_client::ROSSrvInHandlerPtr& out_handler);

  bool getSrvOutHandler(const TopicParams& params,
                          robofleet_client::ROSSrvOutHandlerPtr& out_handler);

  // instantiate topic handlers
  bool configureTopics(const YAML::Node& publishers_list,
                       const YAML::Node& subscribers_list);

  // instantiate service handlers
  bool configureServices(const YAML::Node& incoming_list,
                         const YAML::Node& outgoing_list);

  // instantiate action handlers
  bool configureActions(const YAML::Node& incoming_list,
                        const YAML::Node& outgoing_list);
};

// template definitions
#include "RosClientNode.tpp"