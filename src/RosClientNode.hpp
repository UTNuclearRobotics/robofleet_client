#pragma once

#include <QObject>
#include <unordered_map>

#include "robofleet_client/ROSMsgHandlers.hpp"

namespace YAML {
  class Node;
}

class RosClientNode : public QObject {
  Q_OBJECT

public:
  enum class Verbosity {MINIMAL, CFG_ONLY, ALL};

  RosClientNode(Verbosity verbosity, MessageScheduler& scheduler);

  bool configure(const YAML::Node& root);

public Q_SLOTS:
  /**
   * @brief Attempt to decode an publish message data.
   *
   * Must call register_msg_type<T> before a message of type T can be decoded.
   * @param data the Flatbuffer-encoded message data
   */
  void decode_net_message(const QByteArray& data);

private:
  typedef std::string TopicString;
  typedef std::string MsgTypeString;

  MessageScheduler* scheduler_;

  struct TopicParams {
    std::string message_package;
    MsgTypeString message_type;
    TopicString from;
    TopicString to;
    double priority;
    double rate_limit;
    bool no_drop;
    bool latched;
  };

  const Verbosity verbosity_;

  ros::NodeHandle nh_;

  std::vector<TopicString> pub_remote_topics_;
  std::unordered_map<TopicString, robofleet_client::ROSSubscribeHandlerPtr> subs_;
  std::unordered_map<TopicString, robofleet_client::ROSPublishHandlerPtr> pubs_;
  
  bool readTopicParams(const YAML::Node& node,
                       TopicParams& out_params,
                       const bool publisher);

  bool getSubscribeHandler(const TopicParams& params,
                           robofleet_client::ROSSubscribeHandlerPtr& out_handler);

  bool getPublishHandler(const TopicParams& params,
                         robofleet_client::ROSPublishHandlerPtr& out_handler);
};