#pragma once

#include <rclcpp/rclcpp.hpp>

#include <robofleet_client/common_conversions.hpp>

/**
 * The RBFSubscribeHandler and RBFPublishHandler interfaces
 * are used by the client to encode, decode, subscribe, and publish
 * ROS message types.
 * 
 * To make a ROS message type available to robofleet_client, create a
 * ROS package <msg_package_name>_robofleet and generate pluginlib
 * classes in it that inherit from these types.
 */

class WsServer;
class MessageScheduler;
class QByteArray;

namespace robofleet_client
{

  class RBFSubscribeHandler
  {
    public:
      virtual void initialize(std::shared_ptr<rclcpp::Node> node,
                              const std::string client_topic,
                              const bool latched) = 0;
      
      virtual void publishROS(const QByteArray& data) = 0;
  };



  class RBFPublishHandler
  {
    public:
      virtual void initialize(std::shared_ptr<rclcpp::Node> node,
                              MessageScheduler& scheduler,
                              const std::string client_topic,
                              const std::string rbf_topic,
                              const double priority,
                              const double rate_limit,
                              const bool no_drop,
                              const int queue_size);
      
      virtual void initialize(std::shared_ptr<rclcpp::Node> node,
                              WsServer& server,
                              const std::string client_topic,
                              const std::string rbf_topic);

    protected:
      typedef flatbuffers::Offset<fb::MsgMetadata> MetaDataOffset;
      
      // sends raw data to the message scheduler
      std::function<void(const QByteArray&)> schedule_function_;

      // loads metadata into the flatbuffer builder
      std::function<MetaDataOffset(flatbuffers::FlatBufferBuilder&)> encode_metadata_function_;
  };
  
  typedef std::shared_ptr<RBFSubscribeHandler> RBFSubscribeHandlerPtr;
  typedef std::shared_ptr<RBFPublishHandler> RBFPublishHandlerPtr;

} // namespace robofleet_client