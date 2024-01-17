#pragma once

#include <rclcpp/rclcpp.hpp>

#include <robofleet_client/common_conversions.hpp>

#include <mutex>

#include <QByteArray>

/**
 * The ROSRequestHandler and ROSResponseHandler interfaces
 * are used by the client to encode, decode, subscribe, and publish
 * ROS service types.
 * 
 * To make a ROS service type available to robofleet_client, create a
 * ROS package <srv_package_name>_robofleet and generate pluginlib
 * classes in it that inherit from these types.
 */

class WsServer;
class MessageScheduler;

namespace robofleet_client
{

  class ROSSrvOutHandler
  {
    public:
      virtual void initialize(std::shared_ptr<rclcpp::Node> node,
                              MessageScheduler& scheduler,
                              const std::string client_service,
                              const std::string rbf_topic);

      virtual void initialize(std::shared_ptr<rclcpp::Node> node,
                              WsServer& scheduler,
                              const std::string client_service,
                              const std::string rbf_topic);

      std::function<void (const QByteArray&)> send_request_function;

    protected:
      typedef flatbuffers::Offset<fb::MsgMetadata> MetaDataOffset;

      // sends raw data to the message scheduler
      std::function<void(const QByteArray&)> schedule_function_;

      // loads metadata into the flatbuffer builder
      std::function<MetaDataOffset(flatbuffers::FlatBufferBuilder&)> encode_metadata_function_;

      virtual void sendRequest(const QByteArray& data,
                               const std::shared_ptr<rclcpp::Node> node,
                               const std::string client_service) = 0;
  };



  class ROSSrvInHandler
  {
    public:
      virtual void initialize(std::shared_ptr<rclcpp::Node> node,
                              MessageScheduler& scheduler,
                              const std::string client_service,
                              const std::string rbf_topic,
                              const rclcpp::Duration timeout);

      virtual void initialize(std::shared_ptr<rclcpp::Node> node,
                              WsServer& scheduler,
                              const std::string client_service,
                              const std::string rbf_topic,
                              const rclcpp::Duration timeout);
      
      virtual void returnResponse(const QByteArray& data);

    protected:
      typedef flatbuffers::Offset<fb::MsgMetadata> MetaDataOffset;
      
      bool awaitReponse();

      bool has_received_response_;

      rclcpp::Duration timeout_ = rclcpp::Duration(0,0);

      std::mutex response_mutex_;

      QByteArray response_data_;

      // sends raw data to the message scheduler
      std::function<void(const QByteArray&)> schedule_function_;

      // loads metadata into the flatbuffer builder
      std::function<MetaDataOffset(flatbuffers::FlatBufferBuilder&)> encode_metadata_function_;
  };
  
  typedef std::shared_ptr<ROSSrvInHandler> ROSSrvInHandlerPtr;
  typedef std::shared_ptr<ROSSrvOutHandler> ROSSrvOutHandlerPtr;

} // namespace robofleet_client