#pragma once

#include <ros/ros.h>

// contains macros used to register plugin classes
#include <pluginlib/class_list_macros.h>

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

class MessageScheduler;

namespace robofleet_client
{

  class ROSSrvOutHandler
  {
    public:
      virtual bool initialize(ros::NodeHandle& nh,
                              MessageScheduler* scheduler,
                              const std::string client_service,
                              const std::string rbf_topic);
      
      virtual void sendRequest(const QByteArray& data) = 0;

    protected:
      typedef flatbuffers::Offset<fb::MsgMetadata> MetaDataOffset;
      ros::ServiceClient client_;

      // sends raw data to the message scheduler
      std::function<void(const QByteArray&)> schedule_function_;

      // loads metadata into the flatbuffer builder
      std::function<MetaDataOffset(flatbuffers::FlatBufferBuilder&)> encode_metadata_function_;
  };



  class ROSSrvInHandler
  {
    public:
      virtual bool initialize(ros::NodeHandle& nh,
                              MessageScheduler* scheduler,
                              const std::string client_service,
                              const std::string rbf_topic,
                              const ros::Duration timeout);
      
      virtual void returnResponse(const QByteArray& data);

    protected:
      typedef flatbuffers::Offset<fb::MsgMetadata> MetaDataOffset;

      ros::ServiceServer server_;
      
      bool awaitReponse();

      bool has_received_response_;

      ros::Duration timeout_;

      std::mutex response_mutex_;

      QByteArray response_data_;

      // sends raw data to the message scheduler
      std::function<void(const QByteArray&)> schedule_function_;

      // loads metadata into the flatbuffer builder
      std::function<MetaDataOffset(flatbuffers::FlatBufferBuilder&)> encode_metadata_function_;
  };
  
  typedef boost::shared_ptr<ROSSrvInHandler> ROSSrvInHandlerPtr;
  typedef boost::shared_ptr<ROSSrvOutHandler> ROSSrvOutHandlerPtr;

} // namespace robofleet_client