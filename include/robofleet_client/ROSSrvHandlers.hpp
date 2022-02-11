#pragma once

#include <ros/ros.h>

// contains macros used to register plugin classes
#include <pluginlib/class_list_macros.h>

#include <robofleet_client/common_conversions.hpp>

#include <mutex>

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
class QByteArray;

namespace robofleet_client
{

  class ROSSrvOutHandler
  {
    public:
      virtual void initialize(ros::NodeHandle& nh,
                              const std::string service_name) = 0;
      
      virtual void sendRequest(const QByteArray& data) = 0;

    protected:
      ros::ServiceClient client_;

      std::function<void(QByteArray&)> schedule_function_;
  };



  class ROSSrvInHandler
  {
    public:
      virtual bool initialize(ros::NodeHandle& nh,
                              MessageScheduler* scheduler,
                              const std::string service_name);
      
      virtual void returnResponse(const QByteArray& data) = 0;
    protected:
      ros::ServiceServer server_;
      
      std::function<void(QByteArray&)> schedule_function_;

      void awaitReponse();

      bool has_received_response_;

      std::mutex has_received_response_mutex_;
  };
  
  typedef boost::shared_ptr<ROSSrvInHandler> ROSSrvInHandlerPtr;
  typedef boost::shared_ptr<ROSSrvOutHandler> ROSSrvOutHandlerPtr;

} // namespace robofleet_client