#pragma once

#include <ros/ros.h>

// contains macros used to register plugin classes
#include <pluginlib/class_list_macros.h>

#include <robofleet_client/common_conversions.hpp>

/**
 * The ROSPublishHandler and ROSSubscribeHandler interfaces
 * are used by the client to encode, decode, subscribe, and publish
 * ROS message types.
 * 
 * To make a ROS message type available to robofleet_client, create a
 * ROS package <msg_package_name>_robofleet and generate pluginlib
 * classes in it that inherit from these types.
 */

class MessageScheduler;
class QByteArray;

namespace robofleet_client
{

  class ROSPublishHandler
  {
    public:
      virtual void initialize(ros::NodeHandle& nh,
                              const std::string to_topic,
                              const bool latched) = 0;
      
      virtual void publish(const QByteArray& data) = 0;
        
    protected:
      ros::Publisher pub_;
  };



  class ROSSubscribeHandler
  {
    public:
      virtual bool initialize(ros::NodeHandle& nh,
                              MessageScheduler* scheduler,
                              const std::string to_topic,
                              const double priority,
                              const double rate_limit,
                              const bool no_drop);
      
    protected:
      ros::Subscriber sub_;
      
      std::function<void(const QByteArray&)> schedule_function_;
  };
  
  typedef boost::shared_ptr<ROSPublishHandler> ROSPublishHandlerPtr;
  typedef boost::shared_ptr<ROSSubscribeHandler> ROSSubscribeHandlerPtr;

} // namespace robofleet_client