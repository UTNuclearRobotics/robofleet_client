#include <std_srvs_robofleet/EmptyRequest.h>
#include <QByteArray>

namespace std_srvs_robofleet {
    
  void EmptyRequestPublishHandler::initialize(ros::NodeHandle& nh,
                                                 const std::string to_topic,
                                                 const bool latched)
  {
    pub_ = nh.advertise<MsgType>(to_topic,
                                 10,
                                 latched);
  }
                  
  void EmptyRequestPublishHandler::publish(const QByteArray& data)
  {
    const MsgType msg = decode(data);
    
    pub_.publish(msg);
  }
  
  EmptyRequestPublishHandler::MsgType
  EmptyRequestPublishHandler::decode(const QByteArray& data)
  {
  }

  PLUGINLIB_EXPORT_CLASS(std_srvs_robofleet::EmptyRequestPublishHandler, robofleet_client::ROSPublishHandler)
  


  bool EmptyRequestSubscribeHandler::initialize(ros::NodeHandle& nh,
                                                   MessageScheduler* scheduler,
                                                   const std::string to_topic,
                                                   const double priority,
                                                   const double rate_limit,
                                                   const bool no_drop)
  {
    if (!robofleet_client::ROSSubscribeHandler::initialize(nh,
                                                           scheduler,
                                                           to_topic,
                                                           priority,
                                                           rate_limit,
                                                           no_drop)) {
      return false;
    }

    sub_ = nh.subscribe(to_topic, 10, &EmptyRequestSubscribeHandler::callback, this);

    return true;
  }

  
  QByteArray EmptyRequestSubscribeHandler::encode(const MsgType& msg)
  {
  }
  
  void EmptyRequestSubscribeHandler::callback(const MsgTypeConstPtr& msg)
  {
  }
} // namespace std_srvs_robofleet

PLUGINLIB_EXPORT_CLASS(std_srvs_robofleet::EmptyRequestSubscribeHandler, robofleet_client::ROSSubscribeHandler)