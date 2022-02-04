#include <std_srvs_robofleet/TriggerResponse.h>
#include <QByteArray>

namespace std_srvs_robofleet {
    
  void TriggerResponsePublishHandler::initialize(ros::NodeHandle& nh,
                                                 const std::string to_topic,
                                                 const bool latched)
  {
    pub_ = nh.advertise<MsgType>(to_topic,
                                 10,
                                 latched);
  }
                  
  void TriggerResponsePublishHandler::publish(const QByteArray& data)
  {
    const MsgType msg = decode(data);
    
    pub_.publish(msg);
  }
  
  TriggerResponsePublishHandler::MsgType
  TriggerResponsePublishHandler::decode(const QByteArray& data)
  {
  }

  PLUGINLIB_EXPORT_CLASS(std_srvs_robofleet::TriggerResponsePublishHandler, robofleet_client::ROSPublishHandler)
  


  bool TriggerResponseSubscribeHandler::initialize(ros::NodeHandle& nh,
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

    sub_ = nh.subscribe(to_topic, 10, &TriggerResponseSubscribeHandler::callback, this);

    return true;
  }

  
  QByteArray TriggerResponseSubscribeHandler::encode(const MsgType& msg)
  {
  }
  
  void TriggerResponseSubscribeHandler::callback(const MsgTypeConstPtr& msg)
  {
  }
} // namespace std_srvs_robofleet

PLUGINLIB_EXPORT_CLASS(std_srvs_robofleet::TriggerResponseSubscribeHandler, robofleet_client::ROSSubscribeHandler)