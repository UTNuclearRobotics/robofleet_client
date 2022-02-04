#include <std_srvs_robofleet/TriggerRequest.h>
#include <QByteArray>

namespace std_srvs_robofleet {
    
  void TriggerRequestPublishHandler::initialize(ros::NodeHandle& nh,
                                                 const std::string to_topic,
                                                 const bool latched)
  {
    pub_ = nh.advertise<MsgType>(to_topic,
                                 10,
                                 latched);
  }
                  
  void TriggerRequestPublishHandler::publish(const QByteArray& data)
  {
    const MsgType msg = decode(data);
    
    pub_.publish(msg);
  }
  
  TriggerRequestPublishHandler::MsgType
  TriggerRequestPublishHandler::decode(const QByteArray& data)
  {
  }

  PLUGINLIB_EXPORT_CLASS(std_srvs_robofleet::TriggerRequestPublishHandler, robofleet_client::ROSPublishHandler)
  


  bool TriggerRequestSubscribeHandler::initialize(ros::NodeHandle& nh,
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

    sub_ = nh.subscribe(to_topic, 10, &TriggerRequestSubscribeHandler::callback, this);

    return true;
  }

  
  QByteArray TriggerRequestSubscribeHandler::encode(const MsgType& msg)
  {
  }
  
  void TriggerRequestSubscribeHandler::callback(const MsgTypeConstPtr& msg)
  {
  }
} // namespace std_srvs_robofleet

PLUGINLIB_EXPORT_CLASS(std_srvs_robofleet::TriggerRequestSubscribeHandler, robofleet_client::ROSSubscribeHandler)