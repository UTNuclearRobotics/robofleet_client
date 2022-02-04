#include <std_srvs_robofleet/SetBoolRequest.h>
#include <QByteArray>

namespace std_srvs_robofleet {
    
  void SetBoolRequestPublishHandler::initialize(ros::NodeHandle& nh,
                                                 const std::string to_topic,
                                                 const bool latched)
  {
    pub_ = nh.advertise<MsgType>(to_topic,
                                 10,
                                 latched);
  }
                  
  void SetBoolRequestPublishHandler::publish(const QByteArray& data)
  {
    const MsgType msg = decode(data);
    
    pub_.publish(msg);
  }
  
  SetBoolRequestPublishHandler::MsgType
  SetBoolRequestPublishHandler::decode(const QByteArray& data)
  {
  }

  PLUGINLIB_EXPORT_CLASS(std_srvs_robofleet::SetBoolRequestPublishHandler, robofleet_client::ROSPublishHandler)
  


  bool SetBoolRequestSubscribeHandler::initialize(ros::NodeHandle& nh,
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

    sub_ = nh.subscribe(to_topic, 10, &SetBoolRequestSubscribeHandler::callback, this);

    return true;
  }

  
  QByteArray SetBoolRequestSubscribeHandler::encode(const MsgType& msg)
  {
  }
  
  void SetBoolRequestSubscribeHandler::callback(const MsgTypeConstPtr& msg)
  {
  }
} // namespace std_srvs_robofleet

PLUGINLIB_EXPORT_CLASS(std_srvs_robofleet::SetBoolRequestSubscribeHandler, robofleet_client::ROSSubscribeHandler)