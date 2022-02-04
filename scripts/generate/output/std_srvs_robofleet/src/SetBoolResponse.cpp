#include <std_srvs_robofleet/SetBoolResponse.h>
#include <QByteArray>

namespace std_srvs_robofleet {
    
  void SetBoolResponsePublishHandler::initialize(ros::NodeHandle& nh,
                                                 const std::string to_topic,
                                                 const bool latched)
  {
    pub_ = nh.advertise<MsgType>(to_topic,
                                 10,
                                 latched);
  }
                  
  void SetBoolResponsePublishHandler::publish(const QByteArray& data)
  {
    const MsgType msg = decode(data);
    
    pub_.publish(msg);
  }
  
  SetBoolResponsePublishHandler::MsgType
  SetBoolResponsePublishHandler::decode(const QByteArray& data)
  {
  }

  PLUGINLIB_EXPORT_CLASS(std_srvs_robofleet::SetBoolResponsePublishHandler, robofleet_client::ROSPublishHandler)
  


  bool SetBoolResponseSubscribeHandler::initialize(ros::NodeHandle& nh,
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

    sub_ = nh.subscribe(to_topic, 10, &SetBoolResponseSubscribeHandler::callback, this);

    return true;
  }

  
  QByteArray SetBoolResponseSubscribeHandler::encode(const MsgType& msg)
  {
  }
  
  void SetBoolResponseSubscribeHandler::callback(const MsgTypeConstPtr& msg)
  {
  }
} // namespace std_srvs_robofleet

PLUGINLIB_EXPORT_CLASS(std_srvs_robofleet::SetBoolResponseSubscribeHandler, robofleet_client::ROSSubscribeHandler)