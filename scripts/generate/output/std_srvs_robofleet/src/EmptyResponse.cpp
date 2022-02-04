#include <std_srvs_robofleet/EmptyResponse.h>
#include <QByteArray>

namespace std_srvs_robofleet {
    
  void EmptyResponsePublishHandler::initialize(ros::NodeHandle& nh,
                                                 const std::string to_topic,
                                                 const bool latched)
  {
    pub_ = nh.advertise<MsgType>(to_topic,
                                 10,
                                 latched);
  }
                  
  void EmptyResponsePublishHandler::publish(const QByteArray& data)
  {
    const MsgType msg = decode(data);
    
    pub_.publish(msg);
  }
  
  EmptyResponsePublishHandler::MsgType
  EmptyResponsePublishHandler::decode(const QByteArray& data)
  {
  }

  PLUGINLIB_EXPORT_CLASS(std_srvs_robofleet::EmptyResponsePublishHandler, robofleet_client::ROSPublishHandler)
  


  bool EmptyResponseSubscribeHandler::initialize(ros::NodeHandle& nh,
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

    sub_ = nh.subscribe(to_topic, 10, &EmptyResponseSubscribeHandler::callback, this);

    return true;
  }

  
  QByteArray EmptyResponseSubscribeHandler::encode(const MsgType& msg)
  {
  }
  
  void EmptyResponseSubscribeHandler::callback(const MsgTypeConstPtr& msg)
  {
  }
} // namespace std_srvs_robofleet

PLUGINLIB_EXPORT_CLASS(std_srvs_robofleet::EmptyResponseSubscribeHandler, robofleet_client::ROSSubscribeHandler)