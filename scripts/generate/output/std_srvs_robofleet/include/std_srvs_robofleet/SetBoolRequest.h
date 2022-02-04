#pragma once

// the plugin parent class
#include <robofleet_client/ROSMsgHandlers.hpp>

// get the msg type
#include <std_srvs/SetBoolRequest.h>

namespace std_srvs_robofleet {

  class SetBoolRequestPublishHandler : public robofleet_client::ROSPublishHandler
  {
  public:
    virtual void initialize(ros::NodeHandle& nh,
                            const std::string to_topic,
                            const bool latched);
                    
    virtual void publish(const QByteArray& data);
    
  private:
    typedef std_srvs::SetBoolRequest MsgType;

    MsgType decode(const QByteArray& data);
  };
  
  
  
  class SetBoolRequestSubscribeHandler : public robofleet_client::ROSSubscribeHandler
  {
  public:
    virtual bool initialize(ros::NodeHandle& nh,
                              MessageScheduler* scheduler,
                              const std::string to_topic,
                              const double priority,
                              const double rate_limit,
                              const bool no_drop);
    
  private:
    typedef std_srvs::SetBoolRequest MsgType;
    typedef boost::shared_ptr<MsgType> MsgTypeConstPtr;
    
    QByteArray encode(const MsgType& msg);
    
    void callback(const MsgTypeConstPtr& msg);
  };
}