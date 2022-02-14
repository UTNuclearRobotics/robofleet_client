#include "robofleet_client/ROSMsgHandlers.hpp"
#include "robofleet_client/MessageScheduler.hpp"
#include "WsServer.hpp"

namespace robofleet_client
{
  void ROSSubscribeHandler::initialize(ros::NodeHandle& nh,
                                       MessageScheduler& scheduler,
                                       const std::string client_topic,
                                       const std::string rbf_topic,
                                       const double priority,
                                       const double rate_limit,
                                       const bool no_drop)
  {    
    schedule_function_ = std::bind(&MessageScheduler::enqueue,
                                   &scheduler,
                                   QString::fromStdString(rbf_topic),
                                   std::placeholders::_1,
                                   priority,
                                   rate_limit,
                                   no_drop);
  }

  void ROSSubscribeHandler::initialize(ros::NodeHandle& nh,
                                       WsServer& server,
                                       const std::string client_topic,
                                       const std::string rbf_topic)
  {    
    schedule_function_ = std::bind(&WsServer::broadcast_message,
                                   &server,
                                   std::placeholders::_1,
                                   nullptr);
  }

} // namespace robofleet_client