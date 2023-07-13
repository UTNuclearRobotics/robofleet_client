#include "robofleet_client/ROSMsgHandlers.hpp"
#include "MessageScheduler.hpp"
#include "WsServer.hpp"

namespace robofleet_client
{
  void RBFPublishHandler::initialize(ros::NodeHandle& nh,
                                       MessageScheduler& scheduler,
                                       const std::string client_topic,
                                       const std::string rbf_topic,
                                       const double priority,
                                       const double rate_limit,
                                       const bool no_drop,
                                       const int queue_size)
  {    
    schedule_function_ = [&scheduler, rbf_topic, priority, rate_limit, no_drop, queue_size](const QByteArray& data)
      {
        scheduler.enqueue(QString::fromStdString(rbf_topic), data, priority, rate_limit, no_drop, queue_size);
      };
  }

  void RBFPublishHandler::initialize(ros::NodeHandle& nh,
                                       WsServer& server,
                                       const std::string client_topic,
                                       const std::string rbf_topic)
  {    
    schedule_function_ = [&server](const QByteArray& data)
      {
        server.broadcast_message(data, nullptr);
      };
  }

} // namespace robofleet_client