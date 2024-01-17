#include "robofleet_client/ROSMsgHandlers.hpp"
#include "MessageScheduler.hpp"
#include "WsServer.hpp"

namespace robofleet_client
{
  void RBFPublishHandler::initialize(std::shared_ptr<rclcpp::Node> node,
                                     MessageScheduler& scheduler,
                                     const std::string client_topic,
                                     const std::string rbf_topic,
                                     const double priority,
                                     const double rate_limit,
                                     const bool no_drop,
                                     const int queue_size)
  {
    (void)node;
    (void)client_topic;
    schedule_function_ = [&scheduler, rbf_topic, priority, rate_limit, no_drop, queue_size](const QByteArray& data)
      {
        scheduler.enqueue(QString::fromStdString(rbf_topic), data, priority, rate_limit, no_drop, queue_size);
      };
  }

  void RBFPublishHandler::initialize(std::shared_ptr<rclcpp::Node> node,
                                     WsServer& server,
                                     const std::string client_topic,
                                     const std::string rbf_topic)
  {
    (void)node;
    (void)client_topic;
    (void)rbf_topic;
    schedule_function_ = [&server](const QByteArray& data)
      {
        server.broadcast_message(data, nullptr);
      };
  }

} // namespace robofleet_client