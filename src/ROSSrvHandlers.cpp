#include "robofleet_client/ROSSrvHandlers.hpp"
#include "MessageScheduler.hpp"
#include "WsServer.hpp"

namespace robofleet_client
{
  void ROSSrvOutHandler::initialize(rclcpp::Node* node,
                                    MessageScheduler& scheduler,
                                    const std::string client_service,
                                    const std::string rbf_topic)
  {
    (void)node;
    (void)client_service;
    schedule_function_ = [&scheduler, rbf_topic](const QByteArray& data)
      {
        scheduler.enqueue(QString::fromStdString(rbf_topic),
                          data,
                          std::numeric_limits<double>::max(),
                          std::numeric_limits<double>::max(),
                          true,
                          1);
      };
  }

  void ROSSrvOutHandler::initialize(rclcpp::Node* node,
                                    WsServer& server,
                                    const std::string client_service,
                                    const std::string rbf_topic)
  {
    (void)node;
    (void)client_service;
    (void)rbf_topic;
    schedule_function_ = [&server](const QByteArray& data)
      {
        server.broadcast_message(data, nullptr);
      };
    
  }


  void ROSSrvInHandler::initialize(rclcpp::Node* node,
                                   MessageScheduler& scheduler,
                                   const std::string client_service,
                                   const std::string rbf_topic,
                                   const rclcpp::Duration timeout)
  {
    (void)node;
    (void)client_service;
    schedule_function_ = [&scheduler, rbf_topic](const QByteArray& data)
      {
        scheduler.enqueue(QString::fromStdString(rbf_topic),
                          data,
                          std::numeric_limits<double>::max(),
                          std::numeric_limits<double>::max(),
                          true,
                          1);
      };

    timeout_ = timeout;
  }

  void ROSSrvInHandler::initialize(rclcpp::Node* node,
                                   WsServer& server,
                                   const std::string client_service,
                                   const std::string rbf_topic,
                                   const rclcpp::Duration timeout)
  {
    (void)node;
    (void)client_service;
    (void)rbf_topic;
    schedule_function_ = [&server](const QByteArray& data)
      {
        server.broadcast_message(data, nullptr);
      };

    timeout_ = timeout;
  }


  void ROSSrvInHandler::returnResponse(const QByteArray& data)
  {
    response_data_ = data;
    
    response_mutex_.lock();
    has_received_response_ = true;
    response_mutex_.unlock();
  }


  bool ROSSrvInHandler::awaitReponse()
  {
    has_received_response_ = false;

    rclcpp::Clock clock;
    const rclcpp::Time start_time = clock.now();
    const bool using_timeout = timeout_ > rclcpp::Duration(0,0);

    while (true)
    {
      // check if we have received the response
      response_mutex_.lock();
      if (has_received_response_)
      {
        response_mutex_.unlock();
        return true;
      }
      response_mutex_.unlock();

      // check if we have timed out
      if (using_timeout && (clock.now()-start_time) > timeout_)
      {
        return false;
      }
    }
  }

} // namespace robofleet_client