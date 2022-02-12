#include "robofleet_client/ROSSrvHandlers.hpp"
#include "robofleet_client/MessageScheduler.hpp"

namespace robofleet_client
{
  bool ROSSrvOutHandler::initialize(ros::NodeHandle& nh,
                                    MessageScheduler* scheduler,
                                    const std::string client_service,
                                    const std::string rbf_topic)
  {
    if (scheduler == nullptr)
    {
      return false;
    }
    
    schedule_function_ = std::bind(&MessageScheduler::enqueue,
                                   scheduler,
                                   QString::fromStdString(rbf_topic),
                                   std::placeholders::_1,
                                   std::numeric_limits<double>::max(),
                                   std::numeric_limits<double>::max(),
                                   true);

    return true;
  }

  bool ROSSrvInHandler::initialize(ros::NodeHandle& nh,
                                   MessageScheduler* scheduler,
                                   const std::string client_service,
                                   const std::string rbf_topic)
  {
    if (scheduler == nullptr)
    {
      return false;
    }
    
    schedule_function_ = std::bind(&MessageScheduler::enqueue,
                                   scheduler,
                                   QString::fromStdString(rbf_topic),
                                   std::placeholders::_1,
                                   std::numeric_limits<double>::max(),
                                   std::numeric_limits<double>::max(),
                                   true);

    return true;
  }


  void ROSSrvInHandler::returnResponse(const QByteArray& data)
  {
    response_data_ = data;
    
    response_mutex_.lock();
    has_received_response_ = true;
    response_mutex_.unlock();
  }


  void ROSSrvInHandler::awaitReponse()
  {
    // TODO: Implement timeout feature
    has_received_response_ = false;

    // start another thread for spinning so that we don't block
    // the whole robofleet client
    ros::AsyncSpinner spinner(1);
    spinner.start();

    while (true)
    {
      response_mutex_.lock();
      if (has_received_response_)
      {
        spinner.stop();
        response_mutex_.unlock();
        return;
      }
      response_mutex_.unlock();
    }
  }

} // namespace robofleet_client