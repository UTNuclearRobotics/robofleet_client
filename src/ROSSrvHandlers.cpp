#include "robofleet_client/ROSSrvHandlers.hpp"
#include "robofleet_client/MessageScheduler.hpp"

namespace robofleet_client
{
  bool ROSSrvInHandler::initialize(ros::NodeHandle& nh,
                                  MessageScheduler* scheduler,
                                  const std::string service_name)
  {
    if (scheduler == nullptr)
    {
      return false;
    }
    
    schedule_function_ = std::bind(&MessageScheduler::enqueue,
                                   scheduler,
                                   QString::fromStdString(service_name),
                                   std::placeholders::_1,
                                   std::numeric_limits<double>::max(),
                                   std::numeric_limits<double>::max(),
                                   true);

    return true;
  }

  void ROSSrvInHandler::awaitReponse()
  {
    // TODO: Implement timeout feature
    has_received_response_ = false;

    while (true)
    {
      has_received_response_mutex_.lock();
      if (!has_received_response_)
      {
        has_received_response_mutex_.unlock();
        return;
      }
      has_received_response_mutex_.unlock();
    }
  }

} // namespace robofleet_client