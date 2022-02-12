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
                                   const std::string rbf_topic,
                                   const ros::Duration timeout)
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

    // start another thread for spinning so that we don't block
    // the whole robofleet client
    ros::AsyncSpinner spinner(1);
    spinner.start();

    const ros::Time start_time = ros::Time::now();
    const bool using_timeout = !timeout_.isZero();

    while (true)
    {
      // check if we have received the response
      response_mutex_.lock();
      if (has_received_response_)
      {
        spinner.stop();
        response_mutex_.unlock();
        return true;
      }
      response_mutex_.unlock();

      // check if we have timed out
      if (using_timeout && (ros::Time::now()-start_time) > timeout_)
      {
        return false;
      }
    }
  }

} // namespace robofleet_client