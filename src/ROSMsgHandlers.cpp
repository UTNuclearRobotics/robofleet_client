#include "robofleet_client/ROSMsgHandlers.hpp"
#include "robofleet_client/MessageScheduler.hpp"

namespace robofleet_client
{
  bool ROSSubscribeHandler::initialize(ros::NodeHandle& nh,
                                       MessageScheduler* scheduler,
                                       const std::string client_topic,
                                       const std::string rbf_topic,
                                       const double priority,
                                       const double rate_limit,
                                       const bool no_drop)
  {
    if (scheduler == nullptr)
    {
      return false;
    }
    
    schedule_function_ = std::bind(&MessageScheduler::enqueue,
                                   scheduler,
                                   QString::fromStdString(rbf_topic),
                                   std::placeholders::_1,
                                   priority,
                                   rate_limit,
                                   no_drop);

    return true;
  }

} // namespace robofleet_client