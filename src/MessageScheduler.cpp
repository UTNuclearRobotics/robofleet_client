#include "MessageScheduler.hpp"

#include <QByteArray>
#include <QHash>
#include <QString>
#include <chrono>
#include <deque>
#include <iostream>
#include <unordered_map>
#include <vector>
#include <functional>

MessageScheduler::MessageScheduler(const uint64_t max_queue_before_waiting) :
  ms_(max_queue_before_waiting,
      Backend::ScheduleCallback(std::bind(&MessageScheduler::scheduling_callback,
                                          this, std::placeholders::_1)))
{
}

void MessageScheduler::scheduling_callback(const QByteArray& data)
{
  Q_EMIT scheduled(data);
}

void MessageScheduler::enqueue(
    const QString& topic, const QByteArray& data, double priority, double rate_limit,
    bool no_drop)
{
  ms_.enqueue(topic.toUtf8().constData(), data, priority, rate_limit, no_drop);
}

void MessageScheduler::backpressure_update(const uint64_t message_index,
                                           const uint64_t last_ponged_index)
{
  ms_.backpressure_update(message_index, last_ponged_index);
}

void MessageScheduler::schedule()
{
  ms_.schedule();
}