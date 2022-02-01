#include <robofleet_client/MessageScheduler.hpp>

#include <QByteArray>
#include <QHash>
#include <QString>
#include <chrono>
#include <deque>
#include <iostream>
#include <unordered_map>
#include <vector>
#include <functional>

MessageScheduler::MessageScheduler(uint64_t max_queue_before_waiting) {
  std::function<void(const QByteArray&)> bound_callback(std::bind(&MessageScheduler::scheduling_callback, this, std::placeholders::_1));
  ms = new MessageSchedulerLib<QByteArray>(max_queue_before_waiting, bound_callback);
}

void MessageScheduler::scheduling_callback(const QByteArray& data) {
  Q_EMIT scheduled(data);
}

void MessageScheduler::enqueue(
    const QString& topic, const QByteArray& data, double priority, double rate_limit,
    bool no_drop) {
  ms->enqueue(topic.toUtf8().constData(), data, priority, rate_limit, no_drop);
}

void MessageScheduler::backpressure_update(uint64_t message_index, uint64_t last_ponged_index) {
  ms->backpressure_update(message_index, last_ponged_index);
}

void MessageScheduler::schedule() {
  ms->schedule();
}