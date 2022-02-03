#pragma once

#include <QObject>
#include "MessageSchedulerLib.hpp"

/**
 * @brief Queues messages and schedules them on demand.
 *
 * Messages are enqueued, and then later scheduled (via the scheduled signal)
 * when schedule() is called.
 */
class MessageScheduler : public QObject {
  Q_OBJECT
  
  MessageSchedulerLib<QByteArray>* ms;

 public:
  MessageScheduler(uint64_t max_queue_before_waiting);

  void scheduling_callback(const QByteArray& data);

 Q_SIGNALS:
  void scheduled(const QByteArray& data);

 public Q_SLOTS:
  void enqueue(const QString& topic,
               const QByteArray& data,
               double priority,
               double rate_limit,
               bool no_drop);

  /**
   * @brief Fire this to indicate that the network is free
   * Updates the counter for network backpressure
   */
  void backpressure_update(uint64_t message_index, uint64_t last_ponged_index);

  /**
   * @brief Schedule messages now.
   * Messages flagged as no_drop are sent first, in FIFO fashion.
   * Then, messages are sent by topic priority.
   */
  void schedule();
};
