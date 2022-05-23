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
  
  // MessageScheduler mostly just exposes this class through Qt
  typedef MessageSchedulerLib<QByteArray> Backend;
  Backend ms_;

 public:
  MessageScheduler(const uint64_t max_queue_before_waiting);

  /**
   * @brief Just emits the scheduled Q_SIGNAL
   * @param data The raw data
   */
  void scheduling_callback(const QByteArray& data);

 Q_SIGNALS:
  /**
   * @brief Submits data to the websocket for transmission
   * @param data The raw data
   */
  void scheduled(const QByteArray& data);

 public Q_SLOTS:
  /**
   * @brief Receives processed data from the client
   * @param topic The Robofleet topic to transmit on
   * @param data The raw data
   * @param priority Importance of this data
   * @param rate_limit The maximum rate at which to transmit the data
   * @param If true, the topic has absolute priority
   */
  void enqueue(const QString& topic,
               const QByteArray& data,
               double priority,
               double rate_limit,
               bool no_drop);

  /**
   * @brief Fire this to indicate that the network is free
   * Updates the counter for network backpressure
   */
  void backpressure_update(const uint64_t message_index,
                           const uint64_t last_ponged_index);

  /**
   * @brief Schedule messages now.
   * Messages flagged as no_drop are sent first, in FIFO fashion.
   * Then, messages are sent by topic priority.
   */
  void schedule();
};
