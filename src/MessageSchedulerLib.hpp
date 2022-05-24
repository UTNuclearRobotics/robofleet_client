#pragma once
#include <chrono>
#include <queue>
#include <iostream>
#include <unordered_map>
#include <vector>
#include <functional>

/**
 * @brief Queues messages and schedules them on demand.
 * This implements Robofleet's backpressure monitoring algorithm for traffic control, and expects backpressure updates via the websocket ping/pong mechanism.
 * This does *not* include rate limiting, which is expected to be done by a client *before* calling `enqueue`,
 * as the client can avoid paying the cost of encoding a message by rate-limiting earlier.
 * Messages are enqueued, and then later scheduled
 * when schedule() is called.
 */
template <typename T>
class MessageSchedulerLib {
private:
  using SchedulerClock = std::chrono::high_resolution_clock;
  using TopicName = std::string;

  struct  TopicQueue {
    TopicQueue(const size_t _max_queue_size,
               const double _priority,
               const double _publish_interval) :
      max_queue_size(_max_queue_size),
      priority(_priority),
      publish_interval(_publish_interval)
    {
    }

    T extractNextMessage()
    {
      const T next = messages.front();
      messages.pop();
      return next;
    }

    std::queue<T> messages;

    const size_t max_queue_size;

    const double priority;

    const double publish_interval;

    // when a message was last sent on this topic
    SchedulerClock::time_point last_send_time = SchedulerClock::now();
  };

public:
  // signature of the function to call when a message is scheduled for transmission
  typedef std::function<void(const T&)> ScheduleCallback;

  MessageSchedulerLib(const uint64_t mq, const ScheduleCallback sc) :
      max_queue_before_waiting_(mq)
    , sc_(sc)
  {
  }

  void enqueue(const TopicName& topic,
               const T& data,
               const double priority,
               const double rate_limit,
               const bool no_drop,
               const int queue_size)
  {
    if (no_drop)
    {
      no_drop_queue_.push(data);
    }
    else
    {
      // check if we have a queue for this topic yet
      typename QueueMap::iterator it = topic_queues_.find(topic);
      if (it == topic_queues_.end())
      {
        // set up the new queue
        const double publish_period = rate_limit != 0 ? 1.0 / rate_limit : 0.0;
        it = topic_queues_.emplace(std::piecewise_construct,
                                   std::forward_as_tuple(topic),
                                   std::forward_as_tuple(queue_size, priority, publish_period)).first;
      }

      TopicQueue& queue = it->second;

      if (queue.messages.size() == queue.max_queue_size)
      {
        // the queue is full, so drop the oldest message (the one in front)
        queue.messages.pop();
      }

      // add new message to the back of the queue
      queue.messages.push(data);
    }

    schedule();
  }

  /**
   * @brief Fire this to indicate that the network is free
   * Updates the counter for network backpressure
   */
  void backpressure_update(uint64_t message_index, uint64_t last_ponged_index)
  {
    network_backpressure_counter_ = message_index - last_ponged_index;
    schedule();
  }

  /**
   * @brief Schedule messages now.
   * Messages flagged as no_drop are sent first, in FIFO fashion.
   * Then, messages are sent by topic priority.
   */
  void schedule()
  {
    // flush no-drop queue
    while (!no_drop_queue_.empty() && !serverIsBusy())
    {
      const T& next = no_drop_queue_.front();
      sc_(next);
      no_drop_queue_.pop();
      ++network_backpressure_counter_;
    }

    if (serverIsBusy())
    {
      return;
    }

    const auto now = SchedulerClock::now();

    struct Candidate
    {
      Candidate(TopicQueue* _queue, const std::chrono::duration<double> _wait_time) :
        queue(_queue),
        wait_time(_wait_time)
      {
      }

      TopicQueue* queue;
      std::chrono::duration<double> wait_time;
    };

    while (!serverIsBusy())
    {
      // apply transmission priorities to eligible topics
      std::vector<Candidate> candidates;
      candidates.reserve(topic_queues_.size());
      for (auto& pair : topic_queues_)
      {
        TopicQueue* queue = &pair.second;

        using Milliseconds = std::chrono::milliseconds;
        constexpr double millisec_to_sec = 1e-3;
        const Milliseconds time_since_sent =
          std::chrono::duration_cast<Milliseconds>(now - queue->last_send_time);

        // check if the topic is ready to transmit
        if (!queue->messages.empty()
            && time_since_sent.count() * millisec_to_sec > queue->publish_interval)
        {
          candidates.emplace_back(queue, time_since_sent * queue->priority);
        }
      }

      // no more topics ready to publish
      if (candidates.empty())
      {
        return;
      }

      // sort candidates by priority
      std::sort(candidates.begin(), candidates.end(),
                [](const Candidate& lhs, const Candidate& rhs)
                {
                  return lhs.wait_time < rhs.wait_time;
                }
               );
      
      // determine how many messages we are allowed to send
      const uint64_t msgs_allowed_to_send = max_queue_before_waiting_ - network_backpressure_counter_;
      const uint64_t msgs_to_send = std::min(msgs_allowed_to_send,(uint64_t)candidates.size());

      // attempt to publish top-K candidates
      for (uint64_t cand_idx = 0; cand_idx < msgs_to_send; ++cand_idx)
      {
        sc_(candidates[cand_idx].queue->extractNextMessage());
        candidates[cand_idx].queue->last_send_time = now;
        ++network_backpressure_counter_;
      }
    }
  }

  inline bool serverIsBusy() const
  {
    return network_backpressure_counter_ >= max_queue_before_waiting_;
  }

private:
  ScheduleCallback sc_;
  uint64_t network_backpressure_counter_ = 0;
  uint64_t max_queue_before_waiting_ = 1;

  std::queue<T> no_drop_queue_;

  typedef std::unordered_map<TopicName, TopicQueue> QueueMap;
  QueueMap topic_queues_;
};