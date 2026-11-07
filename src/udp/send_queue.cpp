#include "udp/send_queue.hpp"

#include <algorithm>
#include <iterator>

#include <ros/ros.h>

namespace trb::udp {

namespace {

uint64_t steadyNowUs() {
  return static_cast<uint64_t>(std::chrono::duration_cast<std::chrono::microseconds>(
      std::chrono::steady_clock::now().time_since_epoch()).count());
}

}  // namespace

uint8_t SendQueue::normalizePriority(uint8_t priority) {
  return static_cast<uint8_t>(std::min<size_t>(priority, kNumPriorityLevels - 1));
}

void SendQueue::setLimits(size_t max_packets, size_t max_bytes, uint32_t max_queue_time_ms) {
  std::lock_guard<std::mutex> lk(mutex_);
  max_packets_ = max_packets;
  max_bytes_ = max_bytes;
  max_queue_time_ms_ = max_queue_time_ms;
}

void SendQueue::stop() {
  {
    std::lock_guard<std::mutex> lk(mutex_);
    running_ = false;
    queue_.clear();
    queue_bytes_ = 0;
  }
  cv_.notify_all();
}

void SendQueue::clear() {
  std::lock_guard<std::mutex> lk(mutex_);
  queue_.clear();
  queue_bytes_ = 0;
}

void SendQueue::push(QueueItem &&item) {
  std::unique_lock<std::mutex> lk(mutex_);
  if (!running_) {
    return;
  }

  if (item.kind == QueueItem::Kind::Datagram) {
    queue_bytes_ += item.wire_bytes;
  }
  item.priority = normalizePriority(item.priority);

  queue_.push_back(std::move(item));

  if (max_bytes_ > 0 && queue_bytes_ > max_bytes_) {
    ROS_WARN_THROTTLE(1.0, "UDP send queue high water: %zu/%zu bytes (%zu packets)",
                      queue_bytes_, max_bytes_, queue_.size());
  }

  while (!queue_.empty() && hasExpiredVideoLocked(steadyNowUs())) {
    dropOldestGroupLocked("latency");
  }

  while (!queue_.empty() &&
         ((max_packets_ > 0 && queue_.size() > max_packets_) || (max_bytes_ > 0 && queue_bytes_ > max_bytes_))) {
    dropOldestGroupLocked("capacity");
  }

  lk.unlock();
  cv_.notify_one();
}

bool SendQueue::pop(QueueItem &out, int timeout_ms) {
  std::unique_lock<std::mutex> lk(mutex_);
  if (!running_) {
    return false;
  }

  if (!cv_.wait_for(lk, std::chrono::milliseconds(timeout_ms), [this] { return !queue_.empty() || !running_; })) {
    return false;
  }
  if (!running_ || queue_.empty()) {
    return false;
  }

  out = std::move(queue_.front());
  if (out.kind == QueueItem::Kind::Datagram) {
    if (queue_bytes_ >= out.wire_bytes) {
      queue_bytes_ -= out.wire_bytes;
    } else {
      queue_bytes_ = 0;
    }
  }
  queue_.pop_front();
  return true;
}

bool SendQueue::popPrioritized(QueueItem &out, int timeout_ms) {
  std::unique_lock<std::mutex> lk(mutex_);
  if (!running_) {
    return false;
  }

  if (!cv_.wait_for(lk, std::chrono::milliseconds(timeout_ms), [this] { return !queue_.empty() || !running_; })) {
    return false;
  }
  if (!running_ || queue_.empty()) {
    return false;
  }

  auto selected = selectPrioritizedLocked();
  if (!selected.has_value()) {
    return false;
  }
  const uint8_t priority = normalizePriority((*selected)->priority);
  last_stream_by_priority_[priority] = (*selected)->stream_key;
  has_last_stream_by_priority_[priority] = true;
  removeItemLocked(*selected, out);
  return true;
}

bool SendQueue::peekPrioritized(ItemMeta &out, int timeout_ms) {
  std::unique_lock<std::mutex> lk(mutex_);
  if (!running_) {
    return false;
  }

  if (!cv_.wait_for(lk, std::chrono::milliseconds(timeout_ms), [this] { return !queue_.empty() || !running_; })) {
    return false;
  }
  if (!running_ || queue_.empty()) {
    return false;
  }

  auto selected = selectPrioritizedLocked();
  if (!selected.has_value()) {
    return false;
  }

  const QueueItem &item = **selected;
  out.kind = item.kind;
  out.stream = item.stream;
  out.group_id = item.group_id;
  out.stream_key = item.stream_key;
  out.priority = normalizePriority(item.priority);
  out.pacing_accounted = item.pacing_accounted;
  out.wire_bytes = item.wire_bytes;
  out.enqueue_steady_us = item.enqueue_steady_us;
  return true;
}

bool SendQueue::waitForDataFor(std::chrono::microseconds timeout) {
  std::unique_lock<std::mutex> lk(mutex_);
  if (!running_) {
    return false;
  }
  if (timeout.count() <= 0) {
    return running_;
  }
  cv_.wait_for(lk, timeout);
  return running_;
}

SendQueue::Status SendQueue::status() {
  std::lock_guard<std::mutex> lk(mutex_);
  Status status;
  status.packets = queue_.size();
  status.bytes = queue_bytes_;
  for (const auto &item : queue_) {
    if (item.enqueue_steady_us > 0 &&
        (status.oldest_enqueue_steady_us == 0 || item.enqueue_steady_us < status.oldest_enqueue_steady_us)) {
      status.oldest_enqueue_steady_us = item.enqueue_steady_us;
    }
    const size_t priority = normalizePriority(item.priority);
    status.priority_packets[priority] += 1;
    if (item.kind == QueueItem::Kind::Datagram) {
      status.priority_bytes[priority] += item.wire_bytes;
    }
  }
  return status;
}

SendQueue::Snapshot SendQueue::snapshot() {
  std::lock_guard<std::mutex> lk(mutex_);
  Snapshot snapshot;
  snapshot.packets = queue_.size();
  snapshot.bytes = queue_bytes_;
  snapshot.dropped_video_groups = dropped_video_groups_;
  snapshot.dropped_video_packets = dropped_video_packets_;
  snapshot.dropped_video_bytes = dropped_video_bytes_;
  dropped_video_groups_ = 0;
  dropped_video_packets_ = 0;
  dropped_video_bytes_ = 0;
  return snapshot;
}

bool SendQueue::hasExpiredVideoLocked(uint64_t now_steady_us) const {
  if (max_queue_time_ms_ == 0) {
    return false;
  }

  const uint64_t max_queue_time_us = static_cast<uint64_t>(max_queue_time_ms_) * 1000ULL;
  for (const auto &item : queue_) {
    if (item.stream != QueueItem::Stream::Video || item.enqueue_steady_us == 0) {
      continue;
    }
    return now_steady_us > item.enqueue_steady_us && (now_steady_us - item.enqueue_steady_us) > max_queue_time_us;
  }
  return false;
}

void SendQueue::dropOldestGroupLocked(const char *reason) {
  if (queue_.empty()) {
    return;
  }

  auto victim = std::find_if(queue_.begin(), queue_.end(), [](const QueueItem &item) {
    return item.stream == QueueItem::Stream::Video;
  });
  if (victim == queue_.end()) {
    victim = queue_.begin();
  }

  const uint32_t drop_group_id = victim->group_id;
  const QueueItem::Stream drop_stream = victim->stream;
  size_t dropped_items = 0;
  size_t dropped_bytes = 0;
  for (auto it = queue_.begin(); it != queue_.end();) {
    if (it->group_id == drop_group_id && it->stream == drop_stream) {
      if (it->kind == QueueItem::Kind::Datagram) {
        dropped_bytes += it->wire_bytes;
        if (queue_bytes_ >= it->wire_bytes) {
          queue_bytes_ -= it->wire_bytes;
        } else {
          queue_bytes_ = 0;
        }
      }
      it = queue_.erase(it);
      ++dropped_items;
    } else {
      ++it;
    }
  }

  if (drop_stream == QueueItem::Stream::Video) {
    dropped_video_groups_ += 1;
    dropped_video_packets_ += dropped_items;
    dropped_video_bytes_ += dropped_bytes;
  }

  // Throttle drop logs so a burst of drops cannot swamp the log stream.
  ROS_WARN_THROTTLE(1.0,
                    "SendQueue DROP: reason=%s stream=%d group_id=%u items=%zu bytes=%zu remaining_queue=%zu remaining_bytes=%zu",
                    reason ? reason : "unknown", static_cast<int>(drop_stream), drop_group_id, dropped_items,
                    dropped_bytes, queue_.size(), queue_bytes_);
}

std::optional<SendQueue::QueueIterator> SendQueue::selectPrioritizedLocked() {
  if (queue_.empty()) {
    return std::nullopt;
  }

  for (size_t priority = 0; priority < kNumPriorityLevels; ++priority) {
    std::vector<QueueIterator> candidates;
    for (auto it = queue_.begin(); it != queue_.end(); ++it) {
      if (normalizePriority(it->priority) != priority) {
        continue;
      }
      const auto already_seen = std::find_if(candidates.begin(), candidates.end(), [it](const QueueIterator &candidate) {
        return candidate->stream_key == it->stream_key;
      });
      if (already_seen == candidates.end()) {
        candidates.push_back(it);
      }
    }

    if (candidates.empty()) {
      continue;
    }
    if (!has_last_stream_by_priority_[priority]) {
      return candidates.front();
    }

    const uint32_t last_stream = last_stream_by_priority_[priority];
    auto last = std::find_if(candidates.begin(), candidates.end(), [last_stream](const QueueIterator &candidate) {
      return candidate->stream_key == last_stream;
    });
    if (last == candidates.end()) {
      return candidates.front();
    }
    ++last;
    if (last == candidates.end()) {
      return candidates.front();
    }
    return *last;
  }

  return std::nullopt;
}

void SendQueue::removeItemLocked(QueueIterator it, QueueItem &out) {
  out = std::move(*it);
  if (out.kind == QueueItem::Kind::Datagram) {
    if (queue_bytes_ >= out.wire_bytes) {
      queue_bytes_ -= out.wire_bytes;
    } else {
      queue_bytes_ = 0;
    }
  }
  queue_.erase(it);
}

}  // namespace trb::udp
