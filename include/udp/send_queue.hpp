#pragma once

#include <array>
#include <chrono>
#include <condition_variable>
#include <cstddef>
#include <cstdint>
#include <deque>
#include <mutex>
#include <optional>
#include <vector>

namespace trb::udp {

struct QueueItem {
  enum class Kind { Datagram, GroupEnd };
  enum class Stream { Video, Audio, Other };
  static constexpr uint8_t kHighestPriority = 0;
  static constexpr uint8_t kLowestPriority = 4;

  Kind kind{Kind::Datagram};
  Stream stream{Stream::Other};
  uint32_t group_id{0};
  uint16_t packet_seq{0};
  uint32_t stream_key{0};
  uint8_t priority{kLowestPriority};
  bool pacing_accounted{true};
  bool is_keyframe{false};
  bool is_fec_parity{false};
  uint64_t enqueue_steady_us{0};
  uint64_t frame_capture_steady_us{0};
  std::vector<uint8_t> bytes;
  size_t wire_bytes{0};
};

class SendQueue {
 public:
  static constexpr size_t kNumPriorityLevels = 5;

  struct Snapshot {
    size_t packets{0};
    size_t bytes{0};
    uint64_t dropped_video_groups{0};
    uint64_t dropped_video_packets{0};
    uint64_t dropped_video_bytes{0};
  };

  struct Status {
    size_t packets{0};
    size_t bytes{0};
    uint64_t oldest_enqueue_steady_us{0};
    std::array<size_t, kNumPriorityLevels> priority_packets{};
    std::array<size_t, kNumPriorityLevels> priority_bytes{};
  };

  struct ItemMeta {
    QueueItem::Kind kind{QueueItem::Kind::Datagram};
    QueueItem::Stream stream{QueueItem::Stream::Other};
    uint32_t group_id{0};
    uint32_t stream_key{0};
    uint8_t priority{QueueItem::kLowestPriority};
    bool pacing_accounted{true};
    size_t wire_bytes{0};
    uint64_t enqueue_steady_us{0};
  };

  void setLimits(size_t max_packets, size_t max_bytes, uint32_t max_queue_time_ms = 0);
  void stop();
  // Drop all queued items but leave running_ == true so producers/consumers
  // can keep working. Used on session reset (e.g. unpair / peer-lost).
  void clear();

  void push(QueueItem &&item);
  bool pop(QueueItem &out, int timeout_ms);
  bool popPrioritized(QueueItem &out, int timeout_ms);
  bool peekPrioritized(ItemMeta &out, int timeout_ms);
  bool waitForDataFor(std::chrono::microseconds timeout);
  Status status();
  Snapshot snapshot();

 private:
  using QueueIterator = std::deque<QueueItem>::iterator;

  std::optional<QueueIterator> selectPrioritizedLocked();
  void dropOldestGroupLocked(const char *reason);
  bool hasExpiredVideoLocked(uint64_t now_steady_us) const;
  void removeItemLocked(QueueIterator it, QueueItem &out);
  static uint8_t normalizePriority(uint8_t priority);

  std::mutex mutex_;
  std::condition_variable cv_;
  std::deque<QueueItem> queue_;
  size_t queue_bytes_{0};
  size_t max_packets_{0};
  size_t max_bytes_{0};
  uint32_t max_queue_time_ms_{0};
  bool running_{true};
  std::array<uint32_t, kNumPriorityLevels> last_stream_by_priority_{};
  std::array<bool, kNumPriorityLevels> has_last_stream_by_priority_{};
  uint64_t dropped_video_groups_{0};
  uint64_t dropped_video_packets_{0};
  uint64_t dropped_video_bytes_{0};
};

}  // namespace trb::udp
