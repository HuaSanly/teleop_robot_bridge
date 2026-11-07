#pragma once

#include <chrono>
#include <cstddef>
#include <cstdint>

namespace trb::udp {

class Pacer {
 public:
  void reset();
  void pace(size_t bytes, bool enabled, uint64_t pacing_bps);

 private:
  std::chrono::steady_clock::time_point next_send_tp_{std::chrono::steady_clock::now()};
};

class WebRtcLikePacer {
 public:
  struct Config {
    bool enabled{true};
    uint64_t pacing_bps{0};
    uint32_t send_burst_interval_ms{5};
    size_t max_burst_bytes{63000};
    uint32_t max_debt_ms{500};
    uint32_t queue_time_limit_ms{150};
    double max_drain_multiplier{2.0};
  };

  struct Stats {
    uint64_t adjusted_pacing_bps{0};
    uint64_t debt_us{0};
    uint64_t expected_queue_us{0};
  };

  void reset();
  void setConfig(const Config &config);
  std::chrono::microseconds timeUntilNextSend(size_t queue_bytes,
                                              bool next_packet_accounted,
                                              std::chrono::steady_clock::time_point now);
  void onPacketSent(size_t bytes,
                    bool accounted,
                    size_t queue_bytes,
                    std::chrono::steady_clock::time_point now);
  Stats stats(size_t queue_bytes, std::chrono::steady_clock::time_point now);

 private:
  void updateBudget(std::chrono::steady_clock::time_point now, size_t queue_bytes);
  uint64_t adjustedRateBps(size_t queue_bytes) const;
  double rateBytesPerSec(size_t queue_bytes) const;
  double allowedBurstBytes(size_t queue_bytes) const;
  double maxDebtBytes(size_t queue_bytes) const;

  Config config_;
  double media_debt_bytes_{0.0};
  uint64_t adjusted_pacing_bps_{0};
  std::chrono::steady_clock::time_point last_process_tp_{std::chrono::steady_clock::now()};
  std::chrono::steady_clock::time_point last_send_tp_{std::chrono::steady_clock::now()};
};

}  // namespace trb::udp
