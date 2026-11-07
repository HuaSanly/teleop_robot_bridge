#include "udp/pacer.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <thread>

namespace trb::udp {

void Pacer::reset() {
  next_send_tp_ = std::chrono::steady_clock::now();
}

void Pacer::pace(size_t bytes, bool enabled, uint64_t pacing_bps) {
  if (!enabled || pacing_bps == 0) {
    next_send_tp_ = std::chrono::steady_clock::now();
    return;
  }

  const double rate_bytes_per_sec = static_cast<double>(pacing_bps) / 8.0;
  if (rate_bytes_per_sec <= 0.0) {
    next_send_tp_ = std::chrono::steady_clock::now();
    return;
  }

  auto now = std::chrono::steady_clock::now();
  if (next_send_tp_ < now) {
    next_send_tp_ = now;
  }

  if (next_send_tp_ > now) {
    std::this_thread::sleep_until(next_send_tp_);
  }

  const double pkt_us_d = (static_cast<double>(bytes) * 1e6) / rate_bytes_per_sec;
  const uint64_t pkt_us = static_cast<uint64_t>(std::max(0.0, std::ceil(pkt_us_d)));
  next_send_tp_ += std::chrono::microseconds(pkt_us);
}

void WebRtcLikePacer::reset() {
  media_debt_bytes_ = 0.0;
  adjusted_pacing_bps_ = config_.pacing_bps;
  last_process_tp_ = std::chrono::steady_clock::now();
  last_send_tp_ = last_process_tp_;
}

void WebRtcLikePacer::setConfig(const Config &config) {
  config_ = config;
  if (config_.max_drain_multiplier < 1.0) {
    config_.max_drain_multiplier = 1.0;
  }
  adjusted_pacing_bps_ = adjustedRateBps(0);
}

std::chrono::microseconds WebRtcLikePacer::timeUntilNextSend(size_t queue_bytes,
                                                             bool next_packet_accounted,
                                                             std::chrono::steady_clock::time_point now) {
  if (!config_.enabled || config_.pacing_bps == 0 || !next_packet_accounted) {
    updateBudget(now, queue_bytes);
    return std::chrono::microseconds(0);
  }

  updateBudget(now, queue_bytes);
  const double allowed_debt = allowedBurstBytes(queue_bytes);
  if (media_debt_bytes_ <= allowed_debt) {
    return std::chrono::microseconds(0);
  }

  const double bytes_per_sec = rateBytesPerSec(queue_bytes);
  if (bytes_per_sec <= 0.0) {
    return std::chrono::microseconds(0);
  }

  const double wait_us = ((media_debt_bytes_ - allowed_debt) * 1e6) / bytes_per_sec;
  return std::chrono::microseconds(static_cast<int64_t>(std::ceil(std::max(0.0, wait_us))));
}

void WebRtcLikePacer::onPacketSent(size_t bytes,
                                   bool accounted,
                                   size_t queue_bytes,
                                   std::chrono::steady_clock::time_point now) {
  updateBudget(now, queue_bytes);
  if (config_.enabled && config_.pacing_bps > 0 && accounted) {
    media_debt_bytes_ += static_cast<double>(bytes);
    media_debt_bytes_ = std::min(media_debt_bytes_, maxDebtBytes(queue_bytes));
  }
  last_send_tp_ = now;
}

WebRtcLikePacer::Stats WebRtcLikePacer::stats(size_t queue_bytes, std::chrono::steady_clock::time_point now) {
  updateBudget(now, queue_bytes);
  Stats out;
  out.adjusted_pacing_bps = adjusted_pacing_bps_;
  const double bytes_per_sec = rateBytesPerSec(queue_bytes);
  if (bytes_per_sec > 0.0) {
    out.debt_us = static_cast<uint64_t>(std::ceil((std::max(0.0, media_debt_bytes_) * 1e6) / bytes_per_sec));
    out.expected_queue_us = static_cast<uint64_t>(std::ceil((static_cast<double>(queue_bytes) * 1e6) / bytes_per_sec));
  }
  return out;
}

void WebRtcLikePacer::updateBudget(std::chrono::steady_clock::time_point now, size_t queue_bytes) {
  if (now < last_process_tp_) {
    last_process_tp_ = now;
    return;
  }

  adjusted_pacing_bps_ = adjustedRateBps(queue_bytes);
  const auto elapsed = std::chrono::duration_cast<std::chrono::microseconds>(now - last_process_tp_);
  last_process_tp_ = now;
  if (elapsed.count() <= 0) {
    return;
  }

  const double bytes_per_sec = rateBytesPerSec(queue_bytes);
  if (bytes_per_sec <= 0.0) {
    media_debt_bytes_ = 0.0;
    return;
  }

  const double elapsed_us = std::min<double>(static_cast<double>(elapsed.count()), 2000000.0);
  media_debt_bytes_ -= (bytes_per_sec * elapsed_us) / 1e6;
  if (media_debt_bytes_ < 0.0) {
    media_debt_bytes_ = 0.0;
  }
  media_debt_bytes_ = std::min(media_debt_bytes_, maxDebtBytes(queue_bytes));
}

uint64_t WebRtcLikePacer::adjustedRateBps(size_t queue_bytes) const {
  if (!config_.enabled || config_.pacing_bps == 0) {
    return config_.pacing_bps;
  }

  double adjusted = static_cast<double>(config_.pacing_bps);
  if (config_.queue_time_limit_ms > 0 && queue_bytes > 0) {
    const double target_bps = (static_cast<double>(queue_bytes) * 8.0 * 1000.0) /
                              static_cast<double>(config_.queue_time_limit_ms);
    adjusted = std::max(adjusted, target_bps);
  }

  const double max_bps = static_cast<double>(config_.pacing_bps) * config_.max_drain_multiplier;
  adjusted = std::min(adjusted, max_bps);
  adjusted = std::max(adjusted, static_cast<double>(config_.pacing_bps));
  if (adjusted > static_cast<double>(std::numeric_limits<uint64_t>::max())) {
    return std::numeric_limits<uint64_t>::max();
  }
  return static_cast<uint64_t>(adjusted);
}

double WebRtcLikePacer::rateBytesPerSec(size_t queue_bytes) const {
  const uint64_t bps = adjusted_pacing_bps_ > 0 ? adjusted_pacing_bps_ : adjustedRateBps(queue_bytes);
  return static_cast<double>(bps) / 8.0;
}

double WebRtcLikePacer::allowedBurstBytes(size_t queue_bytes) const {
  const double bytes_per_sec = rateBytesPerSec(queue_bytes);
  const double interval_bytes = (bytes_per_sec * static_cast<double>(config_.send_burst_interval_ms)) / 1000.0;
  if (config_.max_burst_bytes == 0) {
    return interval_bytes;
  }
  return std::min(interval_bytes, static_cast<double>(config_.max_burst_bytes));
}

double WebRtcLikePacer::maxDebtBytes(size_t queue_bytes) const {
  const double bytes_per_sec = rateBytesPerSec(queue_bytes);
  return (bytes_per_sec * static_cast<double>(config_.max_debt_ms)) / 1000.0;
}

}  // namespace trb::udp
