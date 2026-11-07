#include "udp/control_plane.hpp"

#include <chrono>
#include <thread>

#include <ros/ros.h>

namespace trb::udp {

namespace {
int64_t nowSteadyUs() {
  return std::chrono::duration_cast<std::chrono::microseconds>(
             std::chrono::steady_clock::now().time_since_epoch())
      .count();
}
}  // namespace

ControlPlane::ControlPlane(Config config) : config_(config) {}

ControlPlane::~ControlPlane() {
  stop();
}

void ControlPlane::start(std::function<void(const std::string &)> send_fn,
                         std::function<std::string()> hello_fn,
                         std::function<std::string()> ping_fn) {
  if (running_.exchange(true)) {
    return;
  }

  send_fn_ = std::move(send_fn);
  hello_fn_ = std::move(hello_fn);
  ping_fn_ = std::move(ping_fn);
  handshake_ready_.store(false);
  last_ack_steady_us_.store(0);

  thread_ = std::thread(&ControlPlane::threadMain, this);
}

void ControlPlane::stop() {
  if (!running_.exchange(false)) {
    return;
  }
  if (thread_.joinable()) {
    thread_.join();
  }
}

void ControlPlane::onAckReceived() {
  last_ack_steady_us_.store(nowSteadyUs());
  handshake_ready_.store(true);
}

void ControlPlane::onPeerActivity() {
  // Only count as liveness after handshake is up; otherwise stray packets
  // before handshake could mask a non-existent peer.
  if (handshake_ready_.load()) {
    last_ack_steady_us_.store(nowSteadyUs());
  }
}

bool ControlPlane::handshakeReady() const {
  return handshake_ready_.load();
}

void ControlPlane::resetHandshake() {
  handshake_ready_.store(false);
  last_ack_steady_us_.store(0);
}

void ControlPlane::threadMain() {
  using clock = std::chrono::steady_clock;
  auto next_hello = clock::now();
  auto next_ping = clock::now();

  const int64_t peer_lost_timeout_us =
      static_cast<int64_t>(config_.peer_lost_timeout_sec * 1e6);

  while (running_.load()) {
    if (!config_.enabled) {
      std::this_thread::sleep_for(std::chrono::milliseconds(100));
      continue;
    }

    const auto now = clock::now();
    if (!handshake_ready_.load()) {
      if (now >= next_hello) {
        if (send_fn_ && hello_fn_) {
          auto msg = hello_fn_();
          if (!msg.empty()) {
            send_fn_(msg);
          }
        }
        next_hello = now + std::chrono::milliseconds(static_cast<int>(config_.hello_interval_sec * 1000.0));
      }
    } else {
      // Peer-lost detection: no ACK within timeout => roll back to hello phase.
      const int64_t last_ack_us = last_ack_steady_us_.load();
      if (peer_lost_timeout_us > 0 && last_ack_us > 0 &&
          (nowSteadyUs() - last_ack_us) > peer_lost_timeout_us) {
        ROS_WARN("[ControlPlane] Peer lost: no ACK for %.1fs, rolling back to hello phase",
                 config_.peer_lost_timeout_sec);
        handshake_ready_.store(false);
        last_ack_steady_us_.store(0);
        next_hello = clock::now();
        if (peer_lost_cb_) {
          peer_lost_cb_();
        }
        continue;
      }

      if (now >= next_ping) {
        if (send_fn_ && ping_fn_) {
          auto msg = ping_fn_();
          if (!msg.empty()) {
            send_fn_(msg);
          }
        }
        next_ping = now + std::chrono::milliseconds(static_cast<int>(config_.ping_interval_sec * 1000.0));
      }
    }

    std::this_thread::sleep_for(std::chrono::milliseconds(20));
  }
}

}  // namespace trb::udp
