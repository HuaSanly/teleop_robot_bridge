#pragma once

#include <atomic>
#include <cstdint>
#include <functional>
#include <memory>
#include <string>

#include <ros/ros.h>

#include "udp/udp_manager.hpp"
#include "udp/pose_publisher.hpp"

namespace trb::udp {

/**
 * Facade wrapping UdpManager + PosePublisher + handshake timer.
 *
 * Lifecycle:
 *   UdpModule mod(nh, UdpModule::configFromRosParam(nh));
 *   mod.setAudioDatagramCallback(...);
 *   mod.setHandshakeReadyCallback(...);  // invoked once when handshake ready
 *   mod.start(session_id);
 *   ... mod.sendH264Frame(...) / mod.sendAudioPacket(...) ...
 *   mod.stop();
 */
class UdpModule {
 public:
  struct Config {
    UdpManager::Config udp;
    bool pose_enabled{false};
    PosePublisherConfig pose;
    bool direct_running_mode{false};  // disables all UDP startup and I/O

    static Config configFromRosParam(ros::NodeHandle& nh);
  };

  using AudioDatagramCallback =
      std::function<void(const uint8_t* data, size_t size, uint64_t timestamp_ntp)>;
  using HandshakeReadyCallback = std::function<void()>;
  using PeerLostCallback = std::function<void()>;

  UdpModule(ros::NodeHandle nh, Config config);
  ~UdpModule();

  UdpModule(const UdpModule&) = delete;
  UdpModule& operator=(const UdpModule&) = delete;

  /**
   * Create UdpManager, start socket, initialize pose publisher (if enabled),
   * start handshake polling timer if control_enabled.
   * Returns false without creating resources in direct_running_mode.
   */
  bool start(const std::string& session_id);

  void stop();

  // Reset session state (clear queues, roll back handshake) without
  // destroying the socket/threads. After this, the next handshake will
  // re-trigger the handshake-ready callback.
  void resetSession();
  void requestHandshakeRefresh();

  bool isRunning() const { return udp_manager_ != nullptr; }
  bool isHandshakeReady() const;
  bool controlEnabled() const { return config_.udp.control_enabled; }

  void setAudioDatagramCallback(AudioDatagramCallback cb) { audio_cb_ = std::move(cb); }
  void setHandshakeReadyCallback(HandshakeReadyCallback cb) { on_ready_ = std::move(cb); }
  void setPeerLostCallback(PeerLostCallback cb) { on_peer_lost_ = std::move(cb); }

  void sendH264Frame(const uint8_t* data, size_t size, uint64_t capture_timestamp_us, bool keyframe);
  void sendAudioPacket(const uint8_t* data, size_t size, uint64_t timestamp_ntp);

  UdpManager::VideoStatsSnapshot consumeVideoStats();

  const Config& config() const { return config_; }

 private:
  void onDatagram(const uint8_t* data, size_t size);
  void handshakeTimerCallback(const ros::TimerEvent& event);

  ros::NodeHandle nh_;
  Config config_;
  std::unique_ptr<UdpManager> udp_manager_;
  std::unique_ptr<PosePublisher> pose_publisher_;
  ros::Timer handshake_timer_;
  AudioDatagramCallback audio_cb_;
  HandshakeReadyCallback on_ready_;
  PeerLostCallback on_peer_lost_;
  std::atomic<bool> ready_fired_{false};
};

}  // namespace trb::udp
