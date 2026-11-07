#pragma once

#include <atomic>
#include <cstdint>
#include <functional>
#include <memory>
#include <string>
#include <vector>

#include <ros/ros.h>

#include "grpc/grpc_manager.hpp"
#include "audio/audio_manager.hpp"

// Facade for the gRPC signaling subsystem. Owns the GrpcManager, register
// retry timer, heartbeat timer, and the protobuf message construction for
// VideoConfig / AudioConfig publication.
//
// MainNode only interacts with this class through callbacks and a few
// business methods.
class GrpcModule {
 public:
  using Config = GrpcConfig;
  using SignalingEventCallback = std::function<void(const signaling::EventMessage&)>;
  using RegisteredCallback = std::function<void()>;
  using HeartbeatFailCallback = std::function<void()>;
  // Fired (on gRPC worker thread) when the EventStream ends unexpectedly
  // (server-side disconnect, network loss, etc.). Not fired on explicit
  // StopEventStream().
  using StreamEndCallback = std::function<void()>;

  // Load all `grpc/*` parameters from ROS.
  static Config configFromRosParam(ros::NodeHandle& nh);

  GrpcModule(ros::NodeHandle& nh, const Config& config);
  ~GrpcModule();

  GrpcModule(const GrpcModule&) = delete;
  GrpcModule& operator=(const GrpcModule&) = delete;

  // Start the register/heartbeat lifecycle. Will call onRegistered when
  // registration succeeds, and retry on failure internally.
  void start();

  // Stop all internal timers and the signaling event stream.
  void stop();

  void setRegisteredCallback(RegisteredCallback cb) { on_registered_ = std::move(cb); }
  void setSignalingEventCallback(SignalingEventCallback cb) { on_event_ = std::move(cb); }
  void setHeartbeatFailCallback(HeartbeatFailCallback cb) { on_heartbeat_fail_ = std::move(cb); }
  void setStreamEndCallback(StreamEndCallback cb) { on_stream_end_ = std::move(cb); }

  bool isRegistered() const { return registered_.load(); }
  const std::string& sessionId() const;
  const std::string& deviceTypeCode() const;
  const Config& config() const { return config_; }

  // Drop the current session and perform a fresh Register. This must be
  // called outside the EventStream worker thread.
  void reconnect();
  bool pingNow();

  // Start receiving signaling events (call after pairing state is entered).
  void startEventStream();

  // Accept an incoming pair request.
  bool acceptPair(const std::string& peer_session_id);

  // Subscribe this robot session to the peer's pose stream (prefix 0x02).
  bool subscribePoseStream(const std::string& publisher_session_id);

  // Declare this publisher's UDP streams for the current backend session.
  bool publishMediaManifest(bool publish_audio);

  // Serialize the schema-v1 video payload as UTF-8 JSON and store it through the generic
  // PublishStreamConfig RPC for publisher-local prefix 0x01.
  // |codec| selects H.264 or H.265; |vps| must be non-empty for H.265.
  // |has_fov|=true adds the optional fov object; otherwise it is omitted.
  enum class Codec : int { kH264 = 0, kH265 = 1 };
  bool publishVideoConfig(uint32_t width, uint32_t height, uint32_t fps,
                          Codec codec,
                          const std::vector<uint8_t>& sps,
                          const std::vector<uint8_t>& pps,
                          const std::vector<uint8_t>& vps,
                          bool has_fov, float hfov_deg,
                          float vfov_deg, float dfov_deg);

  // Store the schema-v1 Opus JSON config for publisher-local prefix 0x04.
  bool publishAudioConfig(const trb::audio::AudioManager::Config& audio_cfg);

 private:
  void tryRegister();
  void onRegisterRetry(const ros::TimerEvent&);
  void onHeartbeat(const ros::TimerEvent&);

  ros::NodeHandle& nh_;
  Config config_;
  std::unique_ptr<GrpcManager> manager_;

  std::atomic<bool> registered_{false};
  ros::Timer register_retry_timer_;
  ros::Timer heartbeat_timer_;

  RegisteredCallback on_registered_;
  SignalingEventCallback on_event_;
  HeartbeatFailCallback on_heartbeat_fail_;
  StreamEndCallback on_stream_end_;
};
