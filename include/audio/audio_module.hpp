#pragma once

#include <atomic>
#include <cstddef>
#include <cstdint>
#include <memory>

#include <ros/ros.h>

#include "audio/audio_manager.hpp"

namespace trb::audio {

// Facade for the audio subsystem. Owns the AudioManager and all audio-related
// state (started, config published). MainNode interacts with this class only.
class AudioModule {
 public:
  using Config = AudioManager::Config;
  using EncodedPacketCallback = AudioManager::EncodedPacketCallback;

  // Load all `audio/*` parameters from ROS.
  static Config configFromRosParam(ros::NodeHandle& nh);

  AudioModule(ros::NodeHandle& nh, const Config& config);
  ~AudioModule();

  AudioModule(const AudioModule&) = delete;
  AudioModule& operator=(const AudioModule&) = delete;

  // Initialize underlying AudioManager. Returns false if disabled or failed.
  bool initialize();

  // Start/stop audio capture & encoding. Only effective after initialize() and
  // after the generic audio StreamConfig was stored (markConfigPublished()).
  bool start();
  void stop();
  bool isRunning() const { return started_.load(); }

  // True when module is enabled in config AND initialized successfully.
  bool isEnabled() const { return static_cast<bool>(manager_); }

  // Audio StreamConfig publication tracking (driven by MainNode/GrpcModule).
  bool isConfigPublished() const { return config_published_.load(); }
  void markConfigPublished() { config_published_.store(true); }
  void resetConfigPublished() { config_published_.store(false); }

  void setEncodedPacketCallback(EncodedPacketCallback cb);
  bool pushRemoteOpusPacket(const uint8_t* data, size_t size, uint64_t timestamp_ntp);

  const Config& config() const { return config_; }

 private:
  ros::NodeHandle& nh_;
  Config config_;
  std::unique_ptr<AudioManager> manager_;
  EncodedPacketCallback encoded_cb_;
  std::atomic<bool> started_{false};
  std::atomic<bool> config_published_{false};
};

}  // namespace trb::audio
