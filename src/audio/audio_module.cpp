#include "audio/audio_module.hpp"

#include <algorithm>

#include <ros/ros.h>

namespace trb::audio {

AudioModule::Config AudioModule::configFromRosParam(ros::NodeHandle& nh) {
  Config cfg;
  nh.param<bool>("audio/enabled", cfg.enabled, false);
  nh.param<bool>("audio/uplink_enabled", cfg.uplink_enabled, true);
  nh.param<bool>("audio/downlink_enabled", cfg.downlink_enabled, true);
  nh.param<bool>("audio/aec_enabled", cfg.aec_enabled, false);
  nh.param<std::string>("audio/capture_device", cfg.capture_device, "");
  nh.param<std::string>("audio/playback_device", cfg.playback_device, "");

  int sample_rate = static_cast<int>(cfg.sample_rate);
  nh.param<int>("audio/sample_rate", sample_rate, 48000);
  cfg.sample_rate = static_cast<uint32_t>(std::max(0, sample_rate));

  int channels = static_cast<int>(cfg.channels);
  nh.param<int>("audio/channels", channels, 1);
  cfg.channels = static_cast<uint32_t>(std::max(0, channels));

  int frame_duration_ms = static_cast<int>(cfg.frame_duration_ms);
  nh.param<int>("audio/frame_duration_ms", frame_duration_ms, 20);
  cfg.frame_duration_ms = static_cast<uint32_t>(std::max(0, frame_duration_ms));

  int samples_per_channel = static_cast<int>(cfg.samples_per_channel);
  nh.param<int>("audio/samples_per_channel", samples_per_channel, 960);
  cfg.samples_per_channel = static_cast<uint32_t>(std::max(0, samples_per_channel));

  int bitrate_bps = static_cast<int>(cfg.bitrate_bps);
  nh.param<int>("audio/bitrate_bps", bitrate_bps, 64000);
  cfg.bitrate_bps = static_cast<uint32_t>(std::max(0, bitrate_bps));

  nh.param<bool>("audio/opus_inband_fec_enabled", cfg.opus_inband_fec_enabled, true);
  nh.param<bool>("audio/opus_dtx_enabled", cfg.opus_dtx_enabled, false);
  return cfg;
}

AudioModule::AudioModule(ros::NodeHandle& nh, const Config& config)
    : nh_(nh), config_(config) {}

AudioModule::~AudioModule() {
  stop();
  manager_.reset();
}

bool AudioModule::initialize() {
  if (!config_.enabled) {
    ROS_INFO("AudioModule disabled");
    return false;
  }
  manager_ = std::make_unique<AudioManager>();
  if (encoded_cb_) {
    manager_->setEncodedPacketCallback(encoded_cb_);
  }
  if (!manager_->initialize(config_)) {
    ROS_WARN("AudioManager initialize failed");
    manager_.reset();
    return false;
  }
  ROS_INFO("AudioModule initialized (will start after AV negotiation)");
  return true;
}

bool AudioModule::start() {
  if (!manager_) {
    return false;
  }
  if (started_.load()) {
    return true;
  }
  if (!config_published_.load()) {
    ROS_WARN("Audio StreamConfig not stored yet; AudioModule will not start");
    return false;
  }
  if (!manager_->start()) {
    ROS_WARN("AudioManager start failed");
    return false;
  }
  started_.store(true);
  ROS_INFO("AudioModule started (capture_device=%s playback_device=%s)",
           config_.capture_device.empty() ? "<default>" : config_.capture_device.c_str(),
           config_.playback_device.empty() ? "<default>" : config_.playback_device.c_str());
  return true;
}

void AudioModule::stop() {
  if (manager_ && started_.exchange(false)) {
    manager_->stop();
  }
}

void AudioModule::setEncodedPacketCallback(EncodedPacketCallback cb) {
  encoded_cb_ = std::move(cb);
  if (manager_) {
    manager_->setEncodedPacketCallback(encoded_cb_);
  }
}

bool AudioModule::pushRemoteOpusPacket(const uint8_t* data, size_t size, uint64_t timestamp_ntp) {
  if (!manager_) return false;
  return manager_->pushRemoteOpusPacket(data, size, timestamp_ntp);
}

}  // namespace trb::audio
