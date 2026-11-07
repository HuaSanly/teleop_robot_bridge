#pragma once

#include <atomic>
#include <cstddef>
#include <cstdint>
#include <functional>
#include <mutex>
#include <string>

namespace trb::audio {

class AudioManager {
 public:
  struct Config {
    bool enabled{false};
    bool uplink_enabled{true};
    bool downlink_enabled{true};
    bool aec_enabled{false};
    std::string capture_device;
    std::string playback_device;
    uint32_t sample_rate{48000};
    uint32_t channels{1};
    uint32_t frame_duration_ms{20};
    uint32_t samples_per_channel{960};
    uint32_t bitrate_bps{64000};
    bool opus_inband_fec_enabled{true};
    bool opus_dtx_enabled{false};
  };

  using EncodedPacketCallback = std::function<void(const uint8_t *data, size_t size, uint64_t timestamp_ntp)>;

  AudioManager();
  ~AudioManager();

  bool initialize(const Config &config);
  bool start();
  void stop();
  bool isRunning() const;

  void setEncodedPacketCallback(EncodedPacketCallback callback);
  bool pushRemoteOpusPacket(const uint8_t *data, size_t size, uint64_t timestamp_ntp);

  const Config &config() const;

 private:
  struct Impl;
  Impl *impl_{nullptr};
};

}  // namespace trb::audio