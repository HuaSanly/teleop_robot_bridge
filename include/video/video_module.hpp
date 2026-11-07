#pragma once

#include <atomic>
#include <cstdint>
#include <functional>
#include <memory>
#include <mutex>
#include <vector>

#include <ros/ros.h>

#ifdef JETSON_MM_API_ENABLED
#include "video/video_stream_manager.hpp"
#endif

namespace trb::video {

/**
 * Thin Facade over VideoStreamManager that owns the video profile config
 * and caches SPS/PPS for protocol negotiation.
 *
 * Stage E: shell only; internals still delegate to VideoStreamManager.
 * Stage F will rewrite the internals (2-thread capture/encode model).
 */
class VideoModule {
 public:
  struct Config {
    // Mirror of VideoEncoder::Codec, kept independent so this header does not
    // need to pull in the Jetson MM API.
    enum class Codec : int { kH264 = 0, kH265 = 1 };

    uint32_t width{3840};
    uint32_t height{1520};
    uint32_t fps{30};
    Codec codec{Codec::kH264};

    static Config configFromRosParam(ros::NodeHandle& nh);
  };

  using EncodedFrameCallback =
      std::function<void(const uint8_t* data, size_t size,
                         uint64_t timestamp_us, bool keyframe)>;
  using SpsPpsCallback = std::function<void(const std::vector<uint8_t>& sps,
                                            const std::vector<uint8_t>& pps,
                                            const std::vector<uint8_t>& vps)>;

#ifdef JETSON_MM_API_ENABLED
  using StatsSnapshot = VideoStreamManager::StatsSnapshot;
#else
  struct StatsSnapshot {
    uint64_t capture_frames{0};
    uint64_t decode_frames{0};
    uint64_t convert_frames{0};
    uint64_t encode_frames{0};
    uint64_t decoder_drops{0};
    uint64_t converter_pool_drops{0};
    uint64_t converter_failures{0};
    uint64_t encoder_submit_failures{0};
    uint64_t undistort_frames{0};
    uint64_t undistort_pool_drops{0};
    uint64_t undistort_failures{0};
    uint64_t decode_us_total{0};
    uint64_t transform_us_total{0};
    uint64_t undistort_us_total{0};
    uint64_t encode_us_total{0};
    uint64_t eye_image_enqueued{0};
    uint64_t eye_image_published{0};
    uint64_t eye_image_dropped{0};
    uint64_t eye_image_process_us_total{0};
    uint64_t eye_image_mailbox_depth{0};
    uint64_t eye_image_frame_age_us_max{0};
    uint64_t eye_image_cuda_submitted{0};
    uint64_t eye_image_cuda_completed{0};
    uint64_t eye_image_cuda_failures{0};
    uint64_t eye_image_cuda_kernel_us_total{0};
    uint64_t eye_image_cuda_device_to_host_us_total{0};
    uint64_t eye_image_ros_publish_count{0};
    uint64_t eye_image_ros_publish_us_total{0};
    uint64_t undistort_cuda_submitted{0};
    uint64_t undistort_cuda_completed{0};
    uint64_t undistort_cuda_failures{0};
    uint64_t undistort_cuda_kernel_us_total{0};
  };
#endif

  VideoModule(ros::NodeHandle nh, Config config);
  ~VideoModule();

  VideoModule(const VideoModule&) = delete;
  VideoModule& operator=(const VideoModule&) = delete;

  bool start();
  void stop();
  bool isRunning() const;

  void setEncodedFrameCallback(EncodedFrameCallback cb);
  void setSpsPpsCallback(SpsPpsCallback cb);

  // Returns true if codec parameter sets have been captured and cached.
  // For H.264 this is SPS+PPS; for H.265 this is VPS+SPS+PPS.
  bool hasSpsPps() const;
  // Copies the cached SPS/PPS/VPS. Returns false if not yet available.
  // |vps| is empty for H.264.
  bool getSpsPps(std::vector<uint8_t>& sps,
                 std::vector<uint8_t>& pps,
                 std::vector<uint8_t>& vps) const;
  // Clear cached SPS/PPS/VPS (e.g. on unpair before next session).
  void clearSpsPps();

  StatsSnapshot consumeStats();

  const Config& config() const { return config_; }

 private:
  ros::NodeHandle nh_;
  Config config_;
  EncodedFrameCallback encoded_cb_;
  SpsPpsCallback sps_pps_cb_;

  mutable std::mutex sps_pps_mutex_;
  std::vector<uint8_t> cached_sps_;
  std::vector<uint8_t> cached_pps_;
  std::vector<uint8_t> cached_vps_;  // H.265 only

#ifdef JETSON_MM_API_ENABLED
  std::unique_ptr<VideoStreamManager> manager_;
#endif
};

}  // namespace trb::video
