#pragma once

#include <memory>
#include <functional>

#include <ros/ros.h>

namespace trb::video
{

    class VideoStreamManager
    {
    public:
        struct StatsSnapshot
        {
            uint64_t capture_frames = 0;
            uint64_t decode_frames = 0;
            uint64_t convert_frames = 0;
            uint64_t encode_frames = 0;
            uint64_t decoder_drops = 0;
            uint64_t converter_pool_drops = 0;
            uint64_t converter_failures = 0;
            uint64_t encoder_submit_failures = 0;
            uint64_t undistort_frames = 0;
            uint64_t undistort_pool_drops = 0;
            uint64_t undistort_failures = 0;
            uint64_t decode_us_total = 0;
            uint64_t transform_us_total = 0;
            uint64_t undistort_us_total = 0;
            uint64_t encode_us_total = 0;
            uint64_t eye_image_enqueued = 0;
            uint64_t eye_image_published = 0;
            uint64_t eye_image_dropped = 0;
            uint64_t eye_image_process_us_total = 0;
            uint64_t eye_image_mailbox_depth = 0;
            uint64_t eye_image_frame_age_us_max = 0;
            uint64_t eye_image_cuda_submitted = 0;
            uint64_t eye_image_cuda_completed = 0;
            uint64_t eye_image_cuda_failures = 0;
            uint64_t eye_image_cuda_kernel_us_total = 0;
            uint64_t eye_image_cuda_device_to_host_us_total = 0;
            uint64_t eye_image_ros_publish_count = 0;
            uint64_t eye_image_ros_publish_us_total = 0;
            uint64_t undistort_cuda_submitted = 0;
            uint64_t undistort_cuda_completed = 0;
            uint64_t undistort_cuda_failures = 0;
            uint64_t undistort_cuda_kernel_us_total = 0;
        };

        explicit VideoStreamManager(ros::NodeHandle &nh);
        ~VideoStreamManager();

        VideoStreamManager(const VideoStreamManager &) = delete;
        VideoStreamManager &operator=(const VideoStreamManager &) = delete;

        // Start the full video pipeline (capture -> convert -> encode -> record).
        // Returns true if startup succeeded.
        bool start();

        // Stop threads and release resources.
        void stop();

        bool isRunning() const;

        using EncodedFrameCallback = std::function<void(const uint8_t *data, size_t size, uint64_t timestamp_us, bool keyframe)>;
        void setEncodedFrameCallback(EncodedFrameCallback callback);

        // Called once when encoder captures the codec parameter sets (without
        // start codes). For H.264 |vps| is empty; for H.265 |vps|, |sps| and
        // |pps| are all populated.
        using SpsPpsCallback = std::function<void(const std::vector<uint8_t> &sps,
                                                  const std::vector<uint8_t> &pps,
                                                  const std::vector<uint8_t> &vps)>;
        void setSpsPpsCallback(SpsPpsCallback callback);

        // Update encoder target bitrate (bps). No-op if encoder is unavailable.
        void setTargetBitrate(uint32_t bitrate_bps);

        // Consume and reset video pipeline stats since the previous snapshot.
        StatsSnapshot consumeStats();

    private:
        class Impl;
        std::unique_ptr<Impl> impl_;
    };

} // namespace trb::video
