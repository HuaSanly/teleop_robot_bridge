#include "video/video_stream_manager.hpp"

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <condition_variable>
#include <cctype>
#include <cinttypes>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <functional>
#include <memory>
#include <mutex>
#include <queue>
#include <thread>
#include <unordered_map>
#include <utility>
#include <vector>

#include <linux/v4l2-controls.h>
#include <linux/videodev2.h>

#include <ros/ros.h>
#include <ros/package.h>

#include "v4l2_nv_extensions.h"

#include "video/video_converter.hpp"
#include "video/video_decoder.hpp"
#include "video/video_encoder.hpp"
#include "video/video_v4l2_capturer.hpp"
#include "video/eye_image_publisher.hpp"
#include "video/video_undistorter.hpp"
#include "video/nvbuf_mutex.hpp"

// Jetson Multimedia API for DMA-BUF surface access and transforms
#include "nvbufsurface.h"
#include "nvbufsurftransform.h"

namespace trb::video
{

    /**
     * @brief Architecture: 3 pipeline threads (post Stage F refactor).
     *
     * Threads:
     *   capture_thread_  : V4L2 dequeue -> decoder.submit(MJPEG) -> V4L2 requeue
     *   encode_thread_   : decoder.dequeueSync -> converter.transformSync
     *                      -> encoder.submit + encoder.dequeueOne -> onEncoderPacket
     *   eye_image_thread_: consume NV12 frames from nv12_queue_ -> ROS Image publish
     *
     * Data Flow:
     *   V4L2 Camera (MJPEG)                              [capture_thread_]
     *        -> NvVideoDecoder  (MJPEG -> YUV dma-buf)   [encode_thread_]
     *        -> VideoConverter  (YUV  -> NV12 via VIC)   [encode_thread_]
     *        -> VideoEncoder    (NV12 -> H.264)          [encode_thread_]
     *        -> onEncoderPacket -> UDP/gRPC              [encode_thread_]
        *
     *   In parallel: a ref-counted rectified NV12 fd is handed to
     *   eye_image_thread_ for CUDA crop/scale/RGB conversion and ROS publish.
     *   The fd is returned to its pool only after both NVENC input-done and
     *   eye_image processing have released their references.
     *
     * The CUDA eye path does not use the global VIC mutex. The mutex remains
     * available only for the explicit legacy fallback backend.
     */

    struct SharedNv12Frame
    {
        int fd{-1};
        uint64_t timestamp_us{0};
    };

    // NV12 frame data for eye image queue
    struct NV12FrameData
    {
        std::shared_ptr<SharedNv12Frame> frame;
        int64_t enqueued_steady_us{0};
    };

    class VideoStreamManager::Impl
    {
    public:
        explicit Impl(ros::NodeHandle &nh)
            : nh_(nh)
        {
        }

        ~Impl()
        {
            stop();
        }

        void setEncodedFrameCallback(EncodedFrameCallback callback)
        {
            std::lock_guard<std::mutex> lk(callback_mutex_);
            encoded_frame_callback_ = callback;
        }

        void setSpsPpsCallback(SpsPpsCallback callback)
        {
            std::shared_ptr<trb::video::VideoEncoder> enc;
            {
                std::lock_guard<std::mutex> lk(callback_mutex_);
                sps_pps_callback_ = callback;
                enc = encoder_;
            }
            if (enc)
            {
                enc->setSpsPpsCallback(callback);
            }
        }

        void setTargetBitrate(uint32_t bitrate_bps)
        {
            if (bitrate_bps == 0)
            {
                return;
            }

            const auto now = std::chrono::steady_clock::now();
            {
                std::lock_guard<std::mutex> lk(bitrate_mutex_);
                const auto elapsed = now - last_bitrate_update_;
                if (last_bitrate_bps_ == bitrate_bps && elapsed < std::chrono::milliseconds(200))
                {
                    return;
                }
                if (elapsed < std::chrono::milliseconds(100))
                {
                    return;
                }
                last_bitrate_update_ = now;
                last_bitrate_bps_ = bitrate_bps;
            }

            std::shared_ptr<trb::video::VideoEncoder> enc;
            {
                std::lock_guard<std::mutex> lk(callback_mutex_);
                enc = encoder_;
            }
            if (enc)
            {
                const bool ok = enc->setBitrate(bitrate_bps);
                if (!ok)
                {
                    ROS_WARN("VideoEncoder setBitrate failed: %u bps", bitrate_bps);
                }
            }
        }

        bool start()
        {
            if (running_.load())
                return true;

            capturer_ = std::make_shared<trb::video::VideoV4L2Capturer>();

            // Unified profile: [width, height, framerate]
            std::vector<int> profile;
            nh_.param<std::vector<int>>("video/profile", profile, {3840, 1520, 30});
            uint32_t width = 3840;
            uint32_t height = 1520;
            uint32_t framerate = 30;
            if (profile.size() == 3 && profile[0] > 0 && profile[1] > 0 && profile[2] > 0)
            {
                width = static_cast<uint32_t>(profile[0]);
                height = static_cast<uint32_t>(profile[1]);
                framerate = static_cast<uint32_t>(profile[2]);
            }
            else
            {
                ROS_WARN("Invalid video/profile; expected [width,height,framerate]. Using defaults.");
            }

            std::string video_device;
            nh_.param<std::string>("video/device", video_device, "/dev/video0");
            std::string pixel_format_str;
            nh_.param<std::string>("video/pixel_format", pixel_format_str, "mjpeg");
            int v4l2_buffer_count_param;
            nh_.param<int>("video/v4l2/buffer_count", v4l2_buffer_count_param, 8);
            const uint32_t v4l2_buffer_count = (v4l2_buffer_count_param > 0) ? static_cast<uint32_t>(v4l2_buffer_count_param) : 8;
            nh_.param<int>("video/v4l2/warmup_drop_frames", v4l2_warmup_drop_frames_, 30);
            if (v4l2_warmup_drop_frames_ < 0)
                v4l2_warmup_drop_frames_ = 0;

            int conv_pool_param;
            nh_.param<int>("video/decoder/buffer_pool_size", conv_pool_param, 12);
            const uint32_t conv_pool_size = (conv_pool_param > 0) ? static_cast<uint32_t>(conv_pool_param) : 12;

            trb::video::VideoV4L2Capturer::Config cap_cfg;
            cap_cfg.width = width;
            cap_cfg.height = height;
            cap_cfg.framerate = framerate;
            cap_cfg.device_path = video_device;
            cap_cfg.buffer_count = v4l2_buffer_count;

            // Pixel format mapping
            std::string fmt = pixel_format_str;
            std::transform(fmt.begin(), fmt.end(), fmt.begin(), [](unsigned char c)
                           { return static_cast<char>(std::tolower(c)); });
            if (fmt == "mjpeg" || fmt == "mjpg")
            {
                cap_cfg.pixel_format = V4L2_PIX_FMT_MJPEG;
            }
            else if (fmt == "yuyv" || fmt == "yuy2")
            {
                cap_cfg.pixel_format = V4L2_PIX_FMT_YUYV;
            }
            else
            {
                ROS_WARN("Unknown video/pixel_format='%s', falling back to mjpeg", pixel_format_str.c_str());
                cap_cfg.pixel_format = V4L2_PIX_FMT_MJPEG;
            }

            if (!capturer_->initialize(cap_cfg))
            {
                ROS_ERROR("Failed to initialize V4L2 capturer");
                return false;
            }

            // Use negotiated device format for downstream
            const uint32_t actual_w = capturer_->width();
            const uint32_t actual_h = capturer_->height();
            if (actual_w > 0 && actual_h > 0 && (actual_w != width || actual_h != height))
            {
                ROS_WARN(
                    "V4L2 negotiated format differs from requested: requested=%ux%u, actual=%ux%u. Using actual for converter/encoder.",
                    width, height, actual_w, actual_h);
                width = actual_w;
                height = actual_h;
            }

            // Store dimensions for eye image thread
            frame_width_ = width;
            frame_height_ = height;
            frame_rate_ = framerate;

            // Save converter config for deferred initialization in encode thread
            decoder_config_.width = width;
            decoder_config_.height = height;
            int decoder_output_plane_buffers_param;
            int decoder_capture_plane_buffers_param;
            int decoder_max_mjpeg_size_param;
            const uint32_t default_max_mjpeg_size = std::max<uint32_t>(4 * 1024 * 1024u, width * height * 2u);
            nh_.param<int>("video/decoder/output_plane_buffers", decoder_output_plane_buffers_param, 6);
            nh_.param<int>("video/decoder/capture_plane_buffers", decoder_capture_plane_buffers_param, static_cast<int>(conv_pool_size));
            nh_.param<int>("video/decoder/max_mjpeg_size_bytes", decoder_max_mjpeg_size_param, static_cast<int>(default_max_mjpeg_size));
            decoder_config_.output_plane_buffers = (decoder_output_plane_buffers_param > 0) ? static_cast<uint32_t>(decoder_output_plane_buffers_param) : 6u;
            decoder_config_.capture_plane_buffers = (decoder_capture_plane_buffers_param > 0) ? static_cast<uint32_t>(decoder_capture_plane_buffers_param) : std::max<uint32_t>(conv_pool_size, 4u);
            decoder_config_.max_mjpeg_size_bytes = (decoder_max_mjpeg_size_param > 0) ? static_cast<uint32_t>(decoder_max_mjpeg_size_param) : default_max_mjpeg_size;

            converter_config_.width = width;
            converter_config_.height = height;
            converter_config_.input_width = width;
            converter_config_.input_height = height;
            converter_config_.buffer_pool_size = conv_pool_size;

            // Undistort + stereo-rectify (optional, off by default). When
            // enabled we insert a VPI CUDA Remap stage between converter
            // and encoder; the same NV12 fd is also forwarded to the
            // eye_image_publisher, so both downstream consumers see the
            // rectified image.
            nh_.param<bool>("video/undistort/enabled", undistort_enabled_, false);
            if (undistort_enabled_)
            {
                undistorter_config_.width = width;
                undistorter_config_.height = height;
                undistorter_config_.buffer_pool_size = conv_pool_size;
                undistorter_config_.output_surface_layout = converter_config_.output_surface_layout;
                nh_.param<std::string>("video/undistort/calib_file",
                                        undistorter_config_.calibration_file,
                                        "");
                if (!undistorter_config_.calibration_file.empty() &&
                    undistorter_config_.calibration_file.front() != '/')
                {
                    const std::string pkg = ros::package::getPath("teleop_robot_bridge");
                    if (!pkg.empty())
                    {
                        undistorter_config_.calibration_file = pkg + "/" +
                            undistorter_config_.calibration_file;
                    }
                }
                nh_.param<bool>("video/undistort/require_calibration",
                                undistorter_config_.require_calibration,
                                true);
                std::string undistort_backend;
                nh_.param<std::string>("video/undistort/backend", undistort_backend, "cuda");
                std::transform(undistort_backend.begin(), undistort_backend.end(),
                               undistort_backend.begin(), [](unsigned char c) {
                                   return static_cast<char>(std::tolower(c));
                               });
                undistorter_config_.use_cuda = undistort_backend != "legacy" &&
                                               undistort_backend != "vpi";
            }

            auto parse_layout = [](const std::string &v) -> int32_t
            {
                std::string s = v;
                std::transform(s.begin(), s.end(), s.begin(), [](unsigned char c)
                               { return static_cast<char>(std::tolower(c)); });
                if (s == "block" || s == "blocklinear" || s == "block-linear")
                    return 1;
                return 0;
            };

            std::string conv_dec_layout, conv_out_layout, conv_layout_legacy;
            nh_.param<std::string>("video/converter/dec_layout", conv_dec_layout, "");
            nh_.param<std::string>("video/converter/out_layout", conv_out_layout, "");
            nh_.param<std::string>("video/converter/layout", conv_layout_legacy, "");

            if (!conv_dec_layout.empty())
            {
                converter_config_.decode_surface_layout = parse_layout(conv_dec_layout);
            }
            else if (!conv_layout_legacy.empty())
            {
                converter_config_.decode_surface_layout = parse_layout(conv_layout_legacy);
            }

            if (!conv_out_layout.empty())
            {
                converter_config_.output_surface_layout = parse_layout(conv_out_layout);
            }
            else
            {
                converter_config_.output_surface_layout = 0;
            }

            std::string conv_compute;
            nh_.param<std::string>("video/converter/compute", conv_compute, "default");
            {
                std::string s = conv_compute;
                std::transform(s.begin(), s.end(), s.begin(), [](unsigned char c)
                               { return static_cast<char>(std::tolower(c)); });
                if (s == "gpu")
                    converter_config_.transform_compute_mode = 1;
                else if (s == "vic")
                    converter_config_.transform_compute_mode = 2;
                else
                    converter_config_.transform_compute_mode = 0;
            }

            // Save encoder config for deferred initialization
            encoder_config_.width = width;
            encoder_config_.height = height;
            encoder_config_.framerate = framerate;

            std::string codec_str;
            nh_.param<std::string>("video/encoder/codec", codec_str, "h264");
            {
                std::string s = codec_str;
                std::transform(s.begin(), s.end(), s.begin(), [](unsigned char c)
                               { return static_cast<char>(std::tolower(c)); });
                if (s == "h265" || s == "hevc")
                {
                    encoder_config_.codec = trb::video::VideoEncoder::Codec::kH265;
                }
                else if (s == "h264" || s == "avc" || s.empty())
                {
                    encoder_config_.codec = trb::video::VideoEncoder::Codec::kH264;
                }
                else
                {
                    ROS_WARN("Unknown video/encoder/codec='%s' (use h264|h265). Falling back to h264.",
                             codec_str.c_str());
                    encoder_config_.codec = trb::video::VideoEncoder::Codec::kH264;
                }
            }

            // ---- User-facing knobs (yaml) ----
            int bitrate_param;
            nh_.param<int>("video/encoder/bitrate", bitrate_param, 40000000);
            encoder_config_.bitrate = static_cast<uint32_t>(std::max(1, bitrate_param));

            // I-frame interval (GOP length, in frames). Each I-frame is intra-coded.
            int i_frame_interval_param;
            nh_.param<int>("video/encoder/i_frame_interval", i_frame_interval_param, 30);
            encoder_config_.idr_interval = static_cast<uint32_t>(std::max(1, i_frame_interval_param));

            // IDR interval (in frames). IDR is a special I-frame that flushes the
            // reference list -> decoder can independently start here. Typically
            // == i_frame_interval (every I is IDR) for fastest packet-loss recovery,
            // or a multiple of it for longer GOPs.
            int idr_interval_param;
            nh_.param<int>("video/encoder/idr_interval", idr_interval_param, 30);
            encoder_config_.idr_interval_gops = static_cast<uint32_t>(std::max(1, idr_interval_param));

            int force_idr_every_n_param;
            nh_.param<int>("video/encoder/force_idr_every_n", force_idr_every_n_param, 0);
            encoder_config_.force_idr_every_n = static_cast<uint32_t>(std::max(0, force_idr_every_n_param));

            bool intra_refresh_enabled;
            nh_.param<bool>("video/encoder/intra_refresh/enabled", intra_refresh_enabled, false);
            int intra_refresh_interval_slices_param;
            nh_.param<int>("video/encoder/intra_refresh/interval_slices", intra_refresh_interval_slices_param, 0);
            encoder_config_.slice_intra_refresh_interval_slices = intra_refresh_enabled
                ? static_cast<uint32_t>(std::max(1, intra_refresh_interval_slices_param))
                : 0u;

            bool low_freq_idr_enabled;
            nh_.param<bool>("video/encoder/low_freq_idr/enabled", low_freq_idr_enabled, false);
            int low_freq_idr_interval_sec;
            nh_.param<int>("video/encoder/low_freq_idr/interval_sec", low_freq_idr_interval_sec, 2);
            if (low_freq_idr_enabled && low_freq_idr_interval_sec > 0)
            {
                const uint64_t frames_u64 = static_cast<uint64_t>(low_freq_idr_interval_sec) * static_cast<uint64_t>(framerate);
                const uint32_t frames = static_cast<uint32_t>(std::max<uint64_t>(1, frames_u64));
                encoder_config_.idr_interval = frames;
                encoder_config_.idr_interval_gops = frames;
            }

            bool max_perf_mode;
            nh_.param<bool>("video/encoder/max_perf_mode", max_perf_mode, true);
            encoder_config_.max_perf_mode = max_perf_mode;

            // ---- Encoder low-level knobs (all from yaml for manual tuning) ----
            std::string rc_str;
            nh_.param<std::string>("video/encoder/rate_control", rc_str, "vbr");
            {
                std::string s = rc_str;
                std::transform(s.begin(), s.end(), s.begin(), [](unsigned char c)
                               { return static_cast<char>(std::tolower(c)); });
                if (s == "cbr")
                    encoder_config_.rate_control_mode = static_cast<int32_t>(V4L2_MPEG_VIDEO_BITRATE_MODE_CBR);
                else
                    encoder_config_.rate_control_mode = static_cast<int32_t>(V4L2_MPEG_VIDEO_BITRATE_MODE_VBR);
            }

            int peak_bitrate_param;
            nh_.param<int>("video/encoder/peak_bitrate", peak_bitrate_param, 100000000);
            encoder_config_.peak_bitrate = static_cast<uint32_t>(std::max(1, peak_bitrate_param));

            int vbv_size_param;
            nh_.param<int>("video/encoder/virtual_buffer_size", vbv_size_param, 50000000);
            encoder_config_.virtual_buffer_size = static_cast<uint32_t>(std::max(1, vbv_size_param));

            std::string preset_str;
            nh_.param<std::string>("video/encoder/hw_preset", preset_str, "fast");
            {
                std::string s = preset_str;
                std::transform(s.begin(), s.end(), s.begin(), [](unsigned char c)
                               { return static_cast<char>(std::tolower(c)); });
                if (s == "ultrafast")
                    encoder_config_.hw_preset_type = static_cast<int32_t>(V4L2_ENC_HW_PRESET_ULTRAFAST);
                else if (s == "medium")
                    encoder_config_.hw_preset_type = static_cast<int32_t>(V4L2_ENC_HW_PRESET_MEDIUM);
                else if (s == "slow")
                    encoder_config_.hw_preset_type = static_cast<int32_t>(V4L2_ENC_HW_PRESET_SLOW);
                else
                    encoder_config_.hw_preset_type = static_cast<int32_t>(V4L2_ENC_HW_PRESET_FAST);
            }

            int num_ref_param;
            nh_.param<int>("video/encoder/num_reference_frames", num_ref_param, 1);
            encoder_config_.num_reference_frames = static_cast<uint32_t>(std::max(0, num_ref_param));

            int num_b_param;
            nh_.param<int>("video/encoder/num_b_frames", num_b_param, 0);
            encoder_config_.num_b_frames = static_cast<uint32_t>(std::max(0, num_b_param));

            bool insert_sps_pps_at_idr;
            nh_.param<bool>("video/encoder/insert_sps_pps_at_idr", insert_sps_pps_at_idr, true);
            encoder_config_.insert_sps_pps_at_idr = insert_sps_pps_at_idr ? 1 : 0;

            bool slice_level_encode;
            nh_.param<bool>("video/encoder/slice_level_encode", slice_level_encode, false);
            encoder_config_.slice_level_encode = slice_level_encode;

            int num_slices_param;
            nh_.param<int>("video/encoder/num_slices", num_slices_param, 1);
            encoder_config_.num_slices = static_cast<uint32_t>(std::max(1, num_slices_param));

            int poc_type_param;
            nh_.param<int>("video/encoder/poc_type", poc_type_param, 2);
            encoder_config_.poc_type = static_cast<uint32_t>(std::max(0, poc_type_param));

            bool insert_aud;
            nh_.param<bool>("video/encoder/insert_aud", insert_aud, false);
            encoder_config_.insert_aud = insert_aud;

            int framerate_override_param;
            nh_.param<int>("video/encoder/framerate_override", framerate_override_param, 0);
            encoder_config_.encoder_framerate_override = static_cast<uint32_t>(std::max(0, framerate_override_param));

            int output_plane_buffers_param;
            nh_.param<int>("video/encoder/output_plane_buffers", output_plane_buffers_param, 2);
            encoder_config_.output_plane_buffers = static_cast<uint32_t>(std::max(1, output_plane_buffers_param));

            int capture_plane_buffers_param;
            nh_.param<int>("video/encoder/capture_plane_buffers", capture_plane_buffers_param, 2);
            encoder_config_.capture_plane_buffers = static_cast<uint32_t>(std::max(1, capture_plane_buffers_param));

            int qp_i_min, qp_i_max, qp_p_min, qp_p_max;
            nh_.param<int>("video/encoder/qp_range/i_min", qp_i_min, 0);
            nh_.param<int>("video/encoder/qp_range/i_max", qp_i_max, 0);
            nh_.param<int>("video/encoder/qp_range/p_min", qp_p_min, 0);
            nh_.param<int>("video/encoder/qp_range/p_max", qp_p_max, 0);
            encoder_config_.qp_range_i_min = static_cast<uint32_t>(std::max(0, qp_i_min));
            encoder_config_.qp_range_i_max = static_cast<uint32_t>(std::max(0, qp_i_max));
            encoder_config_.qp_range_p_min = static_cast<uint32_t>(std::max(0, qp_p_min));
            encoder_config_.qp_range_p_max = static_cast<uint32_t>(std::max(0, qp_p_max));

            nh_.param<int>("video/encoder/warmup_ms", warmup_duration_ms_, 1500);
            if (warmup_duration_ms_ < 0) warmup_duration_ms_ = 0;
            const int64_t now_us = std::chrono::duration_cast<std::chrono::microseconds>(
                                       std::chrono::steady_clock::now().time_since_epoch())
                                       .count();
            warmup_deadline_us_.store(now_us + static_cast<int64_t>(warmup_duration_ms_) * 1000,
                                      std::memory_order_relaxed);
            warmup_active_.store(warmup_duration_ms_ > 0, std::memory_order_relaxed);
            warmup_dropped_.store(0, std::memory_order_relaxed);

            // Check if eye image is enabled
            nh_.param<bool>("video/eye_image/enabled", eye_image_enabled_, false);
            if (eye_image_enabled_)
            {
                nh_.param<int>("video/eye_image/output_width", eye_image_output_width_, 640);
                nh_.param<int>("video/eye_image/output_height", eye_image_output_height_, 360);
                nh_.param<int>("video/eye_image/max_fps", eye_image_max_fps_, 30);
                if (eye_image_max_fps_ <= 0)
                {
                    ROS_WARN("video/eye_image/max_fps=%d is invalid for the 30 Hz eye limit; using 30",
                             eye_image_max_fps_);
                    eye_image_max_fps_ = 30;
                }
                if (eye_image_max_fps_ > 30)
                {
                    ROS_WARN("video/eye_image/max_fps=%d exceeds the 30 Hz eye limit; clamping to 30",
                             eye_image_max_fps_);
                    eye_image_max_fps_ = 30;
                }
                eye_image_min_interval_us_ = eye_image_max_fps_ > 0
                    ? static_cast<int64_t>(1000000 / std::max(1, eye_image_max_fps_))
                    : 0;
                nh_.param<std::string>("video/eye_image/left_topic", eye_image_left_topic_, "data/left_eye_image");
                nh_.param<std::string>("video/eye_image/right_topic", eye_image_right_topic_, "data/right_eye_image");
                nh_.param<int>("video/eye_image/queue_depth", eye_image_queue_depth_, 3);
                if (eye_image_queue_depth_ < 1)
                    eye_image_queue_depth_ = 1;
                nh_.param<std::string>("video/eye_image/backend", eye_image_backend_, "cuda");
                std::transform(eye_image_backend_.begin(), eye_image_backend_.end(),
                               eye_image_backend_.begin(), [](unsigned char c) {
                                   return static_cast<char>(std::tolower(c));
                               });
                std::string eye_image_drop_policy;
                nh_.param<std::string>("video/eye_image/drop_policy",
                                       eye_image_drop_policy, "latest");
                std::transform(eye_image_drop_policy.begin(), eye_image_drop_policy.end(),
                               eye_image_drop_policy.begin(), [](unsigned char c) {
                                   return static_cast<char>(std::tolower(c));
                               });
                if (eye_image_drop_policy != "latest")
                {
                    ROS_WARN("video/eye_image/drop_policy='%s' is not supported; using latest",
                             eye_image_drop_policy.c_str());
                }
                std::string eye_image_color_space;
                nh_.param<std::string>("video/eye_image/color_space",
                                       eye_image_color_space, "bt601_full");
                std::transform(eye_image_color_space.begin(), eye_image_color_space.end(),
                               eye_image_color_space.begin(), [](unsigned char c) {
                                   return static_cast<char>(std::tolower(c));
                               });
                if (eye_image_color_space != "bt601_full")
                {
                    ROS_WARN("video/eye_image/color_space='%s' is not supported by the CUDA "
                             "kernel; using bt601_full", eye_image_color_space.c_str());
                }
            }

            const bool cuda_eye_backend = eye_image_enabled_ &&
                eye_image_backend_ != "legacy" && eye_image_backend_ != "vic";
            if ((undistort_enabled_ && undistorter_config_.use_cuda) || cuda_eye_backend)
            {
                // CUDA EGL interop requires pitch-linear NV12 on both the
                // converter input to the rectifier and the shared eye input.
                if (converter_config_.output_surface_layout != 0)
                {
                    ROS_WARN("CUDA video backend requires pitch-linear converter output; "
                             "overriding video/converter/out_layout");
                    converter_config_.output_surface_layout = 0;
                }
            }

            running_.store(true);

            // Start encode pipeline (V4L2 DQ + MJPEG decode + convert + encode) and optional eye-image thread.
            encode_thread_ = std::thread(&Impl::encodeThreadMain, this);
            
            if (eye_image_enabled_)
            {
                eye_image_thread_ = std::thread(&Impl::eyeImageThreadMain, this);
                ROS_INFO("VideoStreamManager: Started with CUDA/latest-frame eye_image pipeline (max_fps=%d)", eye_image_max_fps_);
            }
            else
            {
                ROS_INFO("VideoStreamManager: Started (eye_image disabled)");
            }

            return true;
        }

        void stop()
        {
            const bool was_running = running_.exchange(false);
            if (!was_running)
                return;

            // Wake up eye image thread
            {
                std::lock_guard<std::mutex> lk(nv12_queue_mutex_);
                nv12_queue_cv_.notify_all();
            }

            if (capture_thread_.joinable())
                capture_thread_.join();
            if (encode_thread_.joinable())
                encode_thread_.join();
            if (encoder_dq_thread_.joinable())
                encoder_dq_thread_.join();
            if (eye_image_thread_.joinable())
                eye_image_thread_.join();

            {
                std::lock_guard<std::mutex> lk(nv12_queue_mutex_);
                while (!nv12_queue_.empty())
                    nv12_queue_.pop();
            }

            if (capturer_)
            {
                capturer_->stop();
            }

            {
                // Clear encoder_ under callback_mutex_ so a concurrent caller
                // of setTargetBitrate / setSpsPpsCallback cannot observe the
                // pointer mid-destruction.
                std::lock_guard<std::mutex> lk(callback_mutex_);
                encoder_.reset();
            }

            {
                std::lock_guard<std::mutex> lk(encoder_pending_mutex_);
                encoder_pending_frames_by_fd_.clear();
            }

            decoder_.reset();
            undistorter_.reset();
            converter_.reset();
            capturer_.reset();
            eye_image_publisher_.reset();
            eye_image_publisher_ready_.store(false, std::memory_order_relaxed);
            eye_image_has_subscribers_.store(false, std::memory_order_relaxed);

            {
                std::lock_guard<std::mutex> lk(enc_timing_mutex_);
                enc_start_steady_us_by_ts_.clear();
            }
        }

        bool isRunning() const
        {
            return running_.load();
        }

        VideoStreamManager::StatsSnapshot consumeStats()
        {
            VideoStreamManager::StatsSnapshot snapshot;
            snapshot.capture_frames = stats_capture_frames_.exchange(0, std::memory_order_relaxed);
            snapshot.decode_frames = stats_decode_frames_.exchange(0, std::memory_order_relaxed);
            snapshot.encode_frames = stats_encode_frames_.exchange(0, std::memory_order_relaxed);
            snapshot.decoder_drops = stats_decoder_drops_.exchange(0, std::memory_order_relaxed);
            snapshot.encoder_submit_failures = stats_encoder_submit_failures_.exchange(0, std::memory_order_relaxed);
            snapshot.encode_us_total = stats_encode_us_total_.exchange(0, std::memory_order_relaxed);
            snapshot.eye_image_enqueued = eye_image_enqueued_.exchange(0, std::memory_order_relaxed);
            snapshot.eye_image_published = eye_image_published_.exchange(0, std::memory_order_relaxed);
            snapshot.eye_image_dropped = eye_image_drop_queue_.exchange(0, std::memory_order_relaxed);
            snapshot.eye_image_process_us_total = stats_eye_image_process_us_total_.exchange(0, std::memory_order_relaxed);
            snapshot.eye_image_frame_age_us_max = stats_eye_image_frame_age_us_max_.exchange(0, std::memory_order_relaxed);
            {
                std::lock_guard<std::mutex> lk(nv12_queue_mutex_);
                snapshot.eye_image_mailbox_depth = nv12_queue_.size();
            }

            if (converter_)
            {
                auto converter_stats = converter_->consumeStats();
                snapshot.convert_frames = converter_stats.processed_frames;
                snapshot.converter_pool_drops = converter_stats.pool_drops;
                snapshot.converter_failures = converter_stats.failed_frames;
                snapshot.decode_us_total = static_cast<uint64_t>(std::max<int64_t>(0, converter_stats.decode_us_total));
                snapshot.transform_us_total = static_cast<uint64_t>(std::max<int64_t>(0, converter_stats.transform_us_total));
            }

            if (undistorter_)
            {
                auto und_stats = undistorter_->consumeStats();
                snapshot.undistort_frames = und_stats.processed_frames;
                snapshot.undistort_pool_drops = und_stats.pool_drops;
                snapshot.undistort_failures = und_stats.failed_frames;
                snapshot.undistort_us_total = static_cast<uint64_t>(std::max<int64_t>(0, und_stats.remap_us_total));
                snapshot.undistort_cuda_submitted = und_stats.cuda_submitted;
                snapshot.undistort_cuda_completed = und_stats.cuda_completed;
                snapshot.undistort_cuda_failures = und_stats.cuda_failures;
                snapshot.undistort_cuda_kernel_us_total = und_stats.cuda_kernel_us_total;
            }

            if (eye_image_publisher_)
            {
                const auto eye_stats = eye_image_publisher_->consumeStats();
                snapshot.eye_image_ros_publish_count = eye_stats.ros_publish_count;
                snapshot.eye_image_ros_publish_us_total = eye_stats.ros_publish_us_total;
#ifdef TRB_CUDA_AVAILABLE
                snapshot.eye_image_cuda_submitted = eye_stats.cuda.submitted;
                snapshot.eye_image_cuda_completed = eye_stats.cuda.completed;
                snapshot.eye_image_cuda_failures = eye_stats.cuda.failures;
                snapshot.eye_image_cuda_kernel_us_total = eye_stats.cuda.kernel_us_total;
                snapshot.eye_image_cuda_device_to_host_us_total = eye_stats.cuda.device_to_host_us_total;
#endif
            }

            return snapshot;
        }

    private:
        // ==================== Encode Thread ====================
        // Single unified thread:
        //   1. Dequeue V4L2 MJPEG mmap buffer.
        //   2. Hand pointer to NvVideoDecoder (which memcpys into its output plane).
        //   3. Requeue V4L2 buffer immediately.
        // NvVideoDecoder's own DQ threads drive the rest of the pipeline
        // (decoder capture plane → VIC convert → NvVideoEncoder input) via
        // zero-copy DMA-BUF fds.
        void encodeThreadMain()
        {
            // Initialize decoder, converter and encoder in this thread.
            decoder_ = std::make_shared<trb::video::VideoDecoder>();
            if (!decoder_->initialize(decoder_config_))
            {
                ROS_WARN("[ENCODE] VideoDecoder initialize failed");
                decoder_.reset();
            }
            else
            {
                converter_ = std::make_shared<trb::video::VideoConverter>();
                if (!converter_->initialize(converter_config_))
                {
                    ROS_WARN("[ENCODE] VideoConverter initialize failed");
                    converter_.reset();
                }
                else if (undistort_enabled_)
                {
                    auto und = std::make_shared<trb::video::VideoUndistorter>();
                    if (!und->initialize(undistorter_config_))
                    {
                        ROS_WARN("[ENCODE] VideoUndistorter initialize failed; "
                                 "continuing without undistortion");
                    }
                    else
                    {
                        undistorter_ = und;
                    }
                }
                if (converter_)
                {
                    std::shared_ptr<trb::video::VideoEncoder> new_encoder =
                        std::make_shared<trb::video::VideoEncoder>();
                    if (!new_encoder->initialize(encoder_config_))
                    {
                        ROS_WARN("[ENCODE] VideoEncoder initialize failed");
                        new_encoder.reset();
                    }
                    else
                    {
                        new_encoder->setInputDoneCallback(std::bind(&Impl::onEncoderInputDone, this, std::placeholders::_1));
                        SpsPpsCallback pending_sps_pps;
                        {
                            // Publish the encoder under callback_mutex_ so that external
                            // setTargetBitrate / setSpsPpsCallback never see a partially
                            // constructed or already-destroyed instance.
                            std::lock_guard<std::mutex> lk(callback_mutex_);
                            encoder_ = new_encoder;
                            pending_sps_pps = sps_pps_callback_;
                        }
                        if (pending_sps_pps)
                        {
                            new_encoder->setSpsPpsCallback(pending_sps_pps);
                        }
                    }
                }
            }

            // Once the pipeline is up, spawn the capture thread that pumps
            // MJPEG frames from V4L2 into the decoder, and the dedicated
            // encoder DQ thread that long-blocks on the encoder capture plane
            // so encoded AUs are forwarded the moment NVENC signals readiness
            // (no dependency on this thread's loop cadence).
            if (decoder_ && converter_ && encoder_)
            {
                capture_thread_ = std::thread(&Impl::captureThreadMain, this);
                encoder_dq_thread_ = std::thread(&Impl::encoderDqThreadMain, this);
            }

            // Encode-thread main loop: drain decoder capture plane, run VIC
            // transform, submit to encoder, and drain encoder capture plane.
            while (running_.load())
            {
                // Reap completed encoder input buffers (returns fds to the
                // converter output pool). Encoded AU dequeue runs on a
                // dedicated thread (encoder_dq_thread_) for low-jitter delivery.
                if (encoder_)
                {
                    encoder_->drainInputDone();
                }

                if (!decoder_ || !converter_ || !encoder_)
                {
                    std::this_thread::sleep_for(std::chrono::milliseconds(5));
                    continue;
                }

                trb::video::VideoDecoder::DecodedFrame dec_frame;
                // Block up to 10ms for a decoded YUV frame. Short timeout keeps
                // encoder drain responsive even during decoder idle periods.
                if (!decoder_->dequeueSync(dec_frame, 10))
                {
                    continue;
                }

                stats_decode_frames_.fetch_add(1, std::memory_order_relaxed);

                int nv12_fd = -1;
                const bool ok = converter_->transformSync(dec_frame.yuv_dmabuf_fd,
                                                          dec_frame.capture_timestamp_us,
                                                          dec_frame.decode_us,
                                                          nv12_fd);
                // VIC has consumed the YUV buffer; return it to the decoder.
                decoder_->requeueCapture(dec_frame);

                if (!ok)
                    continue;

                int downstream_fd = nv12_fd;
                if (undistorter_)
                {
                    int undist_fd = -1;
                    const bool und_ok = undistorter_->process(nv12_fd, undist_fd);
                    // Converter output is no longer needed once undistorter
                    // has finished (synchronous CUDA Remap copies the data).
                    converter_->releaseFd(nv12_fd);
                    if (!und_ok)
                    {
                        continue;
                    }
                    downstream_fd = undist_fd;
                }

                (void)onConverterOutput(downstream_fd, dec_frame.capture_timestamp_us);
            }
        }

        // Capture-thread main loop: pull one MJPEG frame from V4L2, submit to
        // decoder, requeue V4L2. Keeps decoder input continuously fed without
        // blocking the encode pipeline on camera frame arrival.
        void captureThreadMain()
        {
            if (!capturer_ || !capturer_->start())
            {
                ROS_ERROR("Failed to start V4L2 capturer");
                return;
            }
            ROS_INFO("V4L2 capturer started (warmup_drop_frames=%d)",
                     v4l2_warmup_drop_frames_);

            // Drop the first N camera frames before feeding the decoder. This
            // skips V4L2 STREAMON jitter and the camera's AE/AWB convergence
            // window. NOTE: this does NOT cover NvVideoDecoder capture-plane
            // init jitter -- that requires backpressure on the decoder side.
            const auto warmup_start = std::chrono::steady_clock::now();
            int warmup_dropped_frames = 0;
            bool warmup_done = (v4l2_warmup_drop_frames_ == 0);

            while (running_.load())
            {
                trb::video::VideoV4L2Capturer::Frame frame;
                if (!capturer_ || !capturer_->dequeue(frame, 100))
                {
                    continue;
                }

                const auto now_tp = std::chrono::steady_clock::now();

                if (!warmup_done)
                {
                    (void)capturer_->requeue(frame.v4l2_buf);
                    if (++warmup_dropped_frames >= v4l2_warmup_drop_frames_)
                    {
                        const double warmup_elapsed_ms = static_cast<double>(
                            std::chrono::duration_cast<std::chrono::microseconds>(
                                now_tp - warmup_start).count()) / 1000.0;
                        ROS_INFO("V4L2 capturer warmup complete: dropped=%d elapsed=%.1fms",
                                 warmup_dropped_frames, warmup_elapsed_ms);
                        warmup_done = true;
                    }
                    continue;
                }

                const uint64_t capture_steady_us = static_cast<uint64_t>(
                    std::chrono::duration_cast<std::chrono::microseconds>(
                        now_tp.time_since_epoch()).count());

                stats_capture_frames_.fetch_add(1, std::memory_order_relaxed);

                const uint64_t frame_id = encode_frame_in_count_.fetch_add(1) + 1;

                if (frame.data != nullptr && frame.size > 0 && decoder_)
                {
                    const bool queued = decoder_->submit(frame.data, frame.size,
                                                         frame_id, capture_steady_us,
                                                         capture_steady_us);
                    if (!queued)
                    {
                        stats_decoder_drops_.fetch_add(1, std::memory_order_relaxed);
                    }
                }

                (void)capturer_->requeue(frame.v4l2_buf);
            }
        }

        // Dedicated encoder capture-plane DQ thread.
        // Long-blocks on encoder_->dequeueOne() so encoded access units are
        // delivered to onEncoderPacket() the moment NVENC signals readiness,
        // independent of the encode_thread_ main loop's cadence.
        void encoderDqThreadMain()
        {
            ROS_INFO("[ENCODE] encoder DQ thread started");
            while (running_.load(std::memory_order_acquire))
            {
                std::shared_ptr<trb::video::VideoEncoder> enc;
                {
                    std::lock_guard<std::mutex> lk(callback_mutex_);
                    enc = encoder_;
                }
                if (!enc)
                {
                    std::this_thread::sleep_for(std::chrono::milliseconds(5));
                    continue;
                }

                trb::video::VideoEncoder::EncodedPacket pkt;
                // O_NONBLOCK + num_retries=ms-timeout (see Jetson MM quirks).
                // 100ms keeps shutdown latency bounded while parking the
                // thread on the V4L2 driver wakeup the rest of the time.
                if (!enc->dequeueOne(pkt, 100))
                {
                    continue;
                }

                onEncoderPacket(pkt.data, pkt.size, pkt.timestamp_us, pkt.keyframe);
                enc->releaseCapture(pkt);
            }
            ROS_INFO("[ENCODE] encoder DQ thread exiting");
        }

        // ==================== Eye Image Thread ====================
        // Responsibility: receive the latest rectified NV12 fd and publish
        // CUDA-produced ROS images at the configured maximum rate.
        void eyeImageThreadMain()
        {
            ROS_INFO("[EYE_IMAGE] Thread started (CUDA latest-frame mailbox)");

            eye_image_publisher_ = std::make_shared<trb::video::EyeImagePublisher>();
            trb::video::EyeImagePublisher::Config eye_cfg;
            eye_cfg.output_width = static_cast<uint32_t>(eye_image_output_width_);
            eye_cfg.output_height = static_cast<uint32_t>(eye_image_output_height_);
            eye_cfg.left_topic = eye_image_left_topic_;
            eye_cfg.right_topic = eye_image_right_topic_;
            eye_cfg.enabled = true;
            eye_cfg.cuda_slot_count = static_cast<uint32_t>(eye_image_queue_depth_);
            eye_cfg.use_cuda = eye_image_backend_ != "legacy" && eye_image_backend_ != "vic";

            if (!eye_image_publisher_->initialize(nh_, frame_width_, frame_height_, eye_cfg))
            {
                ROS_WARN("[EYE_IMAGE] EyeImagePublisher initialize failed");
                eye_image_publisher_.reset();
                return;
            }

            // Only the legacy fallback needs the shared VIC mutex. The CUDA
            // path never enters NvBufSurfTransform or CPU surface mapping.
            if (!eye_image_publisher_->usesCuda())
            {
                NvBufSurfTransformConfigParams transform_config;
                std::memset(&transform_config, 0, sizeof(transform_config));
                transform_config.compute_mode = NvBufSurfTransformCompute_VIC;
                transform_config.gpu_id = 0;
                transform_config.cuda_stream = 0;
                if (NvBufSurfTransformSetSessionParams(&transform_config) != 0)
                {
                    ROS_ERROR("[EYE_IMAGE] NvBufSurfTransformSetSessionParams failed");
                    eye_image_publisher_->stop();
                    eye_image_publisher_.reset();
                    return;
                }
                eye_image_publisher_->setVicMutex(&trb::video::getNvBufMutex());
            }
            eye_image_publisher_ready_.store(true, std::memory_order_release);
            updateEyeImageSubscriberState();

            ROS_INFO("[EYE_IMAGE] Publisher initialized: %ux%u -> %ux%u backend=%s max_fps=%d",
                     frame_width_, frame_height_, eye_cfg.output_width, eye_cfg.output_height,
                     eye_image_publisher_->usesCuda() ? "cuda" : "legacy",
                     eye_image_max_fps_);

            uint64_t frames_processed = 0;
            uint64_t frames_dropped = 0;
            int64_t last_process_us = 0;

            while (running_.load())
            {
                NV12FrameData frame_data;
                updateEyeImageSubscriberState();

                {
                    std::unique_lock<std::mutex> lk(nv12_queue_mutex_);
                    while (running_.load())
                    {
                        if (nv12_queue_.empty())
                        {
                            nv12_queue_cv_.wait_for(lk, std::chrono::milliseconds(100), [this] {
                                return !running_.load() || !nv12_queue_.empty();
                            });
                            // Polling also refreshes subscriber state when no
                            // input frame is available to wake this worker.
                            updateEyeImageSubscriberState();
                            continue;
                        }

                        if (eye_image_min_interval_us_ > 0 && last_process_us > 0)
                        {
                            const int64_t now_us = std::chrono::duration_cast<std::chrono::microseconds>(
                                std::chrono::steady_clock::now().time_since_epoch()).count();
                            const int64_t due_us = last_process_us + eye_image_min_interval_us_;
                            if (now_us < due_us)
                            {
                                nv12_queue_cv_.wait_for(lk, std::chrono::microseconds(due_us - now_us));
                                continue;
                            }
                        }

                        while (nv12_queue_.size() > 1)
                        {
                            frames_dropped++;
                            eye_image_drop_queue_.fetch_add(1, std::memory_order_relaxed);
                            nv12_queue_.pop();
                        }
                        frame_data = nv12_queue_.front();
                        nv12_queue_.pop();
                        break;
                    }
                }

                if (!running_.load())
                    break;
                if (!frame_data.frame || !eye_image_publisher_->isEnabled() ||
                    !eye_image_publisher_->hasSubscribers())
                    continue;

                const auto process_start = std::chrono::steady_clock::now();
                const int64_t process_start_us = std::chrono::duration_cast<std::chrono::microseconds>(
                    process_start.time_since_epoch()).count();
                if (frame_data.enqueued_steady_us > 0 && process_start_us >= frame_data.enqueued_steady_us)
                {
                    const uint64_t age_us = static_cast<uint64_t>(process_start_us - frame_data.enqueued_steady_us);
                    uint64_t current = stats_eye_image_frame_age_us_max_.load(std::memory_order_relaxed);
                    while (age_us > current &&
                           !stats_eye_image_frame_age_us_max_.compare_exchange_weak(
                               current, age_us, std::memory_order_relaxed))
                    {
                    }
                }
                const bool published = eye_image_publisher_->processFrameFd(
                    frame_data.frame->fd, frame_data.frame->timestamp_us);
                const auto process_end = std::chrono::steady_clock::now();
                eye_image_process_us_total_.fetch_add(
                    static_cast<uint64_t>(std::chrono::duration_cast<std::chrono::microseconds>(
                        process_end - process_start).count()), std::memory_order_relaxed);
                stats_eye_image_process_us_total_.fetch_add(
                    static_cast<uint64_t>(std::chrono::duration_cast<std::chrono::microseconds>(
                        process_end - process_start).count()), std::memory_order_relaxed);
                frames_processed++;
                if (published)
                    eye_image_published_.fetch_add(1, std::memory_order_relaxed);
                last_process_us = std::chrono::duration_cast<std::chrono::microseconds>(
                    process_end.time_since_epoch()).count();

                if (frames_processed <= 5 || (frames_processed % 300 == 0))
                {
                    const uint64_t enqueued = eye_image_enqueued_.load(std::memory_order_relaxed);
                    const uint64_t no_sub = eye_image_skip_no_sub_.load(std::memory_order_relaxed);
                    const double avg_ms = frames_processed > 0
                        ? static_cast<double>(eye_image_process_us_total_.load(std::memory_order_relaxed)) /
                              static_cast<double>(frames_processed) / 1000.0
                        : 0.0;
                    ROS_INFO("[EYE_IMAGE] Processed %" PRIu64 " frames, dropped %" PRIu64
                             " enq=%" PRIu64 " skip_no_sub=%" PRIu64
                             " avg_process=%.2fms",
                             frames_processed, frames_dropped, enqueued, no_sub, avg_ms);
                }
            }

            eye_image_publisher_ready_.store(false, std::memory_order_release);
            eye_image_has_subscribers_.store(false, std::memory_order_release);
            eye_image_publisher_->stop();
            ROS_INFO("[EYE_IMAGE] Thread stopped (processed=%" PRIu64 ", dropped=%" PRIu64 ")",
                     frames_processed, frames_dropped);
        }

        void onEncoderInputDone(int dmabuf_fd)
        {
            std::shared_ptr<SharedNv12Frame> released;
            {
                std::lock_guard<std::mutex> lk(encoder_pending_mutex_);
                auto it = encoder_pending_frames_by_fd_.find(dmabuf_fd);
                if (it != encoder_pending_frames_by_fd_.end())
                {
                    released = std::move(it->second);
                    encoder_pending_frames_by_fd_.erase(it);
                }
            }

            if (!released)
            {
                ROS_WARN_THROTTLE(5.0, "VideoStreamManager: input done for unknown fd=%d; releasing directly", dmabuf_fd);
                if (undistorter_)
                {
                    undistorter_->releaseFd(dmabuf_fd);
                }
                else if (converter_)
                {
                    converter_->releaseFd(dmabuf_fd);
                }
            }
        }

        void onEncoderPacket(const uint8_t *data, size_t size, uint64_t timestamp_us, bool keyframe)
        {
            const uint64_t count = enc_frame_count_.fetch_add(1) + 1;

            const auto now_tp = std::chrono::steady_clock::now();
            const int64_t now_us = std::chrono::duration_cast<std::chrono::microseconds>(now_tp.time_since_epoch()).count();

            int64_t cost_us = -1;
            {
                std::lock_guard<std::mutex> lk(enc_timing_mutex_);
                auto it = enc_start_steady_us_by_ts_.find(timestamp_us);
                if (it != enc_start_steady_us_by_ts_.end())
                {
                    cost_us = now_us - it->second;
                    enc_start_steady_us_by_ts_.erase(it);
                }
            }

            // NVENC warmup: drop encoded output + backlogged stale frames until
            // the deadline passes. This prevents polluting FEC/send queues with
            // 200-400 ms old packets during cold start.
            if (warmup_active_.load(std::memory_order_relaxed))
            {
                const int64_t deadline_us = warmup_deadline_us_.load(std::memory_order_relaxed);
                if (now_us < deadline_us)
                {
                    warmup_dropped_.fetch_add(1, std::memory_order_relaxed);
                    return;
                }

                // Crossed the deadline: flip off, reset stats so the first
                // post-warmup [VIDEO] log reflects steady state only.
                bool expected = true;
                if (warmup_active_.compare_exchange_strong(expected, false,
                                                           std::memory_order_acq_rel))
                {
                    stats_capture_frames_.store(0, std::memory_order_relaxed);
                    stats_decode_frames_.store(0, std::memory_order_relaxed);
                    stats_encode_frames_.store(0, std::memory_order_relaxed);
                    stats_decoder_drops_.store(0, std::memory_order_relaxed);
                    stats_encoder_submit_failures_.store(0, std::memory_order_relaxed);
                    stats_encode_us_total_.store(0, std::memory_order_relaxed);
                    if (converter_)
                    {
                        (void)converter_->consumeStats();
                    }
                    if (undistorter_)
                    {
                        (void)undistorter_->consumeStats();
                    }
                    ROS_INFO("VideoStreamManager: NVENC warmup complete, dropped %lu stale packets (warmup_ms=%d)",
                             static_cast<unsigned long>(warmup_dropped_.load(std::memory_order_relaxed)),
                             warmup_duration_ms_);
                }
                // fall through to deliver this (first post-warmup) packet
            }

            // Multi-slice: only the first AU of each frame finds its timestamp
            // in enc_start_steady_us_by_ts_ (subsequent slices share the same
            // ts and the entry was erased above). cost_us<0 therefore marks a
            // non-first slice; skip frame counting / cost accumulation so the
            // [VIDEO] fps and enc_ms reflect frames, not AUs.
            if (cost_us >= 0)
            {
                stats_encode_frames_.fetch_add(1, std::memory_order_relaxed);
                stats_encode_us_total_.fetch_add(static_cast<uint64_t>(cost_us), std::memory_order_relaxed);
            }

            // Copy the callback under the lock then release before invoking.
            // The callback (sendH264Frame) does a ~200KB memcpy at 50Mbps/30fps;
            // holding callback_mutex_ across it serializes encoder_dq_thread_
            // (which also takes callback_mutex_ just to read encoder_) and
            // causes capture-plane backlog -> apparent enc latency inflates.
            EncodedFrameCallback cb_local;
            {
                std::lock_guard<std::mutex> lk(callback_mutex_);
                cb_local = encoded_frame_callback_;
            }
            if (cb_local)
            {
                cb_local(data, size, timestamp_us, keyframe);
            }
        }

        bool onConverterOutput(int dmabuf_fd, uint64_t timestamp_us)
        {
            auto shared_frame = makeSharedNv12Frame(dmabuf_fd, timestamp_us);

            if (!encoder_)
                return false;

            const auto enc_t0 = std::chrono::steady_clock::now();
            const int64_t enc_t0_us = std::chrono::duration_cast<std::chrono::microseconds>(enc_t0.time_since_epoch()).count();
            {
                std::lock_guard<std::mutex> lk(enc_timing_mutex_);
                enc_start_steady_us_by_ts_[timestamp_us] = enc_t0_us;
            }
            {
                std::lock_guard<std::mutex> lk(encoder_pending_mutex_);
                encoder_pending_frames_by_fd_[dmabuf_fd] = shared_frame;
            }

            const bool result = encoder_->submit(dmabuf_fd, timestamp_us);
            if (!result)
            {
                {
                    std::lock_guard<std::mutex> lk(enc_timing_mutex_);
                    enc_start_steady_us_by_ts_.erase(timestamp_us);
                }
                {
                    std::lock_guard<std::mutex> lk(encoder_pending_mutex_);
                    encoder_pending_frames_by_fd_.erase(dmabuf_fd);
                }
                stats_encoder_submit_failures_.fetch_add(1, std::memory_order_relaxed);
                return false;
            }

            maybeEnqueueEyeImageFrame(shared_frame);
            return true;
        }

        std::shared_ptr<SharedNv12Frame> makeSharedNv12Frame(int dmabuf_fd, uint64_t timestamp_us)
        {
            return std::shared_ptr<SharedNv12Frame>(
                new SharedNv12Frame{dmabuf_fd, timestamp_us},
                [this](SharedNv12Frame* frame) {
                    if (frame)
                    {
                        if (frame->fd >= 0)
                        {
                            // The fd carried in SharedNv12Frame is whatever
                            // we forwarded to the encoder/eye_image. When
                            // undistortion is on, that fd belongs to the
                            // undistorter pool; otherwise to the converter.
                            if (undistorter_)
                            {
                                undistorter_->releaseFd(frame->fd);
                            }
                            else if (converter_)
                            {
                                converter_->releaseFd(frame->fd);
                            }
                        }
                        delete frame;
                    }
                });
        }

        void updateEyeImageSubscriberState()
        {
            const bool has_subscribers = eye_image_publisher_ &&
                                         eye_image_publisher_->isEnabled() &&
                                         eye_image_publisher_->hasSubscribers();
            eye_image_has_subscribers_.store(has_subscribers, std::memory_order_release);
        }

        void maybeEnqueueEyeImageFrame(const std::shared_ptr<SharedNv12Frame>& frame)
        {
            if (!eye_image_enabled_ || !frame)
                return;

            if (!eye_image_publisher_ready_.load(std::memory_order_acquire))
            {
                eye_image_skip_not_ready_.fetch_add(1, std::memory_order_relaxed);
                return;
            }

            if (!eye_image_has_subscribers_.load(std::memory_order_acquire))
            {
                eye_image_skip_no_sub_.fetch_add(1, std::memory_order_relaxed);
                return;
            }

            {
                std::lock_guard<std::mutex> lk(nv12_queue_mutex_);
                // The mailbox is intentionally bounded to one frame. The
                // encode thread never waits for the CUDA eye worker.
                while (!nv12_queue_.empty())
                {
                    nv12_queue_.pop();
                    eye_image_drop_queue_.fetch_add(1, std::memory_order_relaxed);
                }
                const int64_t now_us = std::chrono::duration_cast<std::chrono::microseconds>(
                    std::chrono::steady_clock::now().time_since_epoch()).count();
                nv12_queue_.push({frame, now_us});
            }

            eye_image_enqueued_.fetch_add(1, std::memory_order_relaxed);
            nv12_queue_cv_.notify_one();
        }

    private:
        ros::NodeHandle &nh_;

        // Capture components
        std::shared_ptr<trb::video::VideoV4L2Capturer> capturer_;

        // Encode pipeline components (created in encode_thread_)
        std::shared_ptr<trb::video::VideoDecoder> decoder_;
        std::shared_ptr<trb::video::VideoConverter> converter_;
        std::shared_ptr<trb::video::VideoUndistorter> undistorter_;
        std::shared_ptr<trb::video::VideoEncoder> encoder_;

        // Eye image pipeline (created in eye_image_thread_)
        std::shared_ptr<trb::video::EyeImagePublisher> eye_image_publisher_;

        // Saved configs for deferred initialization
        trb::video::VideoDecoder::Config decoder_config_;
        trb::video::VideoConverter::Config converter_config_;
        trb::video::VideoUndistorter::Config undistorter_config_;
        bool undistort_enabled_{false};
        trb::video::VideoEncoder::Config encoder_config_;

        // Frame dimensions
        uint32_t frame_width_{0};
        uint32_t frame_height_{0};
        uint32_t frame_rate_{0};

        // V4L2 startup warmup: camera streaming starts only after the
        // decode/convert/encode pipeline is ready, then the first N camera
        // frames are dropped before feeding the decoder.
        int v4l2_warmup_drop_frames_{30};

        // Eye image config
        bool eye_image_enabled_{false};
        int eye_image_output_width_{640};
        int eye_image_output_height_{360};
        int eye_image_max_fps_{30};
        int eye_image_queue_depth_{3};
        int64_t eye_image_min_interval_us_{33333};
        std::string eye_image_backend_{"cuda"};
        std::string eye_image_left_topic_;
        std::string eye_image_right_topic_;

        // Frame counters
        std::atomic<uint64_t> encode_frame_in_count_{0};
        std::atomic<uint64_t> enc_frame_count_{0};
        std::atomic<uint64_t> stats_capture_frames_{0};
        std::atomic<uint64_t> stats_decode_frames_{0};
        std::atomic<uint64_t> stats_encode_frames_{0};
        std::atomic<uint64_t> stats_decoder_drops_{0};
        std::atomic<uint64_t> stats_encoder_submit_failures_{0};
        std::atomic<uint64_t> stats_encode_us_total_{0};
        std::atomic<bool> eye_image_publisher_ready_{false};
        std::atomic<bool> eye_image_has_subscribers_{false};
        std::atomic<uint64_t> eye_image_enqueued_{0};
        std::atomic<uint64_t> eye_image_published_{0};
        std::atomic<uint64_t> eye_image_skip_not_ready_{0};
        std::atomic<uint64_t> eye_image_skip_no_sub_{0};
        std::atomic<uint64_t> eye_image_drop_queue_{0};
        std::atomic<uint64_t> eye_image_process_us_total_{0};
        std::atomic<uint64_t> stats_eye_image_process_us_total_{0};
        std::atomic<uint64_t> stats_eye_image_frame_age_us_max_{0};

        // NVENC cold-start warmup: the first ~1s after the encoder is opened,
        // NvMMLiteOpen + session init stalls the pipeline ~200-400 ms so the
        // first N encoded packets all come out with huge submit-to-output lag.
        // Forwarding them downstream pollutes FEC/send queues and the receiver
        // sees a burst of stale frames. We drop encoded packets (but let the
        // DQ path run normally) until the warmup window elapses, then reset
        // the stats counters so the first [VIDEO] log reflects steady state.
        std::atomic<int64_t> warmup_deadline_us_{0};
        std::atomic<bool> warmup_active_{false};
        std::atomic<uint64_t> warmup_dropped_{0};
        int warmup_duration_ms_{1500};

        std::mutex enc_timing_mutex_;
        std::unordered_map<uint64_t, int64_t> enc_start_steady_us_by_ts_;
        std::mutex encoder_pending_mutex_;
        std::unordered_map<int, std::shared_ptr<SharedNv12Frame>> encoder_pending_frames_by_fd_;

        std::atomic<bool> running_{false};
        
        // Worker threads
        std::thread encode_thread_;
        std::thread capture_thread_;
        std::thread encoder_dq_thread_;
        std::thread eye_image_thread_;
        
        // NV12 queue for eye image (shared from encode thread)
        std::queue<NV12FrameData> nv12_queue_;
        std::mutex nv12_queue_mutex_;
        std::condition_variable nv12_queue_cv_;

        std::mutex callback_mutex_;
        EncodedFrameCallback encoded_frame_callback_;
        SpsPpsCallback sps_pps_callback_;

        std::mutex bitrate_mutex_;
        uint32_t last_bitrate_bps_{0};
        std::chrono::steady_clock::time_point last_bitrate_update_{std::chrono::steady_clock::time_point::min()};
    };

    VideoStreamManager::VideoStreamManager(ros::NodeHandle &nh)
        : impl_(std::make_unique<Impl>(nh))
    {
    }

    VideoStreamManager::~VideoStreamManager() = default;

    bool VideoStreamManager::start()
    {
        return impl_->start();
    }

    void VideoStreamManager::stop()
    {
        impl_->stop();
    }

    bool VideoStreamManager::isRunning() const
    {
        return impl_->isRunning();
    }

    void VideoStreamManager::setEncodedFrameCallback(EncodedFrameCallback callback)
    {
        impl_->setEncodedFrameCallback(callback);
    }

    void VideoStreamManager::setSpsPpsCallback(SpsPpsCallback callback)
    {
        impl_->setSpsPpsCallback(callback);
    }

    void VideoStreamManager::setTargetBitrate(uint32_t bitrate_bps)
    {
        impl_->setTargetBitrate(bitrate_bps);
    }

    VideoStreamManager::StatsSnapshot VideoStreamManager::consumeStats()
    {
        return impl_->consumeStats();
    }

} // namespace trb::video
