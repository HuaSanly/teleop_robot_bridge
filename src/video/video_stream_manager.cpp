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
#include <dlfcn.h>
#include <functional>
#include <memory>
#include <mutex>
#include <queue>
#include <stdexcept>
#include <string>
#include <thread>
#include <type_traits>
#include <unordered_map>
#include <utility>
#include <vector>

#include <linux/v4l2-controls.h>
#include <linux/videodev2.h>

#include "ament_index_cpp/get_package_share_directory.hpp"
#include "rclcpp/rclcpp.hpp"

#include "v4l2_nv_extensions.h"

#include "video/video_converter.hpp"
#include "video/video_decoder.hpp"
#include "video/video_encoder.hpp"
#include "video/video_scaler.hpp"
#include "video/video_recorder.hpp"
#include "video/video_v4l2_capturer.hpp"
#include "video/eye_image_publisher.hpp"
#include "video/video_undistorter.hpp"
#include "video/nvbuf_mutex.hpp"

// Jetson Multimedia API for DMA-BUF surface access
#include "nvbufsurface.h"

namespace trb::video
{
    namespace
    {
        template <typename T>
        T declareOrGet(rclcpp::Node &node, const std::string &name, const T &default_value)
        {
            if (!node.has_parameter(name))
            {
                return node.declare_parameter<T>(name, default_value);
            }
            return node.get_parameter(name).get_value<T>();
        }

        template <>
        int declareOrGet<int>(rclcpp::Node &node, const std::string &name, const int &default_value)
        {
            if (!node.has_parameter(name))
            {
                return static_cast<int>(node.declare_parameter<int64_t>(name, default_value));
            }
            return static_cast<int>(node.get_parameter(name).as_int());
        }

        std::string packageShareDirectory()
        {
            try
            {
                return ament_index_cpp::get_package_share_directory("teleop_robot_bridge");
            }
            catch (const std::exception &)
            {
                return {};
            }
        }

        bool checkNvjpegRuntime(std::string &error)
        {
            error.clear();
            dlerror();
            void *handle = dlopen("libnvjpeg.so.13", RTLD_NOW | RTLD_LOCAL);
            if (!handle)
            {
                const char *dl_error = dlerror();
                error = dl_error ? dl_error : "unknown dlopen error";
                return false;
            }
            dlclose(handle);
            return true;
        }

        uint32_t fnv1a32(const uint8_t *data, size_t size)
        {
            uint32_t hash = 2166136261u;
            const size_t sample_size = std::min<size_t>(size, 4096);
            for (size_t i = 0; i < sample_size; ++i)
            {
                hash ^= data[i];
                hash *= 16777619u;
            }
            return hash;
        }

        bool isBadDecodeStatus(VideoDecoder::SubmitStatus status)
        {
            return status == VideoDecoder::SubmitStatus::kDecodeFailed ||
                   status == VideoDecoder::SubmitStatus::kInvalidDecodedOutput;
        }

        const char *submitStatusToString(VideoDecoder::SubmitStatus status)
        {
            switch (status)
            {
            case VideoDecoder::SubmitStatus::kQueued:
                return "queued";
            case VideoDecoder::SubmitStatus::kStopped:
                return "stopped";
            case VideoDecoder::SubmitStatus::kInvalidInput:
                return "invalid_input";
            case VideoDecoder::SubmitStatus::kFrameTooLarge:
                return "frame_too_large";
            case VideoDecoder::SubmitStatus::kDecodedQueueFull:
                return "decoded_queue_full";
            case VideoDecoder::SubmitStatus::kNoDecodeBuffer:
                return "no_decode_buffer";
            case VideoDecoder::SubmitStatus::kDecodeFailed:
                return "decode_failed";
            case VideoDecoder::SubmitStatus::kInvalidDecodedOutput:
                return "invalid_decoded_output";
            }
            return "unknown";
        }

        const char *v4l2PixelFormatName(uint32_t format)
        {
            switch (format)
            {
            case V4L2_PIX_FMT_MJPEG:
                return "mjpeg";
            case V4L2_PIX_FMT_YUYV:
                return "yuyv";
            default:
                return "unknown";
            }
        }

        void joinThread(std::thread &thread, const char *name)
        {
            if (!thread.joinable())
            {
                return;
            }
            if (thread.get_id() == std::this_thread::get_id())
            {
                RCLCPP_ERROR(rclcpp::get_logger("teleop_robot_bridge.video"),
                             "VideoStreamManager: refusing to join current thread '%s'",
                             name);
                thread.detach();
                return;
            }
            thread.join();
        }
    }

    /**
     * @brief Architecture: CUDA video postprocess pipeline.
     *
     * MJPEG input:
     *   capture_thread_  : V4L2 MMAP dequeue -> NvJPEGDecoder submit -> V4L2 requeue
     *   encode_thread_   : decoder.dequeueSync -> CUDA postprocess -> encoder.submit
     *
     * YUYV input:
     *   encode_thread_   : V4L2 DMABUF dequeue -> CUDA postprocess -> V4L2 requeue
     *                      -> encoder.submit
     *
     * Data Flow:
     *   V4L2 Camera (MJPEG) -> NvJPEGDecoder -> YUV422 DMA-BUF
     *   V4L2 Camera (YUYV)  -> camera-owned YUYV DMA-BUF
     *        -> CUDA postprocess:
     *             undistort on  : fused YUV422/YUYV -> rectified NV12
     *             undistort off : YUV422/YUYV -> NV12 converter
     *        -> VideoEncoder    (NV12 -> H.264/H.265)
     *        -> onEncoderPacket -> UDP/gRPC
     *
     *   In parallel: a ref-counted NV12 fd is handed to eye_image_thread_
     *   for CUDA crop/scale/RGB publish. The fd returns to its owner only
     *   after NVENC input-done and eye_image processing have released it.
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
    };

    class VideoStreamManager::Impl
    {
    public:
        explicit Impl(rclcpp::Node &nh)
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

        bool requestKeyframe()
        {
            std::shared_ptr<trb::video::VideoEncoder> enc;
            {
                std::lock_guard<std::mutex> lk(callback_mutex_);
                enc = encoder_;
            }
            return enc && enc->forceIDR();
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
                    RCLCPP_WARN(rclcpp::get_logger("teleop_robot_bridge.video"), "VideoEncoder setBitrate failed: %u bps", bitrate_bps);
                }
            }
        }

        bool start()
        {
            if (running_.load())
                return true;

            // Unified profile: [width, height, framerate]
            const auto profile = declareOrGet<std::vector<int64_t>>(nh_, "video.profile", std::vector<int64_t>{3840, 1520, 30});
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
                RCLCPP_WARN(rclcpp::get_logger("teleop_robot_bridge.video"), "Invalid video/profile; expected [width,height,framerate]. Using defaults.");
            }

            std::string video_device;
            video_device = declareOrGet<std::string>(nh_, "video.device", "/dev/video0");
            std::string pixel_format_str;
            pixel_format_str = declareOrGet<std::string>(nh_, "video.pixel_format", "mjpeg");
            int v4l2_buffer_count_param;
            v4l2_buffer_count_param = declareOrGet<int>(nh_, "video.v4l2.buffer_count", 8);
            const uint32_t v4l2_buffer_count = (v4l2_buffer_count_param > 0) ? static_cast<uint32_t>(v4l2_buffer_count_param) : 8;
            v4l2_warmup_drop_frames_ = declareOrGet<int>(nh_, "video.v4l2.warmup_drop_frames", 30);
            if (v4l2_warmup_drop_frames_ < 0)
                v4l2_warmup_drop_frames_ = 0;

            int conv_pool_param;
            conv_pool_param = declareOrGet<int>(nh_, "video.decoder.buffer_pool_size", 12);
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
                cap_cfg.memory_mode = trb::video::VideoV4L2Capturer::MemoryMode::kDmaBuf;
            }
            else
            {
                RCLCPP_WARN(rclcpp::get_logger("teleop_robot_bridge.video"), "Unknown video/pixel_format='%s', falling back to mjpeg", pixel_format_str.c_str());
                cap_cfg.pixel_format = V4L2_PIX_FMT_MJPEG;
            }

            capturer_config_ = cap_cfg;
            capturer_ = std::make_shared<trb::video::VideoV4L2Capturer>();
            if (!capturer_->initialize(cap_cfg))
            {
                RCLCPP_ERROR(rclcpp::get_logger("teleop_robot_bridge.video"), "Failed to initialize V4L2 capturer");
                capturer_.reset();
                return false;
            }

            // Use negotiated device format for downstream
            const uint32_t actual_w = capturer_->width();
            const uint32_t actual_h = capturer_->height();
            const uint32_t actual_pixfmt = capturer_->pixelFormat();
            if (actual_w > 0 && actual_h > 0 && (actual_w != width || actual_h != height))
            {
                RCLCPP_WARN(rclcpp::get_logger("teleop_robot_bridge.video"), 
                    "V4L2 negotiated format differs from requested: requested=%ux%u, actual=%ux%u. Using actual for converter/encoder.",
                    width, height, actual_w, actual_h);
                width = actual_w;
                height = actual_h;
            }
            capturer_config_.width = width;
            capturer_config_.height = height;
            yuyv_direct_enabled_ = (cap_cfg.pixel_format == V4L2_PIX_FMT_YUYV &&
                                    actual_pixfmt == V4L2_PIX_FMT_YUYV &&
                                    cap_cfg.memory_mode == trb::video::VideoV4L2Capturer::MemoryMode::kDmaBuf);
            if (cap_cfg.pixel_format == V4L2_PIX_FMT_YUYV && !yuyv_direct_enabled_)
            {
                RCLCPP_ERROR(rclcpp::get_logger("teleop_robot_bridge.video"),
                             "VideoStreamManager: requested YUYV direct path but V4L2 negotiated pixfmt=%s/0x%08x",
                             v4l2PixelFormatName(actual_pixfmt),
                             actual_pixfmt);
                capturer_.reset();
                return false;
            }
            RCLCPP_INFO(rclcpp::get_logger("teleop_robot_bridge.video"),
                        "VideoStreamManager: input pixfmt=%s path=%s",
                        v4l2PixelFormatName(actual_pixfmt),
                        yuyv_direct_enabled_ ? "yuyv_dmabuf_direct" : "mjpeg_decode");

            // Scaling happens after decode/conversion (and after optional
            // undistortion), so capture and decoder dimensions stay at the
            // negotiated camera resolution while encoder/output dimensions can
            // be reduced independently.
            scale_enabled_ = declareOrGet<bool>(nh_, "video.scale.enabled", false);
            const int scale_width_param = declareOrGet<int>(nh_, "video.scale.width", 3200);
            const int scale_height_param = declareOrGet<int>(nh_, "video.scale.height", 1200);
            uint32_t output_width = width;
            uint32_t output_height = height;
            if (scale_enabled_)
            {
                if (scale_width_param <= 0 || scale_height_param <= 0 ||
                    (scale_width_param & 1) != 0 || (scale_height_param & 1) != 0)
                {
                    RCLCPP_ERROR(rclcpp::get_logger("teleop_robot_bridge.video"),
                                 "Invalid video.scale dimensions=%dx%d; dimensions must be positive and even",
                                 scale_width_param, scale_height_param);
                    return false;
                }
                const uint64_t source_area_ratio = static_cast<uint64_t>(width) *
                                                   static_cast<uint64_t>(scale_height_param);
                const uint64_t target_area_ratio = static_cast<uint64_t>(height) *
                                                   static_cast<uint64_t>(scale_width_param);
                if (source_area_ratio != target_area_ratio)
                {
                    RCLCPP_ERROR(rclcpp::get_logger("teleop_robot_bridge.video"),
                                 "video.scale must preserve aspect ratio: input=%ux%u target=%dx%d",
                                 width, height, scale_width_param, scale_height_param);
                    return false;
                }
                output_width = static_cast<uint32_t>(scale_width_param);
                output_height = static_cast<uint32_t>(scale_height_param);
            }
            if (scale_enabled_)
            {
                RCLCPP_INFO(rclcpp::get_logger("teleop_robot_bridge.video"),
                            "VideoStreamManager: output scaling enabled input=%ux%u output=%ux%u",
                            width, height, output_width, output_height);
            }

            // Store output dimensions for the eye image thread and status
            // reporting. The decoder/camera configs below retain input size.
            frame_width_ = output_width;
            frame_height_ = output_height;
            frame_rate_ = framerate;

            // Save CUDA postprocess config for deferred initialization in encode thread.
            decoder_config_.width = width;
            decoder_config_.height = height;
            int decoder_output_plane_buffers_param;
            int decoder_capture_plane_buffers_param;
            int decoder_max_mjpeg_size_param;
            const uint32_t default_max_mjpeg_size = std::max<uint32_t>(4 * 1024 * 1024u, width * height * 2u);
            decoder_output_plane_buffers_param = declareOrGet<int>(nh_, "video.decoder.output_plane_buffers", 6);
            decoder_capture_plane_buffers_param = declareOrGet<int>(nh_, "video.decoder.capture_plane_buffers", static_cast<int>(conv_pool_size));
            decoder_max_mjpeg_size_param = declareOrGet<int>(nh_, "video.decoder.max_mjpeg_size_bytes", static_cast<int>(default_max_mjpeg_size));
            decoder_config_.output_plane_buffers = (decoder_output_plane_buffers_param > 0) ? static_cast<uint32_t>(decoder_output_plane_buffers_param) : 6u;
            decoder_config_.capture_plane_buffers = (decoder_capture_plane_buffers_param > 0) ? static_cast<uint32_t>(decoder_capture_plane_buffers_param) : std::max<uint32_t>(conv_pool_size, 4u);
            decoder_config_.max_mjpeg_size_bytes = (decoder_max_mjpeg_size_param > 0) ? static_cast<uint32_t>(decoder_max_mjpeg_size_param) : default_max_mjpeg_size;

            converter_config_.width = width;
            converter_config_.height = height;
            converter_config_.input_width = width;
            converter_config_.input_height = height;
            converter_config_.buffer_pool_size = conv_pool_size;

            scaler_config_.input_width = width;
            scaler_config_.input_height = height;
            scaler_config_.output_width = output_width;
            scaler_config_.output_height = output_height;
            scaler_config_.buffer_pool_size = conv_pool_size;

            // Undistort + stereo-rectify (optional). The preferred path is
            // fused CUDA decoded-YUV422 -> rectified-NV12; the same NV12 fd
            // is forwarded to encoder and eye_image_publisher.
            undistort_enabled_ = declareOrGet<bool>(nh_, "video.undistort.enabled", false);
            if (undistort_enabled_)
            {
                undistorter_config_.width = width;
                undistorter_config_.height = height;
                undistorter_config_.buffer_pool_size = conv_pool_size;
                undistorter_config_.calibration_file = declareOrGet<std::string>(nh_, "video.undistort.calib_file", "");
                undistorter_config_.profile = declareOrGet<std::string>(nh_, "video.undistort.profile", "");
                if (!undistorter_config_.calibration_file.empty() &&
                    undistorter_config_.calibration_file.front() != '/')
                {
                    const std::string pkg = packageShareDirectory();
                    if (!pkg.empty())
                    {
                        undistorter_config_.calibration_file = pkg + "/" +
                            undistorter_config_.calibration_file;
                    }
                }
                undistorter_config_.require_calibration = declareOrGet<bool>(nh_, "video.undistort.require_calibration", true);
            }
            decoder_config_.decode_surface_layout = 0;
            encoder_config_.input_format = trb::video::VideoEncoder::InputFormat::kNv12;

            // Save encoder config for deferred initialization
            encoder_config_.width = output_width;
            encoder_config_.height = output_height;
            encoder_config_.framerate = framerate;

            std::string codec_str;
            codec_str = declareOrGet<std::string>(nh_, "video.encoder.codec", "h264");
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
                    RCLCPP_WARN(rclcpp::get_logger("teleop_robot_bridge.video"), "Unknown video.encoder.codec='%s' (use h264|h265). Falling back to h264.",
                             codec_str.c_str());
                    encoder_config_.codec = trb::video::VideoEncoder::Codec::kH264;
                }
            }

            // ---- User-facing knobs (yaml) ----
            int bitrate_param;
            bitrate_param = declareOrGet<int>(nh_, "video.encoder.bitrate", 40000000);
            encoder_config_.bitrate = static_cast<uint32_t>(std::max(1, bitrate_param));

            // I-frame interval (GOP length, in frames). Each I-frame is intra-coded.
            int i_frame_interval_param;
            i_frame_interval_param = declareOrGet<int>(nh_, "video.encoder.i_frame_interval", 30);
            encoder_config_.idr_interval = static_cast<uint32_t>(std::max(1, i_frame_interval_param));

            // IDR interval (in GOPs). IDR is a special I-frame that flushes the
            // reference list -> decoder can independently start here. 1 means
            // every I-frame is also an IDR for fastest loss recovery. Keep
            // `video.encoder.idr_interval` as a compatibility fallback because
            // older configs used that key for the same underlying NVENC knob.
            int idr_interval_gops_param;
            idr_interval_gops_param = declareOrGet<int>(nh_, "video.encoder.idr_interval_gops",
                                                        declareOrGet<int>(nh_, "video.encoder.idr_interval", 1));
            encoder_config_.idr_interval_gops = static_cast<uint32_t>(std::max(1, idr_interval_gops_param));

            int force_idr_every_n_param;
            force_idr_every_n_param = declareOrGet<int>(nh_, "video.encoder.force_idr_every_n", 0);
            encoder_config_.force_idr_every_n = static_cast<uint32_t>(std::max(0, force_idr_every_n_param));

            bool intra_refresh_enabled;
            intra_refresh_enabled = declareOrGet<bool>(nh_, "video.encoder.intra_refresh.enabled", false);
            int intra_refresh_interval_slices_param;
            intra_refresh_interval_slices_param = declareOrGet<int>(nh_, "video.encoder.intra_refresh.interval_slices", 0);
            encoder_config_.slice_intra_refresh_interval_slices = intra_refresh_enabled
                ? static_cast<uint32_t>(std::max(1, intra_refresh_interval_slices_param))
                : 0u;

            bool low_freq_idr_enabled;
            low_freq_idr_enabled = declareOrGet<bool>(nh_, "video.encoder.low_freq_idr.enabled", false);
            int low_freq_idr_interval_sec;
            low_freq_idr_interval_sec = declareOrGet<int>(nh_, "video.encoder.low_freq_idr.interval_sec", 2);
            if (low_freq_idr_enabled && low_freq_idr_interval_sec > 0)
            {
                const uint64_t frames_u64 = static_cast<uint64_t>(low_freq_idr_interval_sec) * static_cast<uint64_t>(framerate);
                const uint32_t frames = static_cast<uint32_t>(std::max<uint64_t>(1, frames_u64));
                encoder_config_.idr_interval = frames;
                encoder_config_.idr_interval_gops = frames;
            }

            bool max_perf_mode;
            max_perf_mode = declareOrGet<bool>(nh_, "video.encoder.max_perf_mode", false);
            encoder_config_.max_perf_mode = max_perf_mode;

            // ---- Encoder low-level knobs (all from yaml for manual tuning) ----
            std::string rc_str;
            rc_str = declareOrGet<std::string>(nh_, "video.encoder.rate_control", "vbr");
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
            peak_bitrate_param = declareOrGet<int>(nh_, "video.encoder.peak_bitrate", 100000000);
            encoder_config_.peak_bitrate = static_cast<uint32_t>(std::max(1, peak_bitrate_param));

            int vbv_size_param;
            vbv_size_param = declareOrGet<int>(nh_, "video.encoder.virtual_buffer_size", 50000000);
            encoder_config_.virtual_buffer_size = static_cast<uint32_t>(std::max(1, vbv_size_param));

            std::string preset_str;
            preset_str = declareOrGet<std::string>(nh_, "video.encoder.hw_preset", "fast");
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
            num_ref_param = declareOrGet<int>(nh_, "video.encoder.num_reference_frames", 1);
            encoder_config_.num_reference_frames = static_cast<uint32_t>(std::max(0, num_ref_param));

            int num_b_param;
            num_b_param = declareOrGet<int>(nh_, "video.encoder.num_b_frames", 0);
            encoder_config_.num_b_frames = static_cast<uint32_t>(std::max(0, num_b_param));

            bool insert_sps_pps_at_idr;
            insert_sps_pps_at_idr = declareOrGet<bool>(nh_, "video.encoder.insert_sps_pps_at_idr", true);
            encoder_config_.insert_sps_pps_at_idr = insert_sps_pps_at_idr ? 1 : 0;

            bool slice_level_encode;
            slice_level_encode = declareOrGet<bool>(nh_, "video.encoder.slice_level_encode", false);
            encoder_config_.slice_level_encode = slice_level_encode;

            int num_slices_param;
            num_slices_param = declareOrGet<int>(nh_, "video.encoder.num_slices", 1);
            encoder_config_.num_slices = static_cast<uint32_t>(std::max(1, num_slices_param));

            int poc_type_param;
            poc_type_param = declareOrGet<int>(nh_, "video.encoder.poc_type", -1);
            encoder_config_.poc_type = poc_type_param;

            bool insert_aud;
            insert_aud = declareOrGet<bool>(nh_, "video.encoder.insert_aud", false);
            encoder_config_.insert_aud = insert_aud;

            int framerate_override_param;
            framerate_override_param = declareOrGet<int>(nh_, "video.encoder.framerate_override", 0);
            encoder_config_.encoder_framerate_override = static_cast<uint32_t>(std::max(0, framerate_override_param));

            int output_plane_buffers_param;
            output_plane_buffers_param = declareOrGet<int>(nh_, "video.encoder.output_plane_buffers", 2);
            encoder_config_.output_plane_buffers = static_cast<uint32_t>(std::max(1, output_plane_buffers_param));

            int capture_plane_buffers_param;
            capture_plane_buffers_param = declareOrGet<int>(nh_, "video.encoder.capture_plane_buffers", 2);
            encoder_config_.capture_plane_buffers = static_cast<uint32_t>(std::max(1, capture_plane_buffers_param));

            int qp_i_min, qp_i_max, qp_p_min, qp_p_max;
            qp_i_min = declareOrGet<int>(nh_, "video.encoder.qp_range.i_min", 0);
            qp_i_max = declareOrGet<int>(nh_, "video.encoder.qp_range.i_max", 0);
            qp_p_min = declareOrGet<int>(nh_, "video.encoder.qp_range.p_min", 0);
            qp_p_max = declareOrGet<int>(nh_, "video.encoder.qp_range.p_max", 0);
            encoder_config_.qp_range_i_min = static_cast<uint32_t>(std::max(0, qp_i_min));
            encoder_config_.qp_range_i_max = static_cast<uint32_t>(std::max(0, qp_i_max));
            encoder_config_.qp_range_p_min = static_cast<uint32_t>(std::max(0, qp_p_min));
            encoder_config_.qp_range_p_max = static_cast<uint32_t>(std::max(0, qp_p_max));

            warmup_duration_ms_ = declareOrGet<int>(nh_, "video.encoder.warmup_ms", 1500);
            if (warmup_duration_ms_ < 0) warmup_duration_ms_ = 0;

            recorder_config_.enabled = declareOrGet<bool>(nh_, "video.recording.enabled", false);
            recorder_config_.output_dir = declareOrGet<std::string>(nh_, "video.recording.output_dir", "");

            // Check if eye image is enabled
            eye_image_enabled_ = declareOrGet<bool>(nh_, "video.eye_image.enabled", false);
            if (eye_image_enabled_)
            {
                eye_image_output_width_ = declareOrGet<int>(nh_, "video.eye_image.output_width", 640);
                eye_image_output_height_ = declareOrGet<int>(nh_, "video.eye_image.output_height", 360);
                eye_image_max_fps_ = declareOrGet<int>(nh_, "video.eye_image.max_fps", 10);
                if (eye_image_max_fps_ < 0)
                    eye_image_max_fps_ = 0;
                eye_image_min_interval_us_ = eye_image_max_fps_ > 0
                    ? static_cast<int64_t>(1000000 / std::max(1, eye_image_max_fps_))
                    : 0;
                eye_image_left_topic_ = declareOrGet<std::string>(nh_, "video.eye_image.left_topic", "data/left_eye_image");
                eye_image_right_topic_ = declareOrGet<std::string>(nh_, "video.eye_image.right_topic", "data/right_eye_image");
            }

            restart_on_bad_frames_ = declareOrGet<bool>(nh_, "video.decoder.restart_on_bad_frames", true);
            consecutive_bad_frame_threshold_ = declareOrGet<int>(nh_, "video.decoder.consecutive_bad_frame_threshold", 5);
            if (consecutive_bad_frame_threshold_ < 1)
                consecutive_bad_frame_threshold_ = 1;
            restart_backoff_ms_ = declareOrGet<int>(nh_, "video.decoder.restart_backoff_ms", 500);
            if (restart_backoff_ms_ < 0)
                restart_backoff_ms_ = 0;

            if (cap_cfg.pixel_format == V4L2_PIX_FMT_MJPEG)
            {
                std::string nvjpeg_error;
                if (!checkNvjpegRuntime(nvjpeg_error))
                {
                    RCLCPP_ERROR(rclcpp::get_logger("teleop_robot_bridge.video"),
                                 "VideoStreamManager: MJPEG decode runtime unavailable: failed to load libnvjpeg.so.13 (%s). "
                                 "libnvmm_jpeg.so will abort when the first MJPEG frame is decoded; install the matching CUDA nvJPEG runtime or fix the library path.",
                                 nvjpeg_error.c_str());
                    return false;
                }
            }

            {
                std::lock_guard<std::mutex> lk(restart_mutex_);
                restart_requested_ = false;
                restart_reason_.clear();
            }
            running_.store(true, std::memory_order_release);

            if (!startPipeline())
            {
                running_.store(false, std::memory_order_release);
                stopPipeline();
                return false;
            }

            assignWorkerThread(restart_supervisor_thread_,
                               std::thread(&Impl::restartSupervisorThreadMain, this),
                               "restart_supervisor",
                               false);
            return true;
        }

        void stop()
        {
            const bool was_running = running_.exchange(false);
            if (!was_running)
                return;

            {
                std::lock_guard<std::mutex> lk(restart_mutex_);
                restart_requested_ = false;
                restart_reason_.clear();
            }
            restart_cv_.notify_all();
            joinWorkerThread(restart_supervisor_thread_, "restart_supervisor");
            stopPipeline();
        }

        bool isRunning() const
        {
            return running_.load();
        }

        VideoStreamManager::StatsSnapshot consumeStats()
        {
            VideoStreamManager::StatsSnapshot snapshot;
            snapshot.converter_output_format = "nv12";
            snapshot.capture_frames = stats_capture_frames_.exchange(0, std::memory_order_relaxed);
            snapshot.decode_frames = stats_decode_frames_.exchange(0, std::memory_order_relaxed);
            snapshot.encode_frames = stats_encode_frames_.exchange(0, std::memory_order_relaxed);
            snapshot.decoder_drops = stats_decoder_drops_.exchange(0, std::memory_order_relaxed);
            snapshot.encoder_submit_failures = stats_encoder_submit_failures_.exchange(0, std::memory_order_relaxed);
            snapshot.encode_us_total = stats_encode_us_total_.exchange(0, std::memory_order_relaxed);
            snapshot.decode_us_total = stats_fused_decode_us_total_.exchange(0, std::memory_order_relaxed);

            std::shared_ptr<trb::video::VideoConverter> converter;
            std::shared_ptr<trb::video::VideoUndistorter> undistorter;
            {
                std::lock_guard<std::mutex> lk(pipeline_component_mutex_);
                converter = converter_;
                undistorter = undistorter_;
            }

            if (converter)
            {
                snapshot.converter_output_format = converter->outputFormatName();
                auto converter_stats = converter->consumeStats();
                snapshot.convert_frames = converter_stats.processed_frames;
                snapshot.converter_pool_drops = converter_stats.pool_drops;
                snapshot.converter_failures = converter_stats.failed_frames;
                snapshot.decode_us_total += static_cast<uint64_t>(std::max<int64_t>(0, converter_stats.decode_us_total));
                snapshot.transform_us_total = static_cast<uint64_t>(std::max<int64_t>(0, converter_stats.transform_us_total));
                snapshot.transform_map_us_total = static_cast<uint64_t>(std::max<int64_t>(0, converter_stats.map_us_total));
                snapshot.transform_wait_us_total = static_cast<uint64_t>(std::max<int64_t>(0, converter_stats.transform_wait_us_total));
                snapshot.transform_call_us_total = static_cast<uint64_t>(std::max<int64_t>(0, converter_stats.transform_call_us_total));
            }

            if (undistorter)
            {
                auto und_stats = undistorter->consumeStats();
                snapshot.undistort_frames = und_stats.processed_frames;
                snapshot.undistort_pool_drops = und_stats.pool_drops;
                snapshot.undistort_failures = und_stats.failed_frames;
                snapshot.undistort_fallback_frames = und_stats.fallback_frames;
                snapshot.undistort_us_total = static_cast<uint64_t>(std::max<int64_t>(0, und_stats.remap_us_total));
                snapshot.undistort_map_us_total = static_cast<uint64_t>(std::max<int64_t>(0, und_stats.map_us_total));
                snapshot.undistort_kernel_us_total = static_cast<uint64_t>(std::max<int64_t>(0, und_stats.kernel_us_total));
                snapshot.undistort_sync_us_total = static_cast<uint64_t>(std::max<int64_t>(0, und_stats.sync_us_total));
                snapshot.undistort_backend = und_stats.backend;
            }

            return snapshot;
        }

    private:
        bool startPipeline()
        {
            if (!running_.load(std::memory_order_acquire))
            {
                return false;
            }
            if (pipeline_running_.load(std::memory_order_acquire))
            {
                return true;
            }

            if (!capturer_)
            {
                capturer_ = std::make_shared<trb::video::VideoV4L2Capturer>();
                if (!capturer_->initialize(capturer_config_))
                {
                    RCLCPP_ERROR(rclcpp::get_logger("teleop_robot_bridge.video"),
                                 "VideoStreamManager: failed to initialize V4L2 capturer during pipeline start");
                    capturer_.reset();
                    return false;
                }
            }

            const int64_t now_us = std::chrono::duration_cast<std::chrono::microseconds>(
                                       std::chrono::steady_clock::now().time_since_epoch())
                                       .count();
            warmup_deadline_us_.store(now_us + static_cast<int64_t>(warmup_duration_ms_) * 1000,
                                      std::memory_order_relaxed);
            warmup_active_.store(warmup_duration_ms_ > 0, std::memory_order_relaxed);
            warmup_dropped_.store(0, std::memory_order_relaxed);

            recorder_ = std::make_shared<trb::video::VideoRecorder>(rclcpp::get_logger("teleop_robot_bridge.video"));
            if (!recorder_->initialize(recorder_config_))
            {
                RCLCPP_WARN(rclcpp::get_logger("teleop_robot_bridge.video"),
                            "VideoRecorder initialize failed; recording disabled");
                recorder_.reset();
            }
            recorded_packet_count_.store(0, std::memory_order_relaxed);
            consecutive_bad_decode_frames_.store(0, std::memory_order_relaxed);

            {
                std::lock_guard<std::mutex> lk(nv12_queue_mutex_);
                while (!nv12_queue_.empty())
                    nv12_queue_.pop();
                eye_image_inflight_.store(false, std::memory_order_relaxed);
            }
            eye_image_publisher_ready_.store(false, std::memory_order_relaxed);
            eye_image_has_subscribers_.store(false, std::memory_order_relaxed);

            pipeline_running_.store(true, std::memory_order_release);

            // Start encode pipeline (V4L2 DQ + MJPEG decode + convert + encode)
            // and optional eye-image thread.
            assignWorkerThread(encode_thread_, std::thread(&Impl::encodeThreadMain, this), "encode", true);

            if (eye_image_enabled_)
            {
                assignWorkerThread(eye_image_thread_, std::thread(&Impl::eyeImageThreadMain, this), "eye_image", true);
                RCLCPP_INFO(rclcpp::get_logger("teleop_robot_bridge.video"),
                            "VideoStreamManager: Started with zero-copy eye_image pipeline (max_fps=%d)",
                            eye_image_max_fps_);
            }
            else
            {
                RCLCPP_INFO(rclcpp::get_logger("teleop_robot_bridge.video"),
                            "VideoStreamManager: Started (eye_image disabled)");
            }

            return true;
        }

        void stopPipeline()
        {
            RCLCPP_INFO(rclcpp::get_logger("teleop_robot_bridge.video"),
                        "VideoStreamManager: stopping video pipeline");
            pipeline_running_.store(false, std::memory_order_release);

            if (capturer_)
            {
                // Break poll/DQBUF quickly but keep the mmap buffers alive until
                // capture_thread_ has dropped any in-flight frame reference.
                capturer_->interrupt();
            }

            // Wake up eye image thread.
            {
                std::lock_guard<std::mutex> lk(nv12_queue_mutex_);
                nv12_queue_cv_.notify_all();
            }

            joinWorkerThread(capture_thread_, "capture");
            joinWorkerThread(encode_thread_, "encode");
            if (capturer_)
            {
                // encode_thread_ owns pipeline startup and may have spawned the
                // capture thread after the first join attempt during a fast stop.
                capturer_->interrupt();
            }
            joinWorkerThread(capture_thread_, "capture");
            joinWorkerThread(encoder_dq_thread_, "encoder_dq");
            joinWorkerThread(eye_image_thread_, "eye_image");
            RCLCPP_INFO(rclcpp::get_logger("teleop_robot_bridge.video"),
                        "VideoStreamManager: video worker threads stopped");

            {
                std::lock_guard<std::mutex> lk(nv12_queue_mutex_);
                while (!nv12_queue_.empty())
                    nv12_queue_.pop();
                eye_image_inflight_.store(false, std::memory_order_relaxed);
            }

            auto leak_shared = [](auto &ptr) {
                using SharedPtr = std::decay_t<decltype(ptr)>;
                if (ptr)
                {
                    (void)new SharedPtr(std::move(ptr));
                }
            };
            const bool leak_gpu_objects = !rclcpp::ok();
            // Jetson NvBufSurface/CUDA/NvJPEG objects can still be referenced by
            // driver-side teardown shortly after STREAMOFF/NVENC abort. Destroying
            // them during in-process unpair has been observed to segfault after the
            // capturer is stopped. Keep these pool owners alive until process exit;
            // the next pairing creates a fresh pipeline while the OS reclaims the
            // retired GPU resources when the process terminates.
            constexpr bool retire_nvbuf_pipeline_objects = true;

            std::shared_ptr<trb::video::VideoEncoder> encoder_to_stop;
            {
                // Hide encoder_ before shutdown so late external keyframe/bitrate
                // requests cannot observe a half-stopped NVENC instance.
                std::lock_guard<std::mutex> lk(callback_mutex_);
                encoder_to_stop = std::move(encoder_);
            }
            if (encoder_to_stop)
            {
                RCLCPP_INFO(rclcpp::get_logger("teleop_robot_bridge.video"),
                            "VideoStreamManager: stopping encoder");
                encoder_to_stop->shutdown();
                encoder_to_stop.reset();
                RCLCPP_INFO(rclcpp::get_logger("teleop_robot_bridge.video"),
                            "VideoStreamManager: encoder stopped");
            }

            {
                std::lock_guard<std::mutex> lk(encoder_pending_mutex_);
                if (leak_gpu_objects)
                {
                    for (auto &entry : encoder_pending_frames_by_fd_)
                    {
                        if (entry.second)
                        {
                            entry.second->fd = -1;
                        }
                    }
                }
                encoder_pending_frames_by_fd_.clear();
            }
            RCLCPP_INFO(rclcpp::get_logger("teleop_robot_bridge.video"),
                        "VideoStreamManager: encoder pending frames cleared");

            {
                std::shared_ptr<trb::video::VideoV4L2Capturer> capturer_to_stop;
                capturer_to_stop = std::move(capturer_);
                if (capturer_to_stop)
                {
                    RCLCPP_INFO(rclcpp::get_logger("teleop_robot_bridge.video"),
                                "VideoStreamManager: stopping capturer");
                    capturer_to_stop->stop();
                    if (leak_gpu_objects || retire_nvbuf_pipeline_objects)
                    {
                        leak_shared(capturer_to_stop);
                    }
                    RCLCPP_INFO(rclcpp::get_logger("teleop_robot_bridge.video"),
                                "VideoStreamManager: capturer stopped");
                }
            }

            {
                // These components own NvBufSurface/CUDA/NvJPEG pools. On Jetson,
                // immediate destruction after STREAMOFF/NVENC abort can race with
                // driver teardown, so the normal in-process stop path retires them.
                std::shared_ptr<trb::video::VideoDecoder> decoder_to_release;
                std::shared_ptr<trb::video::VideoUndistorter> undistorter_to_release;
                std::shared_ptr<trb::video::VideoConverter> converter_to_release;
                std::shared_ptr<trb::video::VideoScaler> scaler_to_release;

                {
                    std::lock_guard<std::mutex> lk(getNvBufMutex());
                    decoder_to_release = std::move(decoder_);
                    {
                        std::lock_guard<std::mutex> component_lk(pipeline_component_mutex_);
                        undistorter_to_release = std::move(undistorter_);
                        converter_to_release = std::move(converter_);
                        scaler_to_release = std::move(scaler_);
                    }

                    if (leak_gpu_objects || retire_nvbuf_pipeline_objects)
                    {
                        leak_shared(decoder_to_release);
                        leak_shared(undistorter_to_release);
                        leak_shared(converter_to_release);
                        leak_shared(scaler_to_release);
                    }
                    else
                    {
                        decoder_to_release.reset();
                        undistorter_to_release.reset();
                        converter_to_release.reset();
                        scaler_to_release.reset();
                    }
                }
            }
            RCLCPP_INFO(rclcpp::get_logger("teleop_robot_bridge.video"),
                        retire_nvbuf_pipeline_objects
                            ? "VideoStreamManager: video components retired"
                            : "VideoStreamManager: video components released");

            {
                std::shared_ptr<trb::video::EyeImagePublisher> eye_image_publisher_to_release;
                eye_image_publisher_to_release = std::move(eye_image_publisher_);
                if (leak_gpu_objects)
                {
                    leak_shared(eye_image_publisher_to_release);
                }
            }

            {
                std::shared_ptr<trb::video::VideoRecorder> recorder_to_release;
                recorder_to_release = std::move(recorder_);
            }
            eye_image_publisher_ready_.store(false, std::memory_order_relaxed);
            eye_image_has_subscribers_.store(false, std::memory_order_relaxed);
            consecutive_bad_decode_frames_.store(0, std::memory_order_relaxed);

            {
                std::lock_guard<std::mutex> lk(enc_timing_mutex_);
                enc_start_steady_us_by_ts_.clear();
            }
            RCLCPP_INFO(rclcpp::get_logger("teleop_robot_bridge.video"),
                        "VideoStreamManager: video pipeline stopped");
        }

        void requestPipelineRestart(const std::string &reason)
        {
            if (!running_.load(std::memory_order_acquire) ||
                !pipeline_running_.load(std::memory_order_acquire))
            {
                return;
            }
            {
                std::lock_guard<std::mutex> lk(restart_mutex_);
                if (restart_requested_)
                {
                    return;
                }
                restart_requested_ = true;
                restart_reason_ = reason;
            }
            restart_cv_.notify_one();
        }

        void restartSupervisorThreadMain()
        {
            RCLCPP_INFO(rclcpp::get_logger("teleop_robot_bridge.video"),
                        "VideoStreamManager: restart supervisor started");
            while (running_.load(std::memory_order_acquire))
            {
                std::string reason;
                {
                    std::unique_lock<std::mutex> lk(restart_mutex_);
                    restart_cv_.wait(lk, [this] {
                        return !running_.load(std::memory_order_acquire) || restart_requested_;
                    });
                    if (!running_.load(std::memory_order_acquire))
                    {
                        break;
                    }
                    reason = restart_reason_;
                    restart_requested_ = false;
                    restart_reason_.clear();
                }

                RCLCPP_WARN(rclcpp::get_logger("teleop_robot_bridge.video"),
                            "VideoStreamManager: restarting video pipeline (%s)",
                            reason.c_str());
                stopPipeline();

                while (running_.load(std::memory_order_acquire))
                {
                    if (restart_backoff_ms_ > 0)
                    {
                        std::unique_lock<std::mutex> lk(restart_mutex_);
                        if (restart_cv_.wait_for(lk,
                                                 std::chrono::milliseconds(restart_backoff_ms_),
                                                 [this] { return !running_.load(std::memory_order_acquire); }))
                        {
                            break;
                        }
                    }

                    if (!running_.load(std::memory_order_acquire))
                    {
                        break;
                    }
                    if (startPipeline())
                    {
                        RCLCPP_INFO(rclcpp::get_logger("teleop_robot_bridge.video"),
                                    "VideoStreamManager: video pipeline restart complete");
                        break;
                    }
                    RCLCPP_ERROR(rclcpp::get_logger("teleop_robot_bridge.video"),
                                 "VideoStreamManager: video pipeline restart failed; retrying in %dms",
                                 restart_backoff_ms_);
                    stopPipeline();
                }
            }
            RCLCPP_INFO(rclcpp::get_logger("teleop_robot_bridge.video"),
                        "VideoStreamManager: restart supervisor exiting");
        }

        // ==================== Encode Thread ====================
        bool initializePostprocess()
        {
            bool postprocess_ready = false;
            if (undistort_enabled_)
            {
                auto und = std::make_shared<trb::video::VideoUndistorter>();
                if (!und->initialize(undistorter_config_))
                {
                    RCLCPP_ERROR(rclcpp::get_logger("teleop_robot_bridge.video"),
                                 "[ENCODE] VideoUndistorter initialize failed while video.undistort.enabled=true; "
                                 "pipeline startup aborted");
                }
                else
                {
                    std::lock_guard<std::mutex> lk(pipeline_component_mutex_);
                    undistorter_ = und;
                    converter_.reset();
                    postprocess_ready = true;
                }
            }
            else
            {
                auto converter = std::make_shared<trb::video::VideoConverter>();
                if (!converter->initialize(converter_config_))
                {
                    RCLCPP_WARN(rclcpp::get_logger("teleop_robot_bridge.video"), "[ENCODE] VideoConverter initialize failed");
                }
                else
                {
                    std::lock_guard<std::mutex> lk(pipeline_component_mutex_);
                    converter_ = converter;
                    undistorter_.reset();
                    postprocess_ready = true;
                }
            }

            if (postprocess_ready && scale_enabled_)
            {
                auto scaler = std::make_shared<trb::video::VideoScaler>();
                if (!scaler->initialize(scaler_config_))
                {
                    RCLCPP_ERROR(rclcpp::get_logger("teleop_robot_bridge.video"),
                                 "[ENCODE] VideoScaler initialize failed while video.scale.enabled=true; "
                                 "pipeline startup aborted");
                    postprocess_ready = false;
                }
                else
                {
                    std::lock_guard<std::mutex> lk(pipeline_component_mutex_);
                    scaler_ = scaler;
                }
            }
            else
            {
                std::lock_guard<std::mutex> lk(pipeline_component_mutex_);
                scaler_.reset();
            }
            return postprocess_ready;
        }

        bool initializeEncoder()
        {
            std::shared_ptr<trb::video::VideoEncoder> new_encoder =
                std::make_shared<trb::video::VideoEncoder>();
            if (!new_encoder->initialize(encoder_config_))
            {
                RCLCPP_WARN(rclcpp::get_logger("teleop_robot_bridge.video"), "[ENCODE] VideoEncoder initialize failed");
                return false;
            }

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
            return true;
        }

        bool postprocessYuv422Fd(int src_fd,
                                 uint64_t capture_timestamp_us,
                                 uint64_t decode_us,
                                 int &downstream_fd)
        {
            downstream_fd = -1;
            int converted_fd = -1;
            if (undistorter_)
            {
                if (!undistorter_->processYuv422(src_fd, converted_fd))
                {
                    RCLCPP_WARN_THROTTLE(rclcpp::get_logger("teleop_robot_bridge.video"),
                                         *rclcpp::Clock::make_shared(),
                                         1000,
                                         "[ENCODE] fused CUDA undistort failed; dropping frame");
                    return false;
                }
                if (decode_us > 0)
                {
                    stats_fused_decode_us_total_.fetch_add(decode_us, std::memory_order_relaxed);
                }
            }
            else if (converter_)
            {
                if (!converter_->transformSync(src_fd,
                                               capture_timestamp_us,
                                               decode_us,
                                               converted_fd))
                {
                    return false;
                }
            }
            else
            {
                return false;
            }

            if (!scaler_)
            {
                downstream_fd = converted_fd;
                return true;
            }

            if (!scaler_->transformSync(converted_fd, downstream_fd))
            {
                if (undistorter_)
                {
                    undistorter_->releaseFd(converted_fd);
                }
                else if (converter_)
                {
                    converter_->releaseFd(converted_fd);
                }
                downstream_fd = -1;
                return false;
            }

            // The intermediate full-resolution frame is no longer needed once
            // NvBufSurf::NvTransform has completed synchronously.
            if (undistorter_)
            {
                undistorter_->releaseFd(converted_fd);
            }
            else if (converter_)
            {
                converter_->releaseFd(converted_fd);
            }
            return true;
        }

        void encodeThreadMain()
        {
            if (!initializePostprocess())
            {
                if (pipeline_running_.load(std::memory_order_acquire))
                {
                    requestPipelineRestart("encode_pipeline_init_failed");
                }
                return;
            }
            if (!pipeline_running_.load(std::memory_order_acquire))
            {
                return;
            }
            if (!initializeEncoder())
            {
                if (pipeline_running_.load(std::memory_order_acquire))
                {
                    requestPipelineRestart("encode_pipeline_init_failed");
                }
                return;
            }
            if (!pipeline_running_.load(std::memory_order_acquire))
            {
                return;
            }

            if (!yuyv_direct_enabled_)
            {
                decoder_ = std::make_shared<trb::video::VideoDecoder>();
                if (!decoder_->initialize(decoder_config_))
                {
                    RCLCPP_WARN(rclcpp::get_logger("teleop_robot_bridge.video"), "[ENCODE] VideoDecoder initialize failed");
                    decoder_.reset();
                }
            }
            if (!pipeline_running_.load(std::memory_order_acquire))
            {
                return;
            }

            // Once the pipeline is up, spawn the capture thread that pumps
            // MJPEG frames from V4L2 into the decoder, and the dedicated
            // encoder DQ thread that long-blocks on the encoder capture plane
            // so encoded AUs are forwarded the moment NVENC signals readiness
            // (no dependency on this thread's loop cadence).
            if ((yuyv_direct_enabled_ || decoder_) && (converter_ || undistorter_) && encoder_)
            {
                if (!yuyv_direct_enabled_)
                {
                    assignWorkerThread(capture_thread_, std::thread(&Impl::captureThreadMain, this), "capture", true);
                }
                assignWorkerThread(encoder_dq_thread_, std::thread(&Impl::encoderDqThreadMain, this), "encoder_dq", true);
            }
            else
            {
                if (pipeline_running_.load(std::memory_order_acquire))
                {
                    requestPipelineRestart("encode_pipeline_init_failed");
                }
                return;
            }

            if (yuyv_direct_enabled_)
            {
                encodeYuyvDirectLoop();
                return;
            }

            encodeDecodedLoop();
        }

        void encodeDecodedLoop()
        {
            // Encode-thread main loop: drain decoder capture plane, run CUDA
            // postprocess, submit to encoder, and drain encoder input done.
            while (pipeline_running_.load(std::memory_order_acquire))
            {
                // Reap completed encoder input buffers (returns fds to the
                // converter output pool). Encoded AU dequeue runs on a
                // dedicated thread (encoder_dq_thread_) for low-jitter delivery.
                if (encoder_)
                {
                    encoder_->drainInputDone();
                }

                if (!decoder_ || !encoder_ || (!converter_ && !undistorter_))
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

                int downstream_fd = -1;
                if (!postprocessYuv422Fd(dec_frame.yuv_dmabuf_fd,
                                         dec_frame.capture_timestamp_us,
                                         dec_frame.decode_us,
                                         downstream_fd))
                {
                    decoder_->requeueCapture(dec_frame);
                    continue;
                }

                // Decoder YUV buffer is no longer needed after either the
                // fused CUDA path or the converter path has synchronously
                // consumed it.
                decoder_->requeueCapture(dec_frame);

                (void)onConverterOutput(downstream_fd, dec_frame.capture_timestamp_us);
            }
        }

        void encodeYuyvDirectLoop()
        {
            if (!capturer_ || !capturer_->start())
            {
                RCLCPP_ERROR(rclcpp::get_logger("teleop_robot_bridge.video"),
                             "Failed to start V4L2 YUYV DMABUF capturer");
                requestPipelineRestart("yuyv_direct_capture_start_failed");
                return;
            }

            RCLCPP_INFO(rclcpp::get_logger("teleop_robot_bridge.video"),
                        "V4L2 YUYV direct capturer started (warmup_drop_frames=%d)",
                        v4l2_warmup_drop_frames_);

            const auto warmup_start = std::chrono::steady_clock::now();
            auto last_successful_dequeue = warmup_start;
            int warmup_dropped_frames = 0;
            bool warmup_done = (v4l2_warmup_drop_frames_ == 0);
            bool warned_no_frames = false;

            while (pipeline_running_.load(std::memory_order_acquire))
            {
                if (encoder_)
                {
                    encoder_->drainInputDone();
                }

                trb::video::VideoV4L2Capturer::Frame frame;
                if (!capturer_ || !capturer_->dequeue(frame, 100))
                {
                    const auto now_tp = std::chrono::steady_clock::now();
                    if (!warned_no_frames && (now_tp - last_successful_dequeue) >= std::chrono::seconds(2))
                    {
                        const double stalled_sec = std::chrono::duration<double>(now_tp - last_successful_dequeue).count();
                        RCLCPP_WARN(rclcpp::get_logger("teleop_robot_bridge.video"),
                                    "V4L2 YUYV direct capturer started but no frames dequeued for %.1fs",
                                    stalled_sec);
                        warned_no_frames = true;
                    }
                    continue;
                }

                const auto now_tp = std::chrono::steady_clock::now();
                last_successful_dequeue = now_tp;
                warned_no_frames = false;

                if (!warmup_done)
                {
                    (void)capturer_->requeue(frame.v4l2_buf);
                    if (++warmup_dropped_frames >= v4l2_warmup_drop_frames_)
                    {
                        const double warmup_elapsed_ms = static_cast<double>(
                            std::chrono::duration_cast<std::chrono::microseconds>(
                                now_tp - warmup_start).count()) / 1000.0;
                        RCLCPP_INFO(rclcpp::get_logger("teleop_robot_bridge.video"),
                                    "V4L2 YUYV direct warmup complete: dropped=%d elapsed=%.1fms",
                                    warmup_dropped_frames,
                                    warmup_elapsed_ms);
                        warmup_done = true;
                    }
                    continue;
                }

                const uint64_t capture_steady_us = static_cast<uint64_t>(
                    std::chrono::duration_cast<std::chrono::microseconds>(
                        now_tp.time_since_epoch()).count());

                stats_capture_frames_.fetch_add(1, std::memory_order_relaxed);
                (void)encode_frame_in_count_.fetch_add(1, std::memory_order_relaxed);

                int downstream_fd = -1;
                if (frame.dmabuf_fd < 0 ||
                    !postprocessYuv422Fd(frame.dmabuf_fd, capture_steady_us, 0, downstream_fd))
                {
                    (void)capturer_->requeue(frame.v4l2_buf);
                    continue;
                }

                (void)capturer_->requeue(frame.v4l2_buf);
                (void)onConverterOutput(downstream_fd, capture_steady_us);
            }
        }

        // Capture-thread main loop: pull one MJPEG frame from V4L2, submit to
        // decoder, requeue V4L2. Keeps decoder input continuously fed without
        // blocking the encode pipeline on camera frame arrival.
        void captureThreadMain()
        {
            if (!pipeline_running_.load(std::memory_order_acquire))
            {
                return;
            }
            if (!capturer_ || !capturer_->start())
            {
                RCLCPP_ERROR(rclcpp::get_logger("teleop_robot_bridge.video"), "Failed to start V4L2 capturer");
                return;
            }
            RCLCPP_INFO(rclcpp::get_logger("teleop_robot_bridge.video"), "V4L2 capturer started (warmup_drop_frames=%d)",
                     v4l2_warmup_drop_frames_);

            // Drop the first N camera frames before feeding the decoder. This
            // skips V4L2 STREAMON jitter and the camera's AE/AWB convergence
            // window.
            const auto warmup_start = std::chrono::steady_clock::now();
            auto last_successful_dequeue = warmup_start;
            int warmup_dropped_frames = 0;
            bool warmup_done = (v4l2_warmup_drop_frames_ == 0);
            bool warned_no_frames = false;

            while (pipeline_running_.load(std::memory_order_acquire))
            {
                trb::video::VideoV4L2Capturer::Frame frame;
                if (!capturer_ || !capturer_->dequeue(frame, 100))
                {
                    const auto now_tp = std::chrono::steady_clock::now();
                    if (!warned_no_frames && (now_tp - last_successful_dequeue) >= std::chrono::seconds(2))
                    {
                        const double stalled_sec = std::chrono::duration<double>(now_tp - last_successful_dequeue).count();
                        RCLCPP_WARN(rclcpp::get_logger("teleop_robot_bridge.video"),
                                    "V4L2 capturer started but no frames dequeued for %.1fs",
                                    stalled_sec);
                        warned_no_frames = true;
                    }
                    continue;
                }

                const auto now_tp = std::chrono::steady_clock::now();
                last_successful_dequeue = now_tp;
                warned_no_frames = false;

                if (!warmup_done)
                {
                    (void)capturer_->requeue(frame.v4l2_buf);
                    if (++warmup_dropped_frames >= v4l2_warmup_drop_frames_)
                    {
                        const double warmup_elapsed_ms = static_cast<double>(
                            std::chrono::duration_cast<std::chrono::microseconds>(
                                now_tp - warmup_start).count()) / 1000.0;
                        RCLCPP_INFO(rclcpp::get_logger("teleop_robot_bridge.video"), "V4L2 capturer warmup complete: dropped=%d elapsed=%.1fms",
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
                bool restart_after_requeue = false;
                std::string restart_reason;

                if (frame.data != nullptr && frame.size > 0 && decoder_)
                {
                    const auto status = decoder_->submitDetailed(frame.data, frame.size,
                                                                 frame_id, capture_steady_us,
                                                                 capture_steady_us);
                    if (status == trb::video::VideoDecoder::SubmitStatus::kQueued)
                    {
                        consecutive_bad_decode_frames_.store(0, std::memory_order_relaxed);
                    }
                    else
                    {
                        stats_decoder_drops_.fetch_add(1, std::memory_order_relaxed);
                        if (isBadDecodeStatus(status))
                        {
                            const int streak = consecutive_bad_decode_frames_.fetch_add(1, std::memory_order_relaxed) + 1;
                            if (restart_on_bad_frames_ && streak >= consecutive_bad_frame_threshold_)
                            {
                                restart_after_requeue = true;
                                restart_reason = "consecutive_bad_mjpeg_frames status=" +
                                                 std::string(submitStatusToString(status)) +
                                                 " streak=" + std::to_string(streak);
                            }
                        }
                    }
                }

                (void)capturer_->requeue(frame.v4l2_buf);
                if (restart_after_requeue)
                {
                    RCLCPP_WARN(rclcpp::get_logger("teleop_robot_bridge.video"),
                                "VideoStreamManager: %d consecutive MJPEG decode failures; requesting pipeline restart",
                                consecutive_bad_decode_frames_.load(std::memory_order_relaxed));
                    requestPipelineRestart(restart_reason);
                    break;
                }
            }
        }

        // Dedicated encoder capture-plane DQ thread.
        // Long-blocks on encoder_->dequeueOne() so encoded access units are
        // delivered to onEncoderPacket() the moment NVENC signals readiness,
        // independent of the encode_thread_ main loop's cadence.
        void encoderDqThreadMain()
        {
            RCLCPP_INFO(rclcpp::get_logger("teleop_robot_bridge.video"), "[ENCODE] encoder DQ thread started");
            rclcpp::Clock steady_clock(RCL_STEADY_TIME);
            while (pipeline_running_.load(std::memory_order_acquire))
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

                const size_t packet_size = pkt.size;
                const bool packet_keyframe = pkt.keyframe;
                const auto packet_start = std::chrono::steady_clock::now();
                onEncoderPacket(pkt.data, pkt.size, pkt.timestamp_us, pkt.keyframe);
                const auto before_release = std::chrono::steady_clock::now();
                enc->releaseCapture(pkt);
                const auto after_release = std::chrono::steady_clock::now();

                const double callback_hold_ms = static_cast<double>(
                    std::chrono::duration_cast<std::chrono::microseconds>(before_release - packet_start).count()) / 1000.0;
                if (callback_hold_ms > 5.0)
                {
                    const double release_ms = static_cast<double>(
                        std::chrono::duration_cast<std::chrono::microseconds>(after_release - before_release).count()) / 1000.0;
                    RCLCPP_WARN_THROTTLE(rclcpp::get_logger("teleop_robot_bridge.video"), steady_clock, 1000,
                                         "[ENCODE] output callback held capture buffer for %.2fms before release (release=%.2fms size=%zu key=%d)",
                                         callback_hold_ms, release_ms, packet_size, packet_keyframe ? 1 : 0);
                }
            }
            RCLCPP_INFO(rclcpp::get_logger("teleop_robot_bridge.video"), "[ENCODE] encoder DQ thread exiting");
        }

        // ==================== Eye Image Thread ====================
        // Responsibility: Receive NV12 fd from encode thread, CUDA crop/scale/RGB, publish ROS images.
        void eyeImageThreadMain()
        {
            if (!pipeline_running_.load(std::memory_order_acquire))
            {
                return;
            }
            RCLCPP_INFO(rclcpp::get_logger("teleop_robot_bridge.video"), "[EYE_IMAGE] Thread started (CUDA shared fd mode)");

            // Initialize eye image publisher (no independent decoder)
            eye_image_publisher_ = std::make_shared<trb::video::EyeImagePublisher>();
            trb::video::EyeImagePublisher::Config eye_cfg;
            eye_cfg.output_width = static_cast<uint32_t>(eye_image_output_width_);
            eye_cfg.output_height = static_cast<uint32_t>(eye_image_output_height_);
            eye_cfg.left_topic = eye_image_left_topic_;
            eye_cfg.right_topic = eye_image_right_topic_;
            eye_cfg.enabled = true;

            if (!eye_image_publisher_->initialize(nh_, frame_width_, frame_height_, eye_cfg))
            {
                RCLCPP_WARN(rclcpp::get_logger("teleop_robot_bridge.video"), "[EYE_IMAGE] EyeImagePublisher initialize failed");
                eye_image_publisher_.reset();
                return;
            }

            eye_image_publisher_ready_.store(true, std::memory_order_release);
            updateEyeImageSubscriberState();

            RCLCPP_INFO(rclcpp::get_logger("teleop_robot_bridge.video"), "[EYE_IMAGE] Publisher initialized: %ux%u -> %ux%u max_fps=%d",
                     frame_width_, frame_height_, eye_cfg.output_width, eye_cfg.output_height,
                     eye_image_max_fps_);

            uint64_t frames_processed = 0;
            uint64_t frames_dropped = 0;

            while (pipeline_running_.load(std::memory_order_acquire))
            {
                NV12FrameData frame_data;
                updateEyeImageSubscriberState();
                
                // Wait for NV12 frame fd from encode thread.
                {
                    std::unique_lock<std::mutex> lk(nv12_queue_mutex_);
                    nv12_queue_cv_.wait_for(lk, std::chrono::milliseconds(100), [this] {
                        return !pipeline_running_.load(std::memory_order_acquire) || !nv12_queue_.empty();
                    });

                    if (!pipeline_running_.load(std::memory_order_acquire))
                    {
                        while (!nv12_queue_.empty())
                        {
                            nv12_queue_.pop();
                        }
                        eye_image_inflight_.store(false, std::memory_order_relaxed);
                        break;
                    }
                    
                    if (nv12_queue_.empty())
                        continue;
                    
                    // Skip to latest if queue is backing up, release skipped buffers
                    while (nv12_queue_.size() > 1)
                    {
                        frames_dropped++;
                        eye_image_drop_queue_.fetch_add(1, std::memory_order_relaxed);
                        nv12_queue_.pop();
                    }
                    
                    frame_data = nv12_queue_.front();
                    nv12_queue_.pop();
                }

                if (!frame_data.frame || !eye_image_publisher_->isEnabled() || !eye_image_publisher_->hasSubscribers())
                {
                    eye_image_inflight_.store(false, std::memory_order_release);
                    continue;
                }

                NvBufSurface* src_surface = nullptr;
                {
                    std::lock_guard<std::mutex> lk(getNvBufMutex());
                    if (NvBufSurfaceFromFd(frame_data.frame->fd, reinterpret_cast<void **>(&src_surface)) != 0 || !src_surface)
                    {
                        RCLCPP_WARN_THROTTLE(rclcpp::get_logger("teleop_robot_bridge.video"), *rclcpp::Clock::make_shared(), static_cast<uint64_t>((5.0) * 1000.0), "[EYE_IMAGE] NvBufSurfaceFromFd failed for fd=%d", frame_data.frame->fd);
                        eye_image_inflight_.store(false, std::memory_order_release);
                        continue;
                    }
                }

                const auto process_start = std::chrono::steady_clock::now();
                eye_image_publisher_->processFrame(src_surface, frame_data.frame->timestamp_us);
                const auto process_end = std::chrono::steady_clock::now();
                eye_image_process_us_total_.fetch_add(
                    static_cast<uint64_t>(std::chrono::duration_cast<std::chrono::microseconds>(process_end - process_start).count()),
                    std::memory_order_relaxed);
                frames_processed++;
                eye_image_inflight_.store(false, std::memory_order_release);

                if (frames_processed <= 5 || (frames_processed % 300 == 0))
                {
                    const uint64_t enqueued = eye_image_enqueued_.load(std::memory_order_relaxed);
                    const uint64_t busy = eye_image_skip_busy_.load(std::memory_order_relaxed);
                    const uint64_t rate = eye_image_skip_rate_limit_.load(std::memory_order_relaxed);
                    const uint64_t no_sub = eye_image_skip_no_sub_.load(std::memory_order_relaxed);
                    const double avg_ms = frames_processed > 0
                        ? static_cast<double>(eye_image_process_us_total_.load(std::memory_order_relaxed)) /
                              static_cast<double>(frames_processed) / 1000.0
                        : 0.0;
                    RCLCPP_INFO(rclcpp::get_logger("teleop_robot_bridge.video"), "[EYE_IMAGE] Processed %" PRIu64 " frames, dropped %" PRIu64
                             " enq=%" PRIu64 " skip_busy=%" PRIu64 " skip_rate=%" PRIu64
                             " skip_no_sub=%" PRIu64 " avg_process=%.2fms",
                             frames_processed, frames_dropped, enqueued, busy, rate, no_sub, avg_ms);
                }
            }

            eye_image_publisher_ready_.store(false, std::memory_order_release);
            eye_image_has_subscribers_.store(false, std::memory_order_release);
            eye_image_publisher_->stop();
            RCLCPP_INFO(rclcpp::get_logger("teleop_robot_bridge.video"), "[EYE_IMAGE] Thread stopped (processed=%" PRIu64 ", dropped=%" PRIu64 ")", 
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
                RCLCPP_WARN_THROTTLE(rclcpp::get_logger("teleop_robot_bridge.video"), *rclcpp::Clock::make_shared(), static_cast<uint64_t>((5.0) * 1000.0), "VideoStreamManager: input done for unknown fd=%d; releasing directly", dmabuf_fd);
                std::shared_ptr<trb::video::VideoUndistorter> undistorter;
                std::shared_ptr<trb::video::VideoConverter> converter;
                {
                    std::lock_guard<std::mutex> component_lk(pipeline_component_mutex_);
                    undistorter = undistorter_;
                    converter = converter_;
                }
                if (undistorter)
                {
                    undistorter->releaseFd(dmabuf_fd);
                }
                else if (converter)
                {
                    converter->releaseFd(dmabuf_fd);
                }
            }
        }

        void onEncoderPacket(const uint8_t *data, size_t size, uint64_t timestamp_us, bool keyframe)
        {
            const uint64_t packet_count = enc_frame_count_.fetch_add(1, std::memory_order_relaxed) + 1;

            const auto now_tp = std::chrono::steady_clock::now();
            const int64_t now_us = std::chrono::duration_cast<std::chrono::microseconds>(now_tp.time_since_epoch()).count();

            int64_t cost_us = -1;
            int64_t enc_submit_steady_us = -1;
            {
                std::lock_guard<std::mutex> lk(enc_timing_mutex_);
                auto it = enc_start_steady_us_by_ts_.find(timestamp_us);
                if (it != enc_start_steady_us_by_ts_.end())
                {
                    enc_submit_steady_us = it->second;
                    cost_us = now_us - enc_submit_steady_us;
                    enc_start_steady_us_by_ts_.erase(it);
                }
            }

            // NVENC warmup: drop encoded output + backlogged stale frames until
            // the deadline passes. This prevents polluting FEC/send queues with
            // 200-400 ms old packets during cold start.
            if (warmup_active_.load(std::memory_order_relaxed))
            {
                const int64_t deadline_us = warmup_deadline_us_.load(std::memory_order_relaxed);
                const bool packet_submitted_during_warmup = enc_submit_steady_us >= 0 && enc_submit_steady_us < deadline_us;
                if (now_us < deadline_us || packet_submitted_during_warmup)
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
                    stats_fused_decode_us_total_.store(0, std::memory_order_relaxed);
                    if (converter_)
                    {
                        (void)converter_->consumeStats();
                    }
                    if (undistorter_)
                    {
                        (void)undistorter_->consumeStats();
                    }
                    RCLCPP_INFO(rclcpp::get_logger("teleop_robot_bridge.video"), "VideoStreamManager: NVENC warmup complete, dropped %lu stale packets (warmup_ms=%d)",
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

            if (recorder_)
            {
                const uint64_t record_index = recorded_packet_count_.fetch_add(1, std::memory_order_relaxed);
                recorder_->writeFrame(record_index, data, size, keyframe);
            }

            if (packet_count <= 5 || (packet_count % 300) == 0)
            {
                RCLCPP_INFO(rclcpp::get_logger("teleop_robot_bridge.video"),
                            "[ENCODE] packet=%" PRIu64 " size=%zu key=%d ts=%" PRIu64 " hash=0x%08x",
                            packet_count, size, keyframe ? 1 : 0,
                            static_cast<uint64_t>(timestamp_us), fnv1a32(data, size));
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
            std::shared_ptr<trb::video::VideoUndistorter> undistorter_owner;
            std::shared_ptr<trb::video::VideoConverter> converter_owner;
            std::shared_ptr<trb::video::VideoScaler> scaler_owner;
            {
                std::lock_guard<std::mutex> component_lk(pipeline_component_mutex_);
                undistorter_owner = undistorter_;
                converter_owner = converter_;
                scaler_owner = scaler_;
            }

            return std::shared_ptr<SharedNv12Frame>(
                new SharedNv12Frame{dmabuf_fd, timestamp_us},
                [undistorter_owner, converter_owner, scaler_owner](SharedNv12Frame* frame) {
                    if (frame)
                    {
                        if (frame->fd >= 0)
                        {
                            // The downstream fd belongs to the scaler whenever
                            // scaling is enabled. The full-resolution
                            // conversion/undistortion fd was released as soon
                            // as the synchronous scale completed.
                            if (scaler_owner)
                            {
                                scaler_owner->releaseFd(frame->fd);
                            }
                            else if (undistorter_owner)
                            {
                                undistorter_owner->releaseFd(frame->fd);
                            }
                            else if (converter_owner)
                            {
                                converter_owner->releaseFd(frame->fd);
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

            const int64_t now_us = std::chrono::duration_cast<std::chrono::microseconds>(
                                       std::chrono::steady_clock::now().time_since_epoch())
                                       .count();
            if (eye_image_min_interval_us_ > 0)
            {
                const int64_t last_us = eye_image_last_enqueue_steady_us_.load(std::memory_order_relaxed);
                if (last_us > 0 && now_us - last_us < eye_image_min_interval_us_)
                {
                    eye_image_skip_rate_limit_.fetch_add(1, std::memory_order_relaxed);
                    return;
                }
            }

            bool expected = false;
            if (!eye_image_inflight_.compare_exchange_strong(expected, true, std::memory_order_acq_rel))
            {
                eye_image_skip_busy_.fetch_add(1, std::memory_order_relaxed);
                return;
            }

            {
                std::lock_guard<std::mutex> lk(nv12_queue_mutex_);
                while (!nv12_queue_.empty())
                {
                    nv12_queue_.pop();
                    eye_image_drop_queue_.fetch_add(1, std::memory_order_relaxed);
                }
                nv12_queue_.push({frame});
            }

            eye_image_last_enqueue_steady_us_.store(now_us, std::memory_order_relaxed);
            eye_image_enqueued_.fetch_add(1, std::memory_order_relaxed);
            nv12_queue_cv_.notify_one();
        }

        void assignWorkerThread(std::thread &slot, std::thread thread, const char *name, bool require_pipeline_running)
        {
            std::thread previous;
            std::thread rejected;
            {
                std::lock_guard<std::mutex> lk(worker_thread_mutex_);
                if (slot.joinable())
                {
                    previous = std::move(slot);
                }
                if (!require_pipeline_running || pipeline_running_.load(std::memory_order_acquire))
                {
                    slot = std::move(thread);
                }
                else
                {
                    rejected = std::move(thread);
                }
            }
            joinThread(previous, name);
            joinThread(rejected, name);
        }

        void joinWorkerThread(std::thread &slot, const char *name)
        {
            std::thread thread;
            {
                std::lock_guard<std::mutex> lk(worker_thread_mutex_);
                if (!slot.joinable())
                {
                    return;
                }
                thread = std::move(slot);
            }
            joinThread(thread, name);
        }

    private:
        rclcpp::Node &nh_;

        // Capture components
        std::shared_ptr<trb::video::VideoV4L2Capturer> capturer_;

        // Encode pipeline components (created in encode_thread_)
        std::shared_ptr<trb::video::VideoDecoder> decoder_;
        std::shared_ptr<trb::video::VideoConverter> converter_;
        std::shared_ptr<trb::video::VideoUndistorter> undistorter_;
        std::shared_ptr<trb::video::VideoScaler> scaler_;
        std::shared_ptr<trb::video::VideoEncoder> encoder_;
        std::shared_ptr<trb::video::VideoRecorder> recorder_;

        // Eye image pipeline (created in eye_image_thread_)
        std::shared_ptr<trb::video::EyeImagePublisher> eye_image_publisher_;

        // Saved configs for deferred initialization
        trb::video::VideoV4L2Capturer::Config capturer_config_;
        trb::video::VideoDecoder::Config decoder_config_;
        trb::video::VideoConverter::Config converter_config_;
        trb::video::VideoUndistorter::Config undistorter_config_;
        trb::video::VideoScaler::Config scaler_config_;
        bool undistort_enabled_{false};
        bool scale_enabled_{false};
        bool yuyv_direct_enabled_{false};
        trb::video::VideoEncoder::Config encoder_config_;
        trb::video::VideoRecorder::Config recorder_config_;

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
        int eye_image_max_fps_{10};
        int64_t eye_image_min_interval_us_{100000};
        std::string eye_image_left_topic_;
        std::string eye_image_right_topic_;

        // Frame counters
        std::atomic<uint64_t> encode_frame_in_count_{0};
        std::atomic<uint64_t> enc_frame_count_{0};
        std::atomic<uint64_t> recorded_packet_count_{0};
        std::atomic<uint64_t> stats_capture_frames_{0};
        std::atomic<uint64_t> stats_decode_frames_{0};
        std::atomic<uint64_t> stats_encode_frames_{0};
        std::atomic<uint64_t> stats_decoder_drops_{0};
        std::atomic<uint64_t> stats_encoder_submit_failures_{0};
        std::atomic<uint64_t> stats_encode_us_total_{0};
        std::atomic<uint64_t> stats_fused_decode_us_total_{0};
        std::atomic<bool> eye_image_publisher_ready_{false};
        std::atomic<bool> eye_image_has_subscribers_{false};
        std::atomic<bool> eye_image_inflight_{false};
        std::atomic<int64_t> eye_image_last_enqueue_steady_us_{0};
        std::atomic<uint64_t> eye_image_enqueued_{0};
        std::atomic<uint64_t> eye_image_skip_not_ready_{0};
        std::atomic<uint64_t> eye_image_skip_no_sub_{0};
        std::atomic<uint64_t> eye_image_skip_rate_limit_{0};
        std::atomic<uint64_t> eye_image_skip_busy_{0};
        std::atomic<uint64_t> eye_image_drop_queue_{0};
        std::atomic<uint64_t> eye_image_process_us_total_{0};

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
        std::mutex pipeline_component_mutex_;

        std::atomic<bool> running_{false};
        std::atomic<bool> pipeline_running_{false};

        bool restart_on_bad_frames_{true};
        int consecutive_bad_frame_threshold_{5};
        int restart_backoff_ms_{500};
        std::atomic<int> consecutive_bad_decode_frames_{0};
        std::mutex restart_mutex_;
        std::condition_variable restart_cv_;
        bool restart_requested_{false};
        std::string restart_reason_;

        // Worker threads
        std::mutex worker_thread_mutex_;
        std::thread encode_thread_;
        std::thread capture_thread_;
        std::thread encoder_dq_thread_;
        std::thread eye_image_thread_;
        std::thread restart_supervisor_thread_;
        
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

    VideoStreamManager::VideoStreamManager(rclcpp::Node &nh)
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

    bool VideoStreamManager::requestKeyframe()
    {
        return impl_->requestKeyframe();
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
