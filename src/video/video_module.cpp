#include "video/video_module.hpp"

#include <algorithm>
#include <cctype>
#include <utility>

namespace trb::video
{
    namespace
    {
        enum class StreamFormat
        {
            AnnexB,
            Avcc,
            Unknown,
        };

        template <typename T>
        T declareOrGet(rclcpp::Node &node, const std::string &name, const T &default_value)
        {
            if (!node.has_parameter(name))
            {
                return node.declare_parameter<T>(name, default_value);
            }
            T value{};
            node.get_parameter(name, value);
            return value;
        }

        bool hasAnnexBStartCode(const uint8_t *data, size_t size)
        {
            if (!data || size < 4)
            {
                return false;
            }
            for (size_t index = 0; index + 3 < size; ++index)
            {
                if (data[index] == 0x00 && data[index + 1] == 0x00 && data[index + 2] == 0x01)
                {
                    return true;
                }
                if (index + 4 < size && data[index] == 0x00 && data[index + 1] == 0x00 &&
                    data[index + 2] == 0x00 && data[index + 3] == 0x01)
                {
                    return true;
                }
            }
            return false;
        }

        bool looksLikeAvcc(const uint8_t *data, size_t size)
        {
            if (!data || size < 8)
            {
                return false;
            }
            size_t offset = 0;
            int nal_count = 0;
            while (offset + 4 <= size && nal_count < 3)
            {
                const uint32_t nal_size = (static_cast<uint32_t>(data[offset]) << 24) |
                                          (static_cast<uint32_t>(data[offset + 1]) << 16) |
                                          (static_cast<uint32_t>(data[offset + 2]) << 8) |
                                          static_cast<uint32_t>(data[offset + 3]);
                offset += 4;
                if (nal_size == 0 || offset + nal_size > size)
                {
                    return false;
                }
                offset += nal_size;
                ++nal_count;
            }
            return nal_count > 0;
        }

        StreamFormat detectStreamFormat(const uint8_t *data, size_t size)
        {
            if (hasAnnexBStartCode(data, size))
            {
                return StreamFormat::AnnexB;
            }
            if (looksLikeAvcc(data, size))
            {
                return StreamFormat::Avcc;
            }
            return StreamFormat::Unknown;
        }

        template <typename Fn>
        void forEachNal(const uint8_t *data, size_t size, StreamFormat format, Fn &&fn)
        {
            if (!data || size < 1)
            {
                return;
            }
            if (format == StreamFormat::AnnexB)
            {
                auto is_start_code = [&](size_t index, size_t &start_code_len) -> bool {
                    if (index + 3 <= size && data[index] == 0x00 && data[index + 1] == 0x00 && data[index + 2] == 0x01)
                    {
                        start_code_len = 3;
                        return true;
                    }
                    if (index + 4 <= size && data[index] == 0x00 && data[index + 1] == 0x00 &&
                        data[index + 2] == 0x00 && data[index + 3] == 0x01)
                    {
                        start_code_len = 4;
                        return true;
                    }
                    return false;
                };

                size_t index = 0;
                while (index + 3 < size)
                {
                    size_t start_code_len = 0;
                    if (!is_start_code(index, start_code_len))
                    {
                        ++index;
                        continue;
                    }
                    index += start_code_len;
                    const size_t nal_start = index;
                    size_t nal_end = size;
                    while (index + 3 < size)
                    {
                        size_t next_start_code_len = 0;
                        if (is_start_code(index, next_start_code_len))
                        {
                            nal_end = index;
                            break;
                        }
                        ++index;
                    }
                    if (nal_end > nal_start)
                    {
                        fn(data + nal_start, nal_end - nal_start);
                    }
                    index = nal_end;
                }
                return;
            }

            if (format == StreamFormat::Avcc)
            {
                size_t offset = 0;
                while (offset + 4 <= size)
                {
                    const uint32_t nal_size = (static_cast<uint32_t>(data[offset]) << 24) |
                                              (static_cast<uint32_t>(data[offset + 1]) << 16) |
                                              (static_cast<uint32_t>(data[offset + 2]) << 8) |
                                              static_cast<uint32_t>(data[offset + 3]);
                    offset += 4;
                    if (nal_size == 0 || offset + nal_size > size)
                    {
                        break;
                    }
                    fn(data + offset, nal_size);
                    offset += nal_size;
                }
            }
        }

        const char *codecToString(VideoModule::Config::Codec codec)
        {
            return codec == VideoModule::Config::Codec::kH265 ? "h265" : "h264";
        }
    } // namespace

    VideoModule::Config VideoModule::Config::configFromRosParam(rclcpp::Node &node)
    {
        Config config;
        const auto profile = declareOrGet<std::vector<int64_t>>(node, "video.profile", std::vector<int64_t>{3840, 1520, 30});
        if (profile.size() == 3 && profile[0] > 0 && profile[1] > 0 && profile[2] > 0)
        {
            config.width = static_cast<uint32_t>(profile[0]);
            config.height = static_cast<uint32_t>(profile[1]);
            config.fps = static_cast<uint32_t>(profile[2]);
        }

        const bool sbs_crop_enabled = declareOrGet<bool>(node, "video.sbs_crop.enabled", false);
        const int64_t sbs_crop_margin_param = declareOrGet<int64_t>(node, "video.sbs_crop.margin_px", 0);
        if (sbs_crop_enabled && sbs_crop_margin_param > 0)
        {
            const uint32_t margin = static_cast<uint32_t>(sbs_crop_margin_param);
            const uint32_t half_width = config.width / 2;
            if (half_width > margin * 2 && config.width > margin * 4)
            {
                config.width = (config.width - margin * 4) & ~1u;
            }
        }

        const bool scale_enabled = declareOrGet<bool>(node, "video.scale.enabled", false);
        const int64_t scale_width_param = declareOrGet<int64_t>(node, "video.scale.width", 3200);
        const int64_t scale_height_param = declareOrGet<int64_t>(node, "video.scale.height", 1200);
        if (scale_enabled && scale_width_param > 0 && scale_height_param > 0 &&
            (scale_width_param % 2) == 0 && (scale_height_param % 2) == 0)
        {
            const uint64_t source_ratio = static_cast<uint64_t>(config.width) *
                                          static_cast<uint64_t>(scale_height_param);
            const uint64_t target_ratio = static_cast<uint64_t>(config.height) *
                                          static_cast<uint64_t>(scale_width_param);
            if (source_ratio == target_ratio)
            {
                config.width = static_cast<uint32_t>(scale_width_param);
                config.height = static_cast<uint32_t>(scale_height_param);
            }
            else
            {
                RCLCPP_ERROR(node.get_logger(),
                             "video.scale must preserve aspect ratio: input=%ux%u target=%ldx%ld; "
                             "VideoConfig will keep the input dimensions",
                             config.width,
                             config.height,
                             static_cast<long>(scale_width_param),
                             static_cast<long>(scale_height_param));
            }
        }
        else if (scale_enabled)
        {
            RCLCPP_ERROR(node.get_logger(),
                         "Invalid video.scale dimensions=%ldx%ld; VideoConfig will keep the input dimensions",
                         static_cast<long>(scale_width_param),
                         static_cast<long>(scale_height_param));
        }

        std::string codec = declareOrGet<std::string>(node, "video.encoder.codec", "h264");
        std::transform(codec.begin(), codec.end(), codec.begin(), [](unsigned char value) {
            return static_cast<char>(std::tolower(value));
        });
        if (codec == "h265" || codec == "hevc")
        {
            config.codec = Config::Codec::kH265;
        }
        else if (codec == "h264" || codec == "avc")
        {
            config.codec = Config::Codec::kH264;
        }
        else
        {
            RCLCPP_WARN(node.get_logger(), "Unknown video.encoder.codec='%s', falling back to h264", codec.c_str());
            config.codec = Config::Codec::kH264;
        }
        return config;
    }

    VideoModule::VideoModule(rclcpp::Node &node, Config config)
        : node_(node), logger_(node.get_logger()), config_(std::move(config))
    {
        manager_ = std::make_unique<VideoStreamManager>(node_);
        manager_->setEncodedFrameCallback([this](const uint8_t *data, size_t size, uint64_t timestamp_us, bool keyframe) {
            onEncodedFrame(data, size, timestamp_us, keyframe);
        });
        // Preserve the ROS1 behavior: cache codec parameter sets as soon as
        // the encoder captures them, even if the corresponding early encoded
        // packets are later dropped by the warmup filter.
        manager_->setSpsPpsCallback([this](const std::vector<uint8_t> &sps,
                                           const std::vector<uint8_t> &pps,
                                           const std::vector<uint8_t> &vps) {
            {
                std::lock_guard<std::mutex> lock(sps_pps_mutex_);
                cached_sps_ = sps;
                cached_pps_ = pps;
                cached_vps_ = vps;
            }
            RCLCPP_INFO(logger_, "SPS/PPS captured: sps=%zu bytes pps=%zu bytes vps=%zu bytes",
                        sps.size(), pps.size(), vps.size());
            emitCachedParameterSetsIfReady();
        });
        RCLCPP_INFO(logger_, "VideoModule created: profile=%ux%u@%u codec=%s",
                    config_.width, config_.height, config_.fps, codecToString(config_.codec));
    }

    VideoModule::~VideoModule()
    {
        stop();
    }

    bool VideoModule::start()
    {
        if (!manager_)
        {
            return false;
        }
        if (manager_->isRunning())
        {
            emitCachedParameterSetsIfReady();
            return true;
        }
        RCLCPP_INFO(logger_, "VideoModule starting VideoStreamManager");
        if (!manager_->start())
        {
            RCLCPP_WARN(logger_, "VideoModule: VideoStreamManager start failed");
            return false;
        }
        RCLCPP_INFO(logger_, "VideoModule started");
        emitCachedParameterSetsIfReady();
        return true;
    }

    void VideoModule::stop()
    {
        if (manager_)
        {
            manager_->stop();
        }
    }

    bool VideoModule::isRunning() const
    {
        return manager_ && manager_->isRunning();
    }

    void VideoModule::setEncodedFrameCallback(EncodedFrameCallback callback)
    {
        encoded_callback_ = std::move(callback);
    }

    void VideoModule::setSpsPpsCallback(SpsPpsCallback callback)
    {
        sps_pps_callback_ = std::move(callback);
        emitCachedParameterSetsIfReady();
    }

    bool VideoModule::requestKeyframe()
    {
        return manager_ && manager_->requestKeyframe();
    }

    bool VideoModule::hasSpsPps() const
    {
        std::lock_guard<std::mutex> lock(sps_pps_mutex_);
        return parameterSetsReadyLocked();
    }

    bool VideoModule::getSpsPps(std::vector<uint8_t> &sps,
                                std::vector<uint8_t> &pps,
                                std::vector<uint8_t> &vps) const
    {
        std::lock_guard<std::mutex> lock(sps_pps_mutex_);
        if (!parameterSetsReadyLocked())
        {
            return false;
        }
        sps = cached_sps_;
        pps = cached_pps_;
        vps = cached_vps_;
        return true;
    }

    void VideoModule::clearSpsPps()
    {
        std::lock_guard<std::mutex> lock(sps_pps_mutex_);
        cached_sps_.clear();
        cached_pps_.clear();
        cached_vps_.clear();
        sps_pps_emitted_.store(false, std::memory_order_relaxed);
    }

    VideoModule::StatsSnapshot VideoModule::consumeStats()
    {
        if (!manager_)
        {
            return StatsSnapshot{};
        }
        return manager_->consumeStats();
    }

    void VideoModule::onEncodedFrame(const uint8_t *data, size_t size, uint64_t timestamp_us, bool keyframe)
    {
        updateParameterSets(data, size);
        if (encoded_callback_)
        {
            encoded_callback_(data, size, timestamp_us, keyframe);
        }
    }

    void VideoModule::updateParameterSets(const uint8_t *data, size_t size)
    {
        if (!data || size == 0)
        {
            return;
        }
        {
            std::lock_guard<std::mutex> lock(sps_pps_mutex_);
            if (parameterSetsReadyLocked())
            {
                return;
            }
        }

        const StreamFormat format = detectStreamFormat(data, size);
        if (format == StreamFormat::Unknown)
        {
            return;
        }

        bool became_ready = false;
        forEachNal(data, size, format, [&](const uint8_t *nal, size_t nal_size) {
            if (nal_size < 1)
            {
                return;
            }
            std::lock_guard<std::mutex> lock(sps_pps_mutex_);
            if (config_.codec == Config::Codec::kH265)
            {
                if (nal_size < 2)
                {
                    return;
                }
                const uint8_t nal_unit_type = static_cast<uint8_t>((nal[0] >> 1) & 0x3Fu);
                if (nal_unit_type == 32 && cached_vps_.empty())
                {
                    cached_vps_.assign(nal, nal + nal_size);
                }
                else if (nal_unit_type == 33 && cached_sps_.empty())
                {
                    cached_sps_.assign(nal, nal + nal_size);
                }
                else if (nal_unit_type == 34 && cached_pps_.empty())
                {
                    cached_pps_.assign(nal, nal + nal_size);
                }
            }
            else
            {
                const uint8_t nal_unit_type = static_cast<uint8_t>(nal[0] & 0x1Fu);
                if (nal_unit_type == 7 && cached_sps_.empty())
                {
                    cached_sps_.assign(nal, nal + nal_size);
                }
                else if (nal_unit_type == 8 && cached_pps_.empty())
                {
                    cached_pps_.assign(nal, nal + nal_size);
                }
            }
            became_ready = parameterSetsReadyLocked();
        });

        if (became_ready)
        {
            std::vector<uint8_t> sps;
            std::vector<uint8_t> pps;
            std::vector<uint8_t> vps;
            {
                std::lock_guard<std::mutex> lock(sps_pps_mutex_);
                sps = cached_sps_;
                pps = cached_pps_;
                vps = cached_vps_;
            }
            RCLCPP_INFO(logger_, "SPS/PPS captured: sps=%zu bytes pps=%zu bytes vps=%zu bytes",
                        sps.size(), pps.size(), vps.size());
            emitCachedParameterSetsIfReady();
        }
    }

    bool VideoModule::parameterSetsReadyLocked() const
    {
        if (config_.codec == Config::Codec::kH265)
        {
            return !cached_vps_.empty() && !cached_sps_.empty() && !cached_pps_.empty();
        }
        return !cached_sps_.empty() && !cached_pps_.empty();
    }

    void VideoModule::emitCachedParameterSetsIfReady()
    {
        SpsPpsCallback callback;
        std::vector<uint8_t> sps;
        std::vector<uint8_t> pps;
        std::vector<uint8_t> vps;
        {
            std::lock_guard<std::mutex> lock(sps_pps_mutex_);
            if (!parameterSetsReadyLocked() || !sps_pps_callback_)
            {
                return;
            }
            if (sps_pps_emitted_.exchange(true, std::memory_order_relaxed))
            {
                return;
            }
            callback = sps_pps_callback_;
            sps = cached_sps_;
            pps = cached_pps_;
            vps = cached_vps_;
        }
        callback(sps, pps, vps);
    }

} // namespace trb::video
