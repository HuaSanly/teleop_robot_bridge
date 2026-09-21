#include "include/main_node.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <chrono>
#include <cstdint>
#include <limits>
#include <optional>
#include <string>
#include <utility>
#include <vector>

#include <linux/videodev2.h>

#include "utils/teleop_logger.hpp"

namespace trb
{
    namespace
    {
        using AgvBatteryStateMsg = teleop_robot_bridge::msg::AgvBatteryState;
        using AgvDeviceStateMsg = teleop_robot_bridge::msg::AgvDeviceState;
        using TemperatureMsg = teleop_robot_bridge::msg::Temperature;
        using JointStateMsg = sensor_msgs::msg::JointState;

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

        bool hasValidContext(rclcpp::Node &node)
        {
            const auto base = node.get_node_base_interface();
            const auto context = base ? base->get_context() : rclcpp::Context::SharedPtr{};
            return context && context->is_valid();
        }

        bool canScheduleRosWork(rclcpp::Node &node, const std::atomic<bool> &shutting_down)
        {
            return !shutting_down.load(std::memory_order_acquire) && hasValidContext(node);
        }

        struct AtomicFlagClearGuard
        {
            explicit AtomicFlagClearGuard(std::atomic<bool> &flag_ref) : flag(flag_ref) {}
            ~AtomicFlagClearGuard()
            {
                flag.store(false, std::memory_order_release);
            }

            std::atomic<bool> &flag;
        };

        std::chrono::nanoseconds periodFromSeconds(double seconds)
        {
            return std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::duration<double>(std::max(0.1, seconds)));
        }

        std::chrono::nanoseconds highRatePeriodFromSeconds(double seconds)
        {
            return std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::duration<double>(std::max(0.005, seconds)));
        }

        uint64_t nowUnixMs()
        {
            return static_cast<uint64_t>(std::chrono::duration_cast<std::chrono::milliseconds>(
                                             std::chrono::system_clock::now().time_since_epoch())
                                             .count());
        }

        uint64_t stampToUnixMs(int32_t sec, uint32_t nanosec)
        {
            if (sec <= 0)
            {
                return nowUnixMs();
            }
            return static_cast<uint64_t>(sec) * 1000ULL + static_cast<uint64_t>(nanosec / 1000000U);
        }

        constexpr auto kVideoStartupKeyframeDelay = std::chrono::seconds(4);
        constexpr auto kVideoStartupTimeout = std::chrono::seconds(8);
        constexpr auto kVideoKeyframeRequestInterval = std::chrono::seconds(1);
        constexpr uint64_t kVideoBackpressureKeyframeThreshold = 100;
        constexpr uint8_t kPoseControllerPrefix = 0x02;
        constexpr uint8_t kAudioPrefix = 0x04;
        constexpr uint8_t kBody24RawPrefix = 0x09;

        constexpr uint16_t kFallbackBatteryVoltageMv = 48000;
        constexpr int16_t kFallbackBatteryCurrentMa = -1200;
        constexpr int16_t kFallbackBatteryTemperatureDeciC = 260;
        constexpr const char *kFallbackFaultCode = "ERROR_NONE";
        constexpr const char *kFallbackNetworkQuality = "uplink=good;q=0ms;blk=0;drop=0;tx=0.0/60.0";
        constexpr const char *kFallbackModel = "teleop_robot_demo";
        constexpr const char *kFallbackFirmwareVersion = "v0.0.0-demo";

        constexpr size_t kRobotJointTelemetryJointCount = 19;
        constexpr std::array<const char *, kRobotJointTelemetryJointCount> kRobotJointTelemetryJointNames{
            "Waist01_Joint",
            "Waist02_Joint",
            "Body0422_Joint",
            "openarm_left_joint1",
            "openarm_left_joint2",
            "openarm_left_joint3",
            "openarm_left_joint4",
            "openarm_left_joint5",
            "openarm_left_joint6",
            "openarm_left_joint7",
            "openarm_right_joint1",
            "openarm_right_joint2",
            "openarm_right_joint3",
            "openarm_right_joint4",
            "openarm_right_joint5",
            "openarm_right_joint6",
            "openarm_right_joint7",
            "Head02_Joint",
            "Head03_Joint",
        };

        std::optional<size_t> exactRobotJointIndex(const std::string &name)
        {
            for (size_t joint_index = 0; joint_index < kRobotJointTelemetryJointNames.size(); ++joint_index)
            {
                if (name == kRobotJointTelemetryJointNames[joint_index])
                {
                    return joint_index;
                }
            }
            return std::nullopt;
        }

        float finiteOrZero(double value)
        {
            return std::isfinite(value) ? static_cast<float>(value) : 0.0f;
        }

        void setJointSample(telemetry::RobotJointTelemetrySnapshot &snapshot,
                            size_t joint_index,
                            double position,
                            double velocity,
                            double effort)
        {
            if (joint_index >= snapshot.joints.size())
            {
                return;
            }
            auto &joint = snapshot.joints[joint_index];
            joint.position_rad = finiteOrZero(position);
            joint.velocity_rad_s = finiteOrZero(velocity);
            joint.effort_nm = finiteOrZero(effort);
            joint.valid = std::isfinite(position);
        }

        uint16_t encodeVoltageMv(double voltage_v)
        {
            if (!std::isfinite(voltage_v))
            {
                return 0xFFFFu;
            }
            const auto scaled_mv = static_cast<long long>(std::llround(voltage_v * 1000.0));
            return static_cast<uint16_t>(std::clamp<long long>(scaled_mv, 0, std::numeric_limits<uint16_t>::max()));
        }

        int16_t encodeCurrentMa(double current_a)
        {
            if (!std::isfinite(current_a))
            {
                return std::numeric_limits<int16_t>::min();
            }
            const auto scaled_ma = static_cast<long long>(std::llround(current_a * 1000.0));
            return static_cast<int16_t>(std::clamp<long long>(
                scaled_ma,
                static_cast<long long>(std::numeric_limits<int16_t>::min()),
                static_cast<long long>(std::numeric_limits<int16_t>::max())));
        }

        int16_t encodeTemperatureDeciC(double temperature_c)
        {
            if (!std::isfinite(temperature_c))
            {
                return std::numeric_limits<int16_t>::min();
            }
            const auto scaled_deci_c = static_cast<long long>(std::llround(temperature_c * 10.0));
            return static_cast<int16_t>(std::clamp<long long>(
                scaled_deci_c,
                static_cast<long long>(std::numeric_limits<int16_t>::min()),
                static_cast<long long>(std::numeric_limits<int16_t>::max())));
        }

        std::string agvErrorDomainToString(uint8_t error_domain)
        {
            switch (error_domain)
            {
            case AgvDeviceStateMsg::ERROR_NONE:
                return "ERROR_NONE";
            case AgvDeviceStateMsg::ERROR_UNKNOWN:
                return "ERROR_UNKNOWN";
            case AgvDeviceStateMsg::ERROR_HW:
                return "ERROR_HW";
            case AgvDeviceStateMsg::ERROR_SW:
                return "ERROR_SW";
            case AgvDeviceStateMsg::ERROR_OVER_TRAVEL:
                return "ERROR_OVER_TRAVEL";
            case AgvDeviceStateMsg::EMERGENCY_STOP_HW:
                return "EMERGENCY_STOP_HW";
            case AgvDeviceStateMsg::EMERGENCY_STOP_SW:
                return "EMERGENCY_STOP_SW";
            case AgvDeviceStateMsg::PROTECTIVE_STOP_HW:
                return "PROTECTIVE_STOP_HW";
            case AgvDeviceStateMsg::PROTECTIVE_STOP_SW:
                return "PROTECTIVE_STOP_SW";
            case AgvDeviceStateMsg::SAFETY_STOP:
                return "SAFETY_STOP";
            case AgvDeviceStateMsg::CALIBRATION_REQUIRED:
                return "CALIBRATION_REQUIRED";
            default:
                return "UNKNOWN_" + std::to_string(static_cast<unsigned int>(error_domain));
            }
        }
    } // namespace

    MainNode::MainNode() : Node("main_node")
    {
        configureLogging();
        peer_subscription_callback_group_ =
            this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
        direct_running_mode_ = declareOrGet<bool>(*this, "debug.direct_running_mode", false);
        loadPairingParams();
        loadRobotTelemetryParams();
        startWristCameras();

        telemetry_battery_sub_ = this->create_subscription<AgvBatteryStateMsg>(
            "/agv_battery_state", 10,
            [this](const AgvBatteryStateMsg::SharedPtr msg) { onTelemetryBatteryState(msg); });
        telemetry_temperature_sub_ = this->create_subscription<TemperatureMsg>(
            "/temperature", 10,
            [this](const TemperatureMsg::SharedPtr msg) { onTelemetryTemperature(msg); });
        telemetry_device_state_sub_ = this->create_subscription<AgvDeviceStateMsg>(
            "/agv_device_state", 10,
            [this](const AgvDeviceStateMsg::SharedPtr msg) { onTelemetryDeviceState(msg); });
        if (joint_telemetry_config_.enabled)
        {
            joint_state_sub_ = this->create_subscription<JointStateMsg>(
                joint_telemetry_config_.joint_state_topic, 10,
                [this](const JointStateMsg::SharedPtr msg) { onJointState(msg); });
        }

        video_module_ = std::make_unique<trb::video::VideoModule>(*this, trb::video::VideoModule::Config::configFromRosParam(*this));
        video_module_->setEncodedFrameCallback(
            [this](const uint8_t *data, size_t size, uint64_t timestamp_us, bool keyframe) {
                onEncodedVideoFrame(data, size, timestamp_us, keyframe);
            });
        video_module_->setSpsPpsCallback(
            [this](const std::vector<uint8_t> &sps, const std::vector<uint8_t> &pps, const std::vector<uint8_t> &vps) {
                onSpsPpsCaptured(sps, pps, vps);
            });

        udp_module_ = std::make_unique<trb::udp::UdpModule>(*this, trb::udp::UdpModule::Config::configFromRosParam(*this));
        udp_module_->setAudioDatagramCallback(
            [this](uint16_t seq, const uint8_t *data, size_t size, uint64_t timestamp_ntp) {
                if (audio_module_)
                {
                    (void)audio_module_->receiveRemotePacket(seq, data, size, timestamp_ntp);
                }
            });
        udp_module_->setHandshakeReadyCallback([this]() { onUdpHandshakeReady(); });
        udp_module_->setPeerLostCallback([this]() { onUdpPeerLost(); });

        audio_module_ = std::make_unique<trb::audio::AudioModule>(*this, trb::audio::AudioModule::configFromRosParam(*this));
        audio_module_->setEncodedPacketCallback(
            [this](const uint8_t *data, size_t size, uint64_t timestamp_ntp) {
                onEncodedAudioPacket(data, size, timestamp_ntp);
            });
        const bool audio_initialized = audio_module_->initialize();
        if (audio_module_->config().enabled && !audio_initialized)
        {
            RCLCPP_WARN(this->get_logger(), "AudioModule initialization failed; audio will stay disabled for this process");
        }

        const bool video_stats_enabled = declareOrGet<bool>(*this, "logging.modules.video_stats", true);
        const double stats_period_sec = std::max(0.1, declareOrGet<double>(*this, "logging.modules.period_sec", 1.0));
        if (video_stats_enabled)
        {
            last_video_stats_time_ = std::chrono::steady_clock::now();
            video_stats_timer_ = this->create_wall_timer(
                std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::duration<double>(stats_period_sec)),
                [this]() { videoStatsTimerCallback(); });
            RCLCPP_INFO(this->get_logger(), "Video stats logging enabled: period=%.2fs", stats_period_sec);
        }

        if (direct_running_mode_)
        {
            if (!udp_module_->start(std::string{}))
            {
                RCLCPP_ERROR(this->get_logger(), "Direct running mode failed: UdpModule start failed");
                return;
            }
            enterDirectRunningMode();
            RCLCPP_INFO(this->get_logger(), "teleop_robot_bridge node initialized in direct running mode");
            return;
        }

        grpc_module_ = std::make_unique<GrpcModule>(*this, GrpcModule::configFromRosParam(*this));
        grpc_module_->setRegisteredCallback([this]() { onGrpcRegistered(); });
        grpc_module_->setHeartbeatFailCallback([this]() { onGrpcHeartbeatFail(); });
        grpc_module_->setStreamEndCallback([this]() { onGrpcStreamEnd(); });
        grpc_module_->setSignalingEventCallback([this](const signaling::EventMessage &message) { onSignalingEvent(message); });
        grpc_module_->start();

        RCLCPP_INFO(this->get_logger(), "teleop_robot_bridge node initialized");
    }

    void MainNode::startWristCameras()
    {
        const bool enabled = declareOrGet<bool>(*this, "video.wrist_cameras.enabled", true);
        if (!enabled)
        {
            RCLCPP_INFO(this->get_logger(), "Wrist cameras disabled by configuration");
            return;
        }

        const int64_t configured_width = declareOrGet<int64_t>(*this, "video.wrist_cameras.width", 640);
        const int64_t configured_height = declareOrGet<int64_t>(*this, "video.wrist_cameras.height", 480);
        const int64_t configured_fps = declareOrGet<int64_t>(*this, "video.wrist_cameras.fps", 30);
        const int64_t configured_buffer_count = declareOrGet<int64_t>(*this, "video.wrist_cameras.buffer_count", 4);
        const int64_t configured_retry_interval =
            declareOrGet<int64_t>(*this, "video.wrist_cameras.retry_interval_ms", 1000);
        const std::string pixel_format = declareOrGet<std::string>(*this, "video.wrist_cameras.pixel_format", "yuyv");

        if (configured_width <= 0 || configured_height <= 0 || configured_fps <= 0 || configured_buffer_count <= 0 ||
            configured_retry_interval <= 0)
        {
            RCLCPP_ERROR(this->get_logger(), "Invalid video.wrist_cameras dimensions, fps, buffer count, or retry interval");
            return;
        }
        if (pixel_format != "yuyv")
        {
            RCLCPP_WARN(this->get_logger(), "video.wrist_cameras.pixel_format='%s' is unsupported; using yuyv",
                        pixel_format.c_str());
        }

        trb::video::WristCameraPublisher::Config common_config;
        common_config.width = static_cast<uint32_t>(configured_width);
        common_config.height = static_cast<uint32_t>(configured_height);
        common_config.framerate = static_cast<uint32_t>(configured_fps);
        common_config.pixel_format = V4L2_PIX_FMT_YUYV;
        common_config.buffer_count = static_cast<uint32_t>(configured_buffer_count);
        common_config.retry_interval_ms = static_cast<uint32_t>(configured_retry_interval);

        auto left_config = common_config;
        left_config.device_path = declareOrGet<std::string>(*this, "video.wrist_cameras.left.device", "/dev/video2");
        left_config.topic = declareOrGet<std::string>(*this, "video.wrist_cameras.left.topic", "data/left_wrist_image");
        left_config.frame_id = declareOrGet<std::string>(*this, "video.wrist_cameras.left.frame_id", "left_wrist_camera");

        auto right_config = common_config;
        right_config.device_path = declareOrGet<std::string>(*this, "video.wrist_cameras.right.device", "/dev/video4");
        right_config.topic = declareOrGet<std::string>(*this, "video.wrist_cameras.right.topic", "data/right_wrist_image");
        right_config.frame_id = declareOrGet<std::string>(*this, "video.wrist_cameras.right.frame_id", "right_wrist_camera");

        left_wrist_camera_ = std::make_unique<trb::video::WristCameraPublisher>(*this, "left", std::move(left_config));
        right_wrist_camera_ = std::make_unique<trb::video::WristCameraPublisher>(*this, "right", std::move(right_config));

        if (!left_wrist_camera_->start())
        {
            left_wrist_camera_.reset();
            RCLCPP_ERROR(this->get_logger(), "Failed to start left wrist camera worker");
        }
        if (!right_wrist_camera_->start())
        {
            right_wrist_camera_.reset();
            RCLCPP_ERROR(this->get_logger(), "Failed to start right wrist camera worker");
        }
    }

    void MainNode::stopWristCameras()
    {
        if (left_wrist_camera_)
        {
            left_wrist_camera_->stop();
            left_wrist_camera_.reset();
        }
        if (right_wrist_camera_)
        {
            right_wrist_camera_->stop();
            right_wrist_camera_.reset();
        }
    }

    MainNode::~MainNode()
    {
        RCLCPP_INFO(this->get_logger(), "Shutting down MainNode...");
        shutting_down_.store(true, std::memory_order_release);
        stopWristCameras();

        if (grpc_module_)
        {
            grpc_module_->setRegisteredCallback({});
            grpc_module_->setHeartbeatFailCallback({});
            grpc_module_->setStreamEndCallback({});
            grpc_module_->setSignalingEventCallback({});
        }
        if (video_module_)
        {
            video_module_->setEncodedFrameCallback({});
            video_module_->setSpsPpsCallback({});
        }
        if (audio_module_)
        {
            audio_module_->setEncodedPacketCallback({});
        }
        if (udp_module_)
        {
            udp_module_->setHandshakeReadyCallback({});
            udp_module_->setPeerLostCallback({});
            udp_module_->setAudioDatagramCallback({});
        }

        stopNegotiationRetryTimer();
        stopPeerSubscriptionRetryTimer();
        stopPairingRetryTimer();
        stopRegisteredSetupRetryTimer();
        if (session_work_timer_)
        {
            session_work_timer_->cancel();
            session_work_timer_.reset();
        }
        {
            std::lock_guard<std::mutex> lock(session_work_mutex_);
            session_work_queue_.clear();
        }
        if (video_stats_timer_)
        {
            video_stats_timer_->cancel();
            video_stats_timer_.reset();
        }
        stopRobotTelemetry();
        if (telemetry_timer_)
        {
            telemetry_timer_.reset();
        }
        if (joint_telemetry_timer_)
        {
            joint_telemetry_timer_.reset();
        }

        if (grpc_module_)
        {
            grpc_module_->stop();
            grpc_module_.reset();
        }
        stopVideo();
        if (audio_module_)
        {
            audio_module_->stop();
            audio_module_.reset();
        }
        if (udp_module_)
        {
            udp_module_->stop();
            udp_module_.reset();
        }

        RCLCPP_INFO(this->get_logger(), "MainNode shutdown complete.");
    }

    void MainNode::configureLogging()
    {
        const bool log_file_enabled = declareOrGet<bool>(*this, "logging.file.enabled", false);
        const std::string log_file_path = declareOrGet<std::string>(*this, "logging.file.path", "/tmp/teleop_robot_bridge.log");
        const auto log_file_topics = declareOrGet<std::vector<std::string>>(*this, "logging.file.topics", std::vector<std::string>{});
        const uint32_t log_topic_mask = trb::utils::TeleopLogger::fileTopicsFromStrings(log_file_topics, this->get_logger());
        trb::utils::TeleopLogger::instance().configureFileLogging(log_file_enabled, log_file_path, log_topic_mask);
    }

    void MainNode::loadPairingParams()
    {
        subscribe_vr_pose_flag_ = declareOrGet<bool>(*this, "subscribe_vr_pose_flag", false);
        pair_mode_ = declareOrGet<std::string>(*this, "pair.mode", "passive");
        std::transform(pair_mode_.begin(), pair_mode_.end(), pair_mode_.begin(), [](unsigned char value) {
            return static_cast<char>(std::tolower(value));
        });
        if (pair_mode_ != "active" && pair_mode_ != "passive")
        {
            RCLCPP_WARN(this->get_logger(), "Invalid pair.mode='%s', falling back to passive", pair_mode_.c_str());
            pair_mode_ = "passive";
        }

        desired_peer_session_id_ = declareOrGet<std::string>(*this, "pair.peer_session_id", "");
        desired_peer_device_type_code_ = declareOrGet<std::string>(*this, "pair.peer_device_type_code", "");
        pair_auto_accept_ = declareOrGet<bool>(*this, "pair.auto_accept", true);
        pair_auto_request_ = declareOrGet<bool>(*this, "pair.auto_request", true);
        pair_list_unpaired_on_start_ = declareOrGet<bool>(*this, "pair.list_unpaired_on_start", false);
    }

    void MainNode::loadRobotTelemetryParams()
    {
        telemetry_config_.enabled = declareOrGet<bool>(*this, "telemetry.low_rate.enabled", false);
        telemetry_config_.period_sec = std::max(0.1, declareOrGet<double>(*this, "telemetry.low_rate.period_sec", 1.0));
        telemetry_config_.snapshot = telemetry::RobotTelemetrySnapshot{};
        telemetry_config_.snapshot.version = telemetry::kRobotTelemetryPayloadVersion;
        telemetry_config_.snapshot.battery_voltage_mv = kFallbackBatteryVoltageMv;
        telemetry_config_.snapshot.battery_current_ma = kFallbackBatteryCurrentMa;
        telemetry_config_.snapshot.battery_temperature_deci_c = kFallbackBatteryTemperatureDeciC;
        telemetry_config_.snapshot.fault_code = kFallbackFaultCode;
        telemetry_config_.snapshot.network_quality = kFallbackNetworkQuality;
        telemetry_config_.snapshot.model = declareOrGet<std::string>(*this, "telemetry.low_rate.model", kFallbackModel);
        if (telemetry_config_.snapshot.model.empty())
        {
            telemetry_config_.snapshot.model = kFallbackModel;
        }
        telemetry_config_.snapshot.firmware_version = kFallbackFirmwareVersion;

        joint_telemetry_config_.enabled = declareOrGet<bool>(*this, "telemetry.high_rate.enabled", true);
        joint_telemetry_config_.period_sec = std::max(0.005, declareOrGet<double>(*this, "telemetry.high_rate.period_sec", 0.02));
        joint_telemetry_config_.joint_state_topic = declareOrGet<std::string>(*this, "telemetry.high_rate.joint_state_topic", "/joint_states");
        const int joint_schema_id = declareOrGet<int>(*this, "telemetry.high_rate.schema_id", telemetry::kRobotJointTelemetrySchemaId);
        joint_telemetry_config_.snapshot = telemetry::RobotJointTelemetrySnapshot{};
        joint_telemetry_config_.snapshot.version = telemetry::kRobotJointTelemetryPayloadVersion;
        joint_telemetry_config_.snapshot.schema_id = static_cast<uint16_t>(std::clamp(joint_schema_id, 0, 0xFFFF));
        joint_telemetry_config_.snapshot.sample_timestamp_ms = nowUnixMs();
        joint_telemetry_config_.snapshot.joints.assign(kRobotJointTelemetryJointCount, telemetry::RobotJointStateSample{});
    }

    void MainNode::onTelemetryBatteryState(const AgvBatteryStateMsg::SharedPtr msg)
    {
        if (!msg)
        {
            return;
        }

        std::lock_guard<std::mutex> lock(telemetry_snapshot_mutex_);
        telemetry_config_.snapshot.battery_voltage_mv = encodeVoltageMv(msg->battery_state.voltage);
        telemetry_config_.snapshot.battery_current_ma = encodeCurrentMa(msg->battery_state.current);
    }

    void MainNode::onTelemetryTemperature(const TemperatureMsg::SharedPtr msg)
    {
        if (!msg)
        {
            return;
        }

        std::lock_guard<std::mutex> lock(telemetry_snapshot_mutex_);
        telemetry_config_.snapshot.battery_temperature_deci_c = encodeTemperatureDeciC(msg->temperature.temperature);
    }

    void MainNode::onTelemetryDeviceState(const AgvDeviceStateMsg::SharedPtr msg)
    {
        if (!msg)
        {
            return;
        }

        std::lock_guard<std::mutex> lock(telemetry_snapshot_mutex_);
        telemetry_config_.snapshot.fault_code = agvErrorDomainToString(msg->error_domain);
        telemetry_config_.snapshot.firmware_version = msg->firmware_version;
    }

    void MainNode::onJointState(const JointStateMsg::SharedPtr msg)
    {
        if (!msg)
        {
            return;
        }

        std::lock_guard<std::mutex> lock(joint_telemetry_snapshot_mutex_);
        joint_telemetry_config_.snapshot.sample_timestamp_ms = stampToUnixMs(msg->header.stamp.sec, msg->header.stamp.nanosec);

        for (auto &joint : joint_telemetry_config_.snapshot.joints)
        {
            joint = telemetry::RobotJointStateSample{};
        }

        const size_t sample_count = std::min(msg->name.size(), msg->position.size());
        for (size_t source_index = 0; source_index < sample_count; ++source_index)
        {
            const auto joint_index = exactRobotJointIndex(msg->name[source_index]);
            if (!joint_index)
            {
                continue;
            }
            const double velocity = source_index < msg->velocity.size() ? msg->velocity[source_index] : 0.0;
            const double effort = source_index < msg->effort.size() ? msg->effort[source_index] : 0.0;
            setJointSample(joint_telemetry_config_.snapshot,
                           *joint_index,
                           msg->position[source_index],
                           velocity,
                           effort);
        }
    }

    void MainNode::enterDirectRunningMode()
    {
        {
            std::lock_guard<std::mutex> lock(video_config_mutex_);
            video_config_sent_ = true;
        }
        if (audio_module_)
        {
            audio_module_->markConfigPublished();
            if (audio_module_->isEnabled())
            {
                (void)audio_module_->applyRemoteConfig(audio_module_->config(), "direct");
            }
        }
        setState(State::kRunning, "direct running mode");
        startVideoAndNegotiate();
        if (audio_module_)
        {
            audio_module_->start();
        }
    }

    void MainNode::onGrpcRegistered()
    {
        if (shutting_down_.load(std::memory_order_acquire))
        {
            return;
        }

        stopNegotiationRetryTimer();
        stopPairingRetryTimer();
        stopRegisteredSetupRetryTimer();
        setState(State::kRegistered, "grpc registered");
        publisher_manifest_published_ = false;
        resetPeerSubscriptionState();

        completeRegisteredSetup();
    }

    bool MainNode::publishLocalStreamManifest()
    {
        if (!grpc_module_ || !grpc_module_->isRegistered())
        {
            return false;
        }

        const bool publishes_audio = audio_module_ && audio_module_->isEnabled() &&
                                     audio_module_->config().uplink_enabled;
        return grpc_module_->publishRobotStreamManifest(
            publishes_audio,
            telemetry_config_.enabled,
            joint_telemetry_config_.enabled);
    }

    void MainNode::completeRegisteredSetup()
    {
        if (shutting_down_.load(std::memory_order_acquire) ||
            !grpc_module_ || !grpc_module_->isRegistered() || !udp_module_)
        {
            stopRegisteredSetupRetryTimer();
            return;
        }

        if (!publisher_manifest_published_)
        {
            if (!publishLocalStreamManifest())
            {
                RCLCPP_WARN(this->get_logger(),
                            "Robot stream manifest publish failed; registered setup will retry");
                ensureRegisteredSetupRetryTimer();
                return;
            }
            publisher_manifest_published_ = true;
        }

        if (!udp_module_->isRunning() && !udp_module_->start(grpc_module_->sessionId()))
        {
            RCLCPP_WARN(this->get_logger(), "UdpModule failed to start; registered setup will retry");
            ensureRegisteredSetupRetryTimer();
            return;
        }

        stopRegisteredSetupRetryTimer();
        updateRobotTelemetryLifecycle();
    }

    void MainNode::registeredSetupRetryTimerCallback()
    {
        if (shutting_down_.load(std::memory_order_acquire) ||
            !grpc_module_ || !grpc_module_->isRegistered())
        {
            stopRegisteredSetupRetryTimer();
            return;
        }
        completeRegisteredSetup();
    }

    void MainNode::ensureRegisteredSetupRetryTimer()
    {
        if (!canScheduleRosWork(*this, shutting_down_))
        {
            return;
        }

        const double retry_sec = grpc_module_ ? grpc_module_->config().register_retry_sec : 3.0;
        if (!registered_setup_retry_timer_)
        {
            try
            {
                registered_setup_retry_timer_ = this->create_wall_timer(
                    periodFromSeconds(retry_sec),
                    [this]() { registeredSetupRetryTimerCallback(); });
            }
            catch (const rclcpp::exceptions::RCLError &ex)
            {
                if (!shutting_down_.load(std::memory_order_acquire))
                {
                    RCLCPP_WARN(this->get_logger(), "Skip registered setup retry timer creation: %s", ex.what());
                }
            }
            return;
        }

        try
        {
            registered_setup_retry_timer_->reset();
        }
        catch (const rclcpp::exceptions::RCLError &ex)
        {
            if (!shutting_down_.load(std::memory_order_acquire))
            {
                RCLCPP_WARN(this->get_logger(), "Skip registered setup retry timer reset: %s", ex.what());
            }
        }
    }

    void MainNode::stopRegisteredSetupRetryTimer()
    {
        if (registered_setup_retry_timer_)
        {
            registered_setup_retry_timer_->cancel();
        }
    }

    void MainNode::onGrpcHeartbeatFail()
    {
        if (shutting_down_.load(std::memory_order_acquire))
        {
            return;
        }

        publisher_manifest_published_ = false;
        resetPeerSubscriptionState();
        stopPeerSubscriptionRetryTimer();
        stopRegisteredSetupRetryTimer();
        teardownActiveSession("heartbeat failed", State::kConnecting);
    }

    void MainNode::onGrpcStreamEnd()
    {
        postSessionWork(
            [this]() {
                if (shutting_down_.load(std::memory_order_acquire))
                {
                    return;
                }
                teardownActiveSession("grpc stream ended", State::kRegistered);
                if (!shutting_down_.load(std::memory_order_acquire) && grpc_module_ && grpc_module_->isRegistered())
                {
                    enterPairingState();
                }
            },
            "gRPC stream-end recovery");
    }

    void MainNode::onUdpHandshakeReady()
    {
        if (direct_running_mode_ || shutting_down_.load())
        {
            return;
        }
        updateRobotTelemetryLifecycle();
        enterPairingState();
    }

    void MainNode::onUdpPeerLost()
    {
        postSessionWork([this]() { handleUdpPeerLost(); }, "UDP peer-lost recovery");
    }

    void MainNode::handleUdpPeerLost()
    {
        if (shutting_down_.load(std::memory_order_acquire))
        {
            return;
        }

        const State current = state_.load();
        if (current != State::kNegotiating && current != State::kRunning)
        {
            updateRobotTelemetryLifecycle();
            return;
        }
        teardownActiveSession("udp peer lost", State::kPairing);
    }

    void MainNode::postSessionWork(std::function<void()> work, const char *label)
    {
        if (!work || !canScheduleRosWork(*this, shutting_down_))
        {
            return;
        }

        {
            std::lock_guard<std::mutex> lock(session_work_mutex_);
            session_work_queue_.push_back(std::move(work));
        }

        try
        {
            std::lock_guard<std::mutex> lock(session_work_mutex_);
            if (!session_work_timer_)
            {
                session_work_timer_ = this->create_wall_timer(std::chrono::milliseconds(1), [this]() { drainSessionWork(); });
            }
            else
            {
                session_work_timer_->reset();
            }
        }
        catch (const rclcpp::exceptions::RCLError &ex)
        {
            if (!shutting_down_.load(std::memory_order_acquire))
            {
                RCLCPP_WARN(this->get_logger(), "Skip %s scheduling: %s", label ? label : "session work", ex.what());
            }
        }
    }

    void MainNode::drainSessionWork()
    {
        while (!shutting_down_.load(std::memory_order_acquire))
        {
            std::function<void()> work;
            {
                std::lock_guard<std::mutex> lock(session_work_mutex_);
                if (session_work_queue_.empty())
                {
                    if (session_work_timer_)
                    {
                        session_work_timer_->cancel();
                    }
                    return;
                }
                work = std::move(session_work_queue_.front());
                session_work_queue_.pop_front();
            }

            if (work)
            {
                work();
            }
        }
    }

    void MainNode::startRobotTelemetry()
    {
        if (!canScheduleRosWork(*this, shutting_down_))
        {
            return;
        }

        if (telemetry_config_.enabled && !telemetry_active_.load(std::memory_order_acquire))
        {
            telemetry_active_.store(true, std::memory_order_release);
            if (!telemetry_timer_)
            {
                try
                {
                    telemetry_timer_ = this->create_wall_timer(periodFromSeconds(telemetry_config_.period_sec),
                                                               [this]() { telemetryTimerCallback(); });
                }
                catch (const rclcpp::exceptions::RCLError &ex)
                {
                    telemetry_active_.store(false, std::memory_order_release);
                    if (!shutting_down_.load(std::memory_order_acquire))
                    {
                        RCLCPP_WARN(this->get_logger(), "Skip telemetry timer creation: %s", ex.what());
                    }
                }
            }
            else
            {
                try
                {
                    telemetry_timer_->reset();
                }
                catch (const rclcpp::exceptions::RCLError &ex)
                {
                    telemetry_active_.store(false, std::memory_order_release);
                    if (!shutting_down_.load(std::memory_order_acquire))
                    {
                        RCLCPP_WARN(this->get_logger(), "Skip telemetry timer reset: %s", ex.what());
                    }
                }
            }

            publishRobotTelemetrySnapshot();
        }

        if (joint_telemetry_config_.enabled && !joint_telemetry_active_.load(std::memory_order_acquire))
        {
            joint_telemetry_active_.store(true, std::memory_order_release);
            if (!joint_telemetry_timer_)
            {
                try
                {
                    joint_telemetry_timer_ = this->create_wall_timer(highRatePeriodFromSeconds(joint_telemetry_config_.period_sec),
                                                                     [this]() { jointTelemetryTimerCallback(); });
                }
                catch (const rclcpp::exceptions::RCLError &ex)
                {
                    joint_telemetry_active_.store(false, std::memory_order_release);
                    if (!shutting_down_.load(std::memory_order_acquire))
                    {
                        RCLCPP_WARN(this->get_logger(), "Skip joint telemetry timer creation: %s", ex.what());
                    }
                }
            }
            else
            {
                try
                {
                    joint_telemetry_timer_->reset();
                }
                catch (const rclcpp::exceptions::RCLError &ex)
                {
                    joint_telemetry_active_.store(false, std::memory_order_release);
                    if (!shutting_down_.load(std::memory_order_acquire))
                    {
                        RCLCPP_WARN(this->get_logger(), "Skip joint telemetry timer reset: %s", ex.what());
                    }
                }
            }

            publishRobotJointTelemetrySnapshot();
        }
    }

    void MainNode::stopRobotTelemetry()
    {
        telemetry_active_.store(false, std::memory_order_release);
        joint_telemetry_active_.store(false, std::memory_order_release);
        if (telemetry_timer_)
        {
            telemetry_timer_->cancel();
        }
        if (joint_telemetry_timer_)
        {
            joint_telemetry_timer_->cancel();
        }
    }

    bool MainNode::isRobotTelemetryOnline() const
    {
        const bool any_telemetry_enabled = telemetry_config_.enabled || joint_telemetry_config_.enabled;
        if (shutting_down_.load(std::memory_order_acquire) || !any_telemetry_enabled)
        {
            return false;
        }
        if (direct_running_mode_)
        {
            return false;
        }
        if (!grpc_module_ || !grpc_module_->isRegistered())
        {
            return false;
        }
        if (!udp_module_ || !udp_module_->isRunning())
        {
            return false;
        }
        if (udp_module_->controlEnabled() && !udp_module_->isHandshakeReady())
        {
            return false;
        }
        return true;
    }

    void MainNode::updateRobotTelemetryLifecycle()
    {
        if (isRobotTelemetryOnline())
        {
            startRobotTelemetry();
            return;
        }
        stopRobotTelemetry();
    }

    void MainNode::telemetryTimerCallback()
    {
        if (!telemetry_active_.load(std::memory_order_acquire))
        {
            return;
        }
        if (!isRobotTelemetryOnline())
        {
            stopRobotTelemetry();
            return;
        }
        publishRobotTelemetrySnapshot();
    }

    void MainNode::jointTelemetryTimerCallback()
    {
        if (!joint_telemetry_active_.load(std::memory_order_acquire))
        {
            return;
        }
        if (!isRobotTelemetryOnline())
        {
            stopRobotTelemetry();
            return;
        }
        publishRobotJointTelemetrySnapshot();
    }

    void MainNode::publishRobotTelemetrySnapshot()
    {
        if (!telemetry_active_.load(std::memory_order_acquire) || !udp_module_)
        {
            return;
        }

        telemetry::RobotTelemetrySnapshot snapshot;
        {
            std::lock_guard<std::mutex> lock(telemetry_snapshot_mutex_);
            snapshot = telemetry_config_.snapshot;
        }

        udp_module_->sendRobotTelemetry(snapshot);
    }

    void MainNode::publishRobotJointTelemetrySnapshot()
    {
        if (!joint_telemetry_active_.load(std::memory_order_acquire) || !udp_module_)
        {
            return;
        }

        telemetry::RobotJointTelemetrySnapshot snapshot;
        {
            std::lock_guard<std::mutex> lock(joint_telemetry_snapshot_mutex_);
            snapshot = joint_telemetry_config_.snapshot;
        }

        udp_module_->sendRobotJointTelemetry(snapshot);
    }

    void MainNode::enterPairingState()
    {
        if (!grpc_module_ || shutting_down_.load())
        {
            return;
        }
        setState(State::kPairing, "waiting for pair request");
        grpc_module_->startEventStream();

        if (pair_mode_ == "active")
        {
            (void)tryActivePairing();
            ensurePairingRetryTimer();
        }
        else
        {
            stopPairingRetryTimer();
        }
    }

    bool MainNode::tryActivePairing()
    {
        if (shutting_down_.load(std::memory_order_acquire))
        {
            return false;
        }

        if (!grpc_module_ || !grpc_module_->isRegistered() || state_.load() != State::kPairing)
        {
            return false;
        }
        if (!pair_auto_request_)
        {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                                 "pair.mode=active but pair.auto_request=false; waiting for incoming pair request");
            return false;
        }

        std::string peer_session_id = desired_peer_session_id_;
        if (peer_session_id.empty())
        {
            if (desired_peer_device_type_code_.empty())
            {
                RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                                     "Active pairing requires pair.peer_session_id or pair.peer_device_type_code");
                return false;
            }

            const auto endpoints = grpc_module_->listUnpaired(desired_peer_device_type_code_);
            if (endpoints.empty())
            {
                RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                                     "No unpaired endpoint found for device type %s; active pairing will retry",
                                     desired_peer_device_type_code_.c_str());
                return false;
            }
            if (pair_list_unpaired_on_start_)
            {
                RCLCPP_INFO(this->get_logger(),
                            "Found %zu unpaired endpoint(s) for device type %s; requesting the first one",
                            endpoints.size(), desired_peer_device_type_code_.c_str());
            }
            peer_session_id = endpoints.front().session_id();
        }

        RCLCPP_INFO(this->get_logger(), "Requesting pair with %s", peer_session_id.c_str());
        if (!grpc_module_->requestPair(peer_session_id))
        {
            RCLCPP_WARN(this->get_logger(), "Pair request failed for %s", peer_session_id.c_str());
            return false;
        }
        return true;
    }

    void MainNode::pairingRetryTimerCallback()
    {
        if (shutting_down_.load(std::memory_order_acquire))
        {
            stopPairingRetryTimer();
            return;
        }

        if (state_.load() != State::kPairing || pair_mode_ != "active")
        {
            stopPairingRetryTimer();
            return;
        }
        (void)tryActivePairing();
    }

    void MainNode::ensurePairingRetryTimer()
    {
        if (!canScheduleRosWork(*this, shutting_down_))
        {
            return;
        }

        if (!pairing_retry_timer_)
        {
            try
            {
                pairing_retry_timer_ = this->create_wall_timer(std::chrono::seconds(2), [this]() { pairingRetryTimerCallback(); });
            }
            catch (const rclcpp::exceptions::RCLError &ex)
            {
                if (!shutting_down_.load(std::memory_order_acquire))
                {
                    RCLCPP_WARN(this->get_logger(), "Skip pairing retry timer creation: %s", ex.what());
                }
            }
            return;
        }
        try
        {
            pairing_retry_timer_->reset();
        }
        catch (const rclcpp::exceptions::RCLError &ex)
        {
            if (!shutting_down_.load(std::memory_order_acquire))
            {
                RCLCPP_WARN(this->get_logger(), "Skip pairing retry timer reset: %s", ex.what());
            }
        }
    }

    void MainNode::stopPairingRetryTimer()
    {
        if (pairing_retry_timer_)
        {
            pairing_retry_timer_->cancel();
        }
    }

    void MainNode::onSignalingEvent(const signaling::EventMessage &msg)
    {
        signaling::EventMessage copied_msg = msg;
        postSessionWork([this, copied_msg]() { handleSignalingEvent(copied_msg); }, "signaling event");
    }

    void MainNode::handleSignalingEvent(const signaling::EventMessage &msg)
    {
        if (shutting_down_.load(std::memory_order_acquire))
        {
            return;
        }

        if (!msg.has_pair())
        {
            if (msg.has_system() && msg.system().action() == signaling::SystemCommand::REQUEST_UDP_HELLO)
            {
                RCLCPP_WARN(this->get_logger(), "Server requested UDP HELLO resend; resetting UDP session");
                if (udp_module_)
                {
                    udp_module_->resetSession();
                }
            }
            return;
        }

        const auto &pair_event = msg.pair();
        if (!pair_event.has_peer())
        {
            RCLCPP_WARN(this->get_logger(), "Received PairEvent without peer info");
            return;
        }

        const auto &peer_info = pair_event.peer();
        const std::string peer = peer_info.session_id();
        const std::string peer_device_type_code = peer_info.device_type_code();
        const bool peer_session_matches = desired_peer_session_id_.empty() || peer == desired_peer_session_id_;
        const bool peer_type_matches = desired_peer_device_type_code_.empty() ||
                                       peer_device_type_code.rfind(desired_peer_device_type_code_, 0) == 0;
        const bool peer_matches = peer_session_matches && peer_type_matches;

        switch (pair_event.op())
        {
        case signaling::PairEvent::REQUEST:
            if (pair_auto_accept_ && peer_matches && grpc_module_ && grpc_module_->acceptPair(peer))
            {
                RCLCPP_INFO(this->get_logger(), "Pair request from %s accepted (device_type=%s)",
                            peer.c_str(),
                            peer_device_type_code.empty() ? "<empty>" : peer_device_type_code.c_str());
                {
                    std::lock_guard<std::mutex> lock(peer_receive_state_mutex_);
                    paired_peer_session_id_ = peer;
                    peer_streams_subscribed_ = false;
                    peer_subscribed_prefixes_.clear();
                    if (audio_module_)
                    {
                        audio_module_->resetRemoteConfig("new peer paired");
                    }
                }
                stopPairingRetryTimer();
                setState(State::kNegotiating, "pair accepted");
                startVideoAndNegotiate();
            }
            else
            {
                RCLCPP_WARN(this->get_logger(), "Pair request from %s rejected", peer.c_str());
                if (grpc_module_)
                {
                    (void)grpc_module_->rejectPair(peer);
                }
            }
            break;

        case signaling::PairEvent::ACCEPT:
            RCLCPP_INFO(this->get_logger(), "Pair accepted by %s (device_type=%s)",
                        peer.c_str(),
                        peer_device_type_code.empty() ? "<empty>" : peer_device_type_code.c_str());
            {
                std::lock_guard<std::mutex> lock(peer_receive_state_mutex_);
                paired_peer_session_id_ = peer;
                peer_streams_subscribed_ = false;
                peer_subscribed_prefixes_.clear();
                if (audio_module_)
                {
                    audio_module_->resetRemoteConfig("new peer paired");
                }
            }
            stopPairingRetryTimer();
            setState(State::kNegotiating, "paired");
            startVideoAndNegotiate();
            break;

        case signaling::PairEvent::REJECT:
            RCLCPP_WARN(this->get_logger(), "Pair rejected by %s", peer.c_str());
            {
                std::lock_guard<std::mutex> lock(peer_receive_state_mutex_);
                if (paired_peer_session_id_ == peer)
                {
                    paired_peer_session_id_.clear();
                    peer_streams_subscribed_ = false;
                    peer_subscribed_prefixes_.clear();
                }
            }
            setState(State::kPairing, "pair rejected");
            if (pair_mode_ == "active")
            {
                ensurePairingRetryTimer();
            }
            break;

        case signaling::PairEvent::UNPAIR:
            RCLCPP_INFO(this->get_logger(), "Unpaired by %s", peer.c_str());
            {
                std::lock_guard<std::mutex> lock(peer_receive_state_mutex_);
                if (paired_peer_session_id_ == peer)
                {
                    paired_peer_session_id_.clear();
                    peer_streams_subscribed_ = false;
                    peer_subscribed_prefixes_.clear();
                }
            }
            teardownActiveSession("unpaired by peer", State::kPairing);
            break;

        default:
            break;
        }
    }

    bool MainNode::ensurePeerSubscriptions()
    {
        if (!grpc_module_)
        {
            return false;
        }

        std::string peer_session_id;
        std::vector<uint8_t> subscribed_prefixes;
        {
            std::lock_guard<std::mutex> lock(peer_receive_state_mutex_);
            if (peer_streams_subscribed_)
            {
                return true;
            }
            if (paired_peer_session_id_.empty())
            {
                return false;
            }
            if (!subscribe_vr_pose_flag_ && !wantsRemoteAudio())
            {
                peer_streams_subscribed_ = true;
                peer_subscribed_prefixes_.clear();
                return true;
            }
            peer_session_id = paired_peer_session_id_;
            subscribed_prefixes = peer_subscribed_prefixes_;
        }

        std::vector<uint8_t> available_prefixes;
        if (!grpc_module_->getStreamPrefixes(peer_session_id, available_prefixes))
        {
            return false;
        }

        const auto has_prefix = [&available_prefixes](uint8_t prefix) {
            return std::find(available_prefixes.begin(), available_prefixes.end(), prefix) != available_prefixes.end();
        };

        std::vector<uint8_t> subscribe_prefixes;
        bool has_pose_stream = false;
        bool all_requested_streams_available = true;
        if (subscribe_vr_pose_flag_)
        {
            if (!has_prefix(kPoseControllerPrefix) || !has_prefix(kBody24RawPrefix))
            {
                RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                                     "Peer manifest does not yet declare pose prefixes 0x02 and 0x09; video will continue while pose subscription retries");
                all_requested_streams_available = false;
            }
            else
            {
                subscribe_prefixes.push_back(kPoseControllerPrefix);
                subscribe_prefixes.push_back(kBody24RawPrefix);
                has_pose_stream = true;
            }
        }

        if (wantsRemoteAudio())
        {
            if (!has_prefix(kAudioPrefix))
            {
                RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                                     "Peer manifest does not yet declare audio prefix 0x04");
                all_requested_streams_available = false;
            }
            else
            {
                subscribe_prefixes.push_back(kAudioPrefix);
            }
        }

        std::sort(subscribe_prefixes.begin(), subscribe_prefixes.end());
        subscribe_prefixes.erase(std::unique(subscribe_prefixes.begin(), subscribe_prefixes.end()),
                                 subscribe_prefixes.end());

        if (subscribe_prefixes != subscribed_prefixes &&
            !subscribe_prefixes.empty() &&
            !grpc_module_->subscribe(peer_session_id, subscribe_prefixes))
        {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                                 "Failed to subscribe to available peer stream prefixes; background retry will continue");
            return false;
        }

        std::lock_guard<std::mutex> lock(peer_receive_state_mutex_);
        if (paired_peer_session_id_ != peer_session_id)
        {
            return false;
        }
        if (subscribe_prefixes != peer_subscribed_prefixes_)
        {
            peer_subscribed_prefixes_ = subscribe_prefixes;
            if (!subscribe_prefixes.empty())
            {
                RCLCPP_INFO(this->get_logger(),
                            "Subscribed to %zu available peer stream(s): pose=%d audio=%d complete=%d",
                            subscribe_prefixes.size(),
                            has_pose_stream ? 1 : 0,
                            std::find(subscribe_prefixes.begin(), subscribe_prefixes.end(), kAudioPrefix) !=
                                    subscribe_prefixes.end()
                                ? 1
                                : 0,
                            all_requested_streams_available ? 1 : 0);
            }
        }

        peer_streams_subscribed_ = all_requested_streams_available;
        return peer_streams_subscribed_;
    }

    void MainNode::peerSubscriptionRetryTimerCallback()
    {
        const State current = state_.load();
        bool has_peer = false;
        {
            std::lock_guard<std::mutex> lock(peer_receive_state_mutex_);
            has_peer = !paired_peer_session_id_.empty();
        }
        if (shutting_down_.load(std::memory_order_acquire) ||
            (current != State::kNegotiating && current != State::kRunning) || !has_peer)
        {
            stopPeerSubscriptionRetryTimer();
            return;
        }

        const bool subscriptions_ready = ensurePeerSubscriptions();
        const bool remote_audio_ready = fetchRemoteAudioConfig();
        if (subscriptions_ready && remote_audio_ready)
        {
            stopPeerSubscriptionRetryTimer();
        }
    }

    void MainNode::ensurePeerSubscriptionRetryTimer()
    {
        if (!canScheduleRosWork(*this, shutting_down_))
        {
            return;
        }

        std::lock_guard<std::mutex> lock(peer_subscription_timer_mutex_);
        if (!peer_subscription_retry_timer_)
        {
            try
            {
                peer_subscription_retry_timer_ = this->create_wall_timer(
                    std::chrono::seconds(1),
                    [this]() { peerSubscriptionRetryTimerCallback(); },
                    peer_subscription_callback_group_);
            }
            catch (const rclcpp::exceptions::RCLError &ex)
            {
                if (!shutting_down_.load(std::memory_order_acquire))
                {
                    RCLCPP_WARN(this->get_logger(), "Skip peer subscription retry timer creation: %s", ex.what());
                }
            }
            return;
        }

        try
        {
            peer_subscription_retry_timer_->reset();
        }
        catch (const rclcpp::exceptions::RCLError &ex)
        {
            if (!shutting_down_.load(std::memory_order_acquire))
            {
                RCLCPP_WARN(this->get_logger(), "Skip peer subscription retry timer reset: %s", ex.what());
            }
        }
    }

    void MainNode::stopPeerSubscriptionRetryTimer()
    {
        std::lock_guard<std::mutex> lock(peer_subscription_timer_mutex_);
        if (peer_subscription_retry_timer_)
        {
            peer_subscription_retry_timer_->cancel();
        }
    }

    void MainNode::resetPeerSubscriptionState()
    {
        std::lock_guard<std::mutex> lock(peer_receive_state_mutex_);
        peer_streams_subscribed_ = false;
        peer_subscribed_prefixes_.clear();
    }

    bool MainNode::fetchRemoteAudioConfig()
    {
        if (!wantsRemoteAudio())
        {
            return true;
        }
        if (!audio_module_ || !grpc_module_)
        {
            return false;
        }

        std::string peer_session_id;
        {
            std::lock_guard<std::mutex> lock(peer_receive_state_mutex_);
            if (paired_peer_session_id_.empty())
            {
                return false;
            }
            if (audio_module_->isRemoteConfigReady())
            {
                return true;
            }
            peer_session_id = paired_peer_session_id_;
        }

        GrpcModule::AudioConfig config;
        std::string config_id;
        if (!grpc_module_->fetchAudioConfig(peer_session_id, config, config_id))
        {
            return false;
        }

        std::string message;
        trb::audio::AudioModule::Config remote_config;
        if (!validateRemoteAudioConfig(config, remote_config, message))
        {
            RCLCPP_WARN(this->get_logger(), "Remote audio StreamConfig rejected: %s", message.c_str());
            return false;
        }
        std::lock_guard<std::mutex> lock(peer_receive_state_mutex_);
        if (paired_peer_session_id_ != peer_session_id)
        {
            return false;
        }
        if (!audio_module_->applyRemoteConfig(remote_config, config_id))
        {
            RCLCPP_WARN(this->get_logger(), "Failed to apply remote audio StreamConfig");
            return false;
        }

        RCLCPP_INFO(this->get_logger(), "Remote audio StreamConfig accepted: config_id=%s",
                    config_id.empty() ? "<empty>" : config_id.c_str());
        return true;
    }

    bool MainNode::validateRemoteAudioConfig(const GrpcModule::AudioConfig &config,
                                             trb::audio::AudioModule::Config &remote_config,
                                             std::string &message) const
    {
        if (!audio_module_ || !audio_module_->isEnabled())
        {
            message = "audio module is disabled";
            return false;
        }
        const auto &local_config = audio_module_->config();
        if (!local_config.downlink_enabled)
        {
            message = "audio downlink is disabled";
            return false;
        }
        if (config.sample_rate == 0 || config.channels == 0 || config.frame_duration_ms == 0 ||
            config.samples_per_channel == 0 || config.bitrate_bps == 0)
        {
            message = "invalid non-positive audio config field";
            return false;
        }

        remote_config = local_config;
        remote_config.sample_rate = config.sample_rate;
        remote_config.channels = config.channels;
        remote_config.frame_duration_ms = config.frame_duration_ms;
        remote_config.samples_per_channel = config.samples_per_channel;
        remote_config.bitrate_bps = config.bitrate_bps;
        remote_config.opus_inband_fec_enabled = config.opus_inband_fec_enabled;
        remote_config.opus_dtx_enabled = config.opus_dtx_enabled;

        if (remote_config.sample_rate != local_config.sample_rate ||
            remote_config.channels != local_config.channels ||
            remote_config.frame_duration_ms != local_config.frame_duration_ms ||
            remote_config.samples_per_channel != local_config.samples_per_channel ||
            remote_config.bitrate_bps != local_config.bitrate_bps ||
            remote_config.opus_inband_fec_enabled != local_config.opus_inband_fec_enabled ||
            remote_config.opus_dtx_enabled != local_config.opus_dtx_enabled)
        {
            message = "remote AudioConfig does not match local audio parameters";
            return false;
        }
        message = "accepted";
        return true;
    }

    bool MainNode::wantsRemoteAudio() const
    {
        return audio_module_ && audio_module_->isEnabled() && audio_module_->config().downlink_enabled;
    }

    void MainNode::startVideoAndNegotiate()
    {
        if (shutting_down_.load(std::memory_order_acquire))
        {
            return;
        }

        if (!video_module_)
        {
            RCLCPP_ERROR(this->get_logger(), "VideoModule is not available");
            return;
        }

        if (video_started_ && video_module_->isRunning())
        {
            ensureNegotiationRetryTimer();
            if (state_.load() == State::kNegotiating && video_module_->hasSpsPps())
            {
                tryAdvanceNegotiation();
            }
            return;
        }

        if (!video_module_->start())
        {
            RCLCPP_ERROR(this->get_logger(), "VideoModule start failed");
            return;
        }

        {
            std::lock_guard<std::mutex> lock(video_config_mutex_);
            video_started_ = true;
        }
        video_start_time_ = std::chrono::steady_clock::now();
        last_video_keyframe_request_time_ = std::chrono::steady_clock::time_point{};
        ensureNegotiationRetryTimer();

        if (state_.load() == State::kNegotiating && video_module_->hasSpsPps())
        {
            tryAdvanceNegotiation();
        }
    }

    void MainNode::onEncodedVideoFrame(const uint8_t *data, size_t size, uint64_t timestamp_us, bool keyframe)
    {
        if (shutting_down_.load(std::memory_order_acquire))
        {
            return;
        }

        const State current = state_.load();
        if (current != State::kRunning)
        {
            const uint64_t dropped = dropped_video_before_running_.fetch_add(1, std::memory_order_relaxed) + 1;
            if ((dropped % 30) == 0)
            {
                RCLCPP_WARN(this->get_logger(), "Dropping encoded video frame before running state: dropped=%lu state=%s",
                            static_cast<unsigned long>(dropped), stateToString(current));
            }
            return;
        }

        if (udp_module_)
        {
            udp_module_->sendH264Frame(data, size, timestamp_us, keyframe);
        }
    }

    void MainNode::onEncodedAudioPacket(const uint8_t *data, size_t size, uint64_t timestamp_ntp)
    {
        if (shutting_down_.load(std::memory_order_acquire))
        {
            return;
        }
        if (state_.load() != State::kRunning)
        {
            return;
        }
        if (!audio_module_ || !audio_module_->isConfigPublished())
        {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                                 "Dropping audio packet because AudioConfig is not published");
            return;
        }
        if (udp_module_)
        {
            udp_module_->sendAudioPacket(data, size, timestamp_ntp);
        }
    }

    void MainNode::onSpsPpsCaptured(const std::vector<uint8_t> &sps,
                                    const std::vector<uint8_t> &pps,
                                    const std::vector<uint8_t> &vps)
    {
        if (shutting_down_.load(std::memory_order_acquire))
        {
            return;
        }

        RCLCPP_INFO(this->get_logger(), "Video parameter sets ready: sps=%zu bytes pps=%zu bytes vps=%zu bytes",
                    sps.size(), pps.size(), vps.size());
        if (state_.load() == State::kNegotiating)
        {
            tryAdvanceNegotiation();
        }
    }

    void MainNode::requestVideoKeyframeRecovery(const char *reason, std::chrono::steady_clock::time_point now)
    {
        if (shutting_down_.load(std::memory_order_acquire) || state_.load() != State::kRunning || !video_module_)
        {
            return;
        }

        if (last_video_keyframe_request_time_.time_since_epoch().count() != 0 &&
            (now - last_video_keyframe_request_time_) < kVideoKeyframeRequestInterval)
        {
            return;
        }

        last_video_keyframe_request_time_ = now;
        const bool requested = video_module_->requestKeyframe();
        RCLCPP_WARN(this->get_logger(),
                    requested
                        ? "Video transport loss detected (%s): requested fresh IDR"
                        : "Video transport loss detected (%s): fresh IDR request not ready yet",
                    reason ? reason : "unknown");
    }

    bool MainNode::publishVideoConfig()
    {
        if (shutting_down_.load(std::memory_order_acquire))
        {
            return false;
        }

        if (direct_running_mode_)
        {
            std::lock_guard<std::mutex> lock(video_config_mutex_);
            video_config_sent_ = true;
            return true;
        }
        if (!grpc_module_)
        {
            RCLCPP_WARN(this->get_logger(), "GrpcModule not available, cannot publish VideoConfig");
            return false;
        }
        if (!video_module_)
        {
            RCLCPP_WARN(this->get_logger(), "VideoModule not available, cannot publish VideoConfig");
            return false;
        }

        std::vector<uint8_t> sps;
        std::vector<uint8_t> pps;
        std::vector<uint8_t> vps;
        {
            std::lock_guard<std::mutex> lock(video_config_mutex_);
            if (video_config_sent_)
            {
                return true;
            }
        }

        if (!video_module_->getSpsPps(sps, pps, vps))
        {
            RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                                 "VideoConfig not ready: waiting for cached SPS/PPS");
            return false;
        }

        const auto &video_config = video_module_->config();
        const GrpcModule::Codec codec = video_config.codec == trb::video::VideoModule::Config::Codec::kH265
                                           ? GrpcModule::Codec::kH265
                                           : GrpcModule::Codec::kH264;

        if (!grpc_module_->publishVideoConfig(video_config.width, video_config.height, video_config.fps, codec,
                                              sps, pps, vps, false, 0.0f, 0.0f, 0.0f))
        {
            return false;
        }

        {
            std::lock_guard<std::mutex> lock(video_config_mutex_);
            video_config_sent_ = true;
        }
        return true;
    }

    bool MainNode::publishAudioConfig()
    {
        if (shutting_down_.load(std::memory_order_acquire))
        {
            return false;
        }
        if (direct_running_mode_)
        {
            if (audio_module_)
            {
                audio_module_->markConfigPublished();
            }
            return true;
        }
        if (!audio_module_ || !audio_module_->isEnabled() || !grpc_module_)
        {
            return false;
        }
        if (audio_module_->isConfigPublished())
        {
            return true;
        }

        const auto &config = audio_module_->config();
        GrpcModule::AudioConfig audio_config;
        audio_config.sample_rate = config.sample_rate;
        audio_config.channels = config.channels;
        audio_config.frame_duration_ms = config.frame_duration_ms;
        audio_config.samples_per_channel = config.samples_per_channel;
        audio_config.bitrate_bps = config.bitrate_bps;
        audio_config.opus_inband_fec_enabled = config.opus_inband_fec_enabled;
        audio_config.opus_dtx_enabled = config.opus_dtx_enabled;
        if (!grpc_module_->publishAudioConfig(audio_config))
        {
            return false;
        }

        audio_module_->markConfigPublished();
        return true;
    }

    void MainNode::tryAdvanceNegotiation()
    {
        if (shutting_down_.load(std::memory_order_acquire))
        {
            return;
        }

        if (state_.load() != State::kNegotiating)
        {
            return;
        }
        if (!publishVideoConfig())
        {
            ensureNegotiationRetryTimer();
            return;
        }

        const bool audio_enabled = audio_module_ && audio_module_->isEnabled();
        const bool publishes_audio = audio_enabled && audio_module_->config().uplink_enabled;
        if (publishes_audio && !audio_module_->isConfigPublished() && !publishAudioConfig())
        {
            ensureNegotiationRetryTimer();
            return;
        }
        if (audio_enabled && !publishes_audio)
        {
            audio_module_->markConfigPublished();
        }

        stopNegotiationRetryTimer();
        setState(State::kRunning,
                 audio_enabled ? "local video/audio stream configs published"
                               : "local video stream config published");
        if (audio_module_)
        {
            audio_module_->start();
        }

        if (subscribe_vr_pose_flag_ || wantsRemoteAudio())
        {
            RCLCPP_INFO(this->get_logger(),
                        "Local stream sending is running; peer pose/audio receive setup will retry independently");
            ensurePeerSubscriptionRetryTimer();
        }
        else
        {
            std::lock_guard<std::mutex> lock(peer_receive_state_mutex_);
            peer_streams_subscribed_ = true;
        }
    }

    void MainNode::negotiationRetryTimerCallback()
    {
        static rclcpp::Clock steady_clock(RCL_STEADY_TIME);

        if (shutting_down_.load(std::memory_order_acquire))
        {
            stopNegotiationRetryTimer();
            return;
        }

        if (state_.load() != State::kNegotiating)
        {
            stopNegotiationRetryTimer();
            return;
        }

        if (!video_module_)
        {
            return;
        }

        if (!video_started_ || !video_module_->isRunning())
        {
            startVideoAndNegotiate();
            return;
        }

        if (video_module_->hasSpsPps())
        {
            tryAdvanceNegotiation();
            return;
        }

        const auto now = std::chrono::steady_clock::now();
        if (video_start_time_.time_since_epoch().count() != 0)
        {
            const auto startup_elapsed = now - video_start_time_;
            if (startup_elapsed >= kVideoStartupTimeout)
            {
                RCLCPP_WARN(this->get_logger(),
                            "Video startup stalled before SPS/PPS became ready; restarting video pipeline");
                stopVideo();
                resetVideoConfigState();
                startVideoAndNegotiate();
                return;
            }

            const bool should_request_keyframe =
                startup_elapsed >= kVideoStartupKeyframeDelay &&
                (last_video_keyframe_request_time_.time_since_epoch().count() == 0 ||
                 (now - last_video_keyframe_request_time_) >= kVideoKeyframeRequestInterval);
            if (should_request_keyframe)
            {
                last_video_keyframe_request_time_ = now;
                const bool requested = video_module_->requestKeyframe();
                RCLCPP_INFO_THROTTLE(this->get_logger(), steady_clock, 5000,
                                     requested
                                         ? "Video startup waiting for SPS/PPS: requested fresh IDR"
                                         : "Video startup waiting for SPS/PPS: fresh IDR request not ready yet");
            }
        }

        tryAdvanceNegotiation();
    }

    void MainNode::videoStatsTimerCallback()
    {
        if (shutting_down_.load(std::memory_order_acquire))
        {
            return;
        }

        const auto now = std::chrono::steady_clock::now();
        double elapsed_sec = 1.0;
        if (last_video_stats_time_.time_since_epoch().count() != 0)
        {
            elapsed_sec = std::max(0.001, std::chrono::duration<double>(now - last_video_stats_time_).count());
        }
        last_video_stats_time_ = now;

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
        std::string converter_output_format = "unknown";
        trb::udp::UdpManager::VideoStatsSnapshot udp_stats;
        bool have_video_module = false;
        bool have_udp_module = false;

        if (video_module_)
        {
            const auto video_stats = video_module_->consumeStats();
            have_video_module = true;
            capture_frames = video_stats.capture_frames;
            decode_frames = video_stats.decode_frames;
            convert_frames = video_stats.convert_frames;
            encode_frames = video_stats.encode_frames;
            decoder_drops = video_stats.decoder_drops;
            converter_pool_drops = video_stats.converter_pool_drops;
            converter_failures = video_stats.converter_failures;
            encoder_submit_failures = video_stats.encoder_submit_failures;
            decode_us_total = video_stats.decode_us_total;
            transform_us_total = video_stats.transform_us_total;
            if (!video_stats.converter_output_format.empty())
            {
                converter_output_format = video_stats.converter_output_format;
            }
            encode_us_total = video_stats.encode_us_total;
            undistort_frames = video_stats.undistort_frames;
            undistort_pool_drops = video_stats.undistort_pool_drops;
            undistort_failures = video_stats.undistort_failures;
            undistort_us_total = video_stats.undistort_us_total;
        }
        if (udp_module_ && udp_module_->isRunning())
        {
            udp_stats = udp_module_->consumeVideoStats();
            have_udp_module = true;
        }

        if (!have_video_module && !have_udp_module)
        {
            return;
        }

        const uint64_t conv_drop_total = converter_pool_drops + converter_failures;
        const uint64_t und_drop_total = undistort_pool_drops + undistort_failures;
        const uint64_t udp_drop_total = udp_stats.input_drops + udp_stats.queue_drops;
        const uint64_t udp_sockblk_total = udp_stats.send_block_events;
        const uint64_t udp_sockdrop_total = udp_stats.send_drop_packets;
        const bool has_activity =
            capture_frames > 0 ||
            decode_frames > 0 ||
            convert_frames > 0 ||
            undistort_frames > 0 ||
            encode_frames > 0 ||
            udp_stats.input_frames > 0 ||
            udp_stats.sent_frames > 0 ||
            udp_stats.queue_packets > 0 ||
            decoder_drops > 0 ||
            conv_drop_total > 0 ||
            und_drop_total > 0 ||
            encoder_submit_failures > 0 ||
            udp_drop_total > 0 ||
            udp_sockblk_total > 0 ||
            udp_sockdrop_total > 0;
        if (!has_activity)
        {
            return;
        }

        const double cap_fps = static_cast<double>(capture_frames) / elapsed_sec;
        const double dec_fps = static_cast<double>(decode_frames) / elapsed_sec;
        const double conv_fps = static_cast<double>(convert_frames) / elapsed_sec;
        const double und_fps = static_cast<double>(undistort_frames) / elapsed_sec;
        const double enc_fps = static_cast<double>(encode_frames) / elapsed_sec;
        const double tx_fps = static_cast<double>(udp_stats.sent_frames) / elapsed_sec;
        const double tx_mbps = (static_cast<double>(udp_stats.sent_bytes) * 8.0 / 1000000.0) / elapsed_sec;

        const double dec_ms = decode_frames > 0
                                  ? static_cast<double>(decode_us_total) / static_cast<double>(decode_frames) / 1000.0
                                  : 0.0;
        const double conv_ms = convert_frames > 0
                                   ? static_cast<double>(transform_us_total) / static_cast<double>(convert_frames) / 1000.0
                                   : 0.0;
        const double und_ms = undistort_frames > 0
                                  ? static_cast<double>(undistort_us_total) / static_cast<double>(undistort_frames) / 1000.0
                                  : 0.0;
        const double enc_ms = encode_frames > 0
                                  ? static_cast<double>(encode_us_total) / static_cast<double>(encode_frames) / 1000.0
                                  : 0.0;
        const double e2e_ms = udp_stats.end_to_end_samples > 0
                                  ? static_cast<double>(udp_stats.end_to_end_us_total) / static_cast<double>(udp_stats.end_to_end_samples) / 1000.0
                                  : 0.0;
        const auto avg_us_to_ms = [](uint64_t total_us, uint64_t samples) {
            return samples > 0 ? static_cast<double>(total_us) / static_cast<double>(samples) / 1000.0 : 0.0;
        };
        const double fec_queue_ms = avg_us_to_ms(udp_stats.fec_internal_wait_us_total, udp_stats.fec_internal_wait_samples);
        const double fec_compute_ms = avg_us_to_ms(udp_stats.fec_compute_us_total, udp_stats.fec_compute_samples);
        const double udp_queue_ms = avg_us_to_ms(udp_stats.send_queue_delay_us_total, udp_stats.send_queue_delay_samples);
        const double tail_queue_ms = avg_us_to_ms(udp_stats.tail_queue_delay_us_total, udp_stats.tail_queue_delay_samples);
        const double pacing_ms = avg_us_to_ms(udp_stats.pacing_us_total, udp_stats.sent_packets);
        const double udp_send_ms = avg_us_to_ms(udp_stats.send_syscall_us_total, udp_stats.sent_packets);
        const double pacer_debt_ms = static_cast<double>(udp_stats.pacer_debt_us) / 1000.0;
        const double pacer_expected_queue_ms = static_cast<double>(udp_stats.pacer_expected_queue_us) / 1000.0;
        const double pacer_adjusted_mbps = static_cast<double>(udp_stats.pacer_adjusted_bps) / 1000000.0;

        if (udp_drop_total > 0 || udp_sockdrop_total > 0)
        {
            requestVideoKeyframeRecovery("udp_drop", now);
        }
        else if (udp_sockblk_total >= kVideoBackpressureKeyframeThreshold)
        {
            requestVideoKeyframeRecovery("udp_backpressure", now);
        }

        RCLCPP_INFO(this->get_logger(),
                    "[VIDEO] fmt=%s fps=cap%.1f dec%.1f conv%.1f und%.1f enc%.1f tx%.1f %.1fMbps drop=dec%lu conv%lu und%lu enc%lu udp%lu sock=blk%lu drop%lu q=%zupkts/%.1fKB lat=dec%.2f conv%.2f und%.2f enc%.2f fecq%.2f fec%.2f udpq%.2f tailq%.2f pace%.2f send%.2f e2e=%.2fms pacer=debt%.2f q%.2f rate%.1fMbps",
                    converter_output_format.c_str(),
                    cap_fps,
                    dec_fps,
                    conv_fps,
                    und_fps,
                    enc_fps,
                    tx_fps,
                    tx_mbps,
                    static_cast<unsigned long>(decoder_drops),
                    static_cast<unsigned long>(conv_drop_total),
                    static_cast<unsigned long>(und_drop_total),
                    static_cast<unsigned long>(encoder_submit_failures),
                    static_cast<unsigned long>(udp_drop_total),
                    static_cast<unsigned long>(udp_sockblk_total),
                    static_cast<unsigned long>(udp_sockdrop_total),
                    udp_stats.queue_packets,
                    static_cast<double>(udp_stats.queue_bytes) / 1024.0,
                    dec_ms,
                    conv_ms,
                    und_ms,
                    enc_ms,
                    fec_queue_ms,
                    fec_compute_ms,
                    udp_queue_ms,
                    tail_queue_ms,
                    pacing_ms,
                    udp_send_ms,
                    e2e_ms,
                    pacer_debt_ms,
                    pacer_expected_queue_ms,
                    pacer_adjusted_mbps);
    }

    void MainNode::ensureNegotiationRetryTimer()
    {
        if (!canScheduleRosWork(*this, shutting_down_))
        {
            return;
        }

        if (!negotiation_retry_timer_)
        {
            try
            {
                negotiation_retry_timer_ = this->create_wall_timer(std::chrono::seconds(1), [this]() { negotiationRetryTimerCallback(); });
            }
            catch (const rclcpp::exceptions::RCLError &ex)
            {
                if (!shutting_down_.load(std::memory_order_acquire))
                {
                    RCLCPP_WARN(this->get_logger(), "Skip negotiation retry timer creation: %s", ex.what());
                }
            }
            return;
        }
        try
        {
            negotiation_retry_timer_->reset();
        }
        catch (const rclcpp::exceptions::RCLError &ex)
        {
            if (!shutting_down_.load(std::memory_order_acquire))
            {
                RCLCPP_WARN(this->get_logger(), "Skip negotiation retry timer reset: %s", ex.what());
            }
        }
    }

    void MainNode::stopNegotiationRetryTimer()
    {
        if (negotiation_retry_timer_)
        {
            negotiation_retry_timer_->cancel();
        }
    }

    void MainNode::resetVideoConfigState()
    {
        {
            std::lock_guard<std::mutex> lock(video_config_mutex_);
            video_config_sent_ = false;
            video_started_ = false;
        }
        video_start_time_ = std::chrono::steady_clock::time_point{};
        last_video_keyframe_request_time_ = std::chrono::steady_clock::time_point{};
        if (video_module_)
        {
            video_module_->clearSpsPps();
        }
    }

    void MainNode::stopVideo()
    {
        if (video_module_)
        {
            video_module_->stop();
        }
        {
            std::lock_guard<std::mutex> lock(video_config_mutex_);
            video_started_ = false;
        }
    }

    void MainNode::teardownActiveSession(const std::string &reason, State target_state)
    {
        if (shutting_down_.load(std::memory_order_acquire) && target_state != State::kConnecting)
        {
            return;
        }
        if (session_teardown_in_progress_.exchange(true, std::memory_order_acq_rel))
        {
            RCLCPP_INFO(this->get_logger(), "Session teardown already in progress; skip duplicate request (%s)", reason.c_str());
            return;
        }
        AtomicFlagClearGuard teardown_guard(session_teardown_in_progress_);

        setState(target_state, reason);

        stopRobotTelemetry();
        stopNegotiationRetryTimer();
        stopPeerSubscriptionRetryTimer();
        stopPairingRetryTimer();
        {
            std::lock_guard<std::mutex> lock(peer_receive_state_mutex_);
            paired_peer_session_id_.clear();
            peer_streams_subscribed_ = false;
            peer_subscribed_prefixes_.clear();
            if (audio_module_)
            {
                audio_module_->stop();
                audio_module_->resetConfigPublished();
                audio_module_->resetRemoteConfig(reason);
            }
        }
        stopVideo();
        resetVideoConfigState();

        if (target_state == State::kConnecting)
        {
            publisher_manifest_published_ = false;
            stopRegisteredSetupRetryTimer();
        }

        if (udp_module_)
        {
            switch (target_state)
            {
            case State::kPairing:
            case State::kRegistered:
                udp_module_->resetSession();
                break;
            case State::kConnecting:
                udp_module_->stop();
                break;
            default:
                break;
            }
        }

        if (target_state == State::kPairing)
        {
            enterPairingState();
        }
        updateRobotTelemetryLifecycle();
    }

    void MainNode::setState(State next, const std::string &reason)
    {
        const State previous = state_.exchange(next);
        if (previous == next)
        {
            return;
        }
        RCLCPP_INFO(this->get_logger(), "State transition: %s -> %s (%s)",
                    stateToString(previous), stateToString(next), reason.c_str());
    }

    const char *MainNode::stateToString(State state)
    {
        switch (state)
        {
        case State::kConnecting:
            return "connecting";
        case State::kRegistered:
            return "registered";
        case State::kPairing:
            return "pairing";
        case State::kNegotiating:
            return "negotiating";
        case State::kRunning:
            return "running";
        default:
            return "unknown";
        }
    }
} // namespace trb

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<trb::MainNode>();
    rclcpp::executors::MultiThreadedExecutor executor(rclcpp::ExecutorOptions(), 2);
    executor.add_node(node);
    executor.spin();
    rclcpp::shutdown();
    return 0;
}
