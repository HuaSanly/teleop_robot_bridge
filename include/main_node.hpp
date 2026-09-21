#ifndef MAIN_NODE_H
#define MAIN_NODE_H

#include <atomic>
#include <chrono>
#include <cstddef>
#include <cstdint>
#include <deque>
#include <functional>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"

#include "teleop_robot_bridge/msg/agv_battery_state.hpp"
#include "teleop_robot_bridge/msg/agv_device_state.hpp"
#include "teleop_robot_bridge/msg/temperature.hpp"

#include "audio/audio_module.hpp"
#include "grpc/grpc_module.hpp"
#include "telemetry/robot_telemetry_types.hpp"
#include "udp/udp_module.hpp"
#include "video/video_module.hpp"
#include "video/wrist_camera_publisher.hpp"

namespace trb
{
    class MainNode : public rclcpp::Node
    {
    public:
        MainNode();
        ~MainNode();

    private:
        enum class State
        {
            kConnecting = 0,
            kRegistered = 1,
            kPairing = 2,
            kNegotiating = 3,
            kRunning = 4,
        };

        void configureLogging();
        void loadPairingParams();
        void loadRobotTelemetryParams();
        void enterDirectRunningMode();
        void enterPairingState();
        void startVideoAndNegotiate();
        void stopVideo();
        void startRobotTelemetry();
        void stopRobotTelemetry();
        void updateRobotTelemetryLifecycle();
        bool isRobotTelemetryOnline() const;
        void telemetryTimerCallback();
        void jointTelemetryTimerCallback();
        void publishRobotTelemetrySnapshot();
        void publishRobotJointTelemetrySnapshot();
        void onTelemetryBatteryState(const teleop_robot_bridge::msg::AgvBatteryState::SharedPtr msg);
        void onTelemetryTemperature(const teleop_robot_bridge::msg::Temperature::SharedPtr msg);
        void onTelemetryDeviceState(const teleop_robot_bridge::msg::AgvDeviceState::SharedPtr msg);
        void onJointState(const sensor_msgs::msg::JointState::SharedPtr msg);
        void teardownActiveSession(const std::string &reason, State target_state);
        void postSessionWork(std::function<void()> work, const char *label);
        void drainSessionWork();

        void onGrpcRegistered();
        void onGrpcHeartbeatFail();
        void onGrpcStreamEnd();
        void completeRegisteredSetup();
        void registeredSetupRetryTimerCallback();
        void ensureRegisteredSetupRetryTimer();
        void stopRegisteredSetupRetryTimer();
        void onUdpHandshakeReady();
        void onUdpPeerLost();
        void handleUdpPeerLost();
        void onSignalingEvent(const signaling::EventMessage &msg);
        void handleSignalingEvent(const signaling::EventMessage &msg);
        bool ensurePeerSubscriptions();
        void peerSubscriptionRetryTimerCallback();
        void ensurePeerSubscriptionRetryTimer();
        void stopPeerSubscriptionRetryTimer();
        void resetPeerSubscriptionState();
        bool fetchRemoteAudioConfig();
        bool validateRemoteAudioConfig(const GrpcModule::AudioConfig &config,
                       trb::audio::AudioModule::Config &remote_config,
                       std::string &message) const;
        bool wantsRemoteAudio() const;
        void onEncodedVideoFrame(const uint8_t *data, size_t size, uint64_t timestamp_us, bool keyframe);
        void onEncodedAudioPacket(const uint8_t *data, size_t size, uint64_t timestamp_ntp);
        void onSpsPpsCaptured(const std::vector<uint8_t> &sps,
                      const std::vector<uint8_t> &pps,
                      const std::vector<uint8_t> &vps);
        void requestVideoKeyframeRecovery(const char *reason, std::chrono::steady_clock::time_point now);

        void negotiationRetryTimerCallback();
        void pairingRetryTimerCallback();
        void videoStatsTimerCallback();
        void ensureNegotiationRetryTimer();
        void stopNegotiationRetryTimer();
        void ensurePairingRetryTimer();
        void stopPairingRetryTimer();
        bool tryActivePairing();

        bool publishLocalStreamManifest();
        bool publishVideoConfig();
        bool publishAudioConfig();
        void tryAdvanceNegotiation();
        void resetVideoConfigState();
        void loadFovTable();

        void setState(State next, const std::string &reason);
        static const char *stateToString(State state);

        std::unique_ptr<GrpcModule> grpc_module_;
        std::unique_ptr<trb::audio::AudioModule> audio_module_;
        std::unique_ptr<trb::udp::UdpModule> udp_module_;
        std::unique_ptr<trb::video::VideoModule> video_module_;
        std::unique_ptr<trb::video::WristCameraPublisher> left_wrist_camera_;
        std::unique_ptr<trb::video::WristCameraPublisher> right_wrist_camera_;

        void startWristCameras();
        void stopWristCameras();

        rclcpp::TimerBase::SharedPtr negotiation_retry_timer_;
        rclcpp::TimerBase::SharedPtr peer_subscription_retry_timer_;
        rclcpp::TimerBase::SharedPtr pairing_retry_timer_;
        rclcpp::TimerBase::SharedPtr registered_setup_retry_timer_;
        rclcpp::TimerBase::SharedPtr session_work_timer_;
        rclcpp::TimerBase::SharedPtr video_stats_timer_;
        rclcpp::TimerBase::SharedPtr telemetry_timer_;
        rclcpp::TimerBase::SharedPtr joint_telemetry_timer_;
        rclcpp::CallbackGroup::SharedPtr peer_subscription_callback_group_;
        std::mutex peer_subscription_timer_mutex_;
        rclcpp::Subscription<teleop_robot_bridge::msg::AgvBatteryState>::SharedPtr telemetry_battery_sub_;
        rclcpp::Subscription<teleop_robot_bridge::msg::Temperature>::SharedPtr telemetry_temperature_sub_;
        rclcpp::Subscription<teleop_robot_bridge::msg::AgvDeviceState>::SharedPtr telemetry_device_state_sub_;
        rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_sub_;
        std::chrono::steady_clock::time_point last_video_stats_time_;
        std::chrono::steady_clock::time_point video_start_time_{};
        std::chrono::steady_clock::time_point last_video_keyframe_request_time_{};

        std::atomic<State> state_{State::kConnecting};
        std::atomic<bool> shutting_down_{false};
        std::atomic<bool> session_teardown_in_progress_{false};
        bool direct_running_mode_{false};

        std::mutex session_work_mutex_;
        std::deque<std::function<void()>> session_work_queue_;

        bool subscribe_vr_pose_flag_{false};
        bool pair_auto_accept_{true};
        bool pair_auto_request_{true};
        bool pair_list_unpaired_on_start_{false};
        std::string desired_peer_session_id_;
        std::string desired_peer_device_type_code_;
        std::string paired_peer_session_id_;
        std::string pair_mode_{"passive"};
        std::mutex peer_receive_state_mutex_;
        bool publisher_manifest_published_{false};
        bool peer_streams_subscribed_{false};
        std::vector<uint8_t> peer_subscribed_prefixes_;

        std::mutex video_config_mutex_;
        bool video_config_sent_{false};
        bool video_started_{false};

        struct FovEntry
        {
            uint32_t width{0};
            uint32_t height{0};
            float hfov{0.0f};
            float vfov{0.0f};
            float dfov{0.0f};
        };
        std::vector<FovEntry> fov_table_;

        struct RobotTelemetryConfig
        {
            bool enabled{false};
            double period_sec{1.0};
            telemetry::RobotTelemetrySnapshot snapshot;
        };

        struct RobotJointTelemetryConfig
        {
            bool enabled{false};
            double period_sec{0.02};
            std::string joint_state_topic{"/joint_states"};
            telemetry::RobotJointTelemetrySnapshot snapshot;
        };

        std::mutex telemetry_snapshot_mutex_;
        RobotTelemetryConfig telemetry_config_;
        std::mutex joint_telemetry_snapshot_mutex_;
        RobotJointTelemetryConfig joint_telemetry_config_;
        std::atomic<bool> telemetry_active_{false};
        std::atomic<bool> joint_telemetry_active_{false};

        std::atomic<uint64_t> dropped_video_before_running_{0};
    };
} // namespace trb

#endif // MAIN_NODE_H
