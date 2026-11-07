#ifndef TELEOP_ROBOT_BRIDGE_MAIN_NODE_HPP_
#define TELEOP_ROBOT_BRIDGE_MAIN_NODE_HPP_

#include <atomic>
#include <memory>
#include <mutex>
#include <string>
#include <vector>
#include <cstdint>

#include <ros/ros.h>

#include "audio/audio_module.hpp"
#include "grpc/grpc_module.hpp"
#include "udp/udp_module.hpp"
#include "video/video_module.hpp"

class MainNode {
 public:
	explicit MainNode(ros::NodeHandle nh);
	~MainNode();

 private:
	void enterDirectRunningMode();
	void negotiationRetryTimerCallback(const ros::TimerEvent &event);
	void manifestRetryTimerCallback(const ros::TimerEvent &event);
	void poseSubscriptionRetryTimerCallback(const ros::TimerEvent &event);
	void videoStatsTimerCallback(const ros::TimerEvent &event);
	void enterPairingState();
	void onGrpcRegistered();
	void onGrpcHeartbeatFail();
	void onGrpcStreamEnd();
	void onUdpHandshakeReady();
	void onUdpPeerLost();
	void onSignalingEvent(const signaling::EventMessage &msg);
	void onEncodedVideoFrame(const uint8_t *data, size_t size, uint64_t timestamp_us, bool keyframe);
	void onEncodedAudioPacket(const uint8_t *data, size_t size, uint64_t timestamp_ntp);
	void onSpsPpsCaptured(const std::vector<uint8_t> &sps,
	                      const std::vector<uint8_t> &pps,
	                      const std::vector<uint8_t> &vps);
	bool publishVideoConfig();
	bool publishAudioConfig();
	bool publishMediaManifest();
	void tryAdvanceNegotiation();
	void ensureManifestRetryTimer();
	void stopManifestRetryTimer();
	void beginDefaultPoseSubscription(const std::string &peer_session_id);
	bool tryDefaultPoseSubscription();
	void clearDefaultPoseSubscription();
	void ensureNegotiationRetryTimer();
	void stopNegotiationRetryTimer();
	void startVideoAndNegotiate();
	void startUdpAfterManifest();
	void scheduleGrpcReconnect(const std::string &reason);

	enum class State {
		kConnecting = 0,
		kRegistered = 1,
		kPairing = 2,
		kNegotiating = 3,
		kRunning = 4,
	};

	// Stop video/audio streams, clear negotiation state, and transition to
	// `target_state`. Used for every downstream failure/disconnect path
	// (UNPAIR, UDP peer-lost, gRPC stream end, heartbeat fail) so the node
	// self-heals back to an earlier lifecycle stage without a restart.
	// target_state semantics:
	//   kPairing    - keep gRPC registration & UDP socket; reset UDP session.
	//   kRegistered - keep gRPC registration & UDP socket; stop AV only.
	//   kConnecting - tear down UDP entirely; await re-register.
	void teardownActiveSession(const std::string &reason, State target_state);

	void setState(State next, const std::string &reason);
	static const char *stateToString(State s);

	ros::NodeHandle nh_;

	std::unique_ptr<GrpcModule> grpc_module_;
	std::unique_ptr<trb::audio::AudioModule> audio_module_;
	std::unique_ptr<trb::udp::UdpModule> udp_module_;
	std::unique_ptr<trb::video::VideoModule> video_module_;

	ros::Timer negotiation_retry_timer_;
	ros::Timer manifest_retry_timer_;
	ros::Timer pose_subscription_retry_timer_;
	ros::Timer video_stats_timer_;
	ros::Timer grpc_stream_end_timer_;
	ros::Timer grpc_reconnect_timer_;
	std::atomic<State> state_{State::kConnecting};
	bool direct_running_mode_{false};

	std::mutex video_config_mutex_;
	bool video_config_sent_{false};
	bool video_started_{false};
	std::atomic<bool> media_manifest_published_{false};
	std::mutex pose_subscription_mutex_;
	std::string pose_publisher_session_id_;
	bool pose_subscription_ready_{false};

	// Camera FOV table loaded from config/fov.yaml (parameter
	// `video/fov_table`). Looked up by encoded width x height to populate
	// the schema-v1 video StreamConfig payload.
	struct FovEntry {
		uint32_t width{0};
		uint32_t height{0};
		float hfov{0.f};
		float vfov{0.f};
		float dfov{0.f};
	};
	std::vector<FovEntry> fov_table_;
	void loadFovTable();
};

#endif  // TELEOP_ROBOT_BRIDGE_MAIN_NODE_HPP_
