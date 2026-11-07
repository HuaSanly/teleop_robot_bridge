#include "udp/udp_module.hpp"

#include <algorithm>
#include <utility>

#include "udp/packet_codec.hpp"

namespace trb::udp {

namespace {
UdpManager::PacingMode parsePacingMode(const std::string& value) {
  if (value == "webrtc_like" || value == "webrtc" || value == "priority") {
    return UdpManager::PacingMode::WebRtcLike;
  }
  return UdpManager::PacingMode::Legacy;
}

const char* pacingModeToString(UdpManager::PacingMode mode) {
  switch (mode) {
    case UdpManager::PacingMode::WebRtcLike:
      return "webrtc_like";
    case UdpManager::PacingMode::Legacy:
    default:
      return "legacy";
  }
}

uint16_t readU16LE(const uint8_t* data) {
  return static_cast<uint16_t>(data[0]) |
         (static_cast<uint16_t>(data[1]) << 8);
}

uint64_t readU64LE(const uint8_t* data) {
  return static_cast<uint64_t>(readU16LE(data)) |
         (static_cast<uint64_t>(readU16LE(data + 2)) << 16) |
         (static_cast<uint64_t>(readU16LE(data + 4)) << 32) |
         (static_cast<uint64_t>(readU16LE(data + 6)) << 48);
}
}  // namespace

UdpModule::Config UdpModule::Config::configFromRosParam(ros::NodeHandle& nh) {
  Config c;

  nh.param<bool>("debug/direct_running_mode", c.direct_running_mode, false);

  auto& u = c.udp;
  nh.param<std::string>("udp/server_udp_ip", u.remote_ip, "");
  nh.param<int>("udp/server_udp_port", u.remote_port, 0);
  nh.param<std::string>("udp/bind_ip", u.bind_ip, "0.0.0.0");
  nh.param<int>("udp/bind_port", u.bind_port, 0);
  nh.param<bool>("udp/enable_receive", u.enable_receive, true);
  nh.param<bool>("udp/handshake/enabled", u.control_enabled, true);
  nh.param<double>("udp/handshake/interval_sec", u.control_hello_interval_sec, 1.0);
  nh.param<double>("udp/handshake/ping_interval_sec", u.control_ping_interval_sec, 5.0);
  nh.param<double>("udp/handshake/peer_lost_timeout_sec",
                   u.control_peer_lost_timeout_sec, 0.0);
  nh.param<int>("udp/recv_timeout_ms", u.recv_timeout_ms, 100);
  int max_payload_bytes = static_cast<int>(u.max_payload_bytes);
  nh.param<int>("udp/max_payload_bytes", max_payload_bytes, 1200);
  u.max_payload_bytes = static_cast<size_t>(std::max(0, max_payload_bytes));
  nh.param<bool>("udp/send/nonblocking", u.send_nonblocking, true);
  int send_buffer_bytes = u.send_buffer_bytes;
  nh.param<int>("udp/send/buffer_bytes", send_buffer_bytes, 0);
  u.send_buffer_bytes = send_buffer_bytes;
  nh.param<bool>("udp/pacing/enabled", u.pacing_enabled, true);
  std::string pacing_mode = pacingModeToString(u.pacing_mode);
  nh.param<std::string>("udp/pacing/mode", pacing_mode, pacing_mode);
  u.pacing_mode = parsePacingMode(pacing_mode);
  int pacing_bps = static_cast<int>(u.pacing_bps);
  nh.param<int>("udp/pacing/bps", pacing_bps, 0);
  u.pacing_bps = static_cast<uint64_t>(std::max(0, pacing_bps));
  int pacing_send_burst_interval_ms = static_cast<int>(u.pacing_send_burst_interval_ms);
  nh.param<int>("udp/pacing/send_burst_interval_ms", pacing_send_burst_interval_ms, pacing_send_burst_interval_ms);
  u.pacing_send_burst_interval_ms = static_cast<uint32_t>(std::max(0, pacing_send_burst_interval_ms));
  int pacing_max_burst_bytes = static_cast<int>(u.pacing_max_burst_bytes);
  nh.param<int>("udp/pacing/max_burst_bytes", pacing_max_burst_bytes, pacing_max_burst_bytes);
  u.pacing_max_burst_bytes = static_cast<size_t>(std::max(0, pacing_max_burst_bytes));
  int pacing_max_debt_ms = static_cast<int>(u.pacing_max_debt_ms);
  nh.param<int>("udp/pacing/max_debt_ms", pacing_max_debt_ms, pacing_max_debt_ms);
  u.pacing_max_debt_ms = static_cast<uint32_t>(std::max(1, pacing_max_debt_ms));
  int pacing_queue_time_limit_ms = static_cast<int>(u.pacing_queue_time_limit_ms);
  nh.param<int>("udp/pacing/queue_time_limit_ms", pacing_queue_time_limit_ms, pacing_queue_time_limit_ms);
  u.pacing_queue_time_limit_ms = static_cast<uint32_t>(std::max(1, pacing_queue_time_limit_ms));
  nh.param<double>("udp/pacing/max_drain_multiplier", u.pacing_max_drain_multiplier, u.pacing_max_drain_multiplier);
  u.pacing_max_drain_multiplier = std::max(1.0, u.pacing_max_drain_multiplier);
  nh.param<bool>("udp/pacing/account_audio", u.pacing_account_audio, u.pacing_account_audio);
  int pacing_max_packets_per_batch = static_cast<int>(u.pacing_max_packets_per_batch);
  nh.param<int>("udp/pacing/max_packets_per_batch", pacing_max_packets_per_batch, pacing_max_packets_per_batch);
  u.pacing_max_packets_per_batch = static_cast<uint32_t>(std::max(1, pacing_max_packets_per_batch));
  int queue_max_bytes = static_cast<int>(u.queue_max_bytes);
  nh.param<int>("udp/pacing/queue_max_bytes", queue_max_bytes, 300000);
  u.queue_max_bytes = static_cast<size_t>(std::max(0, queue_max_bytes));
  int queue_max_packets = static_cast<int>(u.queue_max_packets);
  nh.param<int>("udp/pacing/queue_max_packets", queue_max_packets, 0);
  u.queue_max_packets = static_cast<size_t>(std::max(0, queue_max_packets));

  bool fec_enabled = true;
  nh.param<bool>("udp/fec/enabled", fec_enabled, true);
  int fec_table_id = static_cast<int>(u.fec_table_id);
  nh.param<int>("udp/fec/table_id", fec_table_id, 1);
  u.fec_table_id = fec_enabled ? static_cast<uint8_t>(fec_table_id) : 0;
  int fec_input_queue_max_frames = static_cast<int>(u.fec_input_queue_max_frames);
  nh.param<int>("udp/fec/input_queue_max_frames", fec_input_queue_max_frames, 2);
  u.fec_input_queue_max_frames = static_cast<size_t>(std::max(1, fec_input_queue_max_frames));

  nh.param<bool>("udp/pose_udp/enabled", c.pose_enabled, false);
  auto& p = c.pose;
  p.enabled = c.pose_enabled;
  nh.param<std::string>("udp/pose_udp/frame_id/hmd", p.frame_id_hmd, "vr_hmd");
  nh.param<std::string>("udp/pose_udp/frame_id/left_controller", p.frame_id_left_controller, "vr_left_controller");
  nh.param<std::string>("udp/pose_udp/frame_id/right_controller", p.frame_id_right_controller, "vr_right_controller");
  nh.param<std::string>("udp/pose_udp/frame_id/left_aim", p.frame_id_left_aim, "vr_left_aim");
  nh.param<std::string>("udp/pose_udp/frame_id/right_aim", p.frame_id_right_aim, "vr_right_aim");
  nh.param<bool>("udp/pose_udp/publish_euler", p.publish_euler, true);
  nh.param<bool>("udp/pose_udp/publish_buttons", p.publish_buttons, true);
  nh.param<bool>("udp/pose_udp/publish_aim", p.publish_aim, true);

  return c;
}

UdpModule::UdpModule(ros::NodeHandle nh, Config config)
    : nh_(std::move(nh)), config_(std::move(config)) {}

UdpModule::~UdpModule() { stop(); }

bool UdpModule::start(const std::string& session_id) {
  if (config_.direct_running_mode) {
    ROS_INFO("UDP disabled in direct running mode: skipping socket, receive, FEC, pacing and control plane");
    return false;
  }

  if (udp_manager_) {
    return true;
  }

  if (config_.udp.remote_ip.empty() || config_.udp.remote_port <= 0) {
    ROS_WARN("UDP config missing remote_ip/remote_port; UdpModule will not start");
    return false;
  }

  ROS_INFO("UDP config loaded: remote=%s:%d pacing_mode=%s pacing_enabled=%d pacing_bps=%lu",
           config_.udp.remote_ip.c_str(), config_.udp.remote_port,
           pacingModeToString(config_.udp.pacing_mode),
           config_.udp.pacing_enabled ? 1 : 0,
           static_cast<unsigned long>(config_.udp.pacing_bps));
  ROS_INFO("UDP config loaded: queue_max_bytes=%zu queue_max_packets=%zu fec_table_id=%d fec_input_queue_max_frames=%zu",
           config_.udp.queue_max_bytes, config_.udp.queue_max_packets,
           static_cast<int>(config_.udp.fec_table_id),
           config_.udp.fec_input_queue_max_frames);

  udp_manager_ = std::make_unique<UdpManager>(config_.udp);
  udp_manager_->setSessionId(session_id);

  if (config_.pose_enabled) {
    pose_publisher_ = std::make_unique<PosePublisher>();
    if (!pose_publisher_->initialize(nh_, config_.pose)) {
      ROS_WARN("PosePublisher initialization failed");
      pose_publisher_.reset();
    }
  }

  udp_manager_->setDatagramCallback(
      [this](const uint8_t* data, size_t size) { onDatagram(data, size); });

  // Forward peer-lost signal from ControlPlane worker thread up to MainNode.
  udp_manager_->setPeerLostCallback([this]() {
    // Allow handshake-ready to fire again once the next peer connects.
    ready_fired_.store(false);
    if (on_peer_lost_) {
      on_peer_lost_();
    }
  });

  if (!udp_manager_->start()) {
    ROS_ERROR("UdpManager start failed");
    udp_manager_.reset();
    pose_publisher_.reset();
    return false;
  }

  ready_fired_.store(false);
  if (!config_.udp.control_enabled) {
    // No handshake required: fire ready callback immediately.
    if (on_ready_ && !ready_fired_.exchange(true)) {
      on_ready_();
    }
  } else {
    handshake_timer_ = nh_.createTimer(
        ros::Duration(0.2),
        &UdpModule::handshakeTimerCallback,
        this);
  }
  return true;
}

void UdpModule::stop() {
  if (handshake_timer_.hasStarted()) {
    handshake_timer_.stop();
  }
  if (udp_manager_) {
    udp_manager_->stop();
    udp_manager_.reset();
  }
  pose_publisher_.reset();
  ready_fired_.store(false);
}

void UdpModule::resetSession() {
  if (!udp_manager_) {
    return;
  }
  udp_manager_->resetSession();
  // Allow handshake-ready to fire again for the next peer.
  ready_fired_.store(false);
  // Resume polling for the next handshake if control plane is enabled.
  if (config_.udp.control_enabled && !handshake_timer_.hasStarted()) {
    handshake_timer_ = nh_.createTimer(
        ros::Duration(0.2),
        &UdpModule::handshakeTimerCallback,
        this);
  }
}

void UdpModule::requestHandshakeRefresh() {
  if (!udp_manager_ || !config_.udp.control_enabled) return;
  // This is an endpoint-binding refresh for the current session, not a new
  // peer handshake. Keep ready_fired_ set so ACK does not re-enter pairing.
  udp_manager_->requestUdpHello();
  if (!handshake_timer_.hasStarted()) {
    handshake_timer_ = nh_.createTimer(
        ros::Duration(0.2),
        &UdpModule::handshakeTimerCallback,
        this);
  }
}

bool UdpModule::isHandshakeReady() const {
  return udp_manager_ && udp_manager_->isHandshakeReady();
}

void UdpModule::sendH264Frame(const uint8_t* data, size_t size,
                              uint64_t capture_timestamp_us, bool keyframe) {
  if (udp_manager_) {
    udp_manager_->sendH264Frame(data, size, capture_timestamp_us, keyframe);
  }
}

void UdpModule::sendAudioPacket(const uint8_t* data, size_t size,
                                uint64_t timestamp_ntp) {
  if (udp_manager_) {
    udp_manager_->sendAudioPacket(data, size, timestamp_ntp);
  }
}

UdpManager::VideoStatsSnapshot UdpModule::consumeVideoStats() {
  if (!udp_manager_) return UdpManager::VideoStatsSnapshot{};
  return udp_manager_->consumeVideoStats();
}

void UdpModule::onDatagram(const uint8_t* data, size_t size) {
  if (size > 11 && data[0] == kAudioPacketType) {
    if (audio_cb_) {
      const uint64_t ts_ntp = readU64LE(data + 3);
      audio_cb_(data + 11, size - 11, ts_ntp);
    }
    return;
  }
  if (pose_publisher_ && pose_publisher_->isEnabled()) {
    pose_publisher_->handleDatagram(data, size);
  }
}

void UdpModule::handshakeTimerCallback(const ros::TimerEvent& /*event*/) {
  if (!udp_manager_) {
    if (handshake_timer_.hasStarted()) handshake_timer_.stop();
    return;
  }
  if (udp_manager_->isHandshakeReady()) {
    if (handshake_timer_.hasStarted()) handshake_timer_.stop();
    if (on_ready_ && !ready_fired_.exchange(true)) {
      on_ready_();
    }
    return;
  }
  ROS_WARN_THROTTLE(2.0, "Waiting for UDP handshake ACK...");
}

}  // namespace trb::udp
