#include "grpc/grpc_module.hpp"

#include <algorithm>
#include <cstdio>
#include <iomanip>
#include <locale>
#include <sstream>

#include "signaling.pb.h"

namespace {
constexpr uint32_t kVideoPrefix = 0x01;
constexpr uint32_t kPosePrefix = 0x02;
constexpr uint32_t kAudioPrefix = 0x04;
constexpr uint32_t kMediaConfigSchemaVersion = 1;

std::string base64Encode(const std::vector<uint8_t>& data) {
  static constexpr char kAlphabet[] =
      "ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789+/";
  std::string encoded;
  encoded.reserve(((data.size() + 2) / 3) * 4);

  size_t i = 0;
  while (i + 3 <= data.size()) {
    const uint32_t value = (static_cast<uint32_t>(data[i]) << 16) |
                           (static_cast<uint32_t>(data[i + 1]) << 8) |
                           static_cast<uint32_t>(data[i + 2]);
    encoded.push_back(kAlphabet[(value >> 18) & 0x3f]);
    encoded.push_back(kAlphabet[(value >> 12) & 0x3f]);
    encoded.push_back(kAlphabet[(value >> 6) & 0x3f]);
    encoded.push_back(kAlphabet[value & 0x3f]);
    i += 3;
  }

  const size_t remaining = data.size() - i;
  if (remaining == 1) {
    const uint32_t value = static_cast<uint32_t>(data[i]) << 16;
    encoded.push_back(kAlphabet[(value >> 18) & 0x3f]);
    encoded.push_back(kAlphabet[(value >> 12) & 0x3f]);
    encoded.append("==");
  } else if (remaining == 2) {
    const uint32_t value = (static_cast<uint32_t>(data[i]) << 16) |
                           (static_cast<uint32_t>(data[i + 1]) << 8);
    encoded.push_back(kAlphabet[(value >> 18) & 0x3f]);
    encoded.push_back(kAlphabet[(value >> 12) & 0x3f]);
    encoded.push_back(kAlphabet[(value >> 6) & 0x3f]);
    encoded.push_back('=');
  }
  return encoded;
}

std::string makeVideoConfigJson(uint32_t width, uint32_t height, uint32_t fps,
                                bool is_h265,
                                const std::vector<uint8_t>& sps,
                                const std::vector<uint8_t>& pps,
                                const std::vector<uint8_t>& vps,
                                bool has_fov, float hfov_deg,
                                float vfov_deg, float dfov_deg) {
  std::ostringstream json;
  json.imbue(std::locale::classic());
  json << std::setprecision(9)
       << '{'
       << "\"codec\":\"" << (is_h265 ? "H265" : "H264") << "\","
       << "\"width\":" << width << ','
       << "\"height\":" << height << ','
       << "\"fps\":" << fps << ','
       << "\"sps\":\"" << base64Encode(sps) << "\","
       << "\"pps\":\"" << base64Encode(pps) << "\","
       << "\"vps\":\"" << (is_h265 ? base64Encode(vps) : std::string()) << '"';
  if (has_fov) {
    json << ",\"fov\":{"
         << "\"hfov\":" << hfov_deg << ','
         << "\"vfov\":" << vfov_deg << ','
         << "\"dfov\":" << dfov_deg << '}';
  }
  json << '}';
  return json.str();
}
}

GrpcModule::Config GrpcModule::configFromRosParam(ros::NodeHandle& nh) {
  Config cfg;
  nh.param<std::string>("grpc/server_grpc_ip", cfg.server_grpc_ip, "");
  nh.param<int>("grpc/server_grpc_port", cfg.server_grpc_port, 0);
  nh.param<bool>("grpc/use_ssl", cfg.use_ssl, false);
  nh.param<std::string>("grpc/device_id", cfg.device_id, "");
  nh.param<std::string>("grpc/expected_device_type_code",
                        cfg.expected_device_type_code, "");
  nh.param<std::string>("grpc/runtime/firmware_version", cfg.firmware_version, "");
  nh.param<std::string>("grpc/runtime/software_version", cfg.software_version, "teleop_robot_bridge");
  nh.param<std::string>("grpc/runtime/operating_system", cfg.operating_system, "Linux");
  nh.param<std::string>("grpc/runtime/operating_system_version", cfg.operating_system_version, "");
  nh.param<std::string>("grpc/runtime/runtime_version", cfg.runtime_version, "ROS1");
  nh.param<std::string>("grpc/runtime/build_id", cfg.build_id, "");
  if (!nh.getParam("grpc/runtime/capabilities", cfg.capabilities)) {
    cfg.capabilities = {"publisher_streams", "udp_media"};
  }
  nh.param<int>("grpc/rpc_timeout_ms", cfg.rpc_timeout_ms, 5000);
  nh.param<double>("grpc/register_retry_sec", cfg.register_retry_sec, 3.0);
  nh.param<double>("grpc/heartbeat_sec", cfg.heartbeat_sec, 15.0);
  return cfg;
}

GrpcModule::GrpcModule(ros::NodeHandle& nh, const Config& config)
    : nh_(nh), config_(config) {
  manager_ = std::make_unique<GrpcManager>(config_);
}

GrpcModule::~GrpcModule() {
  stop();
  manager_.reset();
}

void GrpcModule::start() {
  tryRegister();
  if (!registered_.load() && !register_retry_timer_.hasStarted()) {
    register_retry_timer_ = nh_.createTimer(
        ros::Duration(config_.register_retry_sec),
        &GrpcModule::onRegisterRetry,
        this);
    ROS_WARN("gRPC register failed, entering pending state. Will retry every %.1f seconds.",
             config_.register_retry_sec);
  }
}

void GrpcModule::stop() {
  // Cancel any in-flight RPC first so timer callbacks currently mid-RPC
  // return immediately; otherwise ros::Timer::stop() would block on the
  // running callback for up to rpc_timeout_ms (~5s).
  if (manager_) {
    manager_->CancelPendingRpcs();
  }
  if (register_retry_timer_.hasStarted()) {
    register_retry_timer_.stop();
  }
  if (heartbeat_timer_.hasStarted()) {
    heartbeat_timer_.stop();
  }
  if (manager_) {
    manager_->StopEventStream();
  }
}

const std::string& GrpcModule::sessionId() const {
  static const std::string empty;
  return manager_ ? manager_->session_id() : empty;
}

const std::string& GrpcModule::deviceTypeCode() const {
  static const std::string empty;
  return manager_ ? manager_->device_type_code() : empty;
}

void GrpcModule::reconnect() {
  if (!manager_) return;
  registered_.store(false);
  if (heartbeat_timer_.hasStarted()) heartbeat_timer_.stop();
  manager_->ClearSession();
  tryRegister();
  if (!registered_.load() && !register_retry_timer_.hasStarted()) {
    register_retry_timer_ = nh_.createTimer(
        ros::Duration(config_.register_retry_sec),
        &GrpcModule::onRegisterRetry, this);
  }
}

bool GrpcModule::pingNow() {
  return manager_ && registered_.load() && manager_->Heartbeat();
}

void GrpcModule::tryRegister() {
  if (registered_.load() || !manager_) {
    return;
  }
  if (manager_->Register()) {
    registered_.store(true);
    if (register_retry_timer_.hasStarted()) {
      register_retry_timer_.stop();
    }
    if (!heartbeat_timer_.hasStarted()) {
      heartbeat_timer_ = nh_.createTimer(
          ros::Duration(config_.heartbeat_sec),
          &GrpcModule::onHeartbeat,
          this);
    }
    ROS_INFO_STREAM("gRPC register success. session_id=" << manager_->session_id());
    if (on_registered_) {
      on_registered_();
    }
  } else {
    ROS_WARN("gRPC register failed, will retry...");
  }
}

void GrpcModule::onRegisterRetry(const ros::TimerEvent& /*event*/) {
  tryRegister();
}

void GrpcModule::onHeartbeat(const ros::TimerEvent& /*event*/) {
  if (!manager_ || !registered_.load()) {
    return;
  }
  if (manager_->Heartbeat()) {
    return;
  }
  ROS_WARN("gRPC heartbeat failed, will re-register");
  registered_.store(false);
  manager_->StopEventStream();
  if (heartbeat_timer_.hasStarted()) {
    heartbeat_timer_.stop();
  }
  if (on_heartbeat_fail_) {
    on_heartbeat_fail_();
  }
  if (!register_retry_timer_.hasStarted()) {
    register_retry_timer_ = nh_.createTimer(
        ros::Duration(config_.register_retry_sec),
        &GrpcModule::onRegisterRetry,
        this);
  }
}

void GrpcModule::startEventStream() {
  if (!manager_) return;
  manager_->StartEventStream(
      [this](const signaling::EventMessage& msg) {
        if (on_event_) on_event_(msg);
      },
      [this](const grpc::Status& status) {
        ROS_WARN("gRPC EventStream ended unexpectedly: code=%d message=%s",
                 static_cast<int>(status.error_code()),
                 status.error_message().c_str());
        if (on_stream_end_) on_stream_end_();
      });
}

bool GrpcModule::acceptPair(const std::string& peer_session_id) {
  if (!manager_) return false;
  return manager_->AcceptPair(peer_session_id);
}

bool GrpcModule::subscribePoseStream(const std::string& publisher_session_id) {
  if (!manager_) return false;
  return manager_->SubscribeStreams(publisher_session_id, {kPosePrefix});
}

bool GrpcModule::publishMediaManifest(bool publish_audio) {
  if (!manager_) return false;

  std::vector<GrpcManager::StreamDeclaration> streams;
  streams.push_back({
      kVideoPrefix, "video", "Stereo Video",
      "SBS encoded camera stream from the robot", "video", "video"});
  if (publish_audio) {
    streams.push_back({
        kAudioPrefix, "audio", "Robot Audio",
        "Opus microphone uplink from the robot", "audio", "audio_opus"});
  }
  return manager_->PublishStreamManifest(streams);
}

bool GrpcModule::publishVideoConfig(uint32_t width, uint32_t height, uint32_t fps,
                                    Codec codec,
                                    const std::vector<uint8_t>& sps,
                                    const std::vector<uint8_t>& pps,
                                    const std::vector<uint8_t>& vps,
                                    bool has_fov, float hfov_deg,
                                    float vfov_deg, float dfov_deg) {
  if (!manager_) {
    ROS_WARN("gRPC manager not available, cannot send video StreamConfig");
    return false;
  }

  const bool is_h265 = (codec == Codec::kH265);
  if (is_h265 && vps.empty()) {
    ROS_WARN("Video StreamConfig codec=H265 but VPS is empty; aborting publish");
    return false;
  }

  ROS_INFO("====== Video StreamConfig ======");
  ROS_INFO("  codec: %s", is_h265 ? "H265" : "H264");
  ROS_INFO("  width: %u", width);
  ROS_INFO("  height: %u", height);
  ROS_INFO("  fps: %u", fps);
  ROS_INFO("  sps_size: %zu bytes", sps.size());
  ROS_INFO("  pps_size: %zu bytes", pps.size());
  if (is_h265) {
    ROS_INFO("  vps_size: %zu bytes", vps.size());
  }
  if (has_fov) {
    ROS_INFO("  fov: hfov=%.2f vfov=%.2f dfov=%.2f deg",
             hfov_deg, vfov_deg, dfov_deg);
  } else {
    ROS_INFO("  fov: <not set>");
  }

  std::string sps_hex;
  for (size_t i = 0; i < std::min(sps.size(), size_t(32)); ++i) {
    char buf[4];
    snprintf(buf, sizeof(buf), "%02X ", sps[i]);
    sps_hex += buf;
  }
  if (sps.size() > 32) sps_hex += "...";
  ROS_INFO("  sps: %s", sps_hex.c_str());

  std::string pps_hex;
  for (size_t i = 0; i < std::min(pps.size(), size_t(32)); ++i) {
    char buf[4];
    snprintf(buf, sizeof(buf), "%02X ", pps[i]);
    pps_hex += buf;
  }
  if (pps.size() > 32) pps_hex += "...";
  ROS_INFO("  pps: %s", pps_hex.c_str());

  if (is_h265) {
    std::string vps_hex;
    for (size_t i = 0; i < std::min(vps.size(), size_t(32)); ++i) {
      char buf[4];
      snprintf(buf, sizeof(buf), "%02X ", vps[i]);
      vps_hex += buf;
    }
    if (vps.size() > 32) vps_hex += "...";
    ROS_INFO("  vps: %s", vps_hex.c_str());
  }
  ROS_INFO("==================================");

  const std::string payload = makeVideoConfigJson(
      width, height, fps, is_h265, sps, pps, vps,
      has_fov, hfov_deg, vfov_deg, dfov_deg);
  ROS_INFO("  JSON payload: %s", payload.c_str());
  if (!manager_->PublishStreamConfig(
          kVideoPrefix, kMediaConfigSchemaVersion, payload)) {
    ROS_WARN("Video StreamConfig publish failed");
    return false;
  }
  ROS_INFO("Video StreamConfig published successfully");
  return true;
}

bool GrpcModule::publishAudioConfig(const trb::audio::AudioManager::Config& cfg) {
  if (!manager_) return false;
  std::ostringstream json;
  json << '{'
       << "\"codec\":\"OPUS\","
       << "\"sample_rate\":" << cfg.sample_rate << ','
       << "\"channels\":" << cfg.channels << ','
       << "\"frame_duration_ms\":" << cfg.frame_duration_ms << ','
       << "\"samples_per_channel\":" << cfg.samples_per_channel << ','
       << "\"bitrate_bps\":" << cfg.bitrate_bps << ','
       << "\"opus_inband_fec_enabled\":"
       << (cfg.opus_inband_fec_enabled ? "true" : "false") << ','
       << "\"opus_dtx_enabled\":"
       << (cfg.opus_dtx_enabled ? "true" : "false")
       << '}';
  const std::string payload = json.str();
  if (!manager_->PublishStreamConfig(
          kAudioPrefix, kMediaConfigSchemaVersion, payload)) {
    ROS_WARN("Publish audio StreamConfig failed");
    return false;
  }
  ROS_INFO("Audio StreamConfig published successfully");
  return true;
}
