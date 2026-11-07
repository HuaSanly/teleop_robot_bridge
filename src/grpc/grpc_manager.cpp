#include "grpc/grpc_manager.hpp"

#include <chrono>

#include <grpc/grpc.h>
#include <ros/ros.h>

GrpcManager::GrpcManager(const GrpcConfig &config)
    : config_(config) {
  auto channel = CreateChannel();
  stub_ = signaling::Signaling::NewStub(channel);
}

GrpcManager::~GrpcManager() {
  CancelPendingRpcs();
  StopEventStream();
}

GrpcManager::RpcScope::RpcScope(GrpcManager &mgr, grpc::ClientContext &ctx)
    : mgr_(mgr), ctx_(ctx) {
  if (mgr_.shutting_down_.load()) {
    shutting_down_ = true;
    return;
  }
  std::lock_guard<std::mutex> lk(mgr_.pending_rpc_mu_);
  mgr_.pending_rpc_ctxs_.insert(&ctx_);
}

GrpcManager::RpcScope::~RpcScope() {
  if (shutting_down_) return;
  std::lock_guard<std::mutex> lk(mgr_.pending_rpc_mu_);
  mgr_.pending_rpc_ctxs_.erase(&ctx_);
}

void GrpcManager::CancelPendingRpcs() {
  shutting_down_.store(true);
  {
    std::lock_guard<std::mutex> lk(pending_rpc_mu_);
    for (grpc::ClientContext *ctx : pending_rpc_ctxs_) {
      if (ctx) ctx->TryCancel();
    }
  }
  // Also kick the long-lived event stream so its blocking Read returns.
  if (stream_context_) {
    stream_context_->TryCancel();
  }
}

std::shared_ptr<grpc::Channel> GrpcManager::CreateChannel() const {
  std::string target = BuildTarget();
  grpc::ChannelArguments args;
  args.SetInt(GRPC_ARG_ENABLE_HTTP_PROXY, 0);
  if (config_.use_ssl) {
    grpc::SslCredentialsOptions ssl_opts;
    return grpc::CreateCustomChannel(target, grpc::SslCredentials(ssl_opts), args);
  }
  return grpc::CreateCustomChannel(target, grpc::InsecureChannelCredentials(), args);
}

std::string GrpcManager::BuildTarget() const {
  return config_.server_grpc_ip + ":" + std::to_string(config_.server_grpc_port);
}

void GrpcManager::ApplyDeadline(grpc::ClientContext &context) const {
  const int timeout_ms = std::max(1, config_.rpc_timeout_ms);
  context.set_deadline(std::chrono::system_clock::now() + std::chrono::milliseconds(timeout_ms));
}

bool GrpcManager::Register() {
  if (!stub_ || shutting_down_.load()) {
    return false;
  }

  signaling::RegisterRequest request;
  request.set_device_id(config_.device_id);
  auto *runtime = request.mutable_runtime();
  if (!config_.firmware_version.empty()) {
    runtime->set_firmware_version(config_.firmware_version);
  }
  if (!config_.software_version.empty()) {
    runtime->set_software_version(config_.software_version);
  }
  if (!config_.operating_system.empty()) {
    runtime->set_operating_system(config_.operating_system);
  }
  if (!config_.operating_system_version.empty()) {
    runtime->set_operating_system_version(config_.operating_system_version);
  }
  if (!config_.runtime_version.empty()) {
    runtime->set_runtime_version(config_.runtime_version);
  }
  if (!config_.build_id.empty()) {
    runtime->set_build_id(config_.build_id);
  }
  for (const auto &capability : config_.capabilities) {
    if (!capability.empty()) runtime->add_capabilities(capability);
  }

  signaling::RegisterResponse response;
  grpc::ClientContext context;
  ApplyDeadline(context);
  RpcScope scope(*this, context);
  if (scope.shuttingDown()) return false;
  ROS_INFO("Attempting gRPC Register to %s with timeout=%d ms",
           BuildTarget().c_str(), config_.rpc_timeout_ms);
  grpc::Status status = stub_->Register(&context, request, &response);

  if (!status.ok()) {
    ROS_WARN("gRPC Register RPC failed: code=%d message=%s",
             static_cast<int>(status.error_code()), status.error_message().c_str());
    return false;
  }

  if (!response.success()) {
    ROS_WARN("gRPC Register rejected by server: %s", response.message().c_str());
    return false;
  }

  session_id_ = response.session_id();
  device_type_code_ = response.device_type_code();
  if (!config_.expected_device_type_code.empty() &&
      device_type_code_ != config_.expected_device_type_code) {
    ROS_ERROR("Backend device type mismatch: expected=%s actual=%s. The backend registration record is authoritative; check the web device record.",
              config_.expected_device_type_code.c_str(),
              device_type_code_.empty() ? "<not returned>" : device_type_code_.c_str());
  }
  ROS_INFO("gRPC Register accepted: session_id=%s device_type_code=%s hardware_generation=%u hardware_version=%s hardware_revision=%s",
           session_id_.c_str(),
           device_type_code_.empty() ? "<not returned>" : device_type_code_.c_str(),
           response.has_hardware() ? response.hardware().hardware_generation() : 0U,
           response.has_hardware() && !response.hardware().hardware_version().empty()
               ? response.hardware().hardware_version().c_str() : "<not set>",
           response.has_hardware() && !response.hardware().hardware_revision().empty()
               ? response.hardware().hardware_revision().c_str() : "<not set>");
  return true;
}

const std::string &GrpcManager::session_id() const {
  return session_id_;
}

const std::string &GrpcManager::device_type_code() const {
  return device_type_code_;
}

void GrpcManager::ClearSession() {
  StopEventStream();
  session_id_.clear();
  device_type_code_.clear();
}

bool GrpcManager::AcceptPair(const std::string &peer_session_id) {
  if (!stub_ || session_id_.empty() || shutting_down_.load()) {
    return false;
  }

  signaling::PairRequest request;
  request.set_op(signaling::PairRequest::PAIR);
  request.set_peer_session_id(peer_session_id);
  request.set_state(signaling::PairRequest::ACCEPT);

  signaling::PairResponse response;
  grpc::ClientContext context;
  ApplyDeadline(context);
  context.AddMetadata("session-id", session_id_);
  RpcScope scope(*this, context);
  if (scope.shuttingDown()) return false;
  grpc::Status status = stub_->Pair(&context, request, &response);

  if (!status.ok()) {
    ROS_WARN("gRPC Pair RPC failed: code=%d message=%s",
             static_cast<int>(status.error_code()), status.error_message().c_str());
    return false;
  }

  return response.success();
}

bool GrpcManager::SubscribeStreams(
    const std::string &publisher_session_id,
    const std::vector<uint32_t> &prefixes) {
  if (!stub_ || session_id_.empty() || publisher_session_id.empty() ||
      prefixes.empty() || shutting_down_.load()) {
    return false;
  }

  signaling::SubscribeRequest request;
  request.set_op(signaling::SubscribeRequest::SUBSCRIBE);
  request.set_publisher_session_id(publisher_session_id);
  for (const uint32_t prefix : prefixes) {
    request.add_subscribe_prefixes(prefix);
  }

  signaling::SubscribeResponse response;
  grpc::ClientContext context;
  ApplyDeadline(context);
  context.AddMetadata("session-id", session_id_);
  RpcScope scope(*this, context);
  if (scope.shuttingDown()) return false;
  const grpc::Status status = stub_->Subscribe(&context, request, &response);
  if (!status.ok()) {
    ROS_WARN_THROTTLE(
        5.0, "gRPC Subscribe failed: publisher=%s code=%d message=%s",
        publisher_session_id.c_str(), static_cast<int>(status.error_code()),
        status.error_message().c_str());
    return false;
  }
  if (!response.success()) {
    ROS_WARN_THROTTLE(5.0, "gRPC Subscribe rejected: publisher=%s message=%s",
                      publisher_session_id.c_str(), response.message().c_str());
    return false;
  }

  ROS_INFO("gRPC stream subscription ready: publisher=%s prefixes=%zu",
           publisher_session_id.c_str(), prefixes.size());
  return true;
}

bool GrpcManager::PublishStreamManifest(
    const std::vector<StreamDeclaration> &streams) {
  if (!stub_ || session_id_.empty() || shutting_down_.load()) {
    return false;
  }

  signaling::PublishStreamManifestRequest request;
  for (const auto &stream : streams) {
    auto *declaration = request.add_streams();
    declaration->set_prefix(stream.prefix);
    declaration->set_stream_code(stream.stream_code);
    declaration->set_display_name(stream.display_name);
    declaration->set_description(stream.description);
    declaration->set_stream_class_code(stream.stream_class_code);
    declaration->set_protocol_code(stream.protocol_code);
  }

  signaling::StreamManifestMutationResponse response;
  grpc::ClientContext context;
  ApplyDeadline(context);
  context.AddMetadata("session-id", session_id_);
  RpcScope scope(*this, context);
  if (scope.shuttingDown()) return false;
  grpc::Status status = stub_->PublishStreamManifest(&context, request, &response);
  if (!status.ok()) {
    ROS_WARN("gRPC PublishStreamManifest failed: code=%d message=%s",
             static_cast<int>(status.error_code()), status.error_message().c_str());
    return false;
  }

  if (!response.success()) {
    ROS_WARN("gRPC PublishStreamManifest rejected: %s", response.message().c_str());
    return false;
  }
  ROS_INFO("gRPC stream manifest published: version=%lu streams=%d",
           static_cast<unsigned long>(response.version()), response.items_size());
  return true;
}

bool GrpcManager::PublishStreamConfig(uint32_t prefix, uint32_t schema_version,
                                      const std::string &payload) {
  if (!stub_ || session_id_.empty() || shutting_down_.load()) {
    return false;
  }

  signaling::PublishStreamConfigRequest request;
  request.set_prefix(prefix);
  request.set_schema_version(schema_version);
  request.set_payload(payload);

  signaling::StreamConfigMutationResponse response;
  grpc::ClientContext context;
  ApplyDeadline(context);
  context.AddMetadata("session-id", session_id_);
  RpcScope scope(*this, context);
  if (scope.shuttingDown()) return false;
  grpc::Status status = stub_->PublishStreamConfig(&context, request, &response);
  if (!status.ok()) {
    ROS_WARN("gRPC PublishStreamConfig(prefix=0x%02X) failed: code=%d message=%s",
             static_cast<unsigned int>(prefix),
             static_cast<int>(status.error_code()), status.error_message().c_str());
    return false;
  }

  if (!response.success()) {
    ROS_WARN("gRPC PublishStreamConfig(prefix=0x%02X) rejected: %s",
             static_cast<unsigned int>(prefix), response.message().c_str());
    return false;
  }
  ROS_INFO("gRPC StreamConfig published: prefix=0x%02X schema=%u version=%lu config_id=%s payload=%zu bytes",
           static_cast<unsigned int>(prefix), schema_version,
           static_cast<unsigned long>(response.has_item() ? response.item().version() : 0),
           response.has_item() ? response.item().config_id().c_str() : "<not returned>",
           payload.size());
  return true;
}

bool GrpcManager::Heartbeat() {
  if (!stub_ || session_id_.empty() || shutting_down_.load()) {
    return false;
  }

  signaling::Heartbeat request;
  request.set_client_time(static_cast<int64_t>(ros::Time::now().toNSec() / 1000000));

  signaling::HeartbeatAck response;
  grpc::ClientContext context;
  ApplyDeadline(context);
  context.AddMetadata("session-id", session_id_);
  RpcScope scope(*this, context);
  if (scope.shuttingDown()) return false;
  grpc::Status status = stub_->Ping(&context, request, &response);
  if (!status.ok()) {
    ROS_WARN("gRPC Ping failed: code=%d message=%s",
             static_cast<int>(status.error_code()), status.error_message().c_str());
    return false;
  }
  return true;
}

void GrpcManager::StartEventStream(const EventCallback &callback,
                                   const StreamEndCallback &on_end) {
  if (stream_running_.exchange(true)) {
    return;
  }

  // Defensive: a previous stream thread may have exited on its own (e.g.
  // server closed the socket) without StopEventStream() being called. In
  // that case stream_thread_ is still joinable but no longer running; we
  // must join it before move-assigning a fresh std::thread below, otherwise
  // the move-assign would invoke std::terminate.
  if (stream_thread_.joinable() &&
      stream_thread_.get_id() != std::this_thread::get_id()) {
    stream_thread_.join();
  }

  stream_callback_ = callback;
  stream_end_callback_ = on_end;
  stream_context_ = std::make_shared<grpc::ClientContext>();
  if (!session_id_.empty()) {
    stream_context_->AddMetadata("session-id", session_id_);
  }

  stream_thread_ = std::thread([this]() {
    signaling::EventSubscribe request;
    request.set_session_id(session_id_);

    std::unique_ptr<grpc::ClientReader<signaling::EventMessage>> reader(
        stub_->EventStream(stream_context_.get(), request));

    signaling::EventMessage msg;
    while (stream_running_.load() && reader->Read(&msg)) {
      if (stream_callback_) {
        stream_callback_(msg);
      }
    }

    const grpc::Status status = reader->Finish();
    // If the stream ended while we were still expecting events (i.e. the
    // server or network dropped us), surface that to the caller exactly
    // once. Subsequent StopEventStream() will become a no-op.
    const bool unexpected = stream_running_.exchange(false);
    if (unexpected && stream_end_callback_) {
      stream_end_callback_(status);
    }
  });
}

void GrpcManager::StopEventStream() {
  if (!stream_running_.exchange(false)) {
    // Still join an already-finished worker thread to avoid leaks.
    if (stream_thread_.joinable()) {
      stream_thread_.join();
    }
    stream_context_.reset();
    stream_callback_ = nullptr;
    stream_end_callback_ = nullptr;
    return;
  }

  if (stream_context_) {
    stream_context_->TryCancel();
  }

  if (stream_thread_.joinable()) {
    stream_thread_.join();
  }

  stream_context_.reset();
  stream_callback_ = nullptr;
  stream_end_callback_ = nullptr;
}
