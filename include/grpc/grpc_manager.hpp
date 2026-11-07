#ifndef TELEOP_ROBOT_BRIDGE_GRPC_MANAGER_HPP_
#define TELEOP_ROBOT_BRIDGE_GRPC_MANAGER_HPP_

#include <atomic>
#include <functional>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <unordered_set>
#include <vector>

#include <grpcpp/grpcpp.h>

#include "signaling.grpc.pb.h"

struct GrpcConfig {
  std::string server_grpc_ip;
  int server_grpc_port = 0;
  bool use_ssl = false;
  std::string device_id;
  // Local safety/audit value only. The backend remains authoritative and
  // RegisterRequest never sends a device type.
  std::string expected_device_type_code;
  std::string firmware_version;
  std::string software_version;
  std::string operating_system;
  std::string operating_system_version;
  std::string runtime_version;
  std::string build_id;
  std::vector<std::string> capabilities;
  int rpc_timeout_ms = 5000;
  double register_retry_sec = 3.0;
  double heartbeat_sec = 15.0;
};

class GrpcManager {
 public:
  struct StreamDeclaration {
    uint32_t prefix{0};
    std::string stream_code;
    std::string display_name;
    std::string description;
    std::string stream_class_code;
    std::string protocol_code;
  };

  explicit GrpcManager(const GrpcConfig &config);
  ~GrpcManager();

  bool Register();
  const std::string &session_id() const;
  const std::string &device_type_code() const;
  void ClearSession();

  bool AcceptPair(const std::string &peer_session_id);
  bool SubscribeStreams(const std::string &publisher_session_id,
                        const std::vector<uint32_t> &prefixes);
  bool PublishStreamManifest(const std::vector<StreamDeclaration> &streams);
  bool PublishStreamConfig(uint32_t prefix, uint32_t schema_version,
                           const std::string &payload);
  bool Heartbeat();
  using EventCallback = std::function<void(const signaling::EventMessage &)>;
  using StreamEndCallback = std::function<void(const grpc::Status &)>;
  void StartEventStream(const EventCallback &callback,
                        const StreamEndCallback &on_end = {});
  void StopEventStream();

  // Cancel any in-flight unary RPC (Register / Heartbeat / AcceptPair / Subscribe /
  // PublishStreamManifest / PublishStreamConfig) and mark the manager as shutting
  // down so subsequent RPCs return false immediately. Idempotent. Intended
  // for fast Ctrl+C tear-down.
  void CancelPendingRpcs();

 private:
  std::shared_ptr<grpc::Channel> CreateChannel() const;
  std::string BuildTarget() const;
    void ApplyDeadline(grpc::ClientContext &context) const;

  // RAII helper that publishes a ClientContext so CancelPendingRpcs() can
  // TryCancel it, and removes it on scope exit.
  class RpcScope {
   public:
    RpcScope(GrpcManager &mgr, grpc::ClientContext &ctx);
    ~RpcScope();
    RpcScope(const RpcScope &) = delete;
    RpcScope &operator=(const RpcScope &) = delete;
    bool shuttingDown() const { return shutting_down_; }

   private:
    GrpcManager &mgr_;
    grpc::ClientContext &ctx_;
    bool shutting_down_{false};
  };

  GrpcConfig config_;
  std::unique_ptr<signaling::Signaling::Stub> stub_;
  std::string session_id_;
  std::string device_type_code_;

  // Shutdown + pending-RPC tracking for fast Ctrl+C exit.
  std::atomic<bool> shutting_down_{false};
  std::mutex pending_rpc_mu_;
  std::unordered_set<grpc::ClientContext *> pending_rpc_ctxs_;

  std::atomic<bool> stream_running_{false};
  std::shared_ptr<grpc::ClientContext> stream_context_;
  std::thread stream_thread_;
  EventCallback stream_callback_;
  StreamEndCallback stream_end_callback_;
};

#endif  // TELEOP_ROBOT_BRIDGE_GRPC_MANAGER_HPP_
