#include "grpc/grpc_manager.hpp"

#include <algorithm>
#include <chrono>
#include <cstdlib>
#include <utility>

#include <google/protobuf/empty.pb.h>

namespace trb
{
    namespace
    {
        int deadlineMs(const GrpcConfig &config)
        {
            return std::max(1, config.rpc_timeout_ms);
        }
    } // namespace

    GrpcManager::GrpcManager(const GrpcConfig &config, rclcpp::Logger logger)
        : config_(config), logger_(std::move(logger))
    {
        auto channel = CreateChannel();
        stub_ = signaling::Signaling::NewStub(channel);
    }

    GrpcManager::~GrpcManager()
    {
        CancelPendingRpcs();
        StopEventStream();
    }

    GrpcManager::RpcScope::RpcScope(GrpcManager &manager, ::grpc::ClientContext &context)
        : manager_(manager), context_(context)
    {
        std::lock_guard<std::mutex> lock(manager_.pending_rpc_mutex_);
        if (manager_.shutting_down_.load())
        {
            shutting_down_ = true;
            return;
        }
        manager_.pending_rpc_contexts_.insert(&context_);
    }

    GrpcManager::RpcScope::~RpcScope()
    {
        if (shutting_down_)
        {
            return;
        }
        std::lock_guard<std::mutex> lock(manager_.pending_rpc_mutex_);
        manager_.pending_rpc_contexts_.erase(&context_);
    }

    void GrpcManager::CancelPendingRpcs()
    {
        shutting_down_.store(true);
        {
            std::lock_guard<std::mutex> lock(pending_rpc_mutex_);
            for (auto *context : pending_rpc_contexts_)
            {
                if (context)
                {
                    context->TryCancel();
                }
            }
        }
        if (stream_context_)
        {
            stream_context_->TryCancel();
        }
    }

    std::shared_ptr<::grpc::Channel> GrpcManager::CreateChannel() const
    {
        const std::string target = BuildTarget();

        const char *existing_no_proxy = std::getenv("no_proxy");
        const std::string no_proxy_prefix = existing_no_proxy ? std::string(existing_no_proxy) + "," : std::string();
        const std::string no_proxy_value = no_proxy_prefix + config_.server_grpc_ip;
        setenv("no_proxy", no_proxy_value.c_str(), 1);
        setenv("NO_PROXY", no_proxy_value.c_str(), 1);

        ::grpc::ChannelArguments args;
        args.SetInt(GRPC_ARG_KEEPALIVE_TIME_MS, 5000);
        args.SetInt(GRPC_ARG_KEEPALIVE_TIMEOUT_MS, 2000);
        args.SetInt(GRPC_ARG_KEEPALIVE_PERMIT_WITHOUT_CALLS, 1);

        std::shared_ptr<::grpc::ChannelCredentials> credentials;
        if (config_.use_ssl)
        {
            ::grpc::SslCredentialsOptions ssl_options;
            credentials = ::grpc::SslCredentials(ssl_options);
        }
        else
        {
            credentials = ::grpc::InsecureChannelCredentials();
        }

        return ::grpc::CreateCustomChannel(target, credentials, args);
    }

    std::string GrpcManager::BuildTarget() const
    {
        return config_.server_grpc_ip + ":" + std::to_string(config_.server_grpc_port);
    }

    void GrpcManager::ApplyDeadline(::grpc::ClientContext &context) const
    {
        context.set_deadline(std::chrono::system_clock::now() + std::chrono::milliseconds(deadlineMs(config_)));
    }

    bool GrpcManager::Register()
    {
        if (!stub_ || shutting_down_.load())
        {
            return false;
        }

        signaling::RegisterRequest request;
        request.set_device_id(config_.device_id);
        auto *runtime = request.mutable_runtime();
        if (!config_.firmware_version.empty())
        {
            runtime->set_firmware_version(config_.firmware_version);
        }
        if (!config_.software_version.empty())
        {
            runtime->set_software_version(config_.software_version);
        }
        if (!config_.operating_system.empty())
        {
            runtime->set_operating_system(config_.operating_system);
        }
        if (!config_.operating_system_version.empty())
        {
            runtime->set_operating_system_version(config_.operating_system_version);
        }
        if (!config_.runtime_version.empty())
        {
            runtime->set_runtime_version(config_.runtime_version);
        }
        if (!config_.build_id.empty())
        {
            runtime->set_build_id(config_.build_id);
        }
        for (const auto &capability : config_.capabilities)
        {
            if (!capability.empty())
            {
                runtime->add_capabilities(capability);
            }
        }

        signaling::RegisterResponse response;
        ::grpc::ClientContext context;
        ApplyDeadline(context);
        RpcScope scope(*this, context);
        if (scope.shuttingDown())
        {
            return false;
        }

        RCLCPP_INFO(logger_, "Attempting gRPC Register to %s with timeout=%d ms", BuildTarget().c_str(), deadlineMs(config_));
        const ::grpc::Status status = stub_->Register(&context, request, &response);
        if (!status.ok())
        {
            RCLCPP_WARN(logger_, "gRPC Register RPC failed: code=%d message=%s",
                        static_cast<int>(status.error_code()), status.error_message().c_str());
            return false;
        }
        if (!response.success())
        {
            RCLCPP_WARN(logger_, "gRPC Register rejected by server: %s", response.message().c_str());
            return false;
        }

        const std::string session_id = response.session_id();
        const std::string device_type_code = response.device_type_code();
        {
            std::lock_guard<std::mutex> lock(session_mutex_);
            session_id_ = session_id;
            device_type_code_ = device_type_code;
        }
        RCLCPP_INFO(logger_, "gRPC registered device_id=%s device_type_code=%s",
                    config_.device_id.c_str(),
                    device_type_code.empty() ? "<empty>" : device_type_code.c_str());
        return true;
    }

    std::string GrpcManager::session_id() const
    {
        std::lock_guard<std::mutex> lock(session_mutex_);
        return session_id_;
    }

    std::string GrpcManager::device_type_code() const
    {
        std::lock_guard<std::mutex> lock(session_mutex_);
        return device_type_code_;
    }

    bool GrpcManager::Pair(signaling::PairRequest::Operation operation,
                           const std::string &peer_session_id,
                           signaling::PairRequest::PairState state)
    {
        const std::string session_id = this->session_id();
        if (!stub_ || session_id.empty() || shutting_down_.load())
        {
            return false;
        }

        signaling::PairRequest request;
        request.set_op(operation);
        request.set_peer_session_id(peer_session_id);
        request.set_state(state);

        signaling::PairResponse response;
        ::grpc::ClientContext context;
        ApplyDeadline(context);
        context.AddMetadata("session-id", session_id);
        RpcScope scope(*this, context);
        if (scope.shuttingDown())
        {
            return false;
        }

        const ::grpc::Status status = stub_->Pair(&context, request, &response);
        if (!status.ok())
        {
            RCLCPP_WARN(logger_, "gRPC Pair RPC failed: code=%d message=%s",
                        static_cast<int>(status.error_code()), status.error_message().c_str());
            return false;
        }
        if (!response.success())
        {
            RCLCPP_WARN(logger_, "gRPC Pair rejected by server: %s", response.message().c_str());
            return false;
        }
        return true;
    }

    bool GrpcManager::RequestPair(const std::string &peer_session_id)
    {
        return Pair(signaling::PairRequest::PAIR, peer_session_id, signaling::PairRequest::REQUEST);
    }

    bool GrpcManager::AcceptPair(const std::string &peer_session_id)
    {
        return Pair(signaling::PairRequest::PAIR, peer_session_id, signaling::PairRequest::ACCEPT);
    }

    bool GrpcManager::RejectPair(const std::string &peer_session_id)
    {
        return Pair(signaling::PairRequest::PAIR, peer_session_id, signaling::PairRequest::REJECT);
    }

    bool GrpcManager::Unpair(const std::string &peer_session_id)
    {
        return Pair(signaling::PairRequest::UNPAIR, peer_session_id, signaling::PairRequest::REQUEST);
    }

    bool GrpcManager::Subscribe(const std::string &publisher_session_id,
                                const std::vector<uint8_t> &prefixes)
    {
        const std::string session_id = this->session_id();
        if (!stub_ || session_id.empty() || shutting_down_.load())
        {
            return false;
        }

        signaling::SubscribeRequest request;
        request.set_op(signaling::SubscribeRequest::SUBSCRIBE);
        request.set_publisher_session_id(publisher_session_id);
        for (const uint8_t prefix : prefixes)
        {
            request.add_subscribe_prefixes(prefix);
        }

        signaling::SubscribeResponse response;
        ::grpc::ClientContext context;
        ApplyDeadline(context);
        context.AddMetadata("session-id", session_id);
        RpcScope scope(*this, context);
        if (scope.shuttingDown())
        {
            return false;
        }

        const ::grpc::Status status = stub_->Subscribe(&context, request, &response);
        if (!status.ok())
        {
            RCLCPP_WARN(logger_, "gRPC Subscribe failed: code=%d message=%s",
                        static_cast<int>(status.error_code()), status.error_message().c_str());
            return false;
        }
        if (!response.success())
        {
            RCLCPP_WARN(logger_, "gRPC Subscribe rejected by server: %s", response.message().c_str());
            return false;
        }
        return true;
    }

    bool GrpcManager::Unsubscribe(const std::string &publisher_session_id)
    {
        const std::string session_id = this->session_id();
        if (!stub_ || session_id.empty() || shutting_down_.load())
        {
            return false;
        }

        signaling::SubscribeRequest request;
        request.set_op(signaling::SubscribeRequest::UNSUBSCRIBE);
        request.set_publisher_session_id(publisher_session_id);

        signaling::SubscribeResponse response;
        ::grpc::ClientContext context;
        ApplyDeadline(context);
        context.AddMetadata("session-id", session_id);
        RpcScope scope(*this, context);
        if (scope.shuttingDown())
        {
            return false;
        }

        const ::grpc::Status status = stub_->Subscribe(&context, request, &response);
        if (!status.ok())
        {
            RCLCPP_WARN(logger_, "gRPC Unsubscribe failed: code=%d message=%s",
                        static_cast<int>(status.error_code()), status.error_message().c_str());
            return false;
        }
        if (!response.success())
        {
            RCLCPP_WARN(logger_, "gRPC Unsubscribe rejected by server: %s", response.message().c_str());
            return false;
        }
        return true;
    }
    std::vector<signaling::UnpairedEndpoint> GrpcManager::ListUnpaired(const std::string &desired_device_type_code)
    {
        std::vector<signaling::UnpairedEndpoint> endpoints;
        const std::string session_id = this->session_id();
        if (!stub_ || session_id.empty() || shutting_down_.load())
        {
            return endpoints;
        }

        signaling::ListUnpairedRequest request;
        request.set_desired_device_type_code(desired_device_type_code);

        signaling::ListUnpairedResponse response;
        ::grpc::ClientContext context;
        ApplyDeadline(context);
        context.AddMetadata("session-id", session_id);
        RpcScope scope(*this, context);
        if (scope.shuttingDown())
        {
            return endpoints;
        }

        const ::grpc::Status status = stub_->ListUnpaired(&context, request, &response);
        if (!status.ok())
        {
            RCLCPP_WARN(logger_, "gRPC ListUnpaired failed: code=%d message=%s",
                        static_cast<int>(status.error_code()), status.error_message().c_str());
            return endpoints;
        }
        if (!response.success())
        {
            RCLCPP_WARN(logger_, "gRPC ListUnpaired rejected by server: %s", response.message().c_str());
            return endpoints;
        }

        endpoints.reserve(static_cast<size_t>(response.endpoints_size()));
        for (const auto &endpoint : response.endpoints())
        {
            endpoints.push_back(endpoint);
        }
        return endpoints;
    }

    bool GrpcManager::PublishStreamManifest(const signaling::PublishStreamManifestRequest &request)
    {
        const std::string session_id = this->session_id();
        if (!stub_ || session_id.empty() || shutting_down_.load())
        {
            return false;
        }

        ::grpc::ClientContext context;
        ApplyDeadline(context);
        context.AddMetadata("session-id", session_id);
        RpcScope scope(*this, context);
        if (scope.shuttingDown())
        {
            return false;
        }

        signaling::StreamManifestMutationResponse response;
        const ::grpc::Status status = stub_->PublishStreamManifest(&context, request, &response);
        if (!status.ok())
        {
            RCLCPP_WARN(logger_, "gRPC PublishStreamManifest failed: code=%d message=%s",
                        static_cast<int>(status.error_code()), status.error_message().c_str());
            return false;
        }
        if (!response.success())
        {
            RCLCPP_WARN(logger_, "gRPC PublishStreamManifest rejected by server: %s", response.message().c_str());
            return false;
        }
        return true;
    }

    bool GrpcManager::GetStreamManifest(const std::string &publisher_session_id,
                                        signaling::GetStreamManifestResponse &response)
    {
        const std::string session_id = this->session_id();
        if (!stub_ || session_id.empty() || shutting_down_.load())
        {
            return false;
        }

        signaling::GetStreamManifestRequest request;
        request.set_publisher_session_id(publisher_session_id);
        ::grpc::ClientContext context;
        ApplyDeadline(context);
        context.AddMetadata("session-id", session_id);
        RpcScope scope(*this, context);
        if (scope.shuttingDown())
        {
            return false;
        }

        const ::grpc::Status status = stub_->GetStreamManifest(&context, request, &response);
        if (!status.ok())
        {
            RCLCPP_WARN(logger_, "gRPC GetStreamManifest failed: code=%d message=%s",
                        static_cast<int>(status.error_code()), status.error_message().c_str());
            return false;
        }
        return true;
    }

    bool GrpcManager::PublishStreamConfig(uint8_t prefix,
                                          uint32_t schema_version,
                                          const std::string &payload,
                                          signaling::StreamConfigItem *published_item)
    {
        const std::string session_id = this->session_id();
        if (!stub_ || session_id.empty() || shutting_down_.load())
        {
            return false;
        }

        signaling::PublishStreamConfigRequest request;
        request.set_prefix(prefix);
        request.set_schema_version(schema_version);
        request.set_payload(payload);

        signaling::StreamConfigMutationResponse response;
        ::grpc::ClientContext context;
        ApplyDeadline(context);
        context.AddMetadata("session-id", session_id);
        RpcScope scope(*this, context);
        if (scope.shuttingDown())
        {
            return false;
        }

        const ::grpc::Status status = stub_->PublishStreamConfig(&context, request, &response);
        if (!status.ok())
        {
            RCLCPP_WARN(logger_, "gRPC PublishStreamConfig failed: prefix=0x%02X code=%d message=%s",
                        static_cast<unsigned int>(prefix),
                        static_cast<int>(status.error_code()), status.error_message().c_str());
            return false;
        }
        if (!response.success() || !response.has_item())
        {
            RCLCPP_WARN(logger_, "gRPC PublishStreamConfig rejected: prefix=0x%02X message=%s",
                        static_cast<unsigned int>(prefix), response.message().c_str());
            return false;
        }
        if (published_item)
        {
            *published_item = response.item();
        }
        return true;
    }

    bool GrpcManager::GetStreamConfig(const std::string &publisher_session_id,
                                      uint8_t prefix,
                                      signaling::StreamConfigItem &item)
    {
        const std::string session_id = this->session_id();
        if (!stub_ || session_id.empty() || shutting_down_.load())
        {
            return false;
        }

        signaling::GetStreamConfigRequest request;
        request.set_publisher_session_id(publisher_session_id);
        request.set_prefix(prefix);
        ::grpc::ClientContext context;
        ApplyDeadline(context);
        context.AddMetadata("session-id", session_id);
        RpcScope scope(*this, context);
        if (scope.shuttingDown())
        {
            return false;
        }

        const ::grpc::Status status = stub_->GetStreamConfig(&context, request, &item);
        if (!status.ok())
        {
            RCLCPP_WARN(logger_, "gRPC GetStreamConfig failed: publisher=%s prefix=0x%02X code=%d message=%s",
                        publisher_session_id.c_str(),
                        static_cast<unsigned int>(prefix),
                        static_cast<int>(status.error_code()), status.error_message().c_str());
            return false;
        }
        return true;
    }

    bool GrpcManager::GetP2pInfo(signaling::GetP2pInfoResponse &response)
    {
        const std::string session_id = this->session_id();
        if (!stub_ || session_id.empty() || shutting_down_.load())
        {
            return false;
        }

        google::protobuf::Empty request;
        ::grpc::ClientContext context;
        ApplyDeadline(context);
        context.AddMetadata("session-id", session_id);
        RpcScope scope(*this, context);
        if (scope.shuttingDown())
        {
            return false;
        }

        const ::grpc::Status status = stub_->GetP2pInfo(&context, request, &response);
        if (!status.ok())
        {
            RCLCPP_WARN(logger_, "gRPC GetP2pInfo failed: code=%d message=%s",
                        static_cast<int>(status.error_code()), status.error_message().c_str());
            return false;
        }
        return response.ready();
    }

    GrpcManager::HeartbeatResult GrpcManager::Heartbeat()
    {
        const std::string session_id = this->session_id();
        if (!stub_ || session_id.empty() || shutting_down_.load())
        {
            return HeartbeatResult::kSessionInvalid;
        }

        const auto now = std::chrono::system_clock::now().time_since_epoch();
        signaling::Heartbeat request;
        request.set_client_time(std::chrono::duration_cast<std::chrono::milliseconds>(now).count());

        signaling::HeartbeatAck response;
        ::grpc::ClientContext context;
        ApplyDeadline(context);
        context.AddMetadata("session-id", session_id);
        RpcScope scope(*this, context);
        if (scope.shuttingDown())
        {
            return HeartbeatResult::kTransportError;
        }

        const ::grpc::Status status = stub_->Ping(&context, request, &response);
        if (!status.ok())
        {
            RCLCPP_WARN(logger_, "gRPC Ping failed: code=%d message=%s",
                        static_cast<int>(status.error_code()), status.error_message().c_str());
            switch (status.error_code())
            {
            case ::grpc::StatusCode::NOT_FOUND:
            case ::grpc::StatusCode::UNAUTHENTICATED:
            case ::grpc::StatusCode::PERMISSION_DENIED:
                return HeartbeatResult::kSessionInvalid;
            default:
                return HeartbeatResult::kTransportError;
            }
        }
        return HeartbeatResult::kOk;
    }

    void GrpcManager::StartEventStream(const EventCallback &callback, const StreamEndCallback &on_end)
    {
        const std::string session_id = this->session_id();
        if (!stub_ || session_id.empty() || shutting_down_.load())
        {
            return;
        }
        if (stream_running_.exchange(true))
        {
            return;
        }
        if (stream_thread_.joinable() && stream_thread_.get_id() != std::this_thread::get_id())
        {
            stream_thread_.join();
        }

        stream_callback_ = callback;
        stream_end_callback_ = on_end;
        stream_context_ = std::make_shared<::grpc::ClientContext>();
        stream_context_->AddMetadata("session-id", session_id);

        stream_thread_ = std::thread([this, session_id]() {
            signaling::EventSubscribe request;
            request.set_session_id(session_id);

            std::unique_ptr<::grpc::ClientReader<signaling::EventMessage>> reader(
                stub_->EventStream(stream_context_.get(), request));

            signaling::EventMessage message;
            while (stream_running_.load() && reader->Read(&message))
            {
                if (stream_callback_)
                {
                    stream_callback_(message);
                }
            }

            const ::grpc::Status status = reader->Finish();
            const bool unexpected = stream_running_.exchange(false);
            if (unexpected && stream_end_callback_)
            {
                stream_end_callback_(status);
            }
        });
    }

    void GrpcManager::StopEventStream()
    {
        stream_running_.store(false);
        if (stream_context_)
        {
            stream_context_->TryCancel();
        }
        if (stream_thread_.joinable())
        {
            if (stream_thread_.get_id() == std::this_thread::get_id())
            {
                stream_thread_.detach();
            }
            else
            {
                stream_thread_.join();
            }
        }
        stream_context_.reset();
        stream_callback_ = nullptr;
        stream_end_callback_ = nullptr;
    }

} // namespace trb
