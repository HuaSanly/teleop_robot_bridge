# MainNode 生命周期文档

本文档详细描述了 `teleop_robot_bridge` 主节点的完整生命周期管理，包括状态机定义、状态转换条件、以及各阶段的业务逻辑。

## 1. 状态机概览

MainNode 使用有限状态机管理节点生命周期，定义了以下5个状态：

| 状态 | 枚举值 | 说明 |
|------|--------|------|
| `kConnecting` | 0 | 初始状态，正在连接信令服务器 |
| `kRegistered` | 1 | 已在信令服务器注册成功 |
| `kPairing` | 2 | 等待客户端配对请求 |
| `kNegotiating` | 3 | 配对成功后，协商视频参数 |
| `kRunning` | 4 | 视频流传输中 |

## 2. 状态转换图

```
                    ┌────────────────────────────────────────────────────────────────────────────┐
                    │                              heartbeat failed                              │
                    ▼                                                                            │
┌─────────────┐  register   ┌─────────────┐  enter    ┌─────────────┐                           │
│ kConnecting │ ──success──▶│ kRegistered │ ─pairing─▶│  kPairing   │◀──────────────────────────┤
└─────────────┘             └─────────────┘           └─────────────┘                           │
    ▲                                                     │                                    │
    │                                               pair accept                                │
    │                                                     │                                    │
    │                                                     ▼                                    │
    │           ┌─────────────┐  VideoConfig  ┌─────────────────┐                              │
    │           │  kRunning   │◀────ACK──────│  kNegotiating   │                              │
    │           └─────────────┘               └─────────────────┘                              │
    │                  │                                                                       │
    │                  │ UNPAIR                                                                │
    │                  └──────────────────────────────────────────────────────────────────────▶│
    │                                                                                          │
    └──────────────────────────────────────────────────────────────────────────────────────────┘
                         register retry
```

## 3. 详细状态说明

### 3.1 kConnecting (连接中)

**描述**: 节点启动后的初始状态，尝试向信令服务器注册。

**入口动作**:
- 加载配置参数 (`loadParams()`)
- 初始化 gRPC 管理器 (`initGrpc()`)
- 初始化视频管理器 (`initVideo()`)

**业务逻辑**:
```
1. 创建 GrpcManager 实例
2. 调用 tryRegister() 尝试注册
3. 如果注册失败:
   - 启动注册重试定时器 (默认间隔3秒)
   - 持续重试直到成功
```

**出口条件**:
| 条件 | 目标状态 | 触发方法 |
|------|----------|----------|
| gRPC 注册成功 | `kRegistered` | `tryRegister()` |

---

### 3.2 kRegistered (已注册)

**描述**: 已成功在信令服务器注册，准备进入配对状态。

**入口动作**:
- 初始化 UDP 管理器 (`initUdp()`)
- 停止注册重试定时器
- 启动心跳定时器 (默认间隔15秒)

**业务逻辑**:
```
1. 创建 UdpManager 实例
2. 配置 UDP socket (绑定地址、远程地址)
3. 初始化 FEC 编码器
4. 初始化 Pacing 发送队列
5. 如启用，初始化 PosePublisher 用于VR追踪数据
6. 立即进入 kPairing 状态
```

**出口条件**:
| 条件 | 目标状态 | 触发方法 |
|------|----------|----------|
| UDP 初始化完成 | `kPairing` | `enterPairingState()` |

---

### 3.3 kPairing (配对中)

**描述**: 等待远程客户端发起配对请求。

**入口动作**:
- 启动 gRPC 事件流监听
- 停止 UDP 握手定时器(如有)

**业务逻辑**:
```
1. 调用 grpc_manager_->StartEventStream() 开始监听事件
2. 注册回调 onSignalingEvent() 处理信令事件
3. 等待 PairEvent 消息
```

**事件处理**:

| 事件类型 | 处理逻辑 |
|----------|----------|
| `PairEvent::REQUEST` | 自动接受配对请求，调用 `AcceptPair()` |
| `PairEvent::ACCEPT` | 对方接受了我们的配对请求(目前未使用) |
| `PairEvent::REJECT` | 对方拒绝配对，保持在 kPairing 状态 |

**出口条件**:
| 条件 | 目标状态 | 触发方法 |
|------|----------|----------|
| 配对请求被接受 | `kNegotiating` | `onSignalingEvent()` |
| 心跳失败 | `kConnecting` | `heartbeatTimerCallback()` |

---

### 3.4 kNegotiating (协商中)

**描述**: 配对成功后，协商视频参数。等待 SPS/PPS 捕获并发送 VideoConfig。

**入口动作**:
- 启动视频流并触发协商 (`startVideoAndNegotiate()`)

**业务逻辑**:

**业务逻辑**:

```
┌─────────────────────────────────────────────────────────────┐
│                   startVideoAndNegotiate()                  │
├─────────────────────────────────────────────────────────────┤
│  1. 如果是 H.264 文件模式:                                   │
│     - 使用预解析的 SPS/PPS                                   │
│     - 直接调用 onSpsPpsCaptured()                           │
│                                                             │
│  2. 如果是实时编码模式:                                      │
│     - 启动 VideoStreamManager                               │
│     - 等待编码器输出第一个关键帧                              │
│     - 编码器回调 onSpsPpsCaptured()                         │
└─────────────────────────────────────────────────────────────┘
                              │
                              ▼
┌─────────────────────────────────────────────────────────────┐
│                    onSpsPpsCaptured()                       │
├─────────────────────────────────────────────────────────────┤
│  1. 缓存 SPS/PPS 数据                                        │
│  2. 构建 VideoConfig 消息:                                   │
│     - codec: H264                                           │
│     - width/height/fps: 从配置读取                           │
│     - sps/pps: 编码器捕获的参数集                            │
│  3. 调用 grpc_manager_->PublishVideoConfig()                │
│  4. 等待 VideoConfigAck                                      │
└─────────────────────────────────────────────────────────────┘
```

**VideoConfig 消息结构**:
```protobuf
message VideoConfig {
  Codec codec = 1;       // H264
  int32 width = 2;       // 视频宽度 (如 3840)
  int32 height = 3;      // 视频高度 (如 1520)
  int32 fps = 4;         // 帧率 (如 30)
  bytes sps = 5;         // H.264 SPS 参数集
  bytes pps = 6;         // H.264 PPS 参数集
}
```

**出口条件**:
| 条件 | 目标状态 | 触发方法 |
|------|----------|----------|
| VideoConfig ACK 收到 | `kRunning` | `onSpsPpsCaptured()` |
| 收到 UNPAIR 事件 | `kPairing` | `onSignalingEvent()` |
| 心跳失败 | `kConnecting` | `heartbeatTimerCallback()` |

---

### 3.6 kRunning (运行中)

**描述**: 视频流正常传输中。

**入口动作**:
- 设置 `video_config_sent_` 为 true
- 日志记录进入运行状态

**业务逻辑**:

```
┌──────────────────────────────────────────────────────────────┐
│                      视频帧发送流程                           │
├──────────────────────────────────────────────────────────────┤
│                                                              │
│   [编码器/H264文件]                                           │
│         │                                                    │
│         ▼                                                    │
│   onEncodedVideoFrame(data, size, timestamp, keyframe)       │
│         │                                                    │
│         ├──── 状态检查: state != kRunning ? 丢弃帧            │
│         │                                                    │
│         ▼                                                    │
│   udp_manager_->sendH264Frame()                              │
│         │                                                    │
│         ├──── FEC 编码 (如启用)                               │
│         ├──── 分片打包                                        │
│         ├──── Pacing 调度                                    │
│         │                                                    │
│         ▼                                                    │
│   UDP Socket 发送                                            │
│                                                              │
└──────────────────────────────────────────────────────────────┘
```

**并行运行的组件**:
- **心跳定时器**: 定期向信令服务器发送心跳
- **视频编码器**: 持续捕获和编码视频帧
- **UDP 接收线程**: 接收 VR 追踪数据
- **Pacing 线程**: 平滑发送视频数据

**出口条件**:
| 条件 | 目标状态 | 触发方法 |
|------|----------|----------|
| 收到 UNPAIR 事件 | `kPairing` | `onSignalingEvent()` |
| 心跳失败 | `kConnecting` | `heartbeatTimerCallback()` |

---

## 4. 关键转换条件详解

### 4.1 注册成功转换

```cpp
void MainNode::tryRegister() {
    if (grpc_manager_->Register()) {
        grpc_registered_ = true;
        setState(State::kRegistered, "grpc registered");
        initUdp();
        enterPairingState();
        // ...
    }
}
```

**前置条件**:
- GrpcManager 已初始化
- 服务器地址配置正确
- 网络连接正常

### 4.2 配对接受转换

```cpp
case signaling::PairEvent::REQUEST: {
    if (grpc_manager_->AcceptPair(pair_event.peer_session_id())) {
        setState(State::kNegotiating, "pair accepted");
        startVideoAndNegotiate();
    }
    break;
}
```

**前置条件**:
- 当前状态为 kPairing
- 收到有效的 PairEvent::REQUEST
- AcceptPair() 调用成功

### 4.3 协商完成转换

```cpp
if (grpc_manager_->PublishVideoConfig(config, ack)) {
    video_config_sent_ = true;
    setState(State::kRunning, "VideoConfig ACK received");
}
```

**前置条件**:
- 当前状态为 kNegotiating
- SPS/PPS 已捕获
- VideoConfig 发布成功并收到 ACK

### 4.4 Unpair 转换

```cpp
case signaling::PairEvent::UNPAIR: {
    // 停止视频，重置状态
    video_config_sent_ = false;
    video_started_ = false;
    cached_sps_.clear();
    cached_pps_.clear();
    setState(State::kPairing, "unpaired");
    break;
}
```

**触发条件**:
- 客户端主动断开连接
- 客户端异常离线(由服务器检测)

### 4.5 心跳失败转换

```cpp
void MainNode::heartbeatTimerCallback(const ros::TimerEvent &event) {
    if (!grpc_manager_->Heartbeat()) {
        grpc_registered_ = false;
        grpc_manager_->StopEventStream();
        udp_manager_->stop();
        setState(State::kConnecting, "heartbeat failed");
        // 重启注册重试定时器
    }
}
```

**触发条件**:
- 心跳请求超时
- 服务器返回错误
- 网络连接断开

---

## 5. 资源管理

### 5.1 各状态资源占用

| 状态 | gRPC连接 | UDP Socket | 事件流 | 视频编码器 | 定时器 |
|------|----------|------------|--------|------------|--------|
| kConnecting | 尝试中 | - | - | 已创建 | 注册重试 |
| kRegistered | 已建立 | 已创建 | - | 已创建 | 心跳 |
| kPairing | 已建立 | 运行中 | 监听中 | 已创建 | 心跳 |
| kNegotiating | 已建立 | 运行中 | 监听中 | 运行中 | 心跳 |
| kRunning | 已建立 | 运行中 | 监听中 | 运行中 | 心跳 |

### 5.2 资源清理 (析构函数)

```cpp
MainNode::~MainNode() {
    // 1. 停止 H.264 播放线程
    h264_player_running_.store(false);
    h264_player_thread_.join();
    
    // 2. 停止所有定时器
    register_retry_timer_.stop();
    heartbeat_timer_.stop();
    udp_handshake_timer_.stop();
    p2p_handshake_timer_.stop();
    
    // 3. 停止视频管理器
    video_manager_->stop();
    
    // 4. 停止 gRPC 事件流
    grpc_manager_->StopEventStream();
    
    // 5. 停止 UDP 管理器
    udp_manager_->stop();
}
```

---

## 6. 时序图

### 6.1 正常启动到运行流程

```
MainNode                  GrpcManager              UdpManager           VideoStreamManager
   │                           │                        │                        │
   │  loadParams()             │                        │                        │
   │──────────────►            │                        │                        │
   │                           │                        │                        │
   │  Register()               │                        │                        │
   │──────────────────────────►│                        │                        │
   │          session_id       │                        │                        │
   │◄──────────────────────────│                        │                        │
   │                           │                        │                        │
   │ [kConnecting -> kRegistered]                       │                        │
   │                           │                        │                        │
   │  initUdp()                │                        │                        │
   │────────────────────────────────────────────────────►                        │
   │                           │                        │ start()                │
   │                           │                        │──────►                 │
   │                           │                        │                        │
   │  enterPairingState()      │                        │                        │
   │──────────────────────────►│                        │                        │
   │                           │ StartEventStream()     │                        │
   │                           │──────►                 │                        │
   │                           │                        │                        │
   │ [kRegistered -> kPairing] │                        │                        │
   │                           │                        │                        │
   │                           │  PairEvent::REQUEST    │                        │
   │◄──────────────────────────│                        │                        │
   │                           │                        │                        │
   │  AcceptPair()             │                        │                        │
   │──────────────────────────►│                        │                        │
   │                           │                        │                        │
    │ [kPairing -> kNegotiating]                          │                        │
    │                           │                        │                        │
    │  startVideoAndNegotiate() │                        │                        │
   │────────────────────────────────────────────────────────────────────────────►│
   │                           │                        │                        │ start()
   │                           │                        │                        │──────►
   │                           │                        │                        │
   │  onSpsPpsCaptured(sps,pps)│                        │                        │
   │◄────────────────────────────────────────────────────────────────────────────│
   │                           │                        │                        │
   │  PublishVideoConfig()     │                        │                        │
   │──────────────────────────►│                        │                        │
   │          ACK              │                        │                        │
   │◄──────────────────────────│                        │                        │
   │                           │                        │                        │
   │ [kNegotiating -> kRunning]│                        │                        │
   │                           │                        │                        │
   │  onEncodedVideoFrame()    │                        │                        │
   │◄────────────────────────────────────────────────────────────────────────────│
   │                           │                        │                        │
   │  sendH264Frame()          │                        │                        │
   │─────────────────────────────────────────────────────►                       │
   │                           │                        │  UDP packet            │
   │                           │                        │─────────────────────────►
```

---

## 7. 错误处理与恢复

### 7.1 注册失败

- **现象**: `tryRegister()` 返回 false
- **处理**: 启动重试定时器，每3秒尝试一次
- **恢复**: 注册成功后自动进入正常流程

### 7.2 心跳失败

- **现象**: `Heartbeat()` 返回 false
- **处理**: 
  1. 停止事件流监听
  2. 停止 UDP 管理器
  3. 回退到 kConnecting 状态
  4. 启动注册重试定时器
- **恢复**: 重新注册成功后恢复

### 7.3 VideoConfig 发布失败

- **现象**: `PublishVideoConfig()` 返回 false
- **处理**: 保持在 kNegotiating 状态，等待下一个关键帧时重试
- **恢复**: 下次关键帧到来时自动重试

### 7.4 P2P握手失败

- **现象**: 握手超时(3秒)或重试次数达到上限(6次)
- **处理**:
  1. 使用 `default_peer_ip` 和 `default_peer_port` 覆盖当前对端地址
  2. 更新 UdpManager 的远程端点
  3. 继续进入 kNegotiating 状态
- **恢复**: 自动使用默认地址继续，无需人工干预

### 7.5 Unpair 事件

- **现象**: 收到 `PairEvent::UNPAIR`
- **处理**:
  1. 停止视频编码器
  2. 停止 P2P 握手定时器 (如在握手中)
  3. 清空 SPS/PPS 缓存
  4. 重置配对状态
  5. 回退到 kPairing 状态
- **恢复**: 等待新的配对请求

---

## 8. 配置参数

### 8.1 gRPC 相关

| 参数 | 默认值 | 说明 |
|------|--------|------|
| `grpc/server_grpc_ip` | "" | 信令服务器 IP |
| `grpc/server_grpc_port` | 0 | 信令服务器端口 |
| `grpc/device_id` | "" | 设备标识符 |
| `grpc/token` | "" | 认证令牌 |
| `grpc/register_retry_sec` | 3.0 | 注册重试间隔(秒) |
| `grpc/heartbeat_sec` | 15.0 | 心跳间隔(秒) |

### 8.2 UDP 相关

| 参数 | 默认值 | 说明 |
|------|--------|------|
| `udp/server_udp_ip` | "" | 远程 UDP 服务器 IP |
| `udp/server_udp_port` | 0 | 远程 UDP 服务器端口 |
| `udp/bind_ip` | "0.0.0.0" | 本地绑定 IP |
| `udp/bind_port` | 0 | 本地绑定端口 |
| `udp/handshake/enabled` | true | 是否启用握手 |

### 8.3 P2P 握手相关

| 参数 | 默认值 | 说明 |
|------|--------|------|
| `udp/p2p/enabled` | true | 是否启用 P2P 握手 |
| `udp/p2p/timeout_sec` | 3.0 | P2P 握手总超时时间(秒) |
| `udp/p2p/max_retries` | 6 | P2P 握手最大重试次数 |
| `udp/p2p/retry_interval_ms` | 500 | P2P 握手重试间隔(毫秒) |
| `udp/p2p/default_peer_ip` | "" | P2P 握手失败时的默认对端 IP |
| `udp/p2p/default_peer_port` | 0 | P2P 握手失败时的默认对端端口 |

### 8.4 视频相关

| 参数 | 默认值 | 说明 |
|------|--------|------|
| `video/profile` | [3840, 1520, 30] | [宽度, 高度, 帧率] |
| `video/h264_file/enabled` | false | H.264 文件模式 |
| `video/h264_file/path` | "" | H.264 文件路径 |
| `video/h264_file/loop` | true | 是否循环播放 |

---

## 9. 日志关键字

监控节点状态时可关注以下日志：

```bash
# 状态转换
State transition: connecting -> registered (grpc registered)
State transition: registered -> pairing (waiting for pair request)
State transition: pairing -> p2p_handshaking (pair accepted, starting p2p handshake)
State transition: p2p_handshaking -> negotiating (p2p handshake success)
State transition: p2p_handshaking -> negotiating (p2p handshake timeout, using default)
State transition: negotiating -> running (VideoConfig ACK received)

# P2P 握手
P2P handshake started, peer: xxx.xxx.xxx.xxx:xxxxx
P2P handshake succeeded, using peer address: xxx.xxx.xxx.xxx:xxxxx
P2P handshake failed after 6 retries, using default: xxx.xxx.xxx.xxx:xxxxx

# 错误/警告
gRPC register failed, entering pending state
gRPC heartbeat failed, will re-register
VideoConfig publish failed, staying in Negotiating state

# 视频流状态
[ENCODER-PRODUCED] total_frames=xxx total_bytes=xxx
[H264_FILE] Sent frame xxx/xxx size=xxx keyframe=x
```
