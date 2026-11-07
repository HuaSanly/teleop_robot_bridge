# 音频模块实现方案 - GStreamer

## 1. 目标

本文档定义 teleop_robot_bridge 中音频模块的正式实现方案。

本方案直接采用 GStreamer 作为音频采集、播放与预处理框架，不再保留 ALSA-only 或 PCM 直发路线。

实现目标：

- 上行：本地麦克风采集 -> 预处理 -> Opus 编码 -> 复用现有 UDP 通道发送
- 下行：接收 VR 端回传 Opus 音频 -> 解码 -> 本地播放
- A/V 同步：与现有视频流共享同一 NTP64 时钟域，音频追随视频时间线
- 回声处理：为后续“播放中的 VR 音频被本地麦克风回录”问题预留标准化 AEC 处理链
- 工程集成：与现有 MainNode、GrpcManager、UdpManager 保持一致的生命周期管理方式

## 2. 方案结论

正式方案固定如下：

- 音频框架：GStreamer
- 上行输入：GStreamer `alsasrc`
- 上行编码：GStreamer `opusenc`
- 下行解码：GStreamer `opusdec`
- 下行输出：GStreamer 音频 sink
- 音频格式：48kHz / mono / S16_LE
- 编码帧长：20ms
- 编码码率：64kbps
- Opus in-band FEC：开启
- Opus DTX：关闭
- 协议：采用 [docs/PROTOCOL_UDP_AUDIO_V2.md](docs/PROTOCOL_UDP_AUDIO_V2.md)

不采用的方案：

- 不采用 PCM 直发
- 不采用 ALSA + 自行调用 libopus 的编码方案
- 不采用独立 UDP 端口

## 3. 选型理由

### 3.1 为什么选 GStreamer

本项目后续不只是单向采集上传，还包括：

- 接收 VR 端回传音频
- 本地播放下行音频
- 避免扬声器播放音频被本地麦克风再次录入

这意味着系统需要一个完整的双向音频处理框架，而不只是一个录音接口。

GStreamer 的优势：

- 可同时组织采集链和播放链
- 便于接入 Opus 编解码器
- 便于接入 AEC、NS、AGC 等处理插件
- 更适合后续扩展输入源、混音、监控与回放
- 将时间戳、缓冲和 live pipeline 语义统一在一套框架内

### 3.2 为什么不选 ALSA-only

ALSA 只解决设备层 I/O，不解决：

- 回声消除
- 噪声抑制
- 自动增益
- 播放参考信号与采集链协同

如果后续只做“采麦克风并上传”，ALSA 足够；但对当前明确的双向音频需求，GStreamer 更合适。

## 4. 总体架构

音频模块由三层组成：

1. GStreamer 上行采集链
2. GStreamer 下行播放链
3. C++ 管理层与现有 UDP/gRPC 适配层

整体结构如下：

```text
                  +--------------------+
                  |    GrpcManager     |
                  |   PublishAudioCfg  |
                  +---------+----------+
                            |
                            v
                  +--------------------+
                  |   AudioManager      |
                  | lifecycle/config    |
                  +----+-----------+----+
                       |           |
          uplink       |           | downlink
                       |           |
                       v           v
        +-------------------+   +-------------------+
        | AudioCapturePipe  |   | AudioPlaybackPipe |
        | alsasrc -> opusenc|   | appsrc -> opusdec |
        |        -> appsink |   |        -> sink    |
        +---------+---------+   +---------+---------+
                  |                         ^
                  | Opus packet             |
                  v                         | Opus packet
            +-----------+                   |
            | UdpManager |-------------------+
            | sendAudio  |
            +-----------+
```

## 5. 模块划分

建议新增目录：

- `include/audio/`
- `src/audio/`

建议新增模块如下。

### 5.1 AudioManager

职责：

- 对外提供统一的音频生命周期接口
- 管理上行采集链和下行播放链
- 持有当前 `AudioConfig`
- 将上行编码输出回调给 MainNode / UdpManager
- 将下行收到的 Opus 包投递给播放链

建议接口：

```cpp
class AudioManager {
 public:
  struct Config;

  explicit AudioManager(ros::NodeHandle &nh);
  ~AudioManager();

  bool initialize(const Config &config);
  bool start();
  void stop();
  bool isRunning() const;

  void setEncodedPacketCallback(std::function<void(const uint8_t *, size_t, uint64_t)> cb);
  void pushRemoteOpusPacket(const uint8_t *data, size_t size, uint64_t timestamp_ntp);
};
```

### 5.2 AudioCapturePipeline

职责：

- 创建并运行 GStreamer 上行 pipeline
- 从 `appsink` 中取出每个 Opus 包
- 为每个 Opus 包生成上行发送时间戳
- 将编码结果回调给 `AudioManager`

### 5.3 AudioPlaybackPipeline

职责：

- 创建并运行 GStreamer 下行 pipeline
- 通过 `appsrc` 注入接收到的 Opus 包
- 解码并播放到本地音频输出设备
- 为后续 AEC 提供播放参考链

### 5.4 AudioConfigModel

职责：

- 将协议文档中的固定配置映射到本地结构体
- 用于参数加载、gRPC 发布和 pipeline 初始化

## 6. 固定音频参数

本方案固定以下参数，不支持热切换：

- 编码器：Opus
- 采样率：48000
- 声道数：1
- 输入采样格式：S16_LE
- 帧长：20ms
- 每帧样本数：960
- 码率：64000 bps
- Opus in-band FEC：开启
- Opus DTX：关闭

这些参数必须与 [docs/PROTOCOL_UDP_AUDIO_V2.md](docs/PROTOCOL_UDP_AUDIO_V2.md) 和后续 `AudioConfig` proto 保持一致。

## 7. 上行采集链设计

### 7.1 功能目标

上行链负责：

- 从本地麦克风实时采集音频
- 统一转换成 48kHz / mono / S16_LE
- 编码为单个 20ms Opus 包
- 通过 `appsink` 输出给业务层封装 UDP 音频头后发送

### 7.2 基础 pipeline

在不启用 AEC 的情况下，上行 pipeline 为：

```text
alsasrc do-timestamp=true !
audio/x-raw,format=S16LE,rate=48000,channels=1 !
audioconvert !
audioresample !
queue max-size-buffers=4 leaky=downstream !
opusenc audio-type=voice frame-size=20 bitrate=64000 inband-fec=true dtx=false !
appsink emit-signals=true sync=false max-buffers=4 drop=true
```

说明：

- `alsasrc` 负责从本地音频设备采集
- `do-timestamp=true` 让采集 buffer 带有 pipeline 时间戳
- `queue` 必须限制深度，避免音频累积延迟
- `appsink` 必须启用丢旧包策略，避免上行阻塞导致堆积

### 7.3 appsink 输出规则

`appsink` 每取到一个 sample，必须满足：

- sample 对应一个完整 Opus 包
- 该 Opus 包直接对应协议中的一个 UDP datagram payload
- 不做聚合，不做再分片

业务层需要从 sample 中提取：

- Opus 数据指针和长度
- pipeline 时间戳

然后将时间戳换算到统一 NTP64 时钟域，用作 [docs/PROTOCOL_UDP_AUDIO_V2.md](docs/PROTOCOL_UDP_AUDIO_V2.md) 中的 `TimestampNtp`。

## 8. 下行播放链设计

### 8.1 功能目标

下行链负责：

- 接收远端 Opus 音频包
- 将包注入播放 pipeline
- 完成 Opus 解码和本地播放
- 为后续 AEC 提供播放参考信号

### 8.2 基础 pipeline

```text
appsrc is-live=true format=time do-timestamp=false block=false !
queue max-size-buffers=8 leaky=downstream !
opusparse !
opusdec !
audioconvert !
audioresample !
autoaudiosink sync=true
```

说明：

- `appsrc` 用于将接收到的 Opus 包送入播放链
- `sync=true` 保持基于时间线播放
- `queue` 深度必须受控，避免下行声音越播越慢

### 8.3 appsrc 输入规则

播放链接收的数据单元与协议完全一致：

- 一个远端 UDP 音频包
- 解析掉 11 字节协议头后得到一个完整 Opus payload
- 将该 Opus payload 作为一个 `GstBuffer` 推入 `appsrc`

业务层负责依据协议头中的 `TimestampNtp` 和本地视频时间线决定何时推入播放链。

## 9. 回声消除设计

### 9.1 问题定义

后续系统支持：

- 本地麦克风采集上行
- 本地扬声器播放 VR 下行音频

如果播放走扬声器外放，本地麦克风会把扬声器声音再次采集进去，形成回声回传。

### 9.2 方案结论

本方案要求为 AEC 预留 GStreamer 处理链，优先使用 WebRTC 系列音频处理插件。

推荐方向：

- `webrtcdsp`
- `webrtcechoprobe`

设计要求：

- 播放链必须能提供 far-end reference
- 采集链中的 AEC 模块必须消费该 reference
- 若目标环境不具备相关插件，则系统应退化为“耳机模式优先”

### 9.3 产品策略

建议按两种使用模式定义：

1. 耳机模式

- VR 回传音频走耳机
- 麦克风基本不会收到播放信号
- 可不依赖 AEC，作为最稳的兜底模式

2. 外放模式

- VR 回传音频走扬声器
- 必须启用 AEC 才能避免明显回声回传

## 10. 时间戳与同步

### 10.1 时间戳来源

上行采集链输出的每个 Opus 包都必须生成 `TimestampNtp`。

要求：

- 音频时间戳必须与视频使用同一 NTP64 时钟域
- `TimestampNtp` 表示该 20ms 音频帧的起始播放时刻

建议实现：

- 记录本地 steady clock 与 NTP64 的换算基准
- 采样时用 pipeline buffer PTS 映射到 steady timeline
- 再转换为统一 NTP64

### 10.2 下行同步策略

根据 [docs/PROTOCOL_UDP_AUDIO_V2.md](docs/PROTOCOL_UDP_AUDIO_V2.md)，接收端采用 video-master：

- 音频早到则缓存等待
- 音频晚到则做 PLC/静音，不得阻塞视频

因此播放链前必须保留一层轻量级 jitter buffer / 调度器，由业务层决定送入 `appsrc` 的时机。

## 11. 与现有工程的集成点

### 11.1 MainNode

需要在 [include/main_node.hpp](include/main_node.hpp) 和 [src/main_node.cpp](src/main_node.cpp) 中新增：

- `initAudio()`
- `onEncodedAudioPacket()`
- 音频配置加载
- 配对成功后的音频启动逻辑
- unpair / stop 时的音频停止逻辑

建议与现有 `initVideo()` 生命周期保持一致。

### 11.2 UdpManager

需要在 [include/udp/udp_manager.hpp](include/udp/udp_manager.hpp) 和 [src/udp/udp_manager.cpp](src/udp/udp_manager.cpp) 中新增：

- `sendAudioPacket(const uint8_t *data, size_t size, uint64_t timestamp_ntp)`
- 音频 packet type `0x04`
- 音频协议头编码逻辑
- 收包侧对 `Type=0x04` 的解析入口

### 11.3 PacketCodec

需要在 [include/udp/packet_codec.hpp](include/udp/packet_codec.hpp) 和对应实现中新增：

- `kAudioPacketType = 0x04`
- `AudioPacketHeaderV2`
- `buildAudioPacket()`

### 11.4 gRPC / Proto

在 [protos/signaling.proto](protos/signaling.proto) 中新增：

- `AudioConfig`
- `AudioConfigAck`
- `PublishAudioConfig`
- `AckAudioConfig`

同时复用已有 `sub_audio` 语义。

## 12. 代码目录建议

建议新增文件：

- `include/audio/audio_manager.hpp`
- `include/audio/audio_capture_pipeline.hpp`
- `include/audio/audio_playback_pipeline.hpp`
- `src/audio/audio_manager.cpp`
- `src/audio/audio_capture_pipeline.cpp`
- `src/audio/audio_playback_pipeline.cpp`

## 13. 构建依赖

需要新增 GStreamer 开发依赖。

建议通过 `pkg-config` 接入：

- `gstreamer-1.0`
- `gstreamer-app-1.0`
- `gstreamer-audio-1.0`
- `gstreamer-base-1.0`

如果启用 WebRTC 音频处理插件，还需确保目标环境安装对应 runtime plugin。

## 14. 关键实现约束

### 14.1 上行约束

- 一个 GStreamer `appsink` sample 必须对应一个完整 Opus 包
- 一个 Opus 包必须对应一个 UDP datagram
- 不得聚合多帧，不得应用层分片

### 14.2 下行约束

- 一个收到的 UDP 音频包只向 `appsrc` 注入一个 Opus payload
- 播放时机必须受 `TimestampNtp` 和视频时间线约束
- 不得简单按到达顺序立即播放

### 14.3 配置约束

- 音频参数在流运行期间固定
- 修改参数必须停流重启
- 不支持热切换

## 15. 分阶段落地顺序

### 阶段 1：协议与配置接线

- 增加 `AudioConfig` proto
- 增加 UDP 音频 packet type 与 header 编码
- 增加 MainNode / UdpManager 音频入口

### 阶段 2：上行链

- 落地 GStreamer 采集 pipeline
- 输出单个 Opus 包并接入 UDP 发送

### 阶段 3：下行链

- 落地 GStreamer 播放 pipeline
- 收到远端 Opus 包后注入播放链

### 阶段 4：AEC

- 接入 WebRTC AEC 相关插件
- 验证“本地外放 + 本地采集”时的回声抑制效果

## 16. 验证标准

本方案完成后，至少满足以下条件：

1. 机器人端能够稳定采集麦克风音频并以 Opus 发送
2. 远端能够按 [docs/PROTOCOL_UDP_AUDIO_V2.md](docs/PROTOCOL_UDP_AUDIO_V2.md) 正确解包和播放
3. 机器人端能够接收远端 Opus 音频并本地播放
4. 音频播放不会阻塞视频主时间线
5. 耳机模式下，上下行双向音频稳定工作
6. 外放模式下，AEC 具备可验证的回声抑制效果
