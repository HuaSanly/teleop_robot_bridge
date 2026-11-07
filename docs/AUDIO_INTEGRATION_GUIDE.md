# 音频对接文档

## 1. 目的

本文档用于说明 teleop_robot_bridge 当前音频模块的对接方式，覆盖以下内容：

- 音频整体架构
- gRPC / Proto 信令协议
- UDP 音频数据协议
- 音频固定编码参数
- 发送端与接收端时序
- 运行参数与联调方式

本文档面向以下角色：

- 服务端 / 信令侧对接人员
- VR 端或接收端对接人员
- 本地联调和问题排查人员

## 2. 总体结论

当前音频链路的正式方案如下：

- 音频编码：Opus
- 采样率：48kHz
- 声道数：1（mono）
- 帧长：20ms
- 码率：64kbps
- 音频数据面：复用现有 UDP socket 和端口
- 音频包类型：`Type = 0x04`
- 配置面：通过 gRPC `AudioConfig` 下发和同步
- 时间戳：使用与视频相同的 NTP64 时钟域
- 发送节流：复用现有 UDP pacing 发送链路
- 应用层 FEC：音频不单独做 parity/FEC 分片，抗丢依赖 Opus PLC 与 in-band FEC

当前工程中，音频与视频、位姿的多路复用类型约定如下：

- `0x01`：视频
- `0x02`：位姿
- `0x04`：音频

## 3. 架构概览

音频模块分为三层：

1. 信令层：通过 gRPC 发布和同步 `AudioConfig`
2. 数据层：通过 UDP 发送和接收 Opus 音频包
3. 本地媒体层：通过 GStreamer 做采集、编码、解码、播放

整体路径如下：

```text
本地麦克风
  -> GStreamer capture pipeline
  -> opusenc
  -> AudioManager
  -> UdpManager::sendAudioPacket
  -> pacing / send queue
  -> UDP socket

远端 UDP 音频包
  -> UdpManager datagram callback
  -> AudioManager::pushRemoteOpusPacket
  -> appsrc
  -> opusdec
  -> 本地播放设备
```

## 4. Proto / gRPC 对接

### 4.1 所在文件

音频信令定义位于 [protos/signaling.proto](protos/signaling.proto) 中，没有单独拆新的 proto 文件。

### 4.2 AudioConfig

当前音频配置消息如下：

```proto
message AudioConfig {
  CodecType codec = 1;
  int32 sample_rate = 2;
  int32 channels = 3;
  int32 frame_duration_ms = 4;
  int32 samples_per_channel = 5;
  int32 bitrate_bps = 6;
  bool opus_inband_fec_enabled = 7;
  bool opus_dtx_enabled = 8;
  string config_id = 9;

  enum CodecType {
    OPUS = 0;
  }
}
```

当前实现约定的固定值如下：

- `codec = OPUS`
- `sample_rate = 48000`
- `channels = 1`
- `frame_duration_ms = 20`
- `samples_per_channel = 960`
- `bitrate_bps = 64000`
- `opus_inband_fec_enabled = true`
- `opus_dtx_enabled = false`

### 4.3 AudioConfigAck

```proto
message AudioConfigAck {
  bool success = 1;
  string message = 2;
  string config_id = 3;
}
```

### 4.4 EventMessage 扩展

音频配置同步通过 `EventMessage.oneof` 承载：

```proto
message EventMessage {
  string sender_session_id = 1;
  string target_session_id = 2;
  int64 timestamp = 3;

  oneof payload {
    PairEvent pair = 10;
    SystemCommand system = 11;
    VideoConfig video_config = 12;
    AudioConfig audio_config = 13;
  }
}
```

### 4.5 SubscribeRequest 扩展

音频订阅开关为：

```proto
message SubscribeRequest {
  Operation op = 1;
  string publisher_session_id = 2;
  bool sub_video = 3;
  bool sub_pose = 4;
  bool sub_audio = 5;
}
```

### 4.6 gRPC RPC

音频相关 RPC 如下：

```proto
rpc PublishAudioConfig (AudioConfig) returns (AudioConfigAck);
rpc AckAudioConfig (AudioConfigAck) returns (google.protobuf.Empty);
```

### 4.7 对接约定

发送端：

1. 完成 Register / Pair / Subscribe 等上层流程
2. 在音频数据开始发送前，调用 `PublishAudioConfig`
3. 只有在配置已经发布成功后，才开始发送 UDP 音频数据

接收端：

1. 从 `EventMessage.audio_config` 获取远端音频配置
2. 使用该配置初始化或确认本地解码参数
3. 如系统实现了 ACK 流程，则通过 `AckAudioConfig` 回 ACK

服务端：

1. 接收发送端 `PublishAudioConfig`
2. 将 `AudioConfig` 转发给配对端或订阅端
3. 可基于 `config_id` 做超时、重发和 ACK 管理

## 5. UDP 音频协议

### 5.1 版本

当前工程实现和文档统一使用 V2。

详见 [docs/PROTOCOL_UDP_AUDIO_V2.md](docs/PROTOCOL_UDP_AUDIO_V2.md)。

### 5.2 基本原则

- 音频与视频复用同一个 UDP socket 和同一个远端端口
- 一个 UDP datagram 只承载一个完整 Opus 包
- 不做应用层分片
- 不做音频专用 parity/FEC 包
- 接收端通过 datagram 第 0 字节 `Type` 分流

### 5.3 头部格式

音频包结构如下：

```text
[AudioPacketHeaderV2 (11 bytes)] + [OpusPayload]
```

对应结构体：

```c
struct AudioPacketHeaderV2 {
  uint8_t Type;         // 固定 0x04
  uint16_t Seq;         // Little-Endian
  uint64_t TimestampNtp;// Little-Endian
};
```

字段含义：

- `Type`
  固定为 `0x04`

- `Seq`
  每发送一个音频包自增 1，用于丢包检测、乱序整理和统计

- `TimestampNtp`
  音频包起始时间戳，使用 NTP64，与视频处于同一时钟域

### 5.4 载荷规则

- UDP 头之后的全部内容都是一个完整的 Opus 包
- 不允许一个 datagram 内携带多个 Opus 包
- 不允许把一个 Opus 包拆分成多个 datagram

计算方式：

```text
OpusPayloadLen = UdpDatagramLen - 11
```

### 5.5 MTU 和长度约束

当前实现中，UDP 最大 payload 配置值为 `1200` 字节。

因此：

- 音频头固定 11 字节
- Opus 载荷必须满足 `OpusPayloadLen <= 1189`

### 5.6 抗丢策略

音频 V2 不做额外应用层 FEC，当前依赖：

1. Opus in-band FEC
2. Opus PLC
3. 必要时静音补偿

## 6. 当前固定音频参数

当前实现固定使用以下参数：

| 项目 | 值 |
| --- | --- |
| 编码器 | Opus |
| 采样率 | 48000 |
| 声道 | 1 |
| PCM 格式 | S16_LE |
| 帧长 | 20ms |
| 每帧样本数 | 960 |
| 码率 | 64000 bps |
| Opus in-band FEC | true |
| Opus DTX | false |

这些参数必须同时满足：

- 与 `AudioConfig` 一致
- 与 UDP V2 协议文档一致
- 与收发双方 Opus 编解码器配置一致

## 7. 发送端时序

标准模式下，发送端时序如下：

1. gRPC Register
2. UDP 控制面握手 HELLO / PING / ACK
3. Pair / Subscribe 等业务流程完成
4. 发送端通过 `PublishAudioConfig` 发布音频配置
5. 本地 GStreamer 采集链输出 Opus 包
6. `AudioManager` 生成 NTP64 时间戳
7. `UdpManager::sendAudioPacket` 封装 `AudioPacketHeaderV2`
8. 音频包进入 pacing / send queue
9. 通过共享 UDP socket 发送到远端

关键约束：

- 未完成配置发布时，不应发送音频数据
- 标准模式下，未完成 UDP 控制面握手时，不应发送音频数据

## 8. 接收端时序

接收端对接时，建议按以下流程处理：

1. 从 gRPC 事件流获取 `AudioConfig`
2. 初始化或确认本地 Opus 解码参数
3. 接收 UDP datagram
4. 根据 `Type == 0x04` 判断为音频包
5. 解析 `Seq` 和 `TimestampNtp`
6. 将后续字节作为一个完整 Opus 包送入解码器
7. 以 `TimestampNtp` 为主进行音频时间线排序和播放

建议接收端具备以下能力：

- 按 `Seq` 做基础乱序处理
- 支持包丢失检测
- 支持视频主时钟下的 A/V 同步
- 使用 Opus FEC / PLC 处理丢包

## 9. 对接建议

- 必须先拿到 `AudioConfig` 再解释 UDP 音频流
- 按 `Type=0x04` 分流，而不是靠端口区分
- 解码端必须按 48kHz / mono / 20ms Opus 来处理
- 建议实现基础 jitter buffer 和丢包恢复策略


