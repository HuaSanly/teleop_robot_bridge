# 音频流协议文档 - UDP 传输（V2：复用通道 + Opus + gRPC 配置绑定）

## 概述

本文档定义 RoboticSystemPlatform 中音频数据的 UDP 传输协议 V2。

V2 的目标是：

- 音频与视频复用同一 UDP socket 和同一远端端口，通过 `Type` 字段分流。
- 音频数据面固定承载 Opus 码流，不再在每个 UDP 包里重复携带稳定配置字段。
- 音频稳定配置通过 gRPC `AudioConfig` 下发，UDP 数据面只保留逐包必需字段。
- 音频与视频共享同一 NTP64 时钟域，接收端采用 video-master 做 A/V 同步。
- 不引入 RTP/RTCP；不使用应用层分片；不使用视频那套外部 FEC 分片机制。

## 变更记录

- **(V1 -> V2)**:
  - 音频继续与视频复用同一 UDP socket/端口，通过 `Type=0x04` 分流。
  - 音频载荷固定为单个 Opus 包；每个 UDP datagram 承载 1 个 Opus 包。
  - 将 `Codec`、`Channels`、`SamplesPerChannel`、`OpusInbandFecEnabled` 等稳定字段移出 UDP 头，改由 gRPC `AudioConfig` 下发。
  - 移除 `PayloadLen`；接收端通过 UDP datagram 总长度减去头长得到 Opus 载荷长度。
  - 移除 `ConfigVersion`；V2 不支持活动音频流中的动态配置切换。
  - 移除 `Flags`；V2 不在逐包头部承载不连续状态。
  - 头部长度由 18 字节缩减为 **11 字节**。

## 传输层

- 协议：UDP
- Socket 模型：音频与视频复用同一 UDP socket 与同一远端端口
- 字节序：除 `Type` 外，所有多字节字段均采用 Little-Endian
- MTU 策略：单个 UDP payload 总长度必须小于等于 1200 字节
- 分片策略：V2 不做应用层分片；一个 UDP datagram 只承载一个 Opus 包
- FEC 策略：V2 不做应用层 parity/FEC 分片；抗丢依赖 Opus PLC 与 Opus in-band FEC
- 多路复用：音频包与视频包通过 datagram 第 0 字节 `Type` 分流

当前 `Type` 取值约定如下：

- `0x01`：视频流
- `0x02`：位姿流
- `0x04`：音频流

其他值由各自协议另行定义。

## 控制面与建链

音频数据面复用现有 UDP 控制面建链结果：

- HELLO/PING/ACK 文本控制面流程与视频共用同一个 UDP socket
- 当共享 UDP 通道握手就绪后，才允许发送本文档定义的音频数据包
- 音频协议不定义额外的独立握手消息

控制面文本协议不在本文档中定义；本文档只定义音频数据面。

## gRPC 配置面

音频流开始发送前，发送端必须通过 gRPC 发布 `AudioConfig`，接收端必须在持有对应版本配置后才能正确解释 UDP 音频流。

### AudioConfig（规范性要求）

V2 固定使用如下配置：

- `codec = OPUS`
- `sample_rate = 48000`
- `channels = 1`
- `frame_duration_ms = 20`
- `samples_per_channel = 960`
- `bitrate_bps = 64000`
- `opus_inband_fec_enabled = true`
- `opus_dtx_enabled = false`

配置消息必须至少包含以下字段：

- `codec`
- `sample_rate`
- `channels`
- `frame_duration_ms`
- `samples_per_channel`
- `bitrate_bps`
- `opus_inband_fec_enabled`
- `opus_dtx_enabled`

V2 中，上述配置字段的规范值固定如上；接收端若收到与本规范不一致的 `AudioConfig`，必须拒绝该配置。

### 配置生效规则

- `AudioConfig` 在音频流启动前发布一次并固定生效
- V2 不支持活动音频流中的配置热切换
- 若发送端需要修改任一音频编码参数，必须先停止当前音频流，再重新发布 `AudioConfig`，然后以新的流重新开始发送
- 接收端在检测到音频流重启时，必须重置 jitter buffer 与 Opus 解码器状态

## 消息格式

### 整体结构

```text
[AudioPacketHeaderV2 (11 bytes)] + [OpusPayload]
```

### AudioPacketHeaderV2 结构定义

所有多字节字段均采用 Little-Endian。

```c
#pragma pack(push, 1)
struct AudioPacketHeaderV2 {
  uint8_t  Type;           // +0: 固定 0x04
  uint16_t Seq;            // +1: 连续包序号（mod 65536）
  uint64_t TimestampNtp;   // +3: 音频起始时间戳（NTP64，与视频同一时钟域）
};
#pragma pack(pop)
// Total Size: 11 bytes
```

### 字段语义

#### Type（1 字节）

- 固定值：`0x04`
- 因为音频与视频复用同一 UDP 通道，接收端必须依赖 `Type` 做多路分流

#### Seq（2 字节，LE）

- 发送端每发送一个音频 datagram 自增 1
- 取值范围 `0..65535`，回绕后从 0 继续
- 接收端用于：
  - 丢包统计
  - 乱序重排
  - FEC/PLC 恢复时判断前一包是否缺失

#### TimestampNtp（8 字节，LE）

- 表示该 Opus 包对应 PCM 的起始播放时间
- 使用 NTP64，且必须与视频 `Timestamp` 使用同一时钟域
- 接收端以该字段作为 A/V 同步依据，不能以到达时间代替

## Payload 规则

- `AudioPacketHeaderV2` 之后的全部字节都是 `OpusPayload`
- `OpusPayload` 长度计算方式：

```text
OpusPayloadLen = UdpDatagramLen - 11
```

- `OpusPayloadLen` 必须大于 0
- 发送端不得在单个 UDP datagram 中承载多个 Opus 包
- 发送端不得对一个 Opus 包做应用层分片

## 编码规范

V2 音频编码参数固定如下：

- 编码器：Opus
- 采样率：48kHz
- 声道：1（mono）
- 帧长：20ms
- 每帧每声道采样数：960
- 目标码率：64kbps
- Opus in-band FEC：开启
- Opus DTX：关闭

发送端必须保证每个 UDP 包只包含一个 20ms Opus 包。

## 丢包恢复规则

V2 不使用应用层 parity/FEC 包。接收端必须按以下顺序处理丢包：

1. 优先使用 Opus in-band FEC 恢复前一包
2. 若无法使用 Opus in-band FEC，则使用 Opus PLC
3. 若仍无法恢复，则输出静音并继续推进时间线

### Opus in-band FEC 使用规则

- 当包 `N` 丢失、包 `N+1` 到达时，接收端可以尝试使用包 `N+1` 中携带的 FEC 信息恢复包 `N`
- 若包 `N+1` 也丢失，则不能恢复包 `N`，必须退化到 PLC 或静音
- 接收端不得为了等待 FEC 恢复而阻塞视频时间线

## A/V 同步约定（Video-master）

目标：不增加视频延迟，音频追随视频。

- 音频与视频都使用同一时钟域的 NTP64 时间戳
- 接收端以视频播放时间线作为主时钟
- 对于任意音频包：
  - 若音频早到：缓存，直到视频时间线推进到 `TimestampNtp` 再输出
  - 若音频晚到：不得阻塞视频；应直接做 PLC/静音并尽快追上当前视频时间线

## 接收端处理流程

1. 读取 UDP datagram，验证长度必须大于 11 字节
2. 解析 `AudioPacketHeaderV2`
3. 检查当前音频流对应的 `AudioConfig` 是否已就绪；若未就绪，则丢弃该包
4. 按 `Seq` 做乱序整理与丢包检测
5. 按 `TimestampNtp` 将音频包挂入播放时间线
6. 当检测到前一包丢失且后一包已到时，优先尝试 Opus in-band FEC
7. 若 FEC 不可用，则执行 PLC
8. 最终以视频时间线为主完成音频输出

## 与视频协议的关系

- 音频 V2 不引入 `FrameId`
  - 原因：一个 UDP datagram 对应一个完整 Opus 包，`Seq + TimestampNtp` 已足够标识顺序与时间线
- 音频 V2 不引入 `FragmentIndex` / `TotalFragments`
  - 原因：V2 明确禁止应用层分片
- 音频 V2 不引入 `PayloadLength`
  - 原因：UDP datagram 本身已经给出总长度，接收端可直接推导 Opus 载荷长度
- 音频 V2 不引入 `ConfigVersion`
  - 原因：V2 明确要求音频配置在流启动后固定，不支持热切换
- 音频 V2 不引入 `Flags`
  - 原因：V2 采用极简逐包头设计，不在数据面显式传递不连续状态

## 实现约束

- 发送端必须先完成共享 UDP 通道控制面握手，再发送音频数据
- 发送端必须先完成 `AudioConfig` 发布，再开始发送音频数据
- 发送端不得发送与当前 `AudioConfig` 不一致的 Opus 流
- 活动音频流期间，发送端不得修改编码参数
- 若发送端需要修改编码参数，必须停止当前流并重新启动新流
