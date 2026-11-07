# PROTOCOL_UDP_AUDIO_V1

## 1. 概述

本文档定义 UDP 音频数据面协议（V1）。音频与现有 UDP 复用通道共享端口，通过 datagram 第 0 字节 `Type` 分流。音频载荷默认采用 Opus 编码，用于“环境音/麦克风混合后一路输出”的播放场景。

设计目标：
- 与现有协议保持一致：小端（Little-Endian）、首字节 Type 分流、避免 IP 分片（建议 payload ≤ 1200B）。
- 为 A/V 同步提供足够信息：采用与视频一致的 NTP64 时间戳。
- 不引入 RTP/RTCP 栈，仅保留必要字段（seq + timestamp + payloadLen 等）。

## 2. 多路复用（Multiplexing）

同一 UDP socket 上不同数据流通过 `payload[0]` 的 `Type` 区分：

- 0x01：视频流
- 0x02：位姿流
- 0x03：反馈流
- 0x04：音频流（本文档）

说明：
- 控制面（例如 HELLO/PING/ACK/PONG/DACK）属于另一套“文本前缀分流”规则，不在本文档范围。

## 3. 传输与约束

- 传输层：UDP
- 字节序：除 `Type/Channels/Codec/Flags` 外，所有多字节字段均为 Little-Endian
- MTU 策略：建议单个 UDP payload 总长度 ≤ 1200 字节（含本协议头），以降低分片概率

## 4. 数据报格式（Audio Datagram V1）

整体结构：

- AudioPacketHeaderV1（固定 18 字节）
- OpusPayload（长度为 `PayloadLen`）

### 4.1 Header 布局（18B，Little-Endian）

| Offset | Size | Type | Endian | Name              | Description |
|-------:|-----:|------|--------|-------------------|-------------|
| 0      | 1    | u8   | -      | Type              | 固定 0x04 |
| 1      | 2    | u16  | LE     | Seq               | 音频包序号，mod 65536 循环递增 |
| 3      | 8    | u64  | LE     | TimestampNtp       | 音频起始时间戳（NTP64，同视频时钟域） |
| 11     | 2    | u16  | LE     | SamplesPerChannel  | 每声道 sample 数（48kHz 下 20ms=960） |
| 13     | 1    | u8   | -      | Channels           | 声道数，默认 1（mono） |
| 14     | 1    | u8   | -      | Codec              | 编码类型：1=Opus |
| 15     | 1    | u8   | -      | Flags              | 标志位（见下） |
| 16     | 2    | u16  | LE     | PayloadLen          | OpusPayload 长度（字节） |
| 18     | N    | bytes| -      | OpusPayload         | Opus 码流数据 |

头部结构定义（C 风格示意）：

- `AudioPacketHeaderV1` 总长度固定为 18 字节。
- 字段紧凑排列（packed），用于跨语言/跨平台解析。

## 4.2 字段语义

### Type
- 固定 0x04。

### Seq（u16, LE）
- 发送端每发送一个音频 datagram 自增 1（u16 回绕）。
- 接收端可用于丢包/乱序统计与 jitter buffer 排序。

### TimestampNtp（u64, LE）
- 表示该音频 payload 对应 PCM 的“起始时刻”（与视频 `TimestampNtp` 共享同一时钟域）。
- 用于音频追随视频（video-master）同步。

### SamplesPerChannel（u16, LE）
- 表示该包解码后每个声道包含的 PCM sample 数。
- 典型值（48kHz）：
  - 20ms：960
  - 40ms：1920（若将来改为 40ms/包，可保持该字段自描述）

### Channels（u8）
- 声道数。
- 当前默认 1（mono）。

### Codec（u8）
- 1：Opus
- 其他值保留扩展。

### Flags（u8）
- bit0：`OpusInbandFecEnabled`（发送端编码器启用 in-band FEC）
- bit1：`Discontinuous`（可选：标记流不连续/重启等；接收端可据此重置音频缓冲）
- 其他位保留，发送端置 0，接收端忽略未知位。

### PayloadLen（u16, LE）
- OpusPayload 的字节数。
- 建议约束：`18 + PayloadLen <= 1200`（避免 IP 分片）。

### OpusPayload（PayloadLen bytes）
- Opus 编码数据，按发送端编码器输出原样填充。
- V1 默认约定：每个音频 datagram 承载 1 个 Opus 帧。

## 5. A/V 同步约定（Video-master）

目标：不增加视频延迟，音频对齐视频。

- 音频与视频都携带同一时钟域的 `TimestampNtp`（NTP64）。
- 接收端以“当前视频播放/提交的时间戳”作为主时钟（video-master）：
  - 音频若早到：缓存等待，直到视频时间线推进到对应 `TimestampNtp` 再输出。
  - 音频若晚到：不阻塞视频；输出 PLC/静音并尽快追上。
- jitter buffer 深度与策略由实现决定，但必须以 `TimestampNtp` 为对齐依据（不能只按到达时间）。

## 6. 编码建议（Opus）

默认建议（环境音/单声道）：
- 采样率：48kHz
- 声道：1（mono）
- 帧长：20ms
- 码率：48–64kbps（可配置）
- 抗丢：启用 Opus PLC；建议启用 Opus in-band FEC（并设置 Flags.bit0）
