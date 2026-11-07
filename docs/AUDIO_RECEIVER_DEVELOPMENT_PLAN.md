# 音频接收侧开发计划

## 1. 目标

本文档用于指导 teleop_robot_bridge 音频接收侧的后续开发。

目标不是再做一版“能响”的最小实现，而是补齐一条可以稳定用于真实业务的接收链，覆盖：

- UDP 音频包解析
- 乱序重排
- jitter buffer
- 丢包检测与恢复
- 播放时序控制
- A/V 同步
- 流重置与重同步
- 运行期统计与联调手段

后续开发默认以本文档为准。

## 2. 当前现状

当前工程已经具备以下基础能力：

1. 发送侧已经实现 Opus 编码、UDP V2 音频头封装和 pacing 发送。
2. gRPC / proto 已经支持 `AudioConfig`、`AudioConfigAck`、`PublishAudioConfig`、`AckAudioConfig`。
3. 接收侧已经能在 UDP datagram callback 中识别 `Type = 0x04` 的音频包。
4. 接收侧当前会直接从 UDP payload 中取出 Opus 载荷，并立即送入 GStreamer 播放链。
5. 当前播放链只做了非常轻量的时间戳映射：
   - 第一个远端包作为 `playback_base_ntp`
   - 后续包按 `TimestampNtp` 计算 PTS
   - 如果 PTS 倒退，则强制改成单调递增

也就是说，当前接收链属于“最小直通版”：

- 能收
- 能推给播放链
- 但还没有正式的网络接收控制层

## 3. 当前缺口

当前接收侧还缺少以下关键能力：

1. 没有完整解析音频头并建立接收包对象。
2. 没有正式使用 `Seq` 做乱序、丢包和重复包处理。
3. 没有 jitter buffer。
4. 没有 Opus FEC 恢复逻辑。
5. 没有 PLC / 静音补偿策略。
6. 没有面向视频主时钟的音频播放控制。
7. 没有收到新 `AudioConfig` 后的流重建逻辑。
8. 没有接收侧统计和问题定位数据。

## 4. 开发原则

后续开发遵守以下原则：

1. 不破坏现有发送链。
2. 接收侧控制逻辑独立成模块，不把 MainNode 继续堆大。
3. 先把“时序正确”和“状态正确”做好，再追求复杂优化。
4. 先实现可观测性，再做复杂恢复逻辑。
5. 先完成单路音频稳定播放，再接入严格的 A/V 同步策略。

## 5. 推荐目标架构

建议把接收侧拆成以下结构：

```text
UDP datagram
  -> AudioReceiver
      -> parse header
      -> validate packet
      -> reorder by Seq
      -> jitter buffer
      -> loss detection
      -> FEC / PLC decision
      -> playout scheduler
      -> AudioManager::pushRemoteOpusPacket(...)
          -> appsrc
          -> opusdec
          -> local sink
```

建议新增模块：

- `include/audio/audio_receiver.hpp`
- `src/audio/audio_receiver.cpp`

建议职责划分：

### 5.1 MainNode

职责：

- 只做 UDP datagram 分流
- 把音频 datagram 交给 AudioReceiver
- 不再在 MainNode 中手工解析音频头和直接推动播放

### 5.2 AudioReceiver

职责：

- 解析 `AudioPacketHeaderV2`
- 接收包合法性校验
- Seq 状态维护
- 乱序重排
- jitter buffer 管理
- 丢包检测
- FEC / PLC 决策
- 播放时序调度
- 输出给 AudioManager

### 5.3 AudioManager

职责：

- 保留 GStreamer capture / playback 生命周期管理
- 提供播放注入接口
- 不负责网络乱序与恢复策略

## 6. 分阶段开发计划

### Phase 1：建立正式接收入口

目标：把接收逻辑从 MainNode 中剥离出来，形成独立的接收侧控制模块。

任务：

1. 新增 `AudioReceiver` 类。
2. 定义接收包结构，例如：

```cpp
struct AudioRxPacket {
  uint16_t seq;
  uint64_t timestamp_ntp;
  std::vector<uint8_t> payload;
  uint64_t arrival_time_ns;
};
```

3. 把当前 MainNode 中对 `Type=0x04` 的处理改为：
   - 只分流
   - 不直接调用 `pushRemoteOpusPacket`
4. 在 `AudioReceiver` 中完成音频头解析：
   - `Type`
   - `Seq`
   - `TimestampNtp`
   - `OpusPayload`
5. 增加基础校验：
   - datagram 长度必须大于 11
   - `Type` 必须为 `0x04`
   - payload 长度必须大于 0

验收标准：

1. MainNode 中不再直接手动取 `data + 11` 推给播放链。
2. AudioReceiver 能打印结构化接收日志：seq、timestamp、payload size。
3. 保持现有“最小直通播放”能力不回退。

### Phase 2：Seq 状态机和乱序重排

目标：正式使用 `Seq` 解决音频包乱序、重复包、跳号问题。

任务：

1. 为接收流维护状态：
   - `expected_seq`
   - `last_released_seq`
   - `started`
2. 增加一个小型待排队容器，按 `Seq` 排序。
3. 处理以下情况：
   - 正常顺序到达
   - 乱序到达
   - 重复包
   - 明显过旧包
   - 跳号
4. 定义窗口大小，例如最多缓存 8 到 16 个包。

验收标准：

1. 乱序到达的包能按正确顺序释放。
2. 重复包不会重复播放。
3. 能输出明确统计：重复包数、乱序包数、跳号次数。

### Phase 3：jitter buffer

目标：吸收网络抖动，避免到达时间波动直接反映为播放波动。

任务：

1. 在 AudioReceiver 中增加 jitter buffer。
2. jitter buffer 以 `Seq` 或 `TimestampNtp` 为排序键，建议内部仍以 `Seq` 为主。
3. 增加目标缓冲深度配置，例如：
   - `target_delay_ms`
   - `max_delay_ms`
4. 增加释放策略：
   - 达到初始缓冲量后开始播放
   - 缓冲过满时丢弃过旧包或追时钟

验收标准：

1. 在轻微抖动条件下播放连续性明显优于直通模式。
2. 能输出当前 jitter buffer 深度和初始起播时间。

### Phase 4：丢包检测

目标：正式判断哪些包丢了，并为恢复策略提供输入。

任务：

1. 依据 `expected_seq` 检测缺包。
2. 对缺包生成结构化事件，例如：

```cpp
struct AudioLossEvent {
  uint16_t missing_seq_begin;
  uint16_t missing_seq_end;
  uint64_t expected_timestamp_ntp;
};
```

3. 维护以下统计：
   - lost_packets
   - late_packets
   - reordered_packets
   - duplicate_packets

验收标准：

1. 丢包情况下能稳定打印缺包范围。
2. 不因为单个缺包直接打乱后续正常播放。

### Phase 5：Opus FEC 恢复

目标：利用 Opus in-band FEC 恢复前一包丢失场景。

任务：

1. 在检测到包 `N` 丢失且包 `N+1` 到达时，尝试用 `N+1` 恢复 `N`。
2. 明确 FEC 恢复调用路径。
3. 统计：
   - fec_recovered_packets
   - fec_failed_packets

说明：

当前发送侧已经固定启用了 `opus_inband_fec_enabled = true`，所以接收侧必须补齐配套恢复逻辑。

验收标准：

1. 单包丢失场景优先走 FEC 恢复。
2. 恢复成功和失败均可观测。

### Phase 6：PLC / 静音补偿

目标：在 FEC 不可用时保持时间线连续。

任务：

1. 明确包恢复优先级：
   - FEC
   - PLC
   - 静音补偿
2. 对连续丢包场景定义上限策略。
3. 保证即使完全恢复不了，也要推进播放时间线。

验收标准：

1. 丢包时不会因为等待永远卡住播放链。
2. 能统计 PLC 次数和静音补偿次数。

### Phase 7：播放时序控制

目标：把当前“简单单调 PTS”升级成真正的播放调度。

当前已有逻辑：

- 根据 `TimestampNtp` 计算 PTS
- 如果 PTS 倒退，则强制修成 `playback_last_pts_ns + duration`

这只能解决时间戳倒退，不能解决：

- 包早到
- 包晚到
- 网络抖动
- 流重启

任务：

1. 把“数据到达时间”和“计划播放时间”分离。
2. 引入 playout scheduler。
3. 对以下情况分别处理：
   - 包早到：缓存等待
   - 包晚到：丢弃或补 PLC
   - 包严重晚到：直接追时钟
4. 定义可配置阈值：
   - late_drop_threshold_ms
   - max_catchup_ms

验收标准：

1. 播放时钟推进策略清晰、可日志验证。
2. 不会单纯因为一个晚到包把整条播放链拖慢。

### Phase 8：A/V 同步

目标：音频追随视频主时钟，而不是独立自由播放。

任务：

1. 明确视频播放时间线来源。
2. 为 AudioReceiver 提供“当前视频主时钟时间”。
3. 以视频时钟判断音频包：
   - 早到则等
   - 晚到则追
4. 明确允许的音画偏差窗口，例如：
   - `audio_lead_tolerance_ms`
   - `audio_lag_tolerance_ms`

验收标准：

1. 播放时序不再只靠音频自己的时间基线。
2. 在视频稳定时，音频不会长期漂移。

### Phase 9：AudioConfig 驱动的流管理

目标：让接收侧真正由远端 `AudioConfig` 驱动，而不是默认假设固定参数永不变化。

任务：

1. 接收到 `AudioConfig` 后标记配置就绪。
2. 未配置就绪时拒绝播放音频包。
3. 若收到新的 `AudioConfig`：
   - 重置 jitter buffer
   - 重置 Seq 状态
   - 重置播放基线
   - 视需要重建解码链
4. 增加配置版本切换日志。

验收标准：

1. 配置变化能正确触发流重置。
2. 不会把旧流状态带到新流里。

### Phase 10：流重启与重同步

目标：处理远端重启、时间戳回跳、长时间断流等情况。

任务：

1. 定义“流重启”判据：
   - Seq 大幅回绕或异常跳变
   - TimestampNtp 大幅回退
   - 长时间无包后重新来包
2. 触发以下动作：
   - 清空 jitter buffer
   - 清空待排队容器
   - 重置播放时基
   - 必要时软重启播放链

验收标准：

1. 远端重启后接收侧能够自动恢复。
2. 不会长期卡在错误状态里。

### Phase 11：观测与调试能力

目标：让接收链问题能被快速定位。

建议增加以下统计：

- total_received_packets
- total_released_packets
- total_lost_packets
- total_reordered_packets
- total_duplicate_packets
- total_late_dropped_packets
- total_fec_recovered_packets
- total_plc_packets
- total_silence_packets
- current_jitter_depth_packets
- current_jitter_depth_ms
- current_playout_delay_ms

建议日志节奏：

- 每秒打印累计统计
- 在状态切换、流重启、配置切换时打印事件日志

验收标准：

1. 接收侧任何问题都能先从统计中判断类别。
2. 不需要每次靠抓包才能定位基本问题。

## 7. 推荐落地文件改动

### 必改文件

- `include/audio/audio_manager.hpp`
- `src/audio/audio_manager.cpp`
- `src/main_node.cpp`

### 建议新增文件

- `include/audio/audio_receiver.hpp`
- `src/audio/audio_receiver.cpp`

### 可能需要配套修改的文件

- `config/params.yaml`
  增加 jitter buffer、时序阈值、统计开关等参数

- `docs/AUDIO_INTEGRATION_GUIDE.md`
  更新接收侧实际能力说明

## 8. 建议新增配置项

建议后续在参数文件中增加：

```yaml
audio:
  rx:
    jitter_target_ms: 60
    jitter_max_ms: 120
    reorder_window_packets: 16
    late_drop_threshold_ms: 80
    max_catchup_ms: 100
    enable_fec_recovery: true
    enable_plc: true
    stats_log_enabled: true
    stats_log_interval_ms: 1000
```

## 9. 开发顺序建议

建议严格按以下顺序推进，不要倒序开发：

1. Phase 1：接收入口重构
2. Phase 2：Seq 状态机与乱序重排
3. Phase 3：jitter buffer
4. Phase 4：丢包检测
5. Phase 5：FEC 恢复
6. Phase 6：PLC / 静音补偿
7. Phase 7：播放时序控制
8. Phase 8：A/V 同步
9. Phase 9：AudioConfig 驱动流重建
10. Phase 10：流重启与重同步
11. Phase 11：统计与调试能力完善

原因：

- 没有 Seq 状态机，FEC 和 PLC 没法做对
- 没有 jitter buffer，播放时序控制没有基础
- 没有统计能力，复杂恢复逻辑难以联调

## 10. 每阶段完成定义

每个阶段都必须同时满足以下条件，才算完成：

1. 代码完成并编译通过
2. 对应日志和统计可观测
3. 至少完成一轮人工联调验证
4. 文档同步更新

## 11. 联调建议

每一阶段开发完成后，建议至少做以下验证：

1. 正常顺序收包播放
2. 手工构造乱序包
3. 手工丢弃单包
4. 连续丢 2 到 3 包
5. 时间戳回跳
6. 音频流停止后重启
7. 收到新的 AudioConfig

## 12. 文档状态

本文档为接收侧开发主清单。

后续开发默认按照本文档推进；若实现策略与本文档不一致，应先更新本文档，再修改代码。
