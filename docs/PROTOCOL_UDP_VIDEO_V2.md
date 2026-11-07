# 视频流协议文档 - UDP 传输（V2：隐式分组 + 静态 FEC 策略表）

## 概述

本文档描述 RoboticSystemPlatform 中使用 UDP 进行视频数据传输的协议格式（V2）。
V2 通过“隐式分组（不传 GroupId/GroupIndex）+ 静态策略表”将分组拓扑从包头移到接收端推导，从而最小化头部开销。

FEC 的具体规则（N→(G,R)、索引映射、RS 系数生成）由 `Docs/UDP_VIDEO_FEC_STRATEGY.md` 定义。

## 变更记录

- **(V1 -> V2)**: 
  - 移除显式的 `FecGroupId` 和 `FecGroupIndex`，接收端通过 `TotalFragments` 查静态表推算拓扑。
  - 移除 `ReservedFec0`。
  - 保留 `FecTableId` (1字节)，用于支持多表切换；并作为 FEC 启用开关。
  - 移除 `FecProfileId`。
  - 头部总长度固定为 **26 字节**。
- **(V2 扩展)**:
  - 新增 `KeyframeFlag` (1字节)，标记该帧是否为关键帧。
  - 头部总长度变为 **27 字节**。

## 传输层

- **协议**: UDP
- **MTU 策略**: 
  - 目标 Payload 大小: **1200 字节**
  - 协议头大小: **27 字节** (Pack 紧凑排列)
  - 总 UDP Payload 大小: **1227 字节** (远小于 1500，安全)

## 消息格式

### 整体结构

```
[VideoPacketHeaderV2 (27 bytes)] + [视频数据载荷 (Max 1200 bytes)]
```

### VideoPacketHeaderV2 结构定义

所有多字节字段均采用 **小端字节序 (Little-Endian)**。

```c
#pragma pack(push, 1)
struct VideoPacketHeaderV2 {
   // --- V1 基础字段 (25 bytes) ---
   uint8_t  Type;              // +0:  消息类型 (0x01)
   uint16_t PacketSeqNum;      // +1:  连续包序号 (0-65535 循环, SCReAM用)
   uint64_t Timestamp;         // +3:  NTP 时间戳 (64-bit)
   uint32_t FrameId;           // +11: 帧ID
   uint16_t FragmentIndex;     // +15: 当前分片索引 (0 开始, 含冗余分片)
   uint16_t TotalFragments;    // +17: 原始分片总数 (N)
   uint16_t PayloadLength;     // +19: 当前包有效载荷长度
   uint32_t FramePayloadLength;// +21: 当前帧总的有效字节数

  // --- V2 新增字段 (2 bytes) ---
  uint8_t  FecTableId;        // +25: 静态表 ID (8-bit)
                     // 0 表示禁用 FEC；非 0 表示启用并选择对应策略表。
  uint8_t  KeyframeFlag;      // +26: 关键帧标志 (0=非关键帧, 1=关键帧)
};
#pragma pack(pop)
// Total Size: 27 bytes
```

### 字段逻辑说明

#### FecTableId（1 字节）
- **类型**: `uint8_t`
- **用途**:
  - 选择“静态 FEC 策略表”（Table）。
  - 当前默认值：`1`（对应代码中的 `VideoFecStrategyTableV1.TableId`）。
  - 接收端根据 `FecTableId` + `TotalFragments` 推导该帧的 `(GroupCount, ParityCount)`。
  - **禁用方式**：`FecTableId = 0` 表示该帧不启用 FEC。

  #### KeyframeFlag（1 字节）
  - **类型**: `uint8_t`
  - **用途**:
    - 标记该帧是否为关键帧（IDR）。
    - 接收端在首次解码或重建后可据此等待关键帧再开始解码。
  - **取值**:
    - `0`：非关键帧
    - `1`：关键帧

#### 隐式分组推断 (Implicit Grouping)
- **核心机制**: 数据包中不再携带 `GroupId` 和 `GroupIndex`。
- **推断流程**:
  1. 若 `FecTableId == 0`：该帧不走 FEC（按“仅源分片”逻辑组帧）。
  2. 读取 `TotalFragments` 作为 $N$。
  3. 根据 `FecTableId` 选择策略表，并用 $N$ 查表得到 `GroupCount`（记为 $G$）与 `ParityCount`（记为 $R$）。
  4. 由 `FragmentIndex` 推断该包属于哪个组、以及组内序号：
     - 若 `FragmentIndex < N`（源分片）：
       - `GroupId = FragmentIndex % G`
       - `SourceOrdinal = FragmentIndex / G`
     - 若 `FragmentIndex >= N`（parity 分片）：令 `p = FragmentIndex - N`（parityGlobal）
       - `GroupId = p % G`
       - `ParityOrdinal = p / G`

完整映射公式与 parity 索引空间定义见 `Docs/UDP_VIDEO_FEC_STRATEGY.md`。

### 示例推算

假设 `TotalFragments` (N) = 30, 查表得 `GroupCount` = 4, `ParityCount` = 2。
总包数 = $30 + 4 \times 2 = 38$ 包。

- **收到 FragmentIndex = 5 (原始包)**
  - `5 < 30` -> 是原始包。
  - 所属组 = `5 % 4 = 1` (Group 1)。
  
- **收到 FragmentIndex = 30 (冗余包)**
  - `30 >= 30` -> 是冗余包。
  - `p = 30 - 30 = 0`
  - 所属组 = `p % 4 = 0` (Group 0)
  - 这是 Group 0 的第 0 个冗余包 (Parity 0)

- **收到 FragmentIndex = 33 (冗余包)**
  - `33 >= 30` -> 是冗余包。
  - `p = 33 - 30 = 3`
  - 所属组 = `p % 4 = 3` (Group 3)
  - 这是 Group 3 的第 0 个冗余包 (Parity 0)

## 处理流程总结

1. **解析头部**: 获取 `FrameId`, `FragmentIndex`, `TotalFragments`。
2. **查表**: 用 `TotalFragments` 获取 FEC 参数 (Groups, Parity)。
3. **映射**: 将包放入对应的 FEC Group 用于丢失恢复。
4. **重组**: 当某帧的所有原始分片集齐，或通过 FEC 恢复出缺失分片后，提交解码。
