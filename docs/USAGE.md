# Teleop Robot Bridge 使用说明

本文档面向使用者，说明如何运行 `teleop_robot_bridge` 节点并验证最新后端信令与媒体链路。

## 运行前准备

- 已配置 ROS 1 运行环境。
- Jetson 设备已安装 Jetson Multimedia API。
- 相机可通过 V4L2 设备访问（默认 /dev/video2）。

## 编译

在工作空间根目录执行：

```bash
cd /home/nvidia/ROS/tele_ws
catkin_make --pkg teleop_robot_bridge
```

## 配置

主配置文件：

- config/params.yaml

常用配置项：

- video.profile：分辨率与帧率
- video.device：V4L2 设备路径
- video.encoder：码率与 I/IDR 间隔
- video.eye_image：是否启用眼图与输出尺寸
- udp.server_udp_ip / udp.server_udp_port：远端地址

### 仅生产数据的直入模式

设置 `debug.direct_running_mode: true` 后，节点直接进入 RUNNING，跳过 gRPC 注册、配对和协商，完全不启动项目 UDP 收发。不会创建媒体链路的 UDP socket，也不会启动 FEC、pacing、收发线程或握手定时器；编码数据不进入 FEC 输入队列或 UDP 发送队列。

视频采集、解码、转换、去畸变和编码继续按配置运行。音频的启用、上行、下行和 AEC 设置保持生效；由于 UDP 接收关闭，下行没有远端音频输入。`video.eye_image` 的 ROS 话题发布继续遵循启用、尺寸、频率和订阅者等现有配置。这里禁用的是项目的 gRPC/UDP 链路，ROS 通信正常保留。

通过 `[VIDEO]` 的 `cap/dec/conv/und/enc` 和 `[EYE_IMAGE]` 观察产出；直入模式下 `tx` 和 UDP 队列 `q` 应始终为零。`[AUDIO] cap` 表示采集编码产出量，`up` 表示编码包交付给回调的数量和码率，并不表示 UDP 已发送；直入模式下 `up` 仍可能非零。

该参数默认值为 `false`，当前配置为 `true`。参数只在启动时读取，修改后需重启节点；设为 `false` 恢复正常的注册、配对、协商和网络传输流程。

### 最新后端设备参数

当前配置使用后台设备登记页中的稳定身份：

- `grpc.device_id`: `device:44547c67f3364d959918a9d0`
- 后台序列号：`SN-25003655`（仅用于网站管理，不进入 Register Proto）
- `grpc.expected_device_type_code`: `0001000102`（仅用于对照后端响应，不发送给后端）

新后端的 `RegisterRequest` 只提交稳定 `device_id` 和本次运行描述，不再提交 `token`、`role`、`robot_generation` 或序列号。设备类型、显示名和硬件描述都来自后台登记记录；注册成功时节点会打印后端返回的权威设备类型。

### Publisher Manifest 与 StreamConfig

每次取得新 Session 后，节点按 `Register → EventStream → Manifest → UDP HELLO → Pairing → StreamConfig` 启动。EventStream 先于 UDP 建立，以便后端在端点绑定异常时下发 `REQUEST_UDP_HELLO`。

节点为本 Session 声明以下 Publisher-local Manifest，再启动 UDP：

| Prefix | stream_code | stream_class_code | protocol_code | 用途 |
| --- | --- | --- | --- | --- |
| `0x01` | `stereo_video` | `video` | `video` | 双目编码视频 |
| `0x04` | `robot_audio` | `audio` | `audio_opus` | 启用音频上行时的 Opus 音频 |

视频和音频参数不再调用已删除的 `PublishVideoConfig` / `PublishAudioConfig`。节点通过通用 `PublishStreamConfig` 保存配置：

- `prefix=0x01, schema_version=1`：payload 为 UTF-8 JSON；`codec` 为 `H264` 或 `H265`，SPS/PPS/VPS 使用标准 Base64 编码，可选视场角放在 `fov` 对象中。
- `prefix=0x04, schema_version=1`：payload 为 UTF-8 JSON，包含 Opus 编码和采样参数。

接收端应先读取发布者 Manifest，再按 `(publisher_session_id, prefix)` 拉取并解析 StreamConfig；新后端不会通过 EventStream 主动推送配置。

## 运行

启动 ROS core（如未启动）：

```bash
roscore
```

启动节点：

```bash
cd /home/nvidia/ROS/tele_ws/src/teleop_robot_bridge
./bringup.sh
```

## 验证

检查节点与话题：

```bash
rostopic list
```

若启用眼图，验证话题与频率：

```bash
rostopic hz /data/left_eye_image
rostopic hz /data/right_eye_image
```

## 说明

- UDP 视频支持 H.264/H.265，可选 FEC。
- UDP 头部 KeyframeFlag 依据 IDR 判定。
- gRPC 心跳周期按最新后端设置为 3 秒（后端在线超时 8 秒）。
- `sync_back_new/` 仅作为本轮后端参考，已由仓库根 `.gitignore` 忽略。
- 修改 params.yaml 后需重启节点。

## 常见问题

- 视频无法启动：检查相机设备路径与权限。
- Register 被拒绝：确认后台存在对应 DeviceId，且设备登记不是 Disabled/Expired。首次有效 Register 会由后端把 Pending 登记激活。
- Manifest 被拒绝：确认后端已启用 `video` / `audio` 流大类及 `video` / `audio_opus` 协议模板。
- 延迟逐渐升高：检查网络质量与接收端缓冲策略。
- 退出缓慢：确认设备未被其他进程占用。
