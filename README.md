# teleop_robot_bridge
用于机器人端的遥操作视频与姿态桥接节点（ROS 2）。包含 gRPC 信令、V4L2 采集与编码、低延迟 UDP 传输，以及 VR 姿态 UDP 接收并发布 ROS 话题。

## 功能概览
* gRPC 信令：注册、心跳、配对事件
* 视频链路：V4L2 采集 →（Jetson 可选）解码/转换 → H.264 编码
* UDP 传输：握手/心跳、节流发送、队列丢弃、FEC 冗余
* 姿态接收：解析 Pose UDP 报文并发布 Pose/Euler/手柄输入/捏合
## 运行依赖
* ROS 2（ament_cmake、rclcpp 等）
* gRPC + Protobuf
* OpenSSL
* V4L2
* Jetson（可选）：Jetson Multimedia API + CUDA
## 构建与运行（简要）
```shell
colcon build --packages-select teleop_robot_bridge
ros2 launch teleop_robot_bridge bringup.launch.py
```
## 配置
参数位于 config/params.yaml，主要包括：

* 信令服务：grpc.server_grpc_ip/grpc.server_grpc_port/grpc.token/grpc.device_id
* 视频：分辨率/帧率、编码器参数、录像开关；`video.scale.enabled=true` 时在转换后等比缩放到 `video.scale.width/height`（默认 `3200x1200`）
* UDP：udp.server_udp_ip/udp.server_udp_port、FEC、节流与队列策略
* 姿态接收：pose_udp.* 相关开关与话题参数
## 目录结构（核心）
* main_node.cpp：主节点入口
* signaling_client.cpp：gRPC 信令客户端
* src/video/：视频采集/转换/编码/录制
* src/udp/：UDP 传输与姿态解析
* config/params.yaml：默认参数
## 备注
* Jetson 平台未安装 Multimedia API 时，转换/编码模块会以 stub 模式构建

git switch main
git switch feature/video-codec-overhaul
git push -u origin feature/video-codec-overhaul
