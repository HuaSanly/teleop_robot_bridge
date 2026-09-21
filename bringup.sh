#!/usr/bin/env bash

# NvBufSurfaceMapEglImage must use NVIDIA's headless EGL display. An inherited
# SSH/X11 DISPLAY can point at an unavailable or unauthorized X server and make
# CUDA video post-processing fail during startup.
unset DISPLAY
unset WAYLAND_DISPLAY

cd ~/ROS/teleop_ws
ros2 launch teleop_robot_bridge bringup.launch.py
