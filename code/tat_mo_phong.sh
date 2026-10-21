#!/bin/bash
# ==============================================================================
# Script tắt và dọn dẹp sạch toàn bộ tiến trình mô phỏng Gazebo & ROS 2
# ==============================================================================
echo ">>> Đang tắt toàn bộ tiến trình mô phỏng Gazebo, RViz và ROS 2..."
killall -9 gz-sim-server gz ros2 rviz2 async_slam_toolbox_node parameter_bridge 2>/dev/null
pkill -9 -f "gz sim" 2>/dev/null
pkill -9 -f "ruby.*gz" 2>/dev/null
pkill -9 -f "robot_sim_slam.launch.py" 2>/dev/null
pkill -9 -f "target_navigator.py" 2>/dev/null
pkill -9 -f "auto_explorer.py" 2>/dev/null
pkill -9 -f "scan_frame_fixer.py" 2>/dev/null
sleep 0.5
echo ">>> ĐÃ TẮT VÀ DỌN SẠCH TOÀN BỘ HỆ THỐNG!"
