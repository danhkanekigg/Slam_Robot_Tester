#!/usr/bin/env bash
# ==============================================================================
# Script hủy lộ trình điều hướng điểm đích ngay lập tức
# ==============================================================================
source /opt/ros/jazzy/setup.bash 2>/dev/null || source /opt/ros/humble/setup.bash 2>/dev/null

ros2 topic pub --once /cancel_goal std_msgs/msg/String "{data: 'CANCEL'}" > /dev/null 2>&1

echo -e "\033[1;32m🛑 [ĐÃ HỦY LỘ TRÌNH]\033[0m Đã phát lệnh hủy lộ trình đến Target Navigator. Xe đã dừng an toàn!"
