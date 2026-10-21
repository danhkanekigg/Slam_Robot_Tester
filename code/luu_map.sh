#!/bin/bash
# Script lưu bản đồ SLAM thành file ảnh
MAP_NAME=${1:-"ban_do_robot"}
echo "======================================================"
echo "    DANG LUU BAN DO: $MAP_NAME"
echo "======================================================"
source /opt/ros/jazzy/setup.bash
cd /home/nvdong/projects/du-an-moi
ros2 run nav2_map_server map_saver_cli -f "$MAP_NAME"
echo ""
echo "Da luu thanh cong file $MAP_NAME.yaml va $MAP_NAME.pgm vao /home/nvdong/projects/du-an-moi/"
echo "======================================================"
