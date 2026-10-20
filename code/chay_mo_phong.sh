#!/bin/bash
# ==============================================================================
# Script tự động bật mô phỏng Gazebo Harmonic + SLAM + RViz2
# Hỗ trợ chọn map: Mê cung (Maze), Văn phòng (Office), Kho hàng (Warehouse), Sân thử nghiệm (Obstacle Course)
# ==============================================================================

# Xử lý tham số truyền vào từ dòng lệnh (ví dụ: ./chay_mo_phong.sh office)
if [ -n "$1" ]; then
    arg_clean=$(echo "$1" | tr '[:upper:]' '[:lower:]' | tr -d '[:space:]')
    case "$arg_clean" in
        1|maze|me_cung) SELECTED_WORLD="maze" ;;
        2|office|van_phong) SELECTED_WORLD="office" ;;
        3|warehouse|kho|kho_hang) SELECTED_WORLD="warehouse" ;;
        4|obstacle|obstacle_course|vat_can) SELECTED_WORLD="obstacle_course" ;;
        *) SELECTED_WORLD="$1" ;;
    esac
else
    echo "======================================================================"
    echo "       CHỌN BẢN ĐỒ MÔ PHỎNG THU NHỎ & ĐẬM ĐẶC NGÃ RẼ [CẤP ĐỘ 1 -> 4]"
    echo "======================================================================"
    echo "   [1] CẤP ĐỘ 1: Mê cung thu nhỏ 6m x 6m (Maze)"
    echo "       - 4 phân vùng, 8 góc cua 90 độ, ngã 3 ngã 4 đan xen, hẻm hẹp 0.8m."
    echo ""
    echo "   [2] CẤP ĐỘ 2: Văn phòng 5 phòng thu hẹp 7m x 7m (Office)"
    echo "       - Cửa hẹp 0.7m, hành lang chữ thập, đồ nội thất sát tường, nhiều ngã rẽ."
    echo ""
    echo "   [3] CẤP ĐỘ 3: Kho hàng hẹp & 3 Ngõ cụt 8m x 6.5m (Warehouse)"
    echo "       - 4 dãy kệ hàng hẹp 0.75m, 3 ngõ cụt bít kín, cột trụ & cua chữ U."
    echo ""
    echo "   [4] CẤP ĐỘ 4: Mê cung cực hạn, Cửa 58cm & Bẫy chữ U 7m x 7m (Obstacle Course)"
    echo "       - Cửa thắt nút 58cm, đường zíc zắc chữ S, bẫy chữ U sâu, vòng lặp kép."
    echo "======================================================================"
    read -t 20 -p "Lựa chọn của bạn [1-4] (tự động chọn [1] sau 20 giây): " choice
    echo ""
    choice_clean=$(echo "$choice" | tr '[:upper:]' '[:lower:]' | tr -d '[:space:]')
    case "$choice_clean" in
        2|office|van_phong) SELECTED_WORLD="office" ;;
        3|warehouse|kho|kho_hang) SELECTED_WORLD="warehouse" ;;
        4|obstacle|obstacle_course|vat_can) SELECTED_WORLD="obstacle_course" ;;
        *) SELECTED_WORLD="maze" ;;
    esac
fi

echo "======================================================"
echo "    DANG DON DEP TRIET DE CAC TIEN TRINH CU..."
echo "======================================================"
killall -9 gz-sim-server gz ros2 rviz2 async_slam_toolbox_node parameter_bridge 2>/dev/null
pkill -9 -f "gz sim" 2>/dev/null
pkill -9 -f "ruby.*gz" 2>/dev/null
pkill -9 -f "robot_sim_slam.launch.py" 2>/dev/null
pkill -9 -f "target_navigator.py" 2>/dev/null
pkill -9 -f "auto_explorer.py" 2>/dev/null
pkill -9 -f "scan_frame_fixer.py" 2>/dev/null
sleep 1

# Bắt sự kiện thoát (Ctrl+C hoặc đóng terminal) để dọn sạch tiến trình con
cleanup() {
    echo ""
    echo ">>> Dang dung he thong va don dep tien trinh mo phong..."
    killall -9 gz-sim-server gz ros2 rviz2 async_slam_toolbox_node parameter_bridge 2>/dev/null
    pkill -9 -f "gz sim" 2>/dev/null
    pkill -9 -f "ruby.*gz" 2>/dev/null
    pkill -9 -f "robot_sim_slam.launch.py" 2>/dev/null
    pkill -9 -f "target_navigator.py" 2>/dev/null
    pkill -9 -f "auto_explorer.py" 2>/dev/null
    pkill -9 -f "scan_frame_fixer.py" 2>/dev/null
    exit 0
}
trap cleanup SIGHUP SIGINT SIGTERM EXIT

echo "======================================================"
echo "    DANG KHOI DONG MO PHONG ROBOT SLAM VỚI MAP: $SELECTED_WORLD"
echo "======================================================"
source /opt/ros/jazzy/setup.bash
cd /home/nvdong/projects/du-an-moi/code_updated/ros2_ws
source install/setup.bash

export SIM_WORLD="$SELECTED_WORLD"
ros2 launch robot_bringup robot_sim_slam.launch.py world:="$SELECTED_WORLD"
