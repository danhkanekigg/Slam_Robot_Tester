#!/bin/bash
# ==============================================================================
# Script Điều hướng Robot đến vị trí chỉ định (Point-to-Point Navigation)
# ==============================================================================
source /opt/ros/jazzy/setup.bash
cd /home/nvdong/projects/du-an-moi/code_updated/ros2_ws
source install/setup.bash
cd /home/nvdong/projects/du-an-moi

if [ $# -ge 2 ]; then
    # Nếu người dùng truyền toạ độ trực tiếp: ./dieu_huong_diem.sh <X> <Y> [Góc]
    python3 gui_toa_do.py "$@"
else
    echo "======================================================================"
    echo "    ĐIỀU HƯỚNG ĐẾN ĐIỂM CHỈ ĐỊNH (POINT-TO-POINT NAVIGATION)"
    echo "======================================================================"
    echo " Lưu ý quan trọng:"
    echo "   - Khi chạy ./chay_mo_phong.sh, Node Target Navigator đã được BẬT SẴN."
    echo "   - Bạn có thể dùng chuột bấm trực tiếp công cụ '2D Goal Pose' trên RViz2."
    echo "======================================================================"
    echo " Chọn phương thức:"
    echo "   [1] Gửi tọa độ đích (X, Y, Góc) bằng tay đến robot đang chạy"
    echo "   [2] Khởi chạy Node Target Navigator độc lập (nếu chưa bật mô phỏng đầy đủ)"
    echo "======================================================================"
    read -p "Lựa chọn của bạn (1 hoặc 2, mặc định 1): " choice
    choice=${choice:-1}

    if [ "$choice" == "1" ]; then
        python3 gui_toa_do.py
    else
        # Kiểm tra xem có node target_navigator nào đang chạy không để tránh chạy đè 2 node
        if pgrep -f "target_navigator.py" > /dev/null; then
            echo "[CẢNH BÁO] Node Target Navigator đang chạy sẵn trong hệ thống!"
            echo "           Không cần bật lại để tránh xung đột lệnh /cmd_vel."
        else
            echo "Đang khởi động Node Target Navigator độc lập..."
            python3 /home/nvdong/projects/du-an-moi/code_updated/ros2_ws/src/robot_bringup/scripts/target_navigator.py --ros-args -p use_sim_time:=true
        fi
    fi
fi
