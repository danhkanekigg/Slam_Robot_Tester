#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Script gửi tọa độ đích (X, Y, Góc) tới robot:
Cách dùng:
  1. Chạy tương tác:
     python3 gui_toa_do.py
  2. Truyền tham số trực tiếp:
     python3 gui_toa_do.py <X> <Y> [Góc độ]
     Ví dụ: python3 gui_toa_do.py 1.5 2.0 90
"""

import sys
import math
import time
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String

def send_target(x, y, yaw_deg):
    rclpy.init()
    node = Node('cli_goal_sender')
    pub_target = node.create_publisher(PoseStamped, '/target_pose', 10)
    pub_mode = node.create_publisher(String, '/exploration_mode', 10)

    # Chờ kết nối publisher
    time.sleep(0.3)

    # Ngắt Auto cũ trước
    mode_msg = String()
    mode_msg.data = "MANUAL"
    pub_mode.publish(mode_msg)
    time.sleep(0.1)

    yaw_rad = math.radians(yaw_deg)
    msg = PoseStamped()
    msg.header.stamp = node.get_clock().now().to_msg()
    msg.header.frame_id = 'map'
    msg.pose.position.x = float(x)
    msg.pose.position.y = float(y)
    msg.pose.position.z = 0.0

    msg.pose.orientation.z = math.sin(yaw_rad / 2.0)
    msg.pose.orientation.w = math.cos(yaw_rad / 2.0)

    # Gửi 3 lần để đảm bảo nhận
    for _ in range(3):
        pub_target.publish(msg)
        time.sleep(0.05)

    print(f"\n\033[1;32m[THÀNH CÔNG] ĐÃ GỬI TỌA ĐỘ ĐÍCH TỚI ROBOT:\033[0m")
    print(f"  -> X   = {x:.3f} m")
    print(f"  -> Y   = {y:.3f} m")
    print(f"  -> Góc = {yaw_deg:.1f}° ({yaw_rad:.3f} rad)")
    print(f"\033[1;36mRobot đang bắt đầu vạch đường A* và né vật cản để đến đích!\033[0m\n")

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    if len(sys.argv) >= 3:
        x_val = float(sys.argv[1])
        y_val = float(sys.argv[2])
        deg_val = float(sys.argv[3]) if len(sys.argv) >= 4 else 0.0
        send_target(x_val, y_val, deg_val)
    else:
        print("\033[1;36m=== BỘ GỬI TỌA ĐỘ ĐÍCH POINT-TO-POINT ===\033[0m")
        try:
            x_in = float(input("Nhập tọa độ X (mét, ví dụ 1.2): "))
            y_in = float(input("Nhập tọa độ Y (mét, ví dụ -0.8): "))
            deg_in = input("Nhập góc quay mong muốn (độ, mặc định 0): ").strip()
            deg_val = float(deg_in) if deg_in else 0.0
            send_target(x_in, y_in, deg_val)
        except ValueError:
            print("\033[1;31mLỗi: Tọa độ nhập vào phải là số thực!\033[0m")
        except KeyboardInterrupt:
            print("\nĐã hủy.")
