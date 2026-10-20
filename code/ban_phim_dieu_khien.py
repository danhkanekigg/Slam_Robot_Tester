#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Bộ điều khiển xe thông minh đa chế độ (Manual & Auto Exploration):
- MANUAL MODE: Tự lái bằng bàn phím (W-A-S-D / Phím mũi tên), phản hồi 0ms.
- AUTO MODE: Xe tự động tìm vùng chưa quét trên bản đồ và né mọi vật cản qua Nav2.
- HUMAN-OVERRIDE: Khi đang Auto, bấm bất kỳ phím nào sẽ ngắt Auto và trả quyền lái ngay lập tức.
- PHÍM TẮT:
    + [A] hoặc [X] : BẬT CHẾ ĐỘ AUTO (Tự hành khám phá & né tường)
    + [M]          : CHUYỂN MANUAL (Thủ công: Giữ để chạy / Duy trì)
    + [1] [2] [3]  : Chậm (0.15), Vừa (0.25), Nhanh (0.45) m/s
    + [Space] / [K]: PHANH KHẨN CẤP
"""

import sys
import termios
import tty
import os
import fcntl
import time
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import String

from sensor_msgs.msg import LaserScan
import numpy as np

BANNER = """
\033[1;36m================================================================
       BỘ ĐIỀU KHIỂN ĐA CHẾ ĐỘ: MANUAL (TỰ LÁI) & AUTO (TỰ HÀNH)
================================================================\033[0m
  \033[1;35mCHẾ ĐỘ TỰ HÀNH (AUTO EXPLORATION & OBSTACLE AVOIDANCE):\033[0m
     \033[1;32m[X]\033[0m          : \033[1;32mBẬT AUTO\033[0m (Xe tự tìm chỗ chưa quét & tự né mọi vật cản)
     \033[1;33m[Phím bất kỳ]\033[0m: Giành lại quyền lái ngay tức thì (Safety Override)

  \033[1;35mCHẾ ĐỘ TỰ LÁI THỦ CÔNG (MANUAL) + ANTI-SLIP SAFETY GUARD:\033[0m
     \033[1;33m[W]\033[0m hoặc \033[1;33m[▲]\033[0m : Tiến                       \033[1;32m[1]\033[0m : Chậm (0.15 m/s)
  \033[1;33m[A]\033[0m hoặc \033[1;33m[◄]\033[0m  \033[1;33m[D]\033[0m hoặc \033[1;33m[►]\033[0m : Quay trái / phải     \033[1;32m[2]\033[0m : Chuẩn SLAM (0.25 m/s)
     \033[1;33m[S]\033[0m hoặc \033[1;33m[▼]\033[0m : Lùi                        \033[1;32m[3]\033[0m : Nhanh (0.45 m/s)
     \033[1;35m[M]\033[0m     : Đổi [Giữ để chạy] / [Duy trì]    \033[1;32m[+] [-]\033[0m: Tăng/giảm tốc độ

  \033[1;32m[BẢO VỆ SLAM]\033[0m : Tự động khóa lệnh TIẾN khi cách tường < 22cm (chống trượt bánh phá map)
  \033[1;31m[Space] / [C]  : HỦY LỘ TRÌNH ĐIỀU HƯỚNG & PHANH DỪNG KHẨN CẤP\033[0m
  \033[1;31m[Ctrl+C]       : Thoát chương trình\033[0m
\033[1;36m================================================================\033[0m
"""

class MultiModeTeleopNode(Node):
    def __init__(self):
        super().__init__('multi_mode_teleop')
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.mode_pub = self.create_publisher(String, '/exploration_mode', 10)
        self.cancel_pub = self.create_publisher(String, '/cancel_goal', 10)

        # Lắng nghe trạng thái từ Auto Explorer & Target Navigator
        self.explorer_status = "Chờ lệnh..."
        self.nav_status = ""
        self.create_subscription(
            String, '/exploration_status', self.status_callback, 10
        )
        self.create_subscription(
            String, '/navigation_status', self.nav_status_callback, 10
        )

        # Lắng nghe Lidar để khóa an toàn chống đâm tường làm trượt bánh
        self.front_obstacle_dist = 10.0
        self.collision_blocked = False
        self.create_subscription(
            LaserScan, '/scan_relay', self.scan_callback, 10
        )

        # Trạng thái chế độ: 'MANUAL' hoặc 'AUTO'
        self.mode = "MANUAL"
        self.hold_mode = True # Trong Manual: True = giữ để đi, False = duy trì

        # Cấu hình vận tốc mặc định
        self.linear_speed = 0.25   # m/s
        self.angular_speed = 0.70  # rad/s

        # Trạng thái chuyển động hiện tại
        self.current_vx = 0.0
        self.current_wz = 0.0
        self.last_key_time = 0.0
        self.key_press_count = 0
        self.status_msg = "DỪNG"

        self.timeout_repeat = 0.16
        self.timeout_first = 0.38

        # Timer phát lệnh liên tục 50Hz (20ms)
        self.timer = self.create_timer(0.02, self.timer_callback)

        # Đảm bảo đồng bộ hóa chế độ MANUAL và phanh dừng xe ngay khi mở bàn phím
        self.set_system_mode("MANUAL")
        self.stop_motion()

    def scan_callback(self, msg: LaserScan):
        ranges = np.array(msg.ranges)
        n = len(ranges)
        if n == 0:
            return
        angles = np.linspace(float(msg.angle_min), float(msg.angle_max), n, endpoint=False)
        valid = np.isfinite(ranges) & (ranges > 0.05) & (ranges < 4.0)
        if np.any(valid):
            x = ranges[valid] * np.cos(angles[valid])
            y = ranges[valid] * np.sin(angles[valid])

            # KÍCH THƯỚC HÌNH CHỮ NHẬT 3D CHUẨN XÁC:
            # Rộng: 0.170m (nửa rộng = 0.085m), Dài: 0.210m, Mũi xe cách LiDAR: +0.105m.
            # 1. Hành lang mũi xe đi thẳng (bề rộng 0.19m = 8.5cm + 1.0cm đệm mỗi bên)
            front_mask = (x > 0.05) & (x < 1.0) & (np.abs(y) <= 0.095)
            if np.any(front_mask):
                self.front_obstacle_dist = float(np.min(x[front_mask]))
            else:
                self.front_obstacle_dist = 10.0

            # 2. Kiểm tra khoảng hở hông (tránh cạ góc khi xoay sát mép cửa/tường)
            left_flank = (y > 0.085) & (y < 0.115) & (x > -0.09) & (x < 0.13)
            right_flank = (y < -0.085) & (y > -0.115) & (x > -0.09) & (x < 0.13)
            self.left_flank_tight = bool(np.any(left_flank))
            self.right_flank_tight = bool(np.any(right_flank))
        else:
            self.front_obstacle_dist = 10.0
            self.left_flank_tight = False
            self.right_flank_tight = False

    def status_callback(self, msg: String):
        self.explorer_status = msg.data

    def nav_status_callback(self, msg: String):
        self.nav_status = msg.data

    def set_system_mode(self, new_mode: str):
        self.mode = new_mode
        mode_msg = String()
        mode_msg.data = new_mode
        self.mode_pub.publish(mode_msg)
        if new_mode == "MANUAL":
            self.stop_motion()

    def publish_twist(self, vx, wz):
        msg = Twist()
        msg.linear.x = float(vx)
        msg.angular.z = float(wz)
        self.cmd_pub.publish(msg)

    def cancel_navigation_goal(self):
        """Hủy lộ trình điều hướng ngay lập tức và dừng xe"""
        msg = String()
        msg.data = "CANCEL"
        self.cancel_pub.publish(msg)
        self.set_system_mode("MANUAL")
        self.stop_motion()
        self.status_msg = "HỦY LỘ TRÌNH"

    def set_motion(self, vx, wz, status):
        # Nếu đang ở AUTO mà người dùng bấm phím điều khiển -> Tự động chuyển ngay về MANUAL
        if self.mode == "AUTO":
            self.set_system_mode("MANUAL")

        # Nếu đang có lộ trình điều hướng đích -> Hủy ngay lập tức để giành quyền lái
        if self.nav_status and any(self.nav_status.startswith(s) for s in ["NAVIGATING", "PLANNING", "ALIGNING"]):
            cancel_msg = String()
            cancel_msg.data = "CANCEL"
            self.cancel_pub.publish(cancel_msg)

        # KHÓA AN TOÀN CHỐNG ĐÂM TƯỜNG & CHỐNG CẠ GÓC KHI LỌT KHE HẸP:
        # Mũi xe cách LiDAR 0.105m. Ngưỡng an toàn mũi xe = 0.145m (cách mũi xe 4.0cm).
        if vx > 0.0 and self.front_obstacle_dist < 0.145:
            vx = 0.0
            status = f"CHẶN TIẾN (Trước mặt {self.front_obstacle_dist:.2f}m)"
            self.collision_blocked = True
        else:
            self.collision_blocked = False

        # Chống cạ góc hình chữ nhật: nếu hông sát tường/mép cửa (< 3cm), cấm bẻ lái vào tường
        if wz > 0.0 and getattr(self, 'left_flank_tight', False):
            wz = 0.0
            status = "KHÓA QUAY TRÁI (Sát mép)"
        elif wz < 0.0 and getattr(self, 'right_flank_tight', False):
            wz = 0.0
            status = "KHÓA QUAY PHẢI (Sát mép)"

        now = time.time()
        if now - self.last_key_time < 0.10:
            self.key_press_count += 1
        else:
            self.key_press_count = 1

        self.current_vx = vx
        self.current_wz = wz
        self.last_key_time = now
        self.status_msg = status
        self.publish_twist(self.current_vx, self.current_wz)

    def stop_motion(self):
        self.current_vx = 0.0
        self.current_wz = 0.0
        self.status_msg = "DỪNG"
        self.key_press_count = 0
        self.publish_twist(0.0, 0.0)

    def timer_callback(self):
        # Ở chế độ AUTO: xe do AutoExplorer điều khiển, node này không can thiệp cmd_vel
        if self.mode == "AUTO":
            return

        # Kiểm tra an toàn liên tục: Nếu xe đang tiến mà khoảng cách giảm xuống < 0.20m -> Cắt ga ngay
        if self.current_vx > 0.0 and self.front_obstacle_dist < 0.20:
            self.stop_motion()
            self.status_msg = "CHẶN TIẾN (ĐỤNG TƯỜNG)"
            return

        # Ở chế độ MANUAL: kiểm tra timeout thả phím
        if self.hold_mode:
            timeout = self.timeout_repeat if self.key_press_count > 1 else self.timeout_first
            if time.time() - self.last_key_time > timeout:
                if self.current_vx != 0.0 or self.current_wz != 0.0:
                    self.stop_motion()
                    return

        if self.current_vx != 0.0 or self.current_wz != 0.0:
            self.publish_twist(self.current_vx, self.current_wz)


def run():
    print(BANNER)
    rclpy.init()
    node = MultiModeTeleopNode()

    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)
    old_flags = fcntl.fcntl(fd, fcntl.F_GETFL)

    try:
        tty.setraw(fd)
        fcntl.fcntl(fd, fcntl.F_SETFL, old_flags | os.O_NONBLOCK)

        last_display_time = 0.0

        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.001)

            try:
                data = os.read(fd, 1024)
            except (BlockingIOError, InterruptedError):
                data = b""

            if data:
                # Xử lý phím mũi tên ANSI: \x1b[A (Lên), \x1b[B (Xuống), \x1b[C (Phải), \x1b[D (Trái)
                if b'\x1b[A' in data: # Mũi tên Lên
                    node.set_motion(node.linear_speed, 0.0, "TIẾN ▲")
                elif b'\x1b[B' in data: # Mũi tên Xuống
                    node.set_motion(-node.linear_speed, 0.0, "LÙI ▼")
                elif b'\x1b[D' in data: # Mũi tên Trái
                    node.set_motion(0.0, node.angular_speed, "QUAY TRÁI ◄")
                elif b'\x1b[C' in data: # Mũi tên Phải
                    node.set_motion(0.0, -node.angular_speed, "QUAY PHẢI ►")
                else:
                    ch = chr(data[-1]).lower()

                    # BẬT CHẾ ĐỘ AUTO (Phím X hoặc khi bấm chữ a mà không phải quay trái)
                    if ch == 'x':
                        node.set_system_mode("AUTO")
                    elif ch in ['w', 'i']:
                        node.set_motion(node.linear_speed, 0.0, "TIẾN ▲")
                    elif ch in ['s', ',']:
                        node.set_motion(-node.linear_speed, 0.0, "LÙI ▼")
                    elif ch in ['a', 'j']:
                        # Bấm 'a' khi đang lái là quay trái, nếu đang dừng mà bấm X là bật Auto
                        node.set_motion(0.0, node.angular_speed, "QUAY TRÁI ◄")
                    elif ch in ['d', 'l']:
                        node.set_motion(0.0, -node.angular_speed, "QUAY PHẢI ►")
                    elif ch in ['u']:
                        node.set_motion(node.linear_speed, node.angular_speed * 0.7, "TIẾN TRÁI ◤")
                    elif ch in ['o']:
                        node.set_motion(node.linear_speed, -node.angular_speed * 0.7, "TIẾN PHẢI ◥")
                    elif ch in ['k', ' ', 'c']:
                        # Phanh khẩn cấp & HỦY LỘ TRÌNH ĐIỀU HƯỚNG
                        node.cancel_navigation_goal()
                    elif ch == 'm':
                        if node.mode == "AUTO":
                            node.set_system_mode("MANUAL")
                        else:
                            node.hold_mode = not node.hold_mode
                        node.stop_motion()
                    elif ch == '1':
                        node.linear_speed = 0.15
                        node.angular_speed = 0.50
                    elif ch == '2':
                        node.linear_speed = 0.25
                        node.angular_speed = 0.70
                    elif ch == '3':
                        node.linear_speed = 0.45
                        node.angular_speed = 1.20
                    elif ch in ['+', 'q']:
                        node.linear_speed = min(0.60, round(node.linear_speed + 0.05, 2))
                        node.angular_speed = min(2.00, round(node.angular_speed + 0.10, 2))
                    elif ch in ['-', 'z']:
                        node.linear_speed = max(0.05, round(node.linear_speed - 0.05, 2))
                        node.angular_speed = max(0.20, round(node.angular_speed - 0.10, 2))
                    elif ch == '\x03': # Ctrl+C
                        break

            # Cập nhật hiển thị giao diện mỗi 0.08s
            now = time.time()
            if now - last_display_time > 0.08:
                last_display_time = now
                if node.nav_status and (node.nav_status.startswith("NAVIGATING") or node.nav_status.startswith("PLANNING") or node.nav_status.startswith("ALIGNING")):
                    mode_display = "\033[1;45;37m POINT-TO-POINT \033[0m"
                    sys.stdout.write(
                        f"\r\033[KChế độ: {mode_display} | Trạng thái: \033[1;35m{node.nav_status}\033[0m | \033[1;33m[Phím bất kỳ để giành lái]\033[0m"
                    )
                elif node.mode == "AUTO":
                    mode_display = "\033[1;42;37m AUTO TỰ HÀNH \033[0m"
                    detail = f"\033[1;32m{node.explorer_status}\033[0m"
                    sys.stdout.write(
                        f"\r\033[KChế độ: {mode_display} | Trạng thái: {detail} | \033[1;33m[Bấm phím bất kỳ để giành quyền lái]\033[0m"
                    )
                else:
                    mode_type = "[Giữ để đi]" if node.hold_mode else "[Duy trì]"
                    mode_display = f"\033[1;44;37m MANUAL THỦ CÔNG \033[0m \033[1;34m{mode_type}\033[0m"
                    st_color = "\033[1;32m" if node.status_msg != "DỪNG" else "\033[1;31m"
                    sys.stdout.write(
                        f"\r\033[KChế độ: {mode_display} | {st_color}{node.status_msg:<12}\033[0m | "
                        f"V: \033[1;33m{node.linear_speed:.2f}\033[0m m/s | "
                        f"W: \033[1;33m{node.angular_speed:.2f}\033[0m rad/s | "
                        f"\033[1;35m[Bấm X để bật Auto | RViz chọn Goal]\033[0m"
                    )
                sys.stdout.flush()

            time.sleep(0.002)

    except KeyboardInterrupt:
        pass
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
        fcntl.fcntl(fd, fcntl.F_SETFL, old_flags)
        node.set_system_mode("MANUAL")
        node.publish_twist(0.0, 0.0)
        print("\n\n\033[1;32mĐã phanh dừng xe an toàn và thoát.\033[0m\n")
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    run()
