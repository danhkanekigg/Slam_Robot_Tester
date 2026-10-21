#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Target Navigator Node (Point-to-Point Navigation)
--------------------------------------------------
Tính năng: Điều hướng robot đến vị trí chỉ định trên bản đồ đã vẽ (song song với SLAM).
1. Lắng nghe điểm đích:
   - Từ nút "2D Goal Pose" (Nav2 Goal) trên RViz2 (topic: /goal_pose dạng PoseStamped).
   - Hoặc từ topic /target_pose / tham số tọa độ (x, y, yaw).
2. Giải quyết xung đột /cmd_vel:
   - Ngay khi nhận điểm đích mới, phát lệnh "MANUAL" tới /exploration_mode để ngắt
     ngay lập tức node auto_explorer cũ, tránh xung đột 2 node cùng gửi vận tốc.
3. Né vật cản thông minh:
   - Dùng A* vạch đường đi tối ưu dựa trên /map của SLAM, ưu tiên đi tim đường (Corridor Centering).
   - Dò quét LiDAR thời gian thực (/scan_relay) ở 20Hz: tự động giảm tốc khi gần vật cản
     và phanh khẩn cấp tuyệt đối nếu có vật cản đột xuất (< 18cm), không bao giờ ủi vào tường.
   - Nếu đường đi bị bít hoàn toàn không thể đến đích: tự động dừng an toàn và thông báo lỗi.
4. Độ chính xác cao:
   - Dừng chính xác tại đích: sai số vị trí < 0.08m (đạt yêu cầu 0.05m - 0.1m).
   - Xoay hướng xe đúng góc chỉ định: sai số góc < 0.08 rad (đạt yêu cầu < 0.1 rad).
"""

import math
import time
import heapq
from collections import deque
import numpy as np
from scipy.ndimage import distance_transform_edt

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy

from nav_msgs.msg import OccupancyGrid, Path
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist, PoseStamped, PointStamped
from visualization_msgs.msg import Marker
from std_msgs.msg import String
import tf2_ros


class TargetNavigatorNode(Node):
    def __init__(self):
        super().__init__('target_navigator_node')

        # Khai báo tham số cấu hình
        if not self.has_parameter('use_sim_time'):
            self.declare_parameter('use_sim_time', True)
        if not self.has_parameter('max_linear_speed'):
            self.declare_parameter('max_linear_speed', 0.25)
        if not self.has_parameter('max_angular_speed'):
            self.declare_parameter('max_angular_speed', 0.40) # Giới hạn 0.40 rad/s để bảo vệ bản đồ SLAM không bị méo lệch
        if not self.has_parameter('xy_tolerance'):
            self.declare_parameter('xy_tolerance', 0.08)  # Sai số vị trí: 8cm
        if not self.has_parameter('yaw_tolerance'):
            self.declare_parameter('yaw_tolerance', 0.08) # Sai số góc: ~4.5 độ (< 0.1 rad)
        if not self.has_parameter('stop_distance'):
            self.declare_parameter('stop_distance', 0.22) # Khoảng cách phanh an toàn trước vật cản (22cm = mũi xe cách tường 9.5cm, đủ bán kính quay 15.7cm không chạm góc)
        if not self.has_parameter('lookahead_dist'):
            self.declare_parameter('lookahead_dist', 0.35) # Bán kính ngắm Pure Pursuit mượt mà
 
        self.max_v = float(self.get_parameter('max_linear_speed').value)
        self.max_w = float(self.get_parameter('max_angular_speed').value)
        self.xy_tol = float(self.get_parameter('xy_tolerance').value)
        self.yaw_tol = float(self.get_parameter('yaw_tolerance').value)
        self.stop_dist = float(self.get_parameter('stop_distance').value)
        self.lookahead_dist = float(self.get_parameter('lookahead_dist').value)

        # Trạng thái điều hướng: 'IDLE', 'PLANNING', 'NAVIGATING', 'ALIGNING'
        self.state = 'IDLE'

        # Điểm đích mong muốn: (target_x, target_y, target_yaw)
        self.target_goal = None

        # Quỹ đạo toàn cục A* hiện tại [(wx, wy), ...]
        self.current_path = []
        self.path_index = 0

        # Dữ liệu bản đồ & cảm biến
        self.map_data = None
        self.accumulated_grid = None  # Bộ nhớ bản đồ tích lũy: Ghi nhớ vĩnh viễn các vùng map đã vẽ
        self.latest_scan = None

        # Bộ lọc gia tốc mượt mà (Smoothing): chống trượt bánh xe, loại bỏ hoàn toàn giật rung
        self.cmd_v = 0.0
        self.cmd_w = 0.0
        self.accel_v_limit = 0.025 # ~0.5 m/s^2 tăng giảm tốc siêu êm
        self.accel_w_limit = 0.035 # ~0.7 rad/s^2 xoay êm ái, chống trượt bánh và bảo vệ SLAM

        # Đếm thời gian kẹt nếu đường bị bít
        self.blocked_start_time = None

        # Trạng thái phục hồi thoát kẹt (Recovery State): None, 'REVERSING', 'ROTATING'
        self.recovery_mode = None
        self.recovery_end_time = 0.0
        self.recovery_rot_dir = 1.0  # +1.0: quay trái, -1.0: quay phải

        # Phát hiện kẹt cơ học tại chỗ (Stuck Detection)
        self.stuck_check_time = 0.0
        self.stuck_check_pos = None
        self.stuck_counter = 0

        # Thời điểm node tự phát "MANUAL" (để không tự ngắt chính mình)
        self.last_self_override_time = 0.0

        # TF Listener tra cứu tọa độ robot trên bản đồ (/map -> base_footprint)
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # QoS cho /map từ SLAM Toolbox (transient local)
        qos_map = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )
        self.map_sub = self.create_subscription(
            OccupancyGrid, '/map', self.map_callback, qos_map
        )

        # QoS cho LaserScan
        qos_scan = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=5
        )
        self.scan_sub = self.create_subscription(
            LaserScan, '/scan_relay', self.scan_callback, qos_scan
        )

        # Lắng nghe điểm đích từ RViz2 (Nút "2D Goal Pose" / "Nav2 Goal" - Click & Kéo)
        self.goal_sub = self.create_subscription(
            PoseStamped, '/goal_pose', self.goal_pose_callback, 10
        )

        # Lắng nghe click đơn từ RViz2 (Nút "Publish Point" - Click 1 lần bất kỳ trên map!)
        self.point_sub = self.create_subscription(
            PointStamped, '/clicked_point', self.clicked_point_callback, 10
        )

        # Lắng nghe điểm đích bổ sung (topic /target_pose)
        self.target_sub = self.create_subscription(
            PoseStamped, '/target_pose', self.goal_pose_callback, 10
        )

        # Lắng nghe chuyển chế độ hệ thống (nếu có lệnh MANUAL/CANCEL từ bàn phím thì dừng)
        self.mode_sub = self.create_subscription(
            String, '/exploration_mode', self.exploration_mode_callback, 10
        )

        # Lắng nghe topic chuyên biệt HỦY LỘ TRÌNH (/cancel_goal)
        self.cancel_sub = self.create_subscription(
            String, '/cancel_goal', self.cancel_goal_callback, 10
        )

        # Publishers
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.mode_pub = self.create_publisher(String, '/exploration_mode', 10)
        self.status_pub = self.create_publisher(String, '/navigation_status', 10)
        self.path_pub = self.create_publisher(Path, '/global_plan', 10)
        self.marker_pub = self.create_publisher(Marker, '/target_marker', 10)

        # Timer điều khiển chuyển động 20Hz (50ms)
        self.control_timer = self.create_timer(0.05, self.control_loop)

        # Timer kiểm tra và lập kế hoạch lại chu kỳ 1.0s (nếu gặp vật cản động)
        self.replan_timer = self.create_timer(1.0, self.replan_check_loop)

        self.get_logger().info(
            "====================================================================\n"
            "  TargetNavigatorNode (Point-to-Point Navigation) KHỞI ĐỘNG THÀNH CÔNG!\n"
            "  - Dùng công cụ '2D Goal Pose' trên RViz2 hoặc gửi toạ độ /target_pose.\n"
            "  - Tự động ngắt Auto Exploration cũ để tránh xung đột /cmd_vel.\n"
            "  - Tự động vạch đường né vật cản và dừng chính xác tuyệt đối tại đích.\n"
            "===================================================================="
        )
        self.publish_status("IDLE: Đang chờ chọn điểm đích trên RViz2 (/goal_pose)...")
        self.stop_robot()

    def publish_status(self, text: str):
        msg = String()
        msg.data = text
        self.status_pub.publish(msg)

    def publish_path(self, waypoints):
        """Phát đường đi lên RViz2 để hiển thị quỹ đạo màu xanh"""
        path_msg = Path()
        path_msg.header.stamp = self.get_clock().now().to_msg()
        path_msg.header.frame_id = 'map'
        for pt in waypoints:
            pose = PoseStamped()
            pose.header = path_msg.header
            pose.pose.position.x = float(pt[0])
            pose.pose.position.y = float(pt[1])
            pose.pose.position.z = 0.0
            pose.pose.orientation.w = 1.0
            path_msg.poses.append(pose)
        self.path_pub.publish(path_msg)

    def map_callback(self, msg: OccupancyGrid):
        self.map_data = msg
        w = msg.info.width
        h = msg.info.height
        new_data = np.array(msg.data, dtype=np.int8).reshape((h, w))

        # Cập nhật bộ nhớ bản đồ tích lũy (Accumulated Grid Memory)
        if self.accumulated_grid is None or self.accumulated_grid.shape != (h, w):
            self.accumulated_grid = new_data.copy()
        else:
            # Ghi nhớ các vùng đã vẽ:
            # 1. Ô nào là sàn tự do (0) thì lưu lại vào bộ nhớ
            # 2. Ô nào là tường (100) thì cập nhật tường
            known_free = (new_data == 0)
            known_wall = (new_data == 100)
            self.accumulated_grid[known_wall] = 100
            self.accumulated_grid[known_free & (self.accumulated_grid != 100)] = 0
            # Nếu SLAM cập nhật xóa tường cũ (trở thành 0):
            self.accumulated_grid[(self.accumulated_grid == 100) & (new_data == 0)] = 0

    def scan_callback(self, msg: LaserScan):
        self.latest_scan = msg

    def cancel_goal_callback(self, msg: String):
        """Hủy lộ trình điều hướng ngay lập tức khi nhận lệnh từ bàn phím hoặc script"""
        cmd = msg.data.strip().upper()
        if cmd in ["CANCEL", "STOP", "RESET", "MANUAL", "HUY"]:
            self.cancel_navigation(reason=f"Lệnh hủy từ topic /cancel_goal ({cmd})")

    def exploration_mode_callback(self, msg: String):
        """Nếu có lệnh MANUAL/CANCEL từ người dùng (bàn phím), hủy điều hướng ngay lập tức"""
        cmd = msg.data.strip().upper()
        # Tránh tự ngắt chính mình khi vừa publish "MANUAL" để ngắt auto_explorer
        if time.time() - self.last_self_override_time < 0.5:
            return
        if cmd in ["MANUAL", "CANCEL", "STOP"] and self.state in ['NAVIGATING', 'ALIGNING', 'PLANNING']:
            self.cancel_navigation(reason=f"Nhận lệnh {cmd} từ /exploration_mode")

    def cancel_navigation(self, reason: str = ""):
        """Dừng xe ngay lập tức, xóa đường đi, xóa marker điểm đích và đưa về IDLE"""
        self.stop_robot()
        self.target_goal = None
        self.current_path = []
        self.path_index = 0
        self.recovery_mode = None
        self.blocked_start_time = None
        self.state = 'IDLE'
        self.publish_path([])
        self.delete_target_marker()
        msg_str = f"🛑 ĐÃ HỦY LỘ TRÌNH ĐIỀU HƯỚNG! ({reason})" if reason else "🛑 ĐÃ HỦY LỘ TRÌNH ĐIỀU HƯỚNG!"
        self.get_logger().info(msg_str)
        self.publish_status("IDLE: Đã hủy lộ trình điều hướng. Chờ lệnh mới...")

    def delete_target_marker(self):
        """Xóa marker điểm đích trên RViz2"""
        marker = Marker()
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.header.frame_id = 'map'
        marker.ns = 'target_point'
        marker.id = 1
        marker.action = Marker.DELETE
        self.marker_pub.publish(marker)

    def stop_robot(self):
        """Dừng xe hoàn toàn, triệt tiêu quán tính"""
        self.cmd_v = 0.0
        self.cmd_w = 0.0
        try:
            if rclpy.ok():
                cmd = Twist()
                self.cmd_pub.publish(cmd)
        except Exception:
            pass

    def get_robot_pose(self):
        """Lấy toạ độ (x, y, yaw) của robot trên map thông qua TF (không khóa luồng)"""
        try:
            t = self.tf_buffer.lookup_transform(
                'map', 'base_footprint', rclpy.time.Time()
            )
            x = t.transform.translation.x
            y = t.transform.translation.y
            q = t.transform.rotation
            siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
            cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
            yaw = math.atan2(siny_cosp, cosy_cosp)
            return (x, y, yaw)
        except Exception:
            return None

    def publish_target_marker(self, gx: float, gy: float, yaw: float, arrived: bool = False):
        """Phát Marker 3D hình cột tiêu (Pin) màu nổi bật tại điểm đích trên RViz2"""
        marker = Marker()
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.header.frame_id = 'map'
        marker.ns = 'target_point'
        marker.id = 1
        marker.type = Marker.CYLINDER
        marker.action = Marker.ADD

        # Vị trí tâm cột tiêu
        marker.pose.position.x = float(gx)
        marker.pose.position.y = float(gy)
        marker.pose.position.z = 0.25  # Nhô cao 25cm trên sàn
        marker.pose.orientation.z = math.sin(yaw / 2.0)
        marker.pose.orientation.w = math.cos(yaw / 2.0)

        # Kích thước cột tiêu
        marker.scale.x = 0.25  # Đường kính 25cm
        marker.scale.y = 0.25
        marker.scale.z = 0.50  # Chiều cao 50cm

        # Màu sắc: Đang chạy -> Đỏ rực, Đã đến đích -> Xanh lá
        if arrived:
            marker.color.r = 0.0
            marker.color.g = 1.0
            marker.color.b = 0.0
            marker.color.a = 0.95
        else:
            marker.color.r = 1.0
            marker.color.g = 0.15
            marker.color.b = 0.15
            marker.color.a = 0.95

        self.marker_pub.publish(marker)

    def clicked_point_callback(self, msg: PointStamped):
        """Xử lý click đơn từ công cụ 'Publish Point' trên thanh công cụ RViz2 (/clicked_point)"""
        gx = msg.point.x
        gy = msg.point.y

        # Tự động tính góc hướng từ robot tới điểm đích
        robot_pose = self.get_robot_pose()
        if robot_pose is not None:
            rx, ry, _ = robot_pose
            target_yaw = math.atan2(gy - ry, gx - rx)
        else:
            target_yaw = 0.0

        # Tạo PoseStamped tương đương
        pose_msg = PoseStamped()
        pose_msg.header = msg.header
        pose_msg.header.stamp = self.get_clock().now().to_msg()
        if not pose_msg.header.frame_id:
            pose_msg.header.frame_id = 'map'
        pose_msg.pose.position.x = gx
        pose_msg.pose.position.y = gy
        pose_msg.pose.position.z = 0.0
        pose_msg.pose.orientation.z = math.sin(target_yaw / 2.0)
        pose_msg.pose.orientation.w = math.cos(target_yaw / 2.0)

        self.get_logger().info(f"🖱️ [CLICK ĐƠN RVIZ] Đã nhận điểm từ 'Publish Point': X = {gx:.2f}, Y = {gy:.2f}")
        self.goal_pose_callback(pose_msg)

    def goal_pose_callback(self, msg: PoseStamped):
        """Xử lý điểm đích mới từ RViz2 (/goal_pose) hoặc topic /target_pose"""
        gx = msg.pose.position.x
        gy = msg.pose.position.y

        # =========================================================================
        # TÍNH NĂNG CLICK HỦY LỘ TRÌNH:
        # Nếu người dùng click vào ngay vị trí robot (< 25cm) khi đang điều hướng,
        # hệ thống hiểu là muốn HỦY LỘ TRÌNH ngay lập tức!
        # =========================================================================
        robot_pose = self.get_robot_pose()
        if robot_pose is not None and self.state in ['NAVIGATING', 'ALIGNING', 'PLANNING']:
            rx, ry, _ = robot_pose
            if math.hypot(gx - rx, gy - ry) < 0.25:
                self.get_logger().info("🖱️ [CLICK HỦY LỘ TRÌNH] Bạn đã click vào ngay robot trên RViz2 -> Hủy lộ trình!")
                self.cancel_navigation(reason="Click vào vị trí robot trên RViz2")
                return

        # =========================================================================
        # GIẢI QUYẾT XUNG ĐỘT TOPIC /cmd_vel:
        # Ngắt ngay node auto_explorer cũ bằng cách gửi "MANUAL" tới /exploration_mode.
        # Node auto_explorer nhận được sẽ lập tức dừng gửi /cmd_vel và hủy đường đi cũ.
        # Ghi nhận thời điểm tự override để node này không tự hủy chính mình.
        # =========================================================================
        self.last_self_override_time = time.time()
        override_msg = String()
        override_msg.data = "MANUAL"
        self.mode_pub.publish(override_msg)

        # Trích xuất toạ độ và góc xoay đích
        q = msg.pose.orientation
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        target_yaw = math.atan2(siny_cosp, cosy_cosp)

        self.get_logger().info(
            f"\n🎯 [TARGET NAVIGATOR] NHẬN ĐIỂM ĐÍCH MỚI:\n"
            f"   -> Toạ độ: X = {gx:.3f} m, Y = {gy:.3f} m\n"
            f"   -> Hướng góc đích: {math.degrees(target_yaw):.1f}° ({target_yaw:.3f} rad)\n"
            f"   -> Đang ngắt Auto cũ và lập kế hoạch đường đi an toàn..."
        )
        self.publish_status(f"PLANNING: Đang tính đường né vật cản tới ({gx:.2f}, {gy:.2f})...")

        # Đặt mục tiêu mới và chuyển trạng thái lập đường đi
        self.target_goal = (gx, gy, target_yaw)
        self.publish_target_marker(gx, gy, target_yaw, arrived=False)
        self.blocked_start_time = None
        self.recovery_mode = None
        self.state = 'PLANNING'
        self.plan_path_to_target()

    def plan_path_to_target(self):
        """Vạch đường đi tối ưu A* từ vị trí hiện tại tới điểm đích dựa trên bản đồ tích lũy"""
        if self.map_data is None:
            self.get_logger().warn("Chưa có bản đồ từ SLAM, không thể lập đường đi!")
            self.publish_status("ERROR: Chưa có bản đồ /map từ SLAM!")
            self.state = 'IDLE'
            return False

        robot_pose = self.get_robot_pose()
        if robot_pose is None:
            self.get_logger().warn("Không lấy được vị trí robot từ TF (map -> base_footprint)!")
            self.publish_status("ERROR: Không xác định được vị trí robot trên map!")
            self.state = 'IDLE'
            return False

        rx, ry, _ = robot_pose
        gx, gy, target_yaw = self.target_goal

        w = self.map_data.info.width
        h = self.map_data.info.height
        res = self.map_data.info.resolution
        origin_x = self.map_data.info.origin.position.x
        origin_y = self.map_data.info.origin.position.y

        # Sử dụng bản đồ bộ nhớ tích lũy (Accumulated Grid Memory) đã ghi nhớ toàn bộ vùng đã vẽ
        if self.accumulated_grid is not None and self.accumulated_grid.shape == (h, w):
            grid = self.accumulated_grid.copy()
        else:
            grid = np.array(self.map_data.data, dtype=np.int8).reshape((h, w))

        occupied_mask = (grid == 100)
        # Tính khoảng cách từ mọi ô tới bức tường gần nhất (đơn vị: ô)
        dist_to_wall = distance_transform_edt(~occupied_mask)

        # Chuyển toạ độ xuất phát và đích sang toạ độ ô lưới
        start_gx = int(np.clip((rx - origin_x) / res, 0, w - 1))
        start_gy = int(np.clip((ry - origin_y) / res, 0, h - 1))
        goal_gx = int(np.clip((gx - origin_x) / res, 0, w - 1))
        goal_gy = int(np.clip((gy - origin_y) / res, 0, h - 1))

        # TỰ ĐỘNG HÚT ĐIỂM ĐÍCH VỀ VÙNG SÀN TỰ DO ĐÃ QUÉT (Smart Goal Snapping):
        # Hỗ trợ điểm đích ở xa, xuyên phòng hoặc ngoài vùng đã quét:
        # Nếu điểm đích rơi vào vùng chưa quét (-1), tường (100) hoặc quá sát tường (< 12cm):
        # Tự động tìm ô sàn tự do (grid == 0) an toàn gần nhất hướng về điểm đích.
        need_snap = (grid[goal_gy, goal_gx] != 0 or dist_to_wall[goal_gy, goal_gx] < 2.4)
        if need_snap:
            self.get_logger().info(f"🎯 Điểm đích ({gx:.2f}, {gy:.2f}) chưa thông thoáng hoặc ở xa -> Đang tìm điểm sàn tự do an toàn nhất...")
            found_free = False
            best_dist = float('inf')
            best_gx, best_gy = goal_gx, goal_gy
            max_r = int(12.0 / res)  # Quét rộng tới 12 mét để đón nhận điểm ở xa!
            for r in range(1, max_r + 1, 2):
                for dy in range(-r, r + 1, 2):
                    for dx in range(-r, r + 1, 2):
                        if abs(dx) != r and abs(dy) != r:
                            continue
                        nx, ny = goal_gx + dx, goal_gy + dy
                        if 0 <= nx < w and 0 <= ny < h:
                            if grid[ny, nx] == 0 and dist_to_wall[ny, nx] >= 2.4:
                                d = math.hypot(dx, dy)
                                if d < best_dist:
                                    best_dist = d
                                    best_gx, best_gy = nx, ny
                                    found_free = True
                if found_free and best_dist <= (r + 2):
                    break

            if found_free:
                goal_gx, goal_gy = best_gx, best_gy
                gx = origin_x + (goal_gx + 0.5) * res
                gy = origin_y + (goal_gy + 0.5) * res
                self.target_goal = (gx, gy, target_yaw)
                self.publish_target_marker(gx, gy, target_yaw, arrived=False)
                self.get_logger().info(f"✅ Đã điều chỉnh điểm đích an toàn tới: ({gx:.2f}, {gy:.2f})")
            else:
                self.get_logger().error("❌ Điểm đích hoàn toàn không nằm trong vùng bản đồ đã vẽ! Dừng xe an toàn.")
                self.publish_status("ERROR: Điểm đích ngoài vùng map đã vẽ!")
                self.stop_robot()
                self.state = 'IDLE'
                return False

        # Thực hiện A* tìm kiếm đường đi toàn diện
        path_grid = self.run_astar(grid, (start_gx, start_gy), (goal_gx, goal_gy), dist_to_wall)
        if not path_grid or len(path_grid) < 2:
            self.get_logger().error("❌ Không tìm được đường đi đến đích (bị tường/vật cản chặn hoàn toàn)! Dừng xe an toàn.")
            self.publish_status("BLOCKED: Đường đi bị vật cản chặn hoàn toàn! Dừng xe an toàn.")
            self.stop_robot()
            self.publish_path([])
            self.state = 'IDLE'
            return False

        # Làm mượt đường đi (Path Smoothing)
        smooth_grid = self.smooth_path(path_grid, dist_to_wall)

        # Chuyển đổi sang toạ độ thực thế giới
        path_world = [
            (origin_x + (pt[0] + 0.5) * res, origin_y + (pt[1] + 0.5) * res)
            for pt in smooth_grid
        ]
        # Điểm mốc cuối cùng là toạ độ chính xác của đích
        path_world.append((gx, gy))

        self.current_path = path_world
        self.state = 'NAVIGATING'
        self.publish_path(self.current_path)

        dist_total = math.hypot(gx - rx, gy - ry)
        self.get_logger().info(
            f"✅ ĐÃ TẠO QUỸ ĐẠO THÀNH CÔNG! Chiều dài: ~{dist_total:.2f} m ({len(self.current_path)} điểm nút).\n"
            f"   -> Bắt đầu di chuyển bám đường né vật cản, bảo vệ bản đồ SLAM..."
        )
        self.publish_status(f"NAVIGATING: Đang di chuyển né vật cản tới ({gx:.2f}, {gy:.2f})...")
        return True

    def run_astar(self, grid, start_grid, goal_grid, dist_to_wall):
        """
        Thuật toán A* toàn diện với kích thước thân xe 3D thực tế (Rộng 17cm, Dài 21cm):
        - Cấp 1 (Chuẩn): Cách tường >= 2.4 ô (12cm) -> Khe cửa hẹp >= 24cm xe lọt qua an toàn.
        - Cấp 2 (Khe hẹp): Cách tường >= 2.1 ô (10.5cm) -> Khe cửa hẹp >= 21cm (xe rộng 17cm, dư 2cm mỗi bên).
        - Cấp 3 (Cực hạn): Cách tường >= 1.9 ô (9.5cm) -> Khe cửa >= 19cm đi thẳng không chạm.
        """
        path = self._astar_core(grid, start_grid, goal_grid, dist_to_wall, min_wall_dist=2.4, max_steps=18000)
        if path is not None:
            return path

        self.get_logger().warn("Cấp 1 không tìm thấy đường, đang thử Cấp 2 (khe hẹp >= 21cm)...")
        path = self._astar_core(grid, start_grid, goal_grid, dist_to_wall, min_wall_dist=2.1, max_steps=18000)
        if path is not None:
            return path

        self.get_logger().warn("Cấp 2 không tìm thấy đường, đang thử Cấp 3 (khe cực hạn >= 19cm)...")
        return self._astar_core(grid, start_grid, goal_grid, dist_to_wall, min_wall_dist=1.9, max_steps=18000)

    def _astar_core(self, grid, start_grid, goal_grid, dist_to_wall, min_wall_dist, max_steps):
        h, w = grid.shape
        sx, sy = start_grid
        gx, gy = goal_grid

        # Ô đi được: Ô tự do (0) và cách tường >= min_wall_dist
        passable = (grid == 0) & (dist_to_wall >= min_wall_dist)
        passable[sy, sx] = True
        passable[gy, gx] = True

        # Đảm bảo khu vực xuất phát và điểm đích luôn thông thoáng (tránh lỗi cô lập điểm đích)
        for dy in [-1, 0, 1]:
            for dx in [-1, 0, 1]:
                px, py = sx + dx, sy + dy
                if 0 <= px < w and 0 <= py < h and grid[py, px] == 0:
                    passable[py, px] = True
                qx, qy = gx + dx, gy + dy
                if 0 <= qx < w and 0 <= qy < h and grid[qy, qx] == 0:
                    passable[qy, qx] = True

        open_set = []
        heapq.heappush(open_set, (0.0, 0.0, (sx, sy)))
        came_from = {}
        g_score = {(sx, sy): 0.0}
        visited = np.zeros((h, w), dtype=bool)

        neighbors = [
            (1, 0, 1.0), (-1, 0, 1.0), (0, 1, 1.0), (0, -1, 1.0),
            (1, 1, 1.414), (-1, 1, 1.414), (1, -1, 1.414), (-1, -1, 1.414)
        ]

        occupied = (grid == 100)
        found = False
        steps = 0

        while open_set and steps < max_steps:
            steps += 1
            _, cost, (cx, cy) = heapq.heappop(open_set)

            if visited[cy, cx]:
                continue
            visited[cy, cx] = True

            if (cx, cy) == (gx, gy) or math.hypot(cx - gx, cy - gy) <= 1.2:
                gx, gy = cx, cy
                found = True
                break

            for dx, dy, step_cost in neighbors:
                nx, ny = cx + dx, cy + dy
                if 0 <= nx < w and 0 <= ny < h and passable[ny, nx] and not visited[ny, nx]:
                    # Chống cắt góc chéo mép tường
                    if dx != 0 and dy != 0:
                        if occupied[cy, nx] or occupied[ny, cx] or dist_to_wall[cy, nx] < 1.8 or dist_to_wall[ny, cx] < 1.8:
                            continue

                    # Lực đẩy tường vừa phải: Ưu tiên căn tim đường/cửa hẹp nhưng không lấn át khoảng cách
                    clr = dist_to_wall[ny, nx]
                    if clr < 3.2:
                        pen = 3.0 * (3.2 - clr)
                    elif clr < 5.0:
                        pen = 0.8 * (5.0 - clr)
                    else:
                        pen = 0.0

                    tg = cost + step_cost + pen
                    if (nx, ny) not in g_score or tg < g_score[(nx, ny)]:
                        g_score[(nx, ny)] = tg
                        f = tg + 1.05 * math.hypot(nx - gx, ny - gy)
                        came_from[(nx, ny)] = (cx, cy)
                        heapq.heappush(open_set, (f, tg, (nx, ny)))

        if not found:
            return None

        path = []
        curr = (gx, gy)
        while curr in came_from:
            path.append(curr)
            curr = came_from[curr]
        path.append((sx, sy))
        path.reverse()
        return path

    def smooth_path(self, path, dist_to_wall):
        """Giữ toàn bộ điểm nút dày đặc (5cm/điểm) để Pure Pursuit bám liên tục, loại bỏ hiện tượng giật mốc"""
        return path

    def analyze_scan(self):
        """
        Phân tích 360 độ LiDAR theo kích thước hình chữ nhật 3D chuẩn của xe:
        - Dài 0.210m (Mũi xe: x = +0.105m, Đuôi xe: x = -0.105m so với tâm LiDAR).
        - Rộng 0.170m (Hông trái: y = +0.085m, Hông phải: y = -0.085m).
        - 2 Góc trước: (+0.105m, ±0.085m).
        """
        dist_front = 10.0
        dist_rear = 10.0
        dist_left = 10.0
        dist_right = 10.0
        dist_fl_corner = 10.0
        dist_fr_corner = 10.0

        if self.latest_scan is None:
            return dist_front, dist_rear, dist_left, dist_right, dist_fl_corner, dist_fr_corner

        ranges = np.array(self.latest_scan.ranges)
        n = len(ranges)
        if n == 0:
            return dist_front, dist_rear, dist_left, dist_right, dist_fl_corner, dist_fr_corner

        angles = np.linspace(
            float(self.latest_scan.angle_min),
            float(self.latest_scan.angle_max),
            n,
            endpoint=False
        )
        valid = np.isfinite(ranges) & (ranges > 0.05) & (ranges < 4.0)
        if not np.any(valid):
            return dist_front, dist_rear, dist_left, dist_right, dist_fl_corner, dist_fr_corner

        r_v = ranges[valid]
        a_v = angles[valid]
        x_pts = r_v * np.cos(a_v)
        y_pts = r_v * np.sin(a_v)

        # 1. Hành lang mũi xe đi thẳng (bề rộng 0.19m = |y| <= 0.095m, mũi xe x = 0.105m)
        front_mask = (x_pts > 0.05) & (x_pts < 1.0) & (np.abs(y_pts) <= 0.095)
        if np.any(front_mask):
            dist_front = float(np.min(x_pts[front_mask]))

        # 2. Hành lang đuôi xe lùi (bề rộng 0.19m = |y| <= 0.095m)
        rear_mask = (x_pts < -0.05) & (x_pts > -1.0) & (np.abs(y_pts) <= 0.095)
        if np.any(rear_mask):
            dist_rear = float(np.min(np.abs(x_pts[rear_mask])))

        # 3. Khoảng hở hông hai bên (khoảng cách thực tế từ thành xe y=±0.085m tới mép cửa/tường)
        left_mask = (y_pts > 0.085) & (y_pts < 0.35) & (x_pts > -0.09) & (x_pts < 0.13)
        if np.any(left_mask):
            dist_left = float(np.min(y_pts[left_mask] - 0.085))

        right_mask = (y_pts < -0.085) & (y_pts > -0.35) & (x_pts > -0.09) & (x_pts < 0.13)
        if np.any(right_mask):
            dist_right = float(np.min(np.abs(y_pts[right_mask]) - 0.085))

        # 4. Khoảng cách trực tiếp từ 2 góc trước hình chữ nhật (+0.105, ±0.085) tới vật cản
        dist_fl_corner = float(np.min(np.hypot(x_pts - 0.105, y_pts - 0.085)))
        dist_fr_corner = float(np.min(np.hypot(x_pts - 0.105, y_pts + 0.085)))

        return dist_front, dist_rear, dist_left, dist_right, dist_fl_corner, dist_fr_corner

    def replan_check_loop(self):
        """Kiểm tra định kỳ xem có cần vạch lại đường nếu chướng ngại vật mới xuất hiện"""
        if self.target_goal is not None and self.state in ['NAVIGATING', 'PLANNING', 'ALIGNING']:
            gx, gy, target_yaw = self.target_goal
            self.publish_target_marker(gx, gy, target_yaw, arrived=False)

        if self.state != 'NAVIGATING' or self.target_goal is None or self.recovery_mode is not None:
            return

        # Nếu bị chặn quá 2.5 giây mà không thể tiến lên -> Tự động tìm đường vòng thay thế
        if self.blocked_start_time is not None:
            if time.time() - self.blocked_start_time > 2.5:
                self.get_logger().warn("⚠️ Vật cản chặn đường quá 2.5s! Đang thử tìm đường vòng thay thế...")
                success = self.plan_path_to_target()
                if not success:
                    self.get_logger().error("❌ Không còn đường vòng nào khác! Dừng xe an toàn để tránh va chạm.")
                    self.publish_status("BLOCKED: Vật cản chặn hoàn toàn! Dừng xe an toàn.")
                    self.stop_robot()
                    self.state = 'IDLE'
                    self.blocked_start_time = None

    def control_loop(self):
        """Vòng lặp điều khiển vận tốc thời gian thực tần số 20Hz (50ms) - Mượt mà, chống va kẹt mép tường"""
        if self.state not in ['NAVIGATING', 'ALIGNING']:
            return

        pose = self.get_robot_pose()
        if pose is None:
            return
        rx, ry, current_yaw = pose

        gx, gy, target_yaw = self.target_goal
        dist_to_goal = math.hypot(gx - rx, gy - ry)

        # =========================================================================
        # GIAI ĐOẠN 1: BÁM QUỸ ĐẠO ĐẾN ĐÍCH (NAVIGATING)
        # =========================================================================
        if self.state == 'NAVIGATING':
            # Kiểm tra xem đã vào phạm vi sai số cho phép chưa (<= 8cm)
            if dist_to_goal <= self.xy_tol:
                self.get_logger().info(
                    f"🎯 ĐÃ ĐẾN VỊ TRÍ ĐÍCH! (Khoảng cách còn: {dist_to_goal*100:.1f} cm <= {self.xy_tol*100:.1f} cm).\n"
                    f"   -> Bắt đầu xoay tinh chỉnh góc đích..."
                )
                self.state = 'ALIGNING'
                self.stop_robot()
                return

            # Phân tích khoảng cách chuẩn hình học 3D từ LiDAR
            dist_front, dist_rear, dist_left, dist_right, dist_fl, dist_fr = self.analyze_scan()

            # =====================================================================
            # 1. XỬ LÝ TRẠNG THÁI PHỤC HỒI THOÁT KẸT (RECOVERY STATE)
            # =====================================================================
            if self.recovery_mode is not None:
                # Đang trong chu trình lùi thoát kẹt
                if self.recovery_mode == 'REVERSING':
                    if time.time() < self.recovery_end_time and dist_rear >= 0.15:
                        cmd = Twist()
                        cmd.linear.x = -0.10  # Lùi thẳng 10 cm/s
                        cmd.angular.z = 0.0
                        self.cmd_pub.publish(cmd)
                        self.publish_status(f"RECOVERY: Đang lùi thẳng né mép tường (cách sau {dist_rear*100:.0f}cm)...")
                        return
                    else:
                        self.stop_robot()
                        self.recovery_mode = None
                        self.get_logger().info("✅ Đã lùi tạo khoảng trống! Đang vạch lại đường...")
                        self.plan_path_to_target()
                        return

                elif self.recovery_mode == 'ROTATING':
                    if time.time() < self.recovery_end_time:
                        cmd = Twist()
                        cmd.linear.x = 0.0
                        cmd.angular.z = float(self.recovery_rot_dir * 0.30)
                        self.cmd_pub.publish(cmd)
                        self.publish_status(f"RECOVERY: Đang xoay mở góc thoát...")
                        return
                    else:
                        self.stop_robot()
                        self.recovery_mode = None
                        self.get_logger().info("✅ Đã xoay mở góc! Đang vạch lại đường...")
                        self.plan_path_to_target()
                        return

            # =====================================================================
            # 2. PHÁT HIỆN KẸT CƠ HỌC TẠI CHỖ (MECHANICAL STUCK DETECTION)
            # =====================================================================
            if self.stuck_check_pos is None or (time.time() - self.stuck_check_time > 1.2):
                if self.stuck_check_pos is not None and abs(self.cmd_v) > 0.03:
                    moved_dist = math.hypot(rx - self.stuck_check_pos[0], ry - self.stuck_check_pos[1])
                    if moved_dist < 0.035:
                        self.stuck_counter += 1
                        if self.stuck_counter >= 2:  # Bị kẹt liên tục > 2s
                            self.stuck_counter = 0
                            self.get_logger().warn(f"🚨 Phát hiện robot kẹt cơ học! Luôn ưu tiên LÙI THẲNG ra xa tường trước.")
                            self.recovery_mode = 'REVERSING'
                            self.recovery_end_time = time.time() + 1.2
                            self.stop_robot()
                            return
                    else:
                        self.stuck_counter = 0
                self.stuck_check_pos = (rx, ry)
                self.stuck_check_time = time.time()

            # =====================================================================
            # 3. PHANH AN TOÀN CHUẨN XÁC THEO HÌNH HỌC HÌNH CHỮ NHẬT
            # =====================================================================
            target_v = 0.0
            target_w = 0.0

            # Mũi xe tại x = +0.105m từ LiDAR. Ngưỡng phanh an toàn trước mũi: 4.5cm -> dist_front < 0.150m
            # Góc trước xe: Ngưỡng an toàn góc: 3.5cm -> min(dist_fl, dist_fr) < 0.035m
            front_or_corner_blocked = (dist_front < 0.150) or (min(dist_fl, dist_fr) < 0.035)

            if front_or_corner_blocked:
                if self.blocked_start_time is None:
                    self.blocked_start_time = time.time()

                # Nếu bị chặn quá 0.8 giây: kích hoạt lùi thẳng để mở rộng góc xoay
                if time.time() - self.blocked_start_time > 0.8:
                    min_corner = min(dist_fl, dist_fr)
                    self.get_logger().warn(
                        f"⚠️ Sát mép (Trước: {dist_front*100:.0f}cm, Góc: {min_corner*100:.0f}cm)! Lùi thẳng né cạ góc..."
                    )
                    self.recovery_mode = 'REVERSING'
                    self.recovery_end_time = time.time() + 1.2
                    self.blocked_start_time = None
                    self.stop_robot()
                    return

                target_v = 0.0
                target_w = 0.0
                self.cmd_v = 0.0
                self.publish_status(
                    f"WARNING: Sát mép (F:{dist_front*100:.0f}cm, C:{min(dist_fl, dist_fr)*100:.0f}cm)! Đang dừng an toàn..."
                )
            else:
                self.blocked_start_time = None

                if not self.current_path:
                    self.plan_path_to_target()
                    return

                # CHỐNG NHẢY CÓC CẮT QUỸ ĐẠO XUYÊN VÁCH NGĂN:
                # Chỉ tìm điểm nút gần nhất trong cửa sổ 15 điểm tiếp theo
                search_len = min(15, len(self.current_path))
                dists = [math.hypot(p[0] - rx, p[1] - ry) for p in self.current_path[:search_len]]
                closest_idx = int(np.argmin(dists))
                if closest_idx > 0:
                    self.current_path = self.current_path[closest_idx:]

                # Bán kính ngắm thích ứng (Adaptive Lookahead)
                cur_lookahead = self.lookahead_dist
                min_side = min(dist_left, dist_right)
                in_narrow_passage = (min_side < 0.06)  # Hông xe cách mép cửa < 6cm
                if in_narrow_passage or dist_to_goal < 0.50:
                    cur_lookahead = 0.20

                lookahead_pt = self.current_path[-1]
                for p in self.current_path:
                    d = math.hypot(p[0] - rx, p[1] - ry)
                    if d >= cur_lookahead:
                        lookahead_pt = p
                        break

                dx = lookahead_pt[0] - rx
                dy = lookahead_pt[1] - ry
                target_dist = math.hypot(dx, dy)
                target_angle = math.atan2(dy, dx)
                alpha = math.atan2(math.sin(target_angle - current_yaw), math.cos(target_angle - current_yaw))

                # ĐIỀU KHIỂN CHỐNG CẠ GÓC KHI QUA KHE HẸP:
                # Nếu đang trong khe hẹp (cách hông < 6cm): Tuyệt đối đi thẳng/ôm tim, KHÔNG xoay giật cục
                if in_narrow_passage:
                    target_v = 0.14  # Tốc độ ổn định qua khe cửa
                    target_w = float(np.clip(alpha * 0.8, -0.20, 0.20))
                elif abs(alpha) > 0.35:
                    # Góc cua gấp: Xoay tại chỗ (v = 0)
                    target_v = 0.0
                    self.cmd_v = 0.0
                    target_w = float(np.clip(1.2 * alpha, -0.35, 0.35))
                else:
                    kappa = 2.0 * math.sin(alpha) / max(0.20, target_dist)
                    speed_scale = max(0.30, math.cos(alpha))
                    target_v = float(self.max_v * speed_scale)

                    if dist_front < 0.35:
                        target_v *= max(0.35, (dist_front - 0.15) / 0.20)
                    if dist_to_goal < 0.35:
                        target_v *= max(0.30, dist_to_goal / 0.35)

                    target_v = float(np.clip(target_v, 0.06, self.max_v))
                    target_w = float(np.clip(target_v * kappa, -self.max_w, self.max_w))

            # Giới hạn gia tốc mượt mà (Smoothing): chống giật tuyệt đối
            if target_v == 0.0:
                self.cmd_v = 0.0
            else:
                dv = np.clip(target_v - self.cmd_v, -self.accel_v_limit, self.accel_v_limit)
                self.cmd_v += dv
            dw = np.clip(target_w - self.cmd_w, -self.accel_w_limit, self.accel_w_limit)
            self.cmd_w += dw

            cmd = Twist()
            cmd.linear.x = float(self.cmd_v)
            cmd.angular.z = float(self.cmd_w)
            self.cmd_pub.publish(cmd)

            self.publish_status(f"NAVIGATING: Đang đi tới đích (còn {dist_to_goal:.2f}m) | V={self.cmd_v:.2f}m/s")

        # =========================================================================
        # GIAI ĐOẠN 2: XOAY TINH CHỈNH GÓC ĐÍCH (ALIGNING)
        # =========================================================================
        elif self.state == 'ALIGNING':
            # Tính độ lệch góc quay cuối cùng
            yaw_error = math.atan2(math.sin(target_yaw - current_yaw), math.cos(target_yaw - current_yaw))

            # Nếu góc đã vào phạm vi sai số (< 0.08 rad ~ 4.5 độ) -> HOÀN THÀNH!
            if abs(yaw_error) <= self.yaw_tol:
                self.stop_robot()
                self.publish_path([])
                self.publish_target_marker(gx, gy, target_yaw, arrived=True)
                self.state = 'IDLE'

                # Báo cáo kết quả chi tiết
                final_pose = self.get_robot_pose()
                fx, fy, fyaw = final_pose if final_pose else (rx, ry, current_yaw)
                final_dist_err = math.hypot(gx - fx, gy - fy)
                final_yaw_err = abs(math.atan2(math.sin(target_yaw - fyaw), math.cos(target_yaw - fyaw)))

                self.get_logger().info(
                    f"\n🎉 [TARGET NAVIGATOR] ĐÃ ĐẾN ĐÍCH CHÍNH XÁC HOÀN TOÀN!\n"
                    f"   -> Điểm đến: X = {gx:.3f} m, Y = {gy:.3f} m\n"
                    f"   -> Vị trí thực tế: X = {fx:.3f} m, Y = {fy:.3f} m\n"
                    f"   -> Sai số khoảng cách: {final_dist_err*100:.1f} cm (Chuẩn cho phép: {self.xy_tol*100:.0f} cm)\n"
                    f"   -> Sai số góc xoay: {math.degrees(final_yaw_err):.1f}° ({final_yaw_err:.3f} rad < {self.yaw_tol:.3f} rad)\n"
                    f"   -> Robot đã dừng hẳn, giữ vị trí an toàn.\n"
                )
                self.publish_status(
                    f"SUCCESS: ĐÃ ĐẾN ĐÍCH! (Sai số: {final_dist_err*100:.1f}cm, góc: {math.degrees(final_yaw_err):.1f}°)"
                )
                return

            # Xoay tại chỗ tinh chỉnh góc đích thật êm ái (tối đa 0.25 rad/s)
            target_w = float(np.clip(1.0 * yaw_error, -0.25, 0.25))
            if abs(target_w) < 0.06:
                target_w = 0.06 if target_w > 0 else -0.06

            cmd = Twist()
            cmd.linear.x = 0.0
            cmd.angular.z = target_w
            self.cmd_pub.publish(cmd)
            self.publish_status(f"ALIGNING: Đang chỉnh góc đích (lệch {math.degrees(yaw_error):.1f}°)...")


def main(args=None):
    rclpy.init(args=args)
    node = TargetNavigatorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.stop_robot()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
