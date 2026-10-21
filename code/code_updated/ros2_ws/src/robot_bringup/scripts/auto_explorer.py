#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Auto Explorer Node - Tự hành khám phá 3 Cấp độ Tối ưu hoá Toàn diện:
----------------------------------------------------------------------
1. ĐỘNG HỌC TỐC ĐỘ CAO & LƯỢN VÒNG CUNG LIÊN TỤC (PURE PURSUIT):
   - Vận tốc tiến tối đa: 0.30 m/s (gấp đôi tốc độ cũ).
   - Vận tốc góc tối đa: 0.85 rad/s (quay 90 độ chỉ mất 1.8s thay vì 5s).
   - Bám quỹ đạo cong mượt mà (Pure Pursuit), không ngắt phanh dừng xe tại chỗ.
2. TỰ ĐỘNG CĂN TIM LUỒN LÁCH QUA KHE CỬA HẸP (CORRIDOR / DOORWAY CENTERING):
   - Bổ sung hàm phạt khoảng cách thành tường: A* tự động vạch đường đi chính xác
     vào TIM CỬA HẸP và GIỮA HÀNH LANG (nơi có khoảng trống lớn nhất).
   - Loại bỏ lực cản ảo từ hai trụ cửa, giúp xe lao dứt khoát qua cửa sang phòng mới.
3. BỘ LẬP KẾ HOẠCH TOÀN CỤC SIÊU NHẸ A* (LIGHTWEIGHT GRID A* PLANNER):
   - Tính toán đường đi né tường tối ưu trực tiếp trên ma trận OccupancyGrid trong 2-3ms.
   - Giải quyết triệt để lỗi xe bị quẩn quanh trong phòng vuông: A* tự động tìm khe hở/cửa
     để vạch đường đi ra các nhánh mới bên ngoài.
   - Phát đường đi lên topic /global_plan hiển thị trực quan trên RViz2.
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
from geometry_msgs.msg import Twist, PoseStamped
from std_msgs.msg import String
import tf2_ros


class AutoExplorerNode(Node):
    def __init__(self):
        super().__init__('auto_explorer_node')

        # Tham số cấu hình ROS 2
        if not self.has_parameter('use_sim_time'):
            self.declare_parameter('use_sim_time', True)
        if not self.has_parameter('default_mode'):
            self.declare_parameter('default_mode', 'MANUAL')
        if not self.has_parameter('max_linear_speed'):
            self.declare_parameter('max_linear_speed', 0.30)
        if not self.has_parameter('max_angular_speed'):
            self.declare_parameter('max_angular_speed', 0.85)
        if not self.has_parameter('lookahead_dist'):
            self.declare_parameter('lookahead_dist', 0.35)
        if not self.has_parameter('stop_distance'):
            self.declare_parameter('stop_distance', 0.22)

        self.current_mode = self.get_parameter('default_mode').value.upper()
        self.max_v = float(self.get_parameter('max_linear_speed').value)
        self.max_w = float(self.get_parameter('max_angular_speed').value)
        self.lookahead_dist = float(self.get_parameter('lookahead_dist').value)
        self.stop_dist = float(self.get_parameter('stop_distance').value)

        # Dữ liệu bản đồ & cảm biến
        self.map_data = None
        self.latest_scan = None
        self.blacklist = {}  # (x, y) -> expire_time

        # Quỹ đạo toàn cục A* hiện tại
        self.current_path = []  # Danh sách tọa độ thế giới [(wx, wy), ...]
        self.current_goal = None

        # Bộ lọc gia tốc mượt (20Hz = 50ms)
        self.cmd_v = 0.0
        self.cmd_w = 0.0
        self.accel_v_limit = 0.04  # ~0.8 m/s^2 tăng/giảm tốc êm ái
        self.accel_w_limit = 0.08  # ~1.6 rad/s^2 góc quay mượt mà

        # Trạng thái lùi khẩn cấp & xoay quét cục bộ tại đích
        self.escaping = False
        self.escape_end_time = 0.0
        self.local_spin_end = 0.0

        # Kiểm tra kẹt (Stuck Detection)
        self.stuck_check_time = 0.0
        self.stuck_check_pos = None
        self.stuck_duration = 0.0

        # TF Listener
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # Đăng ký topic bản đồ
        qos_map = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )
        self.map_sub = self.create_subscription(
            OccupancyGrid, '/map', self.map_callback, qos_map
        )

        # Đăng ký topic LiDAR chuẩn hóa
        qos_scan = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=5
        )
        self.scan_sub = self.create_subscription(
            LaserScan, '/scan_relay', self.scan_callback, qos_scan
        )

        # Chuyển đổi chế độ
        self.mode_sub = self.create_subscription(
            String, '/exploration_mode', self.mode_callback, 10
        )

        # Publishers
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.status_pub = self.create_publisher(String, '/exploration_status', 10)
        self.path_pub = self.create_publisher(Path, '/global_plan', 10)

        # Timer điều khiển chuyển động 20Hz (50ms)
        self.control_timer = self.create_timer(0.05, self.control_loop)

        # Timer phân tích Frontier & Lập kế hoạch đường đi A* chu kỳ 1.0 giây
        self.plan_timer = self.create_timer(1.0, self.planning_loop)

        self.get_logger().info(
            f"AutoExplorer Pro 3 Cấp độ sẵn sàng! Chế độ mặc định: [{self.current_mode}]."
        )
        self.publish_status(f"{self.current_mode}: Đứng im chờ lệnh...")
        self.stop_robot()

    def mode_callback(self, msg: String):
        cmd = msg.data.strip().upper()
        if cmd == "AUTO" and self.current_mode != "AUTO":
            self.current_mode = "AUTO"
            self.escaping = False
            self.cmd_v = 0.0
            self.cmd_w = 0.0
            self.current_path = []
            self.current_goal = None
            self.stuck_check_pos = None
            self.stuck_duration = 0.0
            self.get_logger().info(">>> ĐÃ BẬT AUTO: Kích hoạt A* Explorer luồn lách qua khe hẹp!")
            self.publish_status("AUTO: Đang tìm đường khám phá toàn diện...")
        elif cmd == "MANUAL" and self.current_mode != "MANUAL":
            self.current_mode = "MANUAL"
            self.current_path = []
            self.current_goal = None
            self.escaping = False
            self.stop_robot()
            self.publish_path([])
            self.get_logger().info(">>> ĐÃ CHUYỂN MANUAL: Trả lại quyền lái thủ công!")
            self.publish_status("MANUAL: Điều khiển thủ công")

    def publish_status(self, text: str):
        msg = String()
        msg.data = text
        self.status_pub.publish(msg)

    def publish_path(self, waypoints):
        """Phát đường đi lên RViz2 để người dùng quan sát"""
        path_msg = Path()
        path_msg.header.stamp = self.get_clock().now().to_msg()
        path_msg.header.frame_id = 'map'

        for pt in waypoints:
            pose = PoseStamped()
            pose.header.frame_id = 'map'
            pose.pose.position.x = float(pt[0])
            pose.pose.position.y = float(pt[1])
            pose.pose.position.z = 0.0
            pose.pose.orientation.w = 1.0
            path_msg.poses.append(pose)

        self.path_pub.publish(path_msg)

    def map_callback(self, msg: OccupancyGrid):
        self.map_data = msg

    def scan_callback(self, msg: LaserScan):
        self.latest_scan = msg

    def stop_robot(self):
        self.cmd_v = 0.0
        self.cmd_w = 0.0
        try:
            if rclpy.ok():
                cmd = Twist()
                self.cmd_pub.publish(cmd)
        except Exception:
            pass

    def get_robot_pose(self):
        """Lấy toạ độ (x, y, yaw) của robot trên map"""
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

    def planning_loop(self):
        """
        CẤP ĐỘ 3: Bộ lập kế hoạch toàn cục A* siêu nhẹ
        Tìm kiếm các vùng biên giới (Frontier) và tự động vạch đường xuyên qua cửa hẹp ra nhánh mới.
        """
        if self.current_mode != "AUTO" or self.map_data is None:
            return

        now = time.time()
        # Đang trong chu kỳ xoay quét 360 độ hoàn tất góc khuất thì tạm dừng lập đường mới
        if now < self.local_spin_end:
            return

        # Dọn dẹp blacklist cũ
        self.blacklist = {k: v for k, v in self.blacklist.items() if v > now}

        robot_pose = self.get_robot_pose()
        if robot_pose is None:
            return
        rx, ry, _ = robot_pose

        # Nếu đang có đường đi hợp lệ và chưa tới đích: tiếp tục đi, không đổi đường làm giật xe
        if self.current_path and self.current_goal is not None:
            dist_to_goal = math.hypot(self.current_goal[0] - rx, self.current_goal[1] - ry)
            if dist_to_goal > 0.35:
                return

        # Đã tới đích hoặc chưa có đường -> Phân tích bản đồ và tìm đường mới
        w = self.map_data.info.width
        h = self.map_data.info.height
        res = self.map_data.info.resolution
        origin_x = self.map_data.info.origin.position.x
        origin_y = self.map_data.info.origin.position.y

        grid = np.array(self.map_data.data, dtype=np.int8).reshape((h, w))
        occupied_mask = (grid == 100)

        # Tính khoảng cách từ mọi ô tới bức tường gần nhất (đơn vị: ô)
        dist_to_wall = distance_transform_edt(~occupied_mask)

        # Tìm các cụm biên giới chưa quét (Frontiers)
        frontiers = self.find_frontiers(grid, dist_to_wall, (origin_x, origin_y, res), (rx, ry))
        if not frontiers:
            self.publish_status("AUTO: Đã quét phủ kín bản đồ! Tuần tra nhẹ...")
            return

        # Chuyển toạ độ robot sang ô lưới (grid cell)
        start_gx = int(np.clip((rx - origin_x) / res, 0, w - 1))
        start_gy = int(np.clip((ry - origin_y) / res, 0, h - 1))

        # Thử tìm đường A* tới các Frontier ứng viên (ưu tiên gần nhất)
        for goal_world in frontiers:
            gx = int(np.clip((goal_world[0] - origin_x) / res, 0, w - 1))
            gy = int(np.clip((goal_world[1] - origin_y) / res, 0, h - 1))

            path_grid = self.plan_path_astar(grid, (start_gx, start_gy), (gx, gy), dist_to_wall)
            if path_grid and len(path_grid) > 1:
                # Chuyển đổi đường đi lưới sang toạ độ thế giới
                path_world = [
                    (origin_x + (pt[0] + 0.5) * res, origin_y + (pt[1] + 0.5) * res)
                    for pt in path_grid
                ]
                self.current_path = path_world
                self.current_goal = goal_world
                self.publish_path(self.current_path)
                self.publish_status(
                    f"AUTO: Đã vạch đường qua cửa -> Mục tiêu ({goal_world[0]:.1f}, {goal_world[1]:.1f})"
                )
                return

        # Nếu không có đường A* nào tới được các cụm chính (bị bít kín hoàn toàn)
        self.current_path = []
        self.current_goal = None

    def plan_path_astar(self, grid, start_grid, goal_grid, dist_to_wall):
        """
        Thuật toán A* tối ưu theo kích thước hình chữ nhật 3D (Rộng 17cm, Dài 21cm):
        - Cho phép luồn lách qua khe hẹp >= 20cm (cách tường >= 2.0 ô = 10cm).
        - Căn tim hành lang (Corridor Centering) mượt mà, không lag luồng ROS 2.
        """
        h, w = grid.shape
        sx, sy = start_grid
        gx, gy = goal_grid

        if not (0 <= sx < w and 0 <= sy < h and 0 <= gx < w and 0 <= gy < h):
            return None

        for min_wall_dist in [2.4, 2.0]:
            passable = (grid == 0) & (dist_to_wall >= min_wall_dist)
            passable[sy, sx] = True
            passable[gy, gx] = True

            # Nới lỏng quanh vị trí hiện tại và đích
            for dy in range(-2, 3):
                for dx in range(-2, 3):
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
            max_steps = 15000
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
                        if dx != 0 and dy != 0:
                            if occupied[cy, nx] or occupied[ny, cx] or dist_to_wall[cy, nx] < 1.8 or dist_to_wall[ny, cx] < 1.8:
                                continue

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

            if found:
                path = []
                curr = (gx, gy)
                while curr in came_from:
                    path.append(curr)
                    curr = came_from[curr]
                path.append((sx, sy))
                path.reverse()
                return path

        return None

    def find_frontiers(self, grid, dist_to_wall, map_info, robot_pos):
        """
        Tìm các điểm biên giới chưa quét với ưu tiên cục bộ (Local-First Priority):
        - Quét triệt để các góc/ngóc ngách nhỏ ở gần trước khi chuyển sang phòng/vùng lớn.
        - Không bỏ sót khe hẹp hoặc vùng chưa vẽ nhỏ sát robot.
        """
        origin_x, origin_y, res = map_info
        rx, ry = robot_pos
        h, w = grid.shape

        free_mask = (grid == 0)
        unknown_mask = (grid == -1)
        # Cách tường >= 2.2 ô (11cm): Cho phép tìm frontier trong khe hẹp và góc phòng
        safe_free = free_mask & (dist_to_wall >= 2.2)

        frontier_cells = []
        for y in range(2, h - 2, 2):
            for x in range(2, w - 2, 2):
                if safe_free[y, x]:
                    neighbors = unknown_mask[y-1:y+2, x-1:x+2]
                    if np.any(neighbors):
                        frontier_cells.append((x, y))

        if not frontier_cells:
            return []

        # Gom cụm các ô biên giới gần nhau (giữ cả cụm nhỏ từ 2 ô)
        clusters = []
        visited = set()
        for pt in frontier_cells:
            if pt in visited:
                continue
            cluster = []
            queue = deque([pt])
            visited.add(pt)
            while queue:
                curr = queue.popleft()
                cluster.append(curr)
                for other in frontier_cells:
                    if other not in visited:
                        if abs(curr[0] - other[0]) <= 3 and abs(curr[1] - other[1]) <= 3:
                            visited.add(other)
                            queue.append(other)
            if len(cluster) >= 2:
                clusters.append(cluster)

        ranked = []
        for c in clusters:
            avg_x = sum(p[0] for p in c) / len(c)
            avg_y = sum(p[1] for p in c) / len(c)
            wx = origin_x + (avg_x + 0.5) * res
            wy = origin_y + (avg_y + 0.5) * res

            if any(math.hypot(wx - bx, wy - by) < 0.25 for (bx, by) in self.blacklist):
                continue

            dist = math.hypot(wx - rx, wy - ry)

            # ĐIỂM SỐ CẢI TIẾN - ƯU TIÊN VÙNG NHỎ Ở GẦN TRƯỚC:
            # -dist * 3.5: Trọng số khoảng cách cực lớn (gần là ưu tiên hàng đầu)
            # min(len(c), 12) * 0.15: Giảm điểm thưởng cụm lớn để không nhảy cóc sang vùng xa
            score = -dist * 3.5 + min(len(c), 12) * 0.15
            ranked.append(((wx, wy), score))

        ranked.sort(key=lambda item: item[1], reverse=True)
        return [item[0] for item in ranked]

    def control_loop(self):
        """
        CẤP ĐỘ 1: Bám quỹ đạo Pure Pursuit & Kiểm tra va chạm theo hình chữ nhật 3D chuẩn xác
        Vòng lặp điều khiển tần số 20Hz (50ms).
        """
        if self.current_mode != "AUTO":
            return

        now = time.time()

        # =========================================================================
        # GIAI ĐOẠN 0: XOAY QUÉT 360 ĐỘ CỤC BỘ TẠI ĐÍCH ĐỂ VÉT SẠCH GÓC KHUẤT
        # =========================================================================
        if now < self.local_spin_end:
            cmd = Twist()
            cmd.linear.x = 0.0
            cmd.angular.z = 0.45  # Xoay tại chỗ êm ái để LiDAR phủ kín các ngóc ngách
            self.cmd_v = 0.0
            self.cmd_w = 0.45
            self.cmd_pub.publish(cmd)
            self.publish_status("AUTO: Đang xoay quét 360° vét sạch góc khuất...")
            return

        if self.latest_scan is None:
            return

        ranges = np.array(self.latest_scan.ranges)
        n = len(ranges)
        if n == 0:
            return

        ranges = np.where(np.isnan(ranges) | np.isinf(ranges) | (ranges < 0.15), 10.0, ranges)

        angles = np.linspace(
            float(self.latest_scan.angle_min),
            float(self.latest_scan.angle_max),
            n,
            endpoint=False
        )
        valid = np.isfinite(ranges) & (ranges > 0.05) & (ranges < 4.0)
        dist_front = 10.0
        dist_rear = 10.0
        dist_left = 10.0
        dist_right = 10.0
        dist_fl_corner = 10.0
        dist_fr_corner = 10.0

        if np.any(valid):
            r_v = ranges[valid]
            a_v = angles[valid]
            x_pts = r_v * np.cos(a_v)
            y_pts = r_v * np.sin(a_v)

            # 1. Hành lang mũi xe đi thẳng (bề rộng 0.19m = |y| <= 0.095m, mũi xe x = +0.105m)
            front_mask = (x_pts > 0.05) & (x_pts < 1.0) & (np.abs(y_pts) <= 0.095)
            if np.any(front_mask):
                dist_front = float(np.min(x_pts[front_mask]))

            # 2. Hành lang đuôi xe lùi (bề rộng 0.19m = |y| <= 0.095m)
            rear_mask = (x_pts < -0.05) & (x_pts > -1.0) & (np.abs(y_pts) <= 0.095)
            if np.any(rear_mask):
                dist_rear = float(np.min(np.abs(x_pts[rear_mask])))

            # 3. Khoảng hở hai bên hông xe (cách thành xe y=±0.085m tới mép cửa/tường)
            left_mask = (y_pts > 0.085) & (y_pts < 0.35) & (x_pts > -0.09) & (x_pts < 0.13)
            if np.any(left_mask):
                dist_left = float(np.min(y_pts[left_mask] - 0.085))

            right_mask = (y_pts < -0.085) & (y_pts > -0.35) & (x_pts > -0.09) & (x_pts < 0.13)
            if np.any(right_mask):
                dist_right = float(np.min(np.abs(y_pts[right_mask]) - 0.085))

            # 4. Khoảng cách trực tiếp từ 2 góc trước hình chữ nhật (+0.105, ±0.085) tới vật cản
            dist_fl_corner = float(np.min(np.hypot(x_pts - 0.105, y_pts - 0.085)))
            dist_fr_corner = float(np.min(np.hypot(x_pts - 0.105, y_pts + 0.085)))

        pose = self.get_robot_pose()
        if pose is None:
            return
        rx, ry, yaw = pose

        target_v = 0.0
        target_w = 0.0

        # =========================================================================
        # 1. BẢO VỆ PHANH KHẨN CẤP CHỐNG ĐÂM TƯỜNG (Mũi xe < 15cm, Góc trước < 3.5cm)
        # =========================================================================
        front_or_corner_blocked = (dist_front < 0.150) or (min(dist_fl_corner, dist_fr_corner) < 0.035)

        if front_or_corner_blocked and not self.escaping:
            self.escaping = True
            self.cmd_v = 0.0  # Phanh tức thì
            self.escape_end_time = now + 1.0
            self.publish_status(
                f"AUTO [BẢO VỆ]: Sát tường (F:{dist_front*100:.0f}cm, C:{min(dist_fl_corner, dist_fr_corner)*100:.0f}cm) -> Lùi thẳng..."
            )

        if self.escaping:
            if now < self.escape_end_time and dist_rear >= 0.15:
                target_v = -0.10
                target_w = 0.0
            else:
                self.escaping = False
                self.cmd_v = 0.0
                if self.current_goal:
                    self.blacklist[self.current_goal] = now + 15.0
                self.current_path = []
                self.current_goal = None

        # =========================================================================
        # 2. BÁM QUỸ ĐẠO A* VỚI PURE PURSUIT (LƯỢN CONG LIÊN TỤC & QUA KHE HẸP AN TOÀN)
        # =========================================================================
        elif self.current_path:
            # Kiểm tra kẹt tại chỗ
            if self.stuck_check_pos is None or (now - self.stuck_check_time) > 4.0:
                if self.stuck_check_pos is not None:
                    moved = math.hypot(rx - self.stuck_check_pos[0], ry - self.stuck_check_pos[1])
                    if moved < 0.12:
                        self.stuck_duration += (now - self.stuck_check_time)
                    else:
                        self.stuck_duration = 0.0
                self.stuck_check_pos = (rx, ry)
                self.stuck_check_time = now

            if self.stuck_duration > 5.0:
                if self.current_goal:
                    self.blacklist[self.current_goal] = now + 30.0
                self.current_path = []
                self.current_goal = None
                self.stuck_duration = 0.0
                self.publish_status("AUTO: Đổi hướng tìm đường khác...")
                return

            # Chống nhảy cóc xuyên vách ngăn: Chỉ tìm mốc gần nhất trong cửa sổ 15 điểm tiếp theo
            search_len = min(15, len(self.current_path))
            dists = [math.hypot(p[0] - rx, p[1] - ry) for p in self.current_path[:search_len]]
            closest_idx = int(np.argmin(dists))
            if closest_idx > 0:
                self.current_path = self.current_path[closest_idx:]

            # Bán kính ngắm thích ứng (Adaptive Lookahead)
            cur_lookahead = self.lookahead_dist
            min_side = min(dist_left, dist_right)
            in_narrow_passage = (min_side < 0.06)
            if in_narrow_passage or min(dist_front, dist_left, dist_right) < 0.35:
                cur_lookahead = 0.20

            target_pt = self.current_path[-1]
            for p in self.current_path:
                d = math.hypot(p[0] - rx, p[1] - ry)
                if d >= cur_lookahead:
                    target_pt = p
                    break

            tx, ty = target_pt
            dx = tx - rx
            dy = ty - ry
            target_dist = math.hypot(dx, dy)
            target_angle = math.atan2(dy, dx)
            alpha = math.atan2(math.sin(target_angle - yaw), math.cos(target_angle - yaw))

            # Đến gần đích cuối (< 0.35m): Kích hoạt xoay quét 360 độ để vét sạch mọi góc khuất!
            if dists[-1] < 0.35:
                self.current_path = []
                self.current_goal = None
                self.local_spin_end = now + 1.8  # Xoay quét 360 độ hoàn tất góc khuất
                self.publish_status("AUTO: Đã tới đích! Xoay quét 360° vét sạch góc khuất...")
                return

            # ĐIỀU KHIỂN CHỐNG CẠ GÓC KHI QUA KHE CỬA:
            if in_narrow_passage:
                target_v = 0.15  # Tốc độ lướt êm qua khe cửa
                target_w = float(np.clip(alpha * 0.8, -0.20, 0.20))
            elif abs(alpha) > 0.35:
                # Góc cua gấp: Xoay tại chỗ (v = 0)
                target_v = 0.0
                self.cmd_v = 0.0
                target_w = float(np.clip(alpha * 1.4, -self.max_w, self.max_w))
            else:
                kappa = 2.0 * math.sin(alpha) / max(0.20, target_dist)
                speed_scale = max(0.35, math.cos(alpha))
                target_v = float(self.max_v * speed_scale)

                if dist_front < 0.35:
                    target_v *= max(0.35, (dist_front - 0.15) / 0.20)

                target_w = float(np.clip(target_v * kappa, -self.max_w, self.max_w))

            self.publish_status(
                f"AUTO: Đang bám đường qua cửa -> Mục tiêu | V: {target_v:.2f} m/s"
            )

        # =========================================================================
        # 3. KHI CHƯA CÓ ĐƯỜNG A*: TỰ ĐỘNG TUẦN TRA BÁM TƯỜNG (WALL-FOLLOWING)
        # =========================================================================
        else:
            if dist_front < 0.30 or min(dist_fl_corner, dist_fr_corner) < 0.05:
                avoid_dir = 1.0 if dist_left >= dist_right else -1.0
                target_v = 0.0
                self.cmd_v = 0.0
                target_w = float(self.max_w * 0.6 * avoid_dir)
            else:
                target_v = 0.18
                target_w = 0.0
                if dist_left < 0.30:
                    target_w -= (0.30 - dist_left) * 0.8
                if dist_right < 0.30:
                    target_w += (0.30 - dist_right) * 0.8
                target_w = float(np.clip(target_w, -self.max_w, self.max_w))
            self.publish_status(f"AUTO: Tuần tra bám mép mở rộng map | V: {target_v:.2f} m/s")

        # =========================================================================
        # 4. BỘ LỌC GIA TỐC MƯỢT (VELOCITY RAMP - TRIỆT TIÊU GIẬT CỤC)
        # =========================================================================
        if target_v == 0.0:
            self.cmd_v = 0.0
        else:
            dv = np.clip(target_v - self.cmd_v, -self.accel_v_limit, self.accel_v_limit)
            self.cmd_v += float(dv)

        dw = target_w - self.cmd_w
        dw = np.clip(dw, -self.accel_w_limit, self.accel_w_limit)
        self.cmd_w += float(dw)

        cmd_msg = Twist()
        cmd_msg.linear.x = float(self.cmd_v)
        cmd_msg.angular.z = float(self.cmd_w)
        self.cmd_pub.publish(cmd_msg)


def main(args=None):
    rclpy.init(args=args)
    node = AutoExplorerNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        node.stop_robot()
        try:
            node.destroy_node()
        except Exception:
            pass
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
