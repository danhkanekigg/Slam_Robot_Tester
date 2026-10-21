#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Scan Frame Fixer & Filter
-------------------------
1. Gazebo Harmonic đặt frame_id của LaserScan theo dạng:
     'diff_drive_robot/base_footprint/ld14_lidar'
   Có dấu '/' gây lỗi TF2 -> Chuẩn hóa về 'laser_frame'.
2. Lọc bỏ các tia phản xạ ngược vào thân xe (< 0.15m) để tránh làm méo SLAM.
3. Publish sang topic /scan_relay để SLAM Toolbox sử dụng.
"""
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import LaserScan
import math

class ScanFrameFixer(Node):
    def __init__(self):
        super().__init__('scan_frame_fixer')
        if not self.has_parameter('use_sim_time'):
            self.declare_parameter('use_sim_time', True)
        qos_sub = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )
        qos_pub = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )
        self.pub = self.create_publisher(LaserScan, '/scan_relay', qos_pub)
        self.sub = self.create_subscription(
            LaserScan, '/scan', self.callback, qos_sub
        )
        self.get_logger().info(
            "ScanFrameFixer & Filter sẵn sàng: /scan -> /scan_relay (frame_id: 'laser_frame', lọc vỏ xe < 0.15m)"
        )

    def callback(self, msg: LaserScan):
        msg.header.frame_id = 'laser_frame'
        
        # Lọc các điểm đo < 0.15m (phản xạ vào thân xe, cọc đồng, pin)
        ranges = list(msg.ranges)
        min_valid = 0.15
        for i, r in enumerate(ranges):
            if math.isfinite(r) and r < min_valid:
                ranges[i] = float('inf')
        msg.ranges = ranges
        
        self.pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = ScanFrameFixer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
