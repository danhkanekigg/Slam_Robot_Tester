import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    pkg_share = get_package_share_directory('robot_bringup')
    slam_params_file = os.path.join(pkg_share, 'config', 'slam_toolbox_params.yaml')
    rviz_config_file = os.path.join(pkg_share, 'rviz', 'slam_view.rviz')

    urdf_file = os.path.join(pkg_share, 'urdf', 'robot.urdf')
    with open(urdf_file, 'r', encoding='utf-8') as f:
        robot_desc = f.read()

    # 1. Node cầu nối UDP giữa ESP32 và ROS 2
    udp_bridge_node = Node(
        package='robot_bringup',
        executable='robot_udp_bridge',
        name='robot_udp_bridge_node',
        output='screen',
        parameters=[{
            'lidar_port': 8888,
            'robot_port': 8889,
            'esp32_ip': '192.168.1.105', # Cập nhật IP ESP32 của bạn
            'esp32_cmd_port': 8889,
            'odom_frame': 'odom',
            'base_frame': 'base_footprint',
            'laser_frame': 'laser_frame'
        }]
    )

    # 2. Robot State Publisher (Công bố mô hình 3D: 2 bánh chủ động + 1 bánh tự do + LiDAR + IMU)
    robot_state_pub_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_desc}]
    )

    # 3. Thuật toán SLAM Toolbox (Tạo bản đồ 2D thời gian thực)
    slam_node = Node(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',
        parameters=[slam_params_file]
    )

    # 4. Giao diện trực quan hoá RViz2
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_file] if os.path.exists(rviz_config_file) else [],
        output='screen'
    )

    return LaunchDescription([
        udp_bridge_node,
        robot_state_pub_node,
        slam_node,
        rviz_node
    ])

