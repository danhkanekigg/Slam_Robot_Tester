import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess, DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

def launch_setup(context, *args, **kwargs):
    pkg_share = get_package_share_directory('robot_bringup')
    world_name = context.launch_configurations.get('world', 'maze').strip()
    if not world_name.endswith('.sdf'):
        world_name += '.sdf'

    world_file = os.path.join(pkg_share, 'worlds', world_name)
    if not os.path.exists(world_file):
        world_file = os.path.join(pkg_share, 'worlds', 'maze.sdf')

    print(f"\n=======================================================")
    print(f"  >>> GAZEBO DANG KHOI DONG VOI BAN DO: {world_name}")
    print(f"  >>> Duong dan file: {world_file}")
    print(f"=======================================================\n")

    gz_sim = ExecuteProcess(
        cmd=['gz', 'sim', '-r', world_file],
        output='screen'
    )
    return [gz_sim]

def generate_launch_description():
    pkg_share = get_package_share_directory('robot_bringup')
    rviz_config_file = os.path.join(pkg_share, 'rviz', 'slam_view.rviz')
    xacro_file = os.path.join(pkg_share, 'urdf', 'robot_gazebo.urdf.xacro')

    world_arg = DeclareLaunchArgument(
        'world',
        default_value=os.environ.get('SIM_WORLD', 'maze'),
        description='Tên map mô phỏng: maze, office, warehouse, obstacle_course'
    )

    # 2. Node spawn (sinh) robot vào thế giới ảo Gazebo
    spawn_robot = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-name', 'diff_drive_robot',
            '-topic', 'robot_description',
            '-x', '0.0',
            '-y', '0.0',
            '-z', '0.05'
        ],
        output='screen'
    )

    # 3. Cầu nối ros_gz_bridge ánh xạ các topic cốt lõi giữa Gazebo và ROS 2:
    bridge_node = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
            '/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist',
            '/odom@nav_msgs/msg/Odometry[gz.msgs.Odometry',
            '/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan',
            '/imu@sensor_msgs/msg/Imu[gz.msgs.IMU',
            '/tf@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V'
        ],
        parameters=[{
            'qos_overrides./scan.reliability': 'best_effort',
            'qos_overrides./imu.reliability': 'best_effort'
        }],
        output='screen'
    )

    # 4. Robot State Publisher công bố TF và mô hình 3D cho RViz2
    robot_state_pub = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robot_description,
            'use_sim_time': True
        }]
    )

    # 5. Giao diện trực quan hoá RViz2
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_file] if os.path.exists(rviz_config_file) else [],
        parameters=[{'use_sim_time': True}],
        output='screen'
    )

    return LaunchDescription([
        world_arg,
        OpaqueFunction(function=launch_setup),
        spawn_robot,
        bridge_node,
        robot_state_pub,
        rviz_node
    ])
