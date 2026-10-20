import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess, TimerAction, IncludeLaunchDescription, DeclareLaunchArgument, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
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
    nav2_bringup_dir = get_package_share_directory('nav2_bringup')

    slam_params_file = os.path.join(pkg_share, 'config', 'slam_toolbox_params.yaml')
    nav2_params_file = os.path.join(pkg_share, 'config', 'nav2_params.yaml')
    rviz_config_file = os.path.join(pkg_share, 'rviz', 'slam_view.rviz')
    xacro_file = os.path.join(pkg_share, 'urdf', 'robot_gazebo.urdf.xacro')

    world_arg = DeclareLaunchArgument(
        'world',
        default_value=os.environ.get('SIM_WORLD', 'maze'),
        description='Tên map mô phỏng: maze, office, warehouse, obstacle_course'
    )

    scan_fixer_abs = '/home/nvdong/projects/du-an-moi/code_updated/ros2_ws/src/robot_bringup/scripts/scan_frame_fixer.py'
    auto_explorer_abs = '/home/nvdong/projects/du-an-moi/code_updated/ros2_ws/src/robot_bringup/scripts/auto_explorer.py'
    target_navigator_abs = '/home/nvdong/projects/du-an-moi/code_updated/ros2_ws/src/robot_bringup/scripts/target_navigator.py'

    robot_description = ParameterValue(
        Command(['xacro ', xacro_file]),
        value_type=str
    )

    # 2. Spawn robot (đặt robot sát mặt đất z=0.005m để tiếp xúc êm ái, không bị cấn sàn khi khởi động)
    spawn_robot = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-name', 'diff_drive_robot',
            '-topic', 'robot_description',
            '-x', '0.0', '-y', '0.0', '-z', '0.005'
        ],
        output='screen'
    )

    # 3. Bridge Gazebo <-> ROS2
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

    # 4. Robot State Publisher
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

    # 5a. Scan Frame Fixer & Filter (< 0.15m)
    scan_frame_fixer = ExecuteProcess(
        cmd=['python3', scan_fixer_abs, '--ros-args', '-p', 'use_sim_time:=true'],
        output='screen'
    )

    # 5b. SLAM Toolbox
    slam_node = Node(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',
        parameters=[
            slam_params_file,
            {'use_sim_time': True}
        ]
    )

    # 5c. Lifecycle Manager cho SLAM
    lifecycle_manager_slam = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_slam',
        output='screen',
        parameters=[
            {'use_sim_time': True},
            {'autostart': True},
            {'node_names': ['slam_toolbox']}
        ]
    )

    # 5e. Auto Explorer Node (Frontier Exploration tự động tìm vùng chưa quét & né tường mượt mà)
    auto_explorer = ExecuteProcess(
        cmd=['python3', auto_explorer_abs, '--ros-args', '-p', 'use_sim_time:=true'],
        output='screen'
    )

    # 5f. Target Navigator Node (Point-to-Point Navigation điều hướng điểm chỉ định RViz2)
    target_navigator = ExecuteProcess(
        cmd=['python3', target_navigator_abs, '--ros-args', '-p', 'use_sim_time:=true'],
        output='screen'
    )

    # 6. RViz2
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
        scan_frame_fixer,
        TimerAction(               # T=3s: Bật SLAM Toolbox
            period=3.0,
            actions=[slam_node]
        ),
        TimerAction(               # T=4s: Kích hoạt SLAM Lifecycle
            period=4.0,
            actions=[lifecycle_manager_slam]
        ),
        TimerAction(               # T=5s: Khởi động Auto Explorer Node
            period=5.0,
            actions=[auto_explorer]
        ),
        TimerAction(               # T=6s: Khởi động Target Navigator Node (lắng nghe /goal_pose)
            period=6.0,
            actions=[target_navigator]
        ),
        TimerAction(               # T=7s: Mở RViz2
            period=7.0,
            actions=[rviz_node]
        ),
    ])
