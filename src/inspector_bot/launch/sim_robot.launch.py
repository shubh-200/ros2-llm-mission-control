import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.substitutions import Command

def generate_launch_description():
    pkg_path = get_package_share_directory('inspector_bot')
    box_sdf_path = os.path.join(pkg_path, 'models', 'cargo_box', 'model.sdf')

    # 1. Parse the URDF/Xacro file
    xacro_file = os.path.join(pkg_path, 'urdf', 'inspector_bot.urdf.xacro')
    robot_description_raw = Command(['xacro ', xacro_file])

    # 2. Robot State Publisher (CRITICAL: use_sim_time must be true in simulation)
    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robot_description_raw,
            'use_sim_time': True
        }]
    )

    # 3. Launch Gazebo Harmonic (gz_sim) with an empty world
    gazebo_pkg = get_package_share_directory('ros_gz_sim')
    
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(gazebo_pkg, 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={'gz_args': '-r https://fuel.gazebosim.org/1.0/MovAi/worlds/tugbot_warehouse'}.items()
    )

    # 4. Spawn the robot entity into the Gazebo world
    spawn_entity = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-topic', 'robot_description',
            '-name', 'inspector_bot',
            '-z', '0.2'  # Drop it 20cm above the floor so it falls naturally
        ],
        output='screen'
    )

    # 5. Bridge ROS 2 and Gazebo (Clock and LiDAR)
    # This translates Gazebo's internal messages into standard ROS2 messages
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
            '/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan',
            '/camera/image@sensor_msgs/msg/Image[gz.msgs.Image',
            '/camera/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo',
            '/camera/points@sensor_msgs/msg/PointCloud2[gz.msgs.PointCloudPacked'
        ],
        output='screen'
    )

    # 6. Spawn the Controllers
    # cmd_vel_relay = Node(
    #     package='topic_tools',
    #     executable='relay',
    #     arguments=['/cmd_vel', '/diff_drive_controller/cmd_vel'],
    #     output='screen'
    # )
    
    spawn_broadcaster = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster', '--controller-manager', '/controller_manager'],
    )

    spawn_diff_drive = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['diff_drive_controller', '--controller-manager', '/controller_manager'],
    )

    # 7. Dynamically Spawn the AprilTag Target for 6-DoF Localization
    spawn_tag = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-name', 'misplaced_cargo',
            '-file', box_sdf_path,
            '-x', '-7.3',
            '-y', '8.3',
            '-z', '0.0',
            '-Y', '3.14159'
        ],
        output='screen'
    )
    
    # 8. Delay the spawner to give Gazebo time to boot
    delayed_spawn = TimerAction(
        period=12.0, # Wait 12 seconds before executing the spawn node
        actions=[spawn_tag]
    )
    
    return LaunchDescription([
        node_robot_state_publisher,
        gazebo_launch,
        spawn_entity,
        bridge,
        spawn_broadcaster,
        spawn_diff_drive,
        delayed_spawn
        # cmd_vel_relay
    ])