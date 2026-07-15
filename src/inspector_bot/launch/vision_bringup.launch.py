"""
Vision Bringup Launch File

Launches the full stack for vision target detection and following:
  1. Gazebo + Robot spawn (via sim_robot.launch.py)
  2. Nav2 (for return-to-start navigation)
  3. Twist stamper (cmd_vel bridge)
  4. RViz with camera visualization
  5. Red target model spawn (moving target for detection)

Usage:
  ros2 launch inspector_bot vision_bringup.launch.py
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    inspector_pkg = get_package_share_directory('inspector_bot')
    nav2_pkg = get_package_share_directory('nav2_bringup')

    map_file = os.path.join(inspector_pkg, 'maps', 'warehouse_map.yaml')
    rviz_config_file = os.path.join(inspector_pkg, 'rviz', 'production.rviz')
    nav2_params_file = os.path.join(inspector_pkg, 'config', 'nav2_params.yaml')

    # Path to the red target model
    red_target_sdf = os.path.join(inspector_pkg, 'models', 'red_target', 'model.sdf')

    # --- 1. Boot Gazebo & Spawn Robot ---
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(inspector_pkg, 'launch', 'sim_robot.launch.py')
        )
    )

    # --- 2. Nav2 (for return-to-start) ---
    nav2_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav2_pkg, 'launch', 'bringup_launch.py')
        ),
        launch_arguments={
            'use_sim_time': 'true',
            'map': map_file,
            'params_file': nav2_params_file,
        }.items()
    )

    # --- 3. Twist Stamper ---
    twist_stamper_node = Node(
        package='twist_stamper',
        executable='twist_stamper',
        parameters=[{'use_sim_time': True}],
        remappings=[
            ('cmd_vel_in', '/cmd_vel'),
            ('cmd_vel_out', '/diff_drive_controller/cmd_vel'),
        ]
    )

    # --- 4. RViz ---
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_file],
        parameters=[{'use_sim_time': True}],
    )

    # --- 5. Spawn red target after Gazebo loads ---
    spawn_red_target = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-name', 'red_target',
            '-file', red_target_sdf,
            '-x', '2.0',
            '-y', '1.0',
            '-z', '0.1',
            '-Y', '0.0'
        ],
        output='screen'
    )

    # --- 6. Orchestration ---
    # Delay spawning the target and loading Nav2 to give Gazebo time to boot.
    # Spawning the target after 8 seconds, and Nav2 after 15 seconds.
    delayed_target_spawn = TimerAction(
        period=8.0,
        actions=[spawn_red_target],
    )

    delayed_navigation = TimerAction(
        period=15.0,
        actions=[nav2_launch, twist_stamper_node, rviz_node],
    )

    return LaunchDescription([
        gazebo_launch,
        delayed_target_spawn,
        delayed_navigation,
    ])
