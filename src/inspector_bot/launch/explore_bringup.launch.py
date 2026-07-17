import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    # 1. Package paths
    inspector_pkg = get_package_share_directory('inspector_bot')
    nav2_pkg = get_package_share_directory('nav2_bringup')

    # Config files
    nav2_params_file = os.path.join(inspector_pkg, 'config', 'nav2_params.yaml')
    slam_params_file = os.path.join(inspector_pkg, 'config', 'slam_params.yaml')
    rviz_config_file = os.path.join(inspector_pkg, 'rviz', 'production.rviz')

    # 2. Gazebo + Robot spawn (reuse existing sim launch)
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(inspector_pkg, 'launch', 'sim_robot.launch.py')
        )
    )

    # 3. Nav2 bringup in SLAM mode
    # When slam:=True, bringup_launch.py launches slam_toolbox
    # instead of map_server + amcl. No static map needed.
    nav2_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav2_pkg, 'launch', 'bringup_launch.py')
        ),
        launch_arguments={
            'use_sim_time': 'true',
            'slam': 'True',
            'params_file': nav2_params_file,
        }.items()
    )

    # 4. Twist stamper bridge
    twist_stamper_node = Node(
        package='twist_stamper',
        executable='twist_stamper',
        parameters=[{'use_sim_time': True}],
        remappings=[
            ('cmd_vel_in', '/cmd_vel'),
            ('cmd_vel_out', '/diff_drive_controller/cmd_vel')
        ]
    )

    # 5. RViz
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_file],
        parameters=[{'use_sim_time': True}]
    )

    # 6. Orchestration
    # Give Gazebo 15s to boot before starting Nav2 + SLAM
    delayed_nav_and_slam = TimerAction(
        period=15.0,
        actions=[
            nav2_launch,
            twist_stamper_node,
            rviz_node,
        ]
    )

    return LaunchDescription([
        gazebo_launch,          # Fires immediately
        delayed_nav_and_slam,   # Fires after 15 seconds
    ])
