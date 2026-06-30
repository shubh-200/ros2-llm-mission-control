import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node, LifecycleNode

def generate_launch_description():
    # --- 1. Define Package Paths ---
    inspector_pkg = get_package_share_directory('inspector_bot')
    nav2_pkg = get_package_share_directory('nav2_bringup')
    
    # Absolute paths for configuration files
    map_file = os.path.join(inspector_pkg, 'maps', 'warehouse_map.yaml') # Update with your actual map name if different
    rviz_config_file = os.path.join(inspector_pkg, 'rviz', 'production.rviz')
    nav2_params_file = os.path.join(inspector_pkg, 'config', 'nav2_params.yaml')

    # --- 2. Define the Launch Actions ---
    
    # Action A: Boot Gazebo & Spawn Robot (Terminal 1)
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(inspector_pkg, 'launch', 'sim_robot.launch.py'))
    )

    # Action B: Nav2 Autonomy Brain (Terminal 2)
    nav2_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(nav2_pkg, 'launch', 'bringup_launch.py')),
        launch_arguments={'use_sim_time': 'true', 'map': map_file, 'params_file': nav2_params_file}.items()
    )

    # Action C: Hardware Bridge (Terminal 3)
    twist_stamper_node = Node(
        package='twist_stamper',
        executable='twist_stamper',
        parameters=[{'use_sim_time': True}],
        remappings=[
            ('cmd_vel_in', '/cmd_vel'),
            ('cmd_vel_out', '/diff_drive_controller/cmd_vel')
        ]
    )

    # Action D: The Custom RViz Dashboard (Terminal 4)
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_file], # Loads your frozen UI state
        parameters=[{'use_sim_time': True}]
    )

    # # Action E: 3D Vision Microservice (Terminal 5)
    # vision_node = LifecycleNode(
    #     package='inspector_vision',
    #     executable='target_locator',
    #     name='target_locator',
    #     namespace='',
    #     parameters=[{'use_sim_time': True}]
    # )

    # # Action F: Vision Lifecycle Manager
    # vision_lifecycle_manager = Node(
    #     package='nav2_lifecycle_manager',
    #     executable='lifecycle_manager',
    #     name='lifecycle_manager_vision',
    #     output='screen',
    #     parameters=[
    #         {'use_sim_time': True},
    #         {'autostart': True},
    #         {'node_names': ['target_locator']},
    #         {'bond_timeout': 0.0}
    #     ]
    # )

    # --- 3. The Orchestration (Handling the Race Condition) ---
    # We must give Gazebo 12 seconds to load the physics engine before booting the vision microservice,
    # and then another 3 seconds for the vision node to configure & activate before booting navigation.
    # delayed_vision = TimerAction(
    #     period=12.0,
    #     actions=[vision_node, vision_lifecycle_manager]
    # )

    delayed_navigation = TimerAction(
        period=15.0,
        actions=[nav2_launch, twist_stamper_node, rviz_node]
    )

    # --- 4. Execute ---
    return LaunchDescription([
        gazebo_launch,          # Fires immediately
        # delayed_vision,         # Fires after 12 seconds
        delayed_navigation      # Fires after 15 seconds
    ])