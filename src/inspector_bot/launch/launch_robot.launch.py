import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, RegisterEventHandler
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node
from launch.event_handlers import OnProcessStart

def generate_launch_description():
    # 1. Locate the package install directory
    pkg_path = get_package_share_directory('inspector_bot')

    # 2. Process the Xacro file into a raw URDF string programmatically
    xacro_file = os.path.join(pkg_path, 'urdf', 'inspector_bot.urdf.xacro')
    robot_description_raw = Command(['xacro ', xacro_file])

    # 3. Define the Robot State Publisher Node
    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description_raw}]
    )

    # 4. Define the ros2_control Controller Manager Node
    controller_manager_yaml = os.path.join(pkg_path, 'config', 'controllers.yaml')
    node_controller_manager = Node(
        package='controller_manager',
        executable='ros2_control_node',
        parameters=[controller_manager_yaml],
        output='screen',
        remappings=[('/controller_manager/robot_description', '/robot_description')]
    )

    # 5. Define the Joint State Broadcaster Spawner Node
    spawn_joint_state_broadcaster = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster'],
        output='screen'
    )

    # 6. Define the Differential Drive Controller Spawner Node
    spawn_diff_drive_controller = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['diff_drive_controller'],
        output='screen'
    )

    # 7. Production Event Handling: Delay controller spawning until the manager is active
    delay_broadcaster_after_manager = RegisterEventHandler(
        event_handler=OnProcessStart(
            target_action=node_controller_manager,
            on_start=[spawn_joint_state_broadcaster]
        )
    )

    delay_diff_drive_after_manager = RegisterEventHandler(
        event_handler=OnProcessStart(
            target_action=node_controller_manager,
            on_start=[spawn_diff_drive_controller]
        )
    )

    # Assemble the final launch system execution list
    return LaunchDescription([
        node_robot_state_publisher,
        node_controller_manager,
        delay_broadcaster_after_manager,
        delay_diff_drive_after_manager
    ])