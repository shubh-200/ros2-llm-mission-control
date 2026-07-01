import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSReliabilityPolicy

from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped

import time 
import math

from inspector_llm.mission_validator import load_map_metadata, is_within_map_bounds, CostmapValidator, validate_waypoints

MAP_YAML = '/home/shubham/omokai_ws/install/inspector_bot/share/inspector_bot/maps/warehouse_map.yaml'

def publish_initial_pose(node):
    pub = node.create_publisher(
        PoseWithCovarianceStamped,
        '/initialpose',
        QoSProfile(reliability=QoSReliabilityPolicy.RELIABLE, depth=10)
    )

    msg = PoseWithCovarianceStamped()
    msg.header.frame_id = 'map'
    msg.header.stamp = node.get_clock().now().to_msg()

    # Robot spawns at x=0, y=0, facing forward (yaw=0)
    msg.pose.pose.position.x = 0.0
    msg.pose.pose.position.y = 0.0
    msg.pose.pose.position.z = 0.0
    msg.pose.pose.orientation.w = 1.0  # yaw=0 → quaternion (0,0,0,1)

    # Covariance: AMCL needs some initial uncertainty — these are standard values
    msg.pose.covariance[0]  = 0.25   # xx
    msg.pose.covariance[7]  = 0.25   # yy
    msg.pose.covariance[35] = 0.07   # yaw

    # Publish a few times — /initialpose uses BEST_EFFORT and can be dropped
    time.sleep(1.0)  # wait for subscriber to connect
    for _ in range(5):
        pub.publish(msg)
        time.sleep(0.3)

    node.get_logger().info('Initial pose published.')

def navigate_to_waypoint(nav_client, node, x, y, yaw):
    # Build the goal
    goal_msg = NavigateToPose.Goal()
    goal_msg.pose = PoseStamped()
    goal_msg.pose.header.frame_id = 'map'
    goal_msg.pose.header.stamp = node.get_clock().now().to_msg()

    goal_msg.pose.pose.position.x = x
    goal_msg.pose.pose.position.y = y
    goal_msg.pose.pose.position.z = 0.0

    # Convert yaw (radians) to quaternion — only z and w needed for 2D
    goal_msg.pose.pose.orientation.z = math.sin(yaw / 2.0)
    goal_msg.pose.pose.orientation.w = math.cos(yaw / 2.0)

    node.get_logger().info(f'Sending goal → x={x}, y={y}, yaw={yaw:.2f}')

    # Send goal and wait for acceptance
    send_goal_future = nav_client.send_goal_async(goal_msg)
    rclpy.spin_until_future_complete(node, send_goal_future)

    goal_handle = send_goal_future.result()
    if not goal_handle.accepted:
        node.get_logger().error('Goal rejected by Nav2!')
        return False

    node.get_logger().info('Goal accepted. Robot is moving...')

    # Wait for the robot to reach the goal
    result_future = goal_handle.get_result_async()
    rclpy.spin_until_future_complete(node, result_future)

    node.get_logger().info('Waypoint reached.')
    return True

def execute_mission(mission: dict):
    rclpy.init()
    node = Node('mission_executor')
     # --- 1. Load static map metadata (instant, no ROS needed) ---
    map_meta = load_map_metadata(MAP_YAML)
    # --- 2. Create costmap subscriber (just registers it, doesn't wait yet) ---
    costmap_validator = CostmapValidator(node)
    # --- 3. Set up Nav2 action client (just creates the client object) ---
    nav_client = ActionClient(node, NavigateToPose, '/navigate_to_pose')
    # --- 4. Publish initial pose FIRST → AMCL starts localizing → 
    #        map→odom TF becomes valid → costmap can build properly ---
    publish_initial_pose(node)
    # --- 5. Now wait for Nav2 action server ---
    node.get_logger().info('Waiting for Nav2...')
    nav_client.wait_for_server()
    # --- 6. NOW wait for costmap — transforms are valid, it arrives quickly ---
    node.get_logger().info('Waiting for costmap...')
    costmap_validator.wait_for_costmap(node)
    node.get_logger().info('Costmap received.')
    # --- 7. Validate all waypoints before robot moves ---
    try:
        validate_waypoints(mission['waypoints'], map_meta, costmap_validator)
        node.get_logger().info('All waypoints validated. Safe to execute.')
    except ValueError as e:
        node.get_logger().error(f'Mission validation FAILED: {e}')
        node.destroy_node()
        rclpy.shutdown()
        return
    # --- 8. Execute waypoints ---
    waypoints  = mission['waypoints']
    loop_count = mission.get('loop_count', 1)
    for loop in range(loop_count):
        node.get_logger().info(f'--- Loop {loop + 1} of {loop_count} ---')
        for wp in waypoints:
            success = navigate_to_waypoint(nav_client, node, wp['x'], wp['y'], wp.get('yaw', 0.0))
            if not success:
                node.get_logger().error('Mission aborted.')
                node.destroy_node()
                rclpy.shutdown()
                return
    node.get_logger().info('Mission complete.')
    node.destroy_node()
    rclpy.shutdown()
def main():
    test_mission = {
        "loop_count": 1,
        "waypoints": [
            {"x":  -0.838, "y": -3.99, "yaw": 0.0},
            {"x":  2.0, "y": -1.0, "yaw": 3.14},
            {"x":  0.0, "y":  0.0, "yaw": 0.0},
        ]
    }
    execute_mission(test_mission)
if __name__ == '__main__':
    main()