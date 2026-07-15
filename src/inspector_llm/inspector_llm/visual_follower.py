"""
Visual Follower — ROS 2 Node

Proportional controller that tracks a detected target by subscribing to
/detected_target (PoseStamped) and publishing velocity commands on /cmd_vel.

Lifecycle:
  1. Starts following when /detected_target is published
  2. Stops following when:
     - Timeout expires (configurable via parameter)
     - Target lost for >5 seconds
  3. After stopping: navigates back to start position using Nav2

The follow controller does NOT use Nav2 for pursuit (too slow to react).
It directly publishes Twist to /cmd_vel for responsive tracking.
Nav2 is only used for the return-to-start phase after following ends.
"""

import math
import time
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from geometry_msgs.msg import PoseStamped, Twist
from std_msgs.msg import String
from nav2_msgs.action import NavigateToPose
import tf2_ros
from tf2_ros import Buffer, TransformListener


# --- Controller gains ---
KP_ANGULAR = 1.5        # Proportional gain for angular velocity (rad/s per rad error)
KP_LINEAR = 0.5         # Proportional gain for linear velocity (m/s per m error)
DESIRED_DISTANCE = 1.5  # metres — how far to stay from the target
MAX_LINEAR_VEL = 0.4    # m/s — cap forward speed
MAX_ANGULAR_VEL = 1.0   # rad/s — cap turning speed
LOST_TIMEOUT = 5.0      # seconds — stop following if target lost for this long


class VisualFollower(Node):
    """
    Follows a detected target using proportional control on /cmd_vel.

    Subscribes:
      /detected_target   (PoseStamped)  — 3D pose from vision_detector
      /detection_status  (String)       — "tracking" | "lost" | "idle"

    Publishes:
      /cmd_vel           (Twist)        — velocity commands for pursuit

    Parameters:
      follow_timeout     (float): Max seconds to follow. Default: 60.0
      return_to_start    (bool): Navigate back to start after following. Default: true
    """

    def __init__(self):
        super().__init__('visual_follower')

        # --- Parameters ---
        self.declare_parameter('follow_timeout', 60.0)
        self.declare_parameter('return_to_start', True)
        self._timeout = self.get_parameter('follow_timeout').value
        self._return_flag = self.get_parameter('return_to_start').value

        # --- State ---
        self._following = False
        self._follow_start_time = None
        self._last_target_time = None
        self._start_pose = None  # saved when following begins
        self._done = False

        # --- TF ---
        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)

        # --- Subscribers ---
        self.create_subscription(
            PoseStamped, '/detected_target', self._target_cb, 10
        )
        self.create_subscription(
            String, '/detection_status', self._status_cb, 10
        )

        # --- Publishers ---
        self._cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        # --- Nav2 Action Client (for return-to-start) ---
        self._nav_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')

        # --- Control loop at 10 Hz ---
        self._timer = self.create_timer(0.1, self._control_loop)

        self.get_logger().info(
            f'Visual follower ready. Timeout: {self._timeout}s, '
            f'Return to start: {self._return_flag}'
        )

    def _save_start_pose(self):
        """Save current robot pose for return-to-start."""
        try:
            t = self._tf_buffer.lookup_transform('map', 'base_link', rclpy.time.Time())
            self._start_pose = PoseStamped()
            self._start_pose.header.frame_id = 'map'
            self._start_pose.pose.position.x = t.transform.translation.x
            self._start_pose.pose.position.y = t.transform.translation.y
            self._start_pose.pose.position.z = 0.0
            self._start_pose.pose.orientation = t.transform.rotation
            self.get_logger().info(
                f'Start pose saved: ({t.transform.translation.x:.2f}, '
                f'{t.transform.translation.y:.2f})'
            )
        except Exception as e:
            self.get_logger().warn(f'Could not save start pose: {e}')
            self._start_pose = None

    def _target_cb(self, msg: PoseStamped):
        """Receive target pose from vision detector."""
        self._last_target_time = time.time()

        if not self._following and not self._done:
            # Start following
            self._following = True
            self._follow_start_time = time.time()
            self._save_start_pose()
            self.get_logger().info('Target acquired — starting pursuit!')

    def _status_cb(self, msg: String):
        """Track detection status."""
        pass  # We use _last_target_time for timing instead

    def _control_loop(self):
        """Main 10Hz control loop."""
        if self._done or not self._following:
            return

        now = time.time()

        # --- Check timeout ---
        elapsed = now - self._follow_start_time
        if elapsed >= self._timeout:
            self.get_logger().info(
                f'Follow timeout reached ({self._timeout}s). Stopping pursuit.'
            )
            self._stop_following('timeout')
            return

        # --- Check target lost ---
        if self._last_target_time is not None:
            since_last = now - self._last_target_time
            if since_last > LOST_TIMEOUT:
                self.get_logger().info(
                    f'Target lost for {since_last:.1f}s (>{LOST_TIMEOUT}s). '
                    'Stopping pursuit.'
                )
                self._stop_following('target_lost')
                return

        # --- Compute follow command ---
        try:
            # Look up target position relative to robot
            t = self._tf_buffer.lookup_transform(
                'base_link', 'detected_target', rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=0.1)
            )
        except Exception:
            # TF not available — send zero velocity
            self._publish_stop()
            return

        target_x = t.transform.translation.x  # forward distance
        target_y = t.transform.translation.y  # lateral offset

        # Bearing angle to target
        bearing = math.atan2(target_y, target_x)

        # Distance to target
        distance = math.sqrt(target_x ** 2 + target_y ** 2)

        # --- Proportional control ---
        cmd = Twist()

        # Angular: turn toward target
        cmd.angular.z = KP_ANGULAR * bearing
        cmd.angular.z = max(-MAX_ANGULAR_VEL, min(MAX_ANGULAR_VEL, cmd.angular.z))

        # Linear: approach/retreat to maintain desired distance
        distance_error = distance - DESIRED_DISTANCE
        cmd.linear.x = KP_LINEAR * distance_error
        cmd.linear.x = max(-0.1, min(MAX_LINEAR_VEL, cmd.linear.x))  # allow slight reverse

        # If target is very close, don't drive forward
        if distance < 0.8:
            cmd.linear.x = 0.0

        # Log periodically (every ~2 seconds)
        if int(elapsed) % 2 == 0 and int(elapsed * 10) % 20 == 0:
            self.get_logger().info(
                f'Following: dist={distance:.2f}m, bearing={math.degrees(bearing):.1f}°, '
                f'cmd=({cmd.linear.x:.2f}, {cmd.angular.z:.2f}), '
                f'elapsed={elapsed:.0f}/{self._timeout:.0f}s'
            )

        self._cmd_pub.publish(cmd)

    def _publish_stop(self):
        """Send zero velocity."""
        self._cmd_pub.publish(Twist())

    def _stop_following(self, reason: str):
        """Stop pursuit and optionally return to start."""
        self._following = False
        self._done = True
        self._publish_stop()

        self.get_logger().info(f'Pursuit ended. Reason: {reason}')

        if self._return_flag and self._start_pose is not None:
            self.get_logger().info('Returning to start position via Nav2...')
            self._navigate_to_start()
        else:
            self.get_logger().info('Follow complete. No return-to-start requested.')

    def _navigate_to_start(self):
        """Use Nav2 to navigate back to the saved start position."""
        if not self._nav_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error('Nav2 action server not available for return!')
            return

        goal = NavigateToPose.Goal()
        goal.pose = self._start_pose
        goal.pose.header.stamp = self.get_clock().now().to_msg()

        self.get_logger().info(
            f'Sending return goal: ({self._start_pose.pose.position.x:.2f}, '
            f'{self._start_pose.pose.position.y:.2f})'
        )

        future = self._nav_client.send_goal_async(goal)
        future.add_done_callback(self._return_goal_response)

    def _return_goal_response(self, future):
        """Handle Nav2 goal acceptance."""
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error('Return-to-start goal rejected by Nav2!')
            return

        self.get_logger().info('Return-to-start goal accepted. Navigating...')
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self._return_result_cb)

    def _return_result_cb(self, future):
        """Handle Nav2 navigation result."""
        result = future.result().result
        if result:
            self.get_logger().info('Returned to start position successfully!')
        else:
            self.get_logger().warn('Return-to-start navigation failed or was preempted.')


def main(args=None):
    rclpy.init(args=args)
    node = VisualFollower()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
