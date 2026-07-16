"""
Target Mover — ROS 2 Node

Publishes continuous velocity commands on /red_target/cmd_vel to make the
red box target drive in circles or loop around, simulating a moving target.
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist


class TargetMover(Node):

    def __init__(self):
        super().__init__('target_mover')
        self._pub = self.create_publisher(Twist, '/red_target/cmd_vel', 10)
        self._timer = self.create_timer(0.1, self._timer_cb)
        self.get_logger().info('Target Mover node started. Publishing to /red_target/cmd_vel...')

    def _timer_cb(self):
        msg = Twist()
        msg.linear.x = 0.25   # m/s
        msg.angular.z = 0.35  # rad/s
        self._pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = TargetMover()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
