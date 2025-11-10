#!/usr/bin/env python3
"""
Simple Controller for EvaRobot

This node relays velocity commands from the standard /cmd_vel topic
to the controller's expected topic with proper stamping.

Converts Twist (unstamped) to TwistStamped for the controller.

Author: Kevin Medrano Ayala
License: BSD-3-Clause
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, TwistStamped


class SimpleController(Node):
    def __init__(self):
        super().__init__("simple_controller")

        # Subscriber to standard cmd_vel topic (unstamped)
        self.cmd_vel_sub = self.create_subscription(
            Twist,
            "/cmd_vel",
            self.cmd_vel_callback,
            10
        )

        # Publisher to controller's stamped cmd_vel topic
        self.cmd_vel_pub = self.create_publisher(
            TwistStamped,
            "/evarobot_controller/cmd_vel",
            10
        )

        self.get_logger().info("Simple Controller started")
        self.get_logger().info("Relaying /cmd_vel (Twist) -> /evarobot_controller/cmd_vel (TwistStamped)")

    def cmd_vel_callback(self, msg):
        """
        Callback for cmd_vel messages
        Converts Twist to TwistStamped and forwards to controller
        """
        twist_stamped = TwistStamped()
        twist_stamped.header.stamp = self.get_clock().now().to_msg()
        twist_stamped.header.frame_id = "base_link"
        twist_stamped.twist = msg
        self.cmd_vel_pub.publish(twist_stamped)


def main(args=None):
    rclpy.init(args=args)
    node = SimpleController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
