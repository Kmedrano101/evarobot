#!/usr/bin/env python3
"""
Joystick Teleop Node for EvaRobot

Translates joystick inputs to cmd_vel Twist messages
for differential drive robot control.

Fixed QoS compatibility for simulation use.

Author: Kevin Medrano Ayala
License: BSD-3-Clause
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Joy


class JoyTeleopNode(Node):
    """
    ROS2 node for joystick teleoperation of differential drive robot
    """

    def __init__(self):
        super().__init__('joy_teleop_node')

        # Declare parameters
        self.declare_parameter('linear_speed', 0.5)   # m/s
        self.declare_parameter('angular_speed', 1.5)  # rad/s
        self.declare_parameter('speed_increment', 0.1)
        self.declare_parameter('joy_deadzone', 0.1)
        self.declare_parameter('cmd_vel_topic', '/cmd_vel')
        self.declare_parameter('joy_topic', '/joy')

        # Get parameters
        self.linear_speed = self.get_parameter('linear_speed').value
        self.angular_speed = self.get_parameter('angular_speed').value
        self.speed_increment = self.get_parameter('speed_increment').value
        self.joy_deadzone = self.get_parameter('joy_deadzone').value
        cmd_vel_topic = self.get_parameter('cmd_vel_topic').value
        joy_topic = self.get_parameter('joy_topic').value

        # QoS Profiles
        # Use RELIABLE for cmd_vel to match simple_controller subscription
        cmd_vel_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10
        )

        # Use BEST_EFFORT for joy (typical for sensor data)
        joy_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10
        )

        # Publishers and Subscribers
        self.cmd_vel_pub = self.create_publisher(
            Twist,
            cmd_vel_topic,
            cmd_vel_qos
        )

        self.joy_sub = self.create_subscription(
            Joy,
            joy_topic,
            self.joy_callback,
            joy_qos
        )

        # Current velocity state
        self.current_linear = 0.0
        self.current_angular = 0.0

        # Joystick state tracking
        self.last_axes = []
        self.last_buttons = []
        self.last_speed_adjust_time = self.get_clock().now()

        self.get_logger().info('='*60)
        self.get_logger().info('Joy Teleop Node Started')
        self.get_logger().info('='*60)
        self.get_logger().info(f'Publishing to: {cmd_vel_topic}')
        self.get_logger().info(f'Subscribing to: {joy_topic}')
        self.get_logger().info(f'Linear speed: {self.linear_speed:.2f} m/s')
        self.get_logger().info(f'Angular speed: {self.angular_speed:.2f} rad/s')
        self.get_logger().info('='*60)
        self.get_logger().info('PS4 Controller Mapping:')
        self.get_logger().info('  Left Stick X-axis  -> Rotate Left/Right')
        self.get_logger().info('  Right Stick Y-axis -> Forward/Backward')
        self.get_logger().info('  D-pad Up           -> Increase Speed')
        self.get_logger().info('  D-pad Down         -> Decrease Speed')
        self.get_logger().info('='*60)

    def apply_deadzone(self, value, deadzone=0.1):
        """Apply deadzone to joystick axis value"""
        if abs(value) < deadzone:
            return 0.0
        # Scale the remaining range
        sign = 1 if value > 0 else -1
        return sign * (abs(value) - deadzone) / (1.0 - deadzone)

    def joy_callback(self, msg):
        """Process joystick input"""
        # Check if input changed (avoid redundant publishing)
        if (msg.axes == self.last_axes and
            msg.buttons == self.last_buttons):
            return

        self.last_axes = msg.axes[:]
        self.last_buttons = msg.buttons[:]

        # PS4 Controller mapping:
        # axes[0] = Left stick X-axis  (rotation left/right)
        # axes[4] = Right stick Y-axis (forward/backward)
        # axes[7] = D-pad Y-axis (speed adjustment)

        if len(msg.axes) < 5:
            self.get_logger().warn('Not enough joystick axes detected')
            return

        # Read joystick axes with deadzone
        linear_axis = self.apply_deadzone(msg.axes[4], self.joy_deadzone)    # Right stick Y (up/down)
        angular_axis = self.apply_deadzone(msg.axes[0], self.joy_deadzone)   # Left stick X (left/right)

        # Speed adjustment with D-pad (prevent rapid adjustments)
        current_time = self.get_clock().now()
        time_since_last = (current_time - self.last_speed_adjust_time).nanoseconds / 1e9

        if len(msg.axes) > 7 and time_since_last > 0.3:  # 300ms debounce
            if msg.axes[7] == 1.0:  # D-pad up
                self.linear_speed += self.speed_increment
                self.angular_speed += self.speed_increment * 0.5
                self.last_speed_adjust_time = current_time
                self.get_logger().info(
                    f'Speed increased -> Linear: {self.linear_speed:.2f} m/s, '
                    f'Angular: {self.angular_speed:.2f} rad/s'
                )
            elif msg.axes[7] == -1.0:  # D-pad down
                self.linear_speed = max(0.1, self.linear_speed - self.speed_increment)
                self.angular_speed = max(0.2, self.angular_speed - self.speed_increment * 0.5)
                self.last_speed_adjust_time = current_time
                self.get_logger().info(
                    f'Speed decreased -> Linear: {self.linear_speed:.2f} m/s, '
                    f'Angular: {self.angular_speed:.2f} rad/s'
                )

        # Calculate velocities
        self.current_linear = linear_axis * self.linear_speed
        self.current_angular = angular_axis * self.angular_speed

        # Publish
        self.publish_twist()

    def publish_twist(self):
        """Publish current velocity as Twist message"""
        twist = Twist()
        twist.linear.x = self.current_linear
        twist.linear.y = 0.0
        twist.linear.z = 0.0
        twist.angular.x = 0.0
        twist.angular.y = 0.0
        twist.angular.z = self.current_angular

        self.cmd_vel_pub.publish(twist)


def main(args=None):
    """Main entry point"""
    rclpy.init(args=args)

    try:
        node = JoyTeleopNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
