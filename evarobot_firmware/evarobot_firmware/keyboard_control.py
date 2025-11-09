#!/usr/bin/env python3
"""
Joystick/Keyboard Teleop Node for EvaRobot

Translates joystick and keyboard inputs to cmd_vel Twist messages
for differential drive robot control.

Author: Kevin Medrano Ayala
Contact: kevin.ejem18@gmail.com
License: BSD-3-Clause
"""

import sys
import threading
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Joy
import termios
import tty


# ============================================================================
# KEYBOARD BINDINGS
# ============================================================================

HELP_MSG = """
EvaRobot Keyboard/Joystick Control
----------------------------------
Keyboard Controls:
  w/s : Increase/decrease linear velocity
  a/d : Increase/decrease angular velocity
  Space : Emergency stop (zero all velocities)
  q/e : Increase/decrease speed multiplier
  x : Quit

Joystick Controls:
  Left Stick Y-axis  : Linear velocity (forward/backward)
  Right Stick X-axis : Angular velocity (rotation)
  D-pad Up/Down      : Increase/decrease speed multiplier

Current Settings:
  Linear velocity : ±{linear:.2f} m/s
  Angular velocity: ±{angular:.2f} rad/s
"""

# Keyboard key bindings for differential drive
KEY_BINDINGS = {
    'w': (1, 0),    # Forward
    's': (-1, 0),   # Backward
    'a': (0, 1),    # Turn left
    'd': (0, -1),   # Turn right
    ' ': (0, 0),    # Stop
}


# ============================================================================
# TERMINAL CONTROL FUNCTIONS
# ============================================================================

def get_key(settings):
    """Read a single key press from terminal"""
    tty.setraw(sys.stdin.fileno())
    key = sys.stdin.read(1)
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key


def save_terminal_settings():
    """Save current terminal settings"""
    return termios.tcgetattr(sys.stdin)


def restore_terminal_settings(old_settings):
    """Restore terminal to previous settings"""
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_settings)


# ============================================================================
# JOYSTICK CONTROL NODE
# ============================================================================

class JoyControlNode(Node):
    """
    ROS2 node for joystick and keyboard teleoperation of differential drive robot
    """

    def __init__(self):
        super().__init__('joy_control_node')

        # Declare parameters
        self.declare_parameter('linear_speed', 0.5)   # m/s
        self.declare_parameter('angular_speed', 1.0)  # rad/s
        self.declare_parameter('speed_increment', 0.1)
        self.declare_parameter('joy_deadzone', 0.1)
        self.declare_parameter('cmd_vel_topic', 'cmd_vel')
        self.declare_parameter('joy_topic', 'joy')

        # Get parameters
        self.linear_speed = self.get_parameter('linear_speed').value
        self.angular_speed = self.get_parameter('angular_speed').value
        self.speed_increment = self.get_parameter('speed_increment').value
        self.joy_deadzone = self.get_parameter('joy_deadzone').value
        cmd_vel_topic = self.get_parameter('cmd_vel_topic').value
        joy_topic = self.get_parameter('joy_topic').value

        # QoS Profiles
        qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10
        )

        # Publishers and Subscribers
        self.cmd_vel_pub = self.create_publisher(
            Twist,
            cmd_vel_topic,
            qos_profile
        )

        self.joy_sub = self.create_subscription(
            Joy,
            joy_topic,
            self.joy_callback,
            qos_profile
        )

        # Current velocity state
        self.current_linear = 0.0
        self.current_angular = 0.0

        # Joystick state tracking
        self.last_axes = []
        self.last_buttons = []

        # Threading for ROS2 spinning
        self.spin_thread = threading.Thread(
            target=rclpy.spin,
            args=(self,),
            daemon=True
        )
        self.spin_thread.start()

        self.get_logger().info('Joy Control Node Started')
        self.get_logger().info(f'Publishing to: {cmd_vel_topic}')
        self.get_logger().info(f'Linear speed: {self.linear_speed} m/s')
        self.get_logger().info(f'Angular speed: {self.angular_speed} rad/s')

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
        # Left stick Y-axis  (axes[1]) -> Linear velocity
        # Right stick X-axis (axes[2]) -> Angular velocity
        # D-pad up/down (axes[7]) -> Speed adjustment

        linear_axis = self.apply_deadzone(msg.axes[1], self.joy_deadzone)
        angular_axis = self.apply_deadzone(msg.axes[2], self.joy_deadzone)

        # Speed adjustment with D-pad
        if len(msg.axes) > 7:
            if msg.axes[7] == 1.0:  # D-pad up
                self.linear_speed += self.speed_increment
                self.get_logger().info(f'Speed increased to {self.linear_speed:.2f} m/s')
            elif msg.axes[7] == -1.0:  # D-pad down
                self.linear_speed = max(0.1, self.linear_speed - self.speed_increment)
                self.get_logger().info(f'Speed decreased to {self.linear_speed:.2f} m/s')

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

    def keyboard_control(self):
        """Handle keyboard teleoperation"""
        settings = save_terminal_settings()

        try:
            print(HELP_MSG.format(
                linear=self.linear_speed,
                angular=self.angular_speed
            ))
            print("\nReady for keyboard input...")

            while rclpy.ok():
                key = get_key(settings)

                if key == 'x' or key == '\x03':  # 'x' or Ctrl+C
                    break

                elif key == 'q':
                    self.linear_speed += self.speed_increment
                    self.get_logger().info(f'Linear speed: {self.linear_speed:.2f} m/s')

                elif key == 'e':
                    self.linear_speed = max(0.1, self.linear_speed - self.speed_increment)
                    self.get_logger().info(f'Linear speed: {self.linear_speed:.2f} m/s')

                elif key in KEY_BINDINGS:
                    linear, angular = KEY_BINDINGS[key]
                    self.current_linear = linear * self.linear_speed
                    self.current_angular = angular * self.angular_speed
                    self.publish_twist()

                    print(f'\rLinear: {self.current_linear:+.2f} m/s  '
                          f'Angular: {self.current_angular:+.2f} rad/s  ',
                          end='', flush=True)

        except Exception as e:
            self.get_logger().error(f'Error: {e}')

        finally:
            # Stop robot
            self.current_linear = 0.0
            self.current_angular = 0.0
            self.publish_twist()
            restore_terminal_settings(settings)
            print('\n\nShutting down...')


def main(args=None):
    """Main entry point"""
    rclpy.init(args=args)

    try:
        node = JoyControlNode()
        node.keyboard_control()
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
