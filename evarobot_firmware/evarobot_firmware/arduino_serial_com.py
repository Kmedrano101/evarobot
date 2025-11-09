#!/usr/bin/env python3
"""
Arduino Serial Communication Bridge for EvaRobot

Bridges ROS2 cmd_vel messages to Arduino firmware via serial communication.
Supports both manual (PWM) and automatic (PID velocity) control modes.
Publishes encoder feedback for odometry.

Author: Kevin Medrano Ayala
Contact: kevin.ejem18@gmail.com
License: BSD-3-Clause
"""

import json
import math
import serial
import threading
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
from geometry_msgs.msg import Twist
from std_msgs.msg import Int32MultiArray, Float32MultiArray, String


# ============================================================================
# ARDUINO SERIAL BRIDGE NODE
# ============================================================================

class ArduinoSerialBridge(Node):
    """
    ROS2 node that bridges cmd_vel commands to Arduino motor controller
    and publishes encoder feedback
    """

    def __init__(self):
        super().__init__('arduino_serial_bridge')

        # Declare parameters
        self.declare_parameter('serial_port', '/dev/ttyACM0')
        self.declare_parameter('baud_rate', 115200)
        self.declare_parameter('timeout', 0.1)
        self.declare_parameter('control_mode', 'manual')  # 'manual' or 'automatic'
        self.declare_parameter('wheel_separation', 0.235)  # meters
        self.declare_parameter('wheel_radius', 0.033)  # meters
        self.declare_parameter('max_linear_velocity', 0.5)  # m/s
        self.declare_parameter('max_angular_velocity', 2.0)  # rad/s
        self.declare_parameter('pwm_min', 38)
        self.declare_parameter('pwm_max', 95)
        self.declare_parameter('cmd_vel_timeout', 1.0)  # seconds
        self.declare_parameter('publish_encoder_feedback', True)

        # Get parameters
        self.serial_port = self.get_parameter('serial_port').value
        self.baud_rate = self.get_parameter('baud_rate').value
        self.timeout = self.get_parameter('timeout').value
        self.control_mode = self.get_parameter('control_mode').value
        self.wheel_separation = self.get_parameter('wheel_separation').value
        self.wheel_radius = self.get_parameter('wheel_radius').value
        self.max_linear_vel = self.get_parameter('max_linear_velocity').value
        self.max_angular_vel = self.get_parameter('max_angular_velocity').value
        self.pwm_min = self.get_parameter('pwm_min').value
        self.pwm_max = self.get_parameter('pwm_max').value
        self.cmd_vel_timeout = self.get_parameter('cmd_vel_timeout').value
        self.publish_feedback = self.get_parameter('publish_encoder_feedback').value

        # QoS Profile
        qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10
        )

        # Subscribers
        self.cmd_vel_sub = self.create_subscription(
            Twist,
            'cmd_vel',
            self.cmd_vel_callback,
            qos_profile
        )

        self.mode_sub = self.create_subscription(
            String,
            'control_mode',
            self.mode_callback,
            qos_profile
        )

        # Publishers
        self.encoder_ticks_pub = self.create_publisher(
            Int32MultiArray,
            'encoder_ticks',
            qos_profile
        )

        self.encoder_velocities_pub = self.create_publisher(
            Float32MultiArray,
            'encoder_velocities',
            qos_profile
        )

        self.status_pub = self.create_publisher(
            String,
            'arduino_status',
            qos_profile
        )

        # Serial connection
        self.serial_conn = None
        self.serial_lock = threading.Lock()
        self.connect_serial()

        # Watchdog timer for cmd_vel timeout
        self.last_cmd_time = self.get_clock().now()
        self.watchdog_timer = self.create_timer(0.1, self.watchdog_callback)

        # Feedback reader thread
        if self.publish_feedback:
            self.feedback_thread = threading.Thread(
                target=self.read_feedback_loop,
                daemon=True
            )
            self.feedback_thread.start()

        # Set initial control mode
        self.set_control_mode(self.control_mode)

        self.get_logger().info('Arduino Serial Bridge Started')
        self.get_logger().info(f'Serial port: {self.serial_port} @ {self.baud_rate} baud')
        self.get_logger().info(f'Control mode: {self.control_mode}')
        self.get_logger().info(f'Wheel separation: {self.wheel_separation} m')
        self.get_logger().info(f'Wheel radius: {self.wheel_radius} m')

    def connect_serial(self):
        """Establish serial connection to Arduino with retry logic"""
        import time

        max_retries = 3
        retry_delay = 1.0  # seconds

        for attempt in range(max_retries):
            try:
                self.get_logger().info(f'Connecting to Arduino on {self.serial_port} (attempt {attempt + 1}/{max_retries})...')

                # Close any existing connection
                if self.serial_conn and self.serial_conn.is_open:
                    self.serial_conn.close()
                    time.sleep(0.5)

                # Open serial connection
                self.serial_conn = serial.Serial(
                    port=self.serial_port,
                    baudrate=self.baud_rate,
                    timeout=self.timeout
                )

                self.get_logger().info(f'Port opened, waiting for Arduino to reset...')

                # Wait for Arduino to reset after serial connection
                time.sleep(2.5)

                # Flush any stale data
                self.serial_conn.reset_input_buffer()
                self.serial_conn.reset_output_buffer()

                # Read startup message
                if self.serial_conn.in_waiting:
                    msg = self.serial_conn.readline().decode('utf-8', errors='ignore').strip()
                    self.get_logger().info(f'Arduino: {msg}')

                self.get_logger().info(f'Successfully connected to Arduino on {self.serial_port}')
                return  # Success!

            except serial.SerialException as e:
                self.get_logger().warn(f'Connection attempt {attempt + 1} failed: {e}')
                self.serial_conn = None

                if attempt < max_retries - 1:
                    self.get_logger().info(f'Retrying in {retry_delay} seconds...')
                    time.sleep(retry_delay)
                else:
                    self.get_logger().error(f'Failed to connect to Arduino after {max_retries} attempts')
                    self.get_logger().error('Please check:')
                    self.get_logger().error(f'  1. Arduino is connected to {self.serial_port}')
                    self.get_logger().error(f'  2. User has permission (member of dialout group)')
                    self.get_logger().error(f'  3. No other program is using the port')
            except Exception as e:
                self.get_logger().error(f'Unexpected error: {e}')
                self.serial_conn = None
                if attempt < max_retries - 1:
                    time.sleep(retry_delay)

    def send_command(self, cmd_dict):
        """Send JSON command to Arduino"""
        if not self.serial_conn or not self.serial_conn.is_open:
            self.get_logger().warn('Serial connection not available')
            return False

        try:
            with self.serial_lock:
                cmd_str = json.dumps(cmd_dict) + '\n'
                self.serial_conn.write(cmd_str.encode('utf-8'))
                self.get_logger().debug(f'Sent: {cmd_str.strip()}')
            return True
        except Exception as e:
            self.get_logger().error(f'Failed to send command: {e}')
            return False

    def set_control_mode(self, mode):
        """Switch between manual and automatic control modes"""
        if mode not in ['manual', 'automatic']:
            self.get_logger().error(f'Invalid control mode: {mode}')
            return

        self.control_mode = mode
        cmd = {"mode": mode}
        if self.send_command(cmd):
            self.get_logger().info(f'Control mode set to: {mode}')

    def mode_callback(self, msg):
        """Handle control mode change requests"""
        self.set_control_mode(msg.data)

    def cmd_vel_callback(self, msg):
        """Convert cmd_vel to Arduino commands"""
        self.last_cmd_time = self.get_clock().now()

        # Extract velocities
        linear_vel = msg.linear.x  # m/s
        angular_vel = msg.angular.z  # rad/s

        # Clamp velocities
        linear_vel = max(-self.max_linear_vel, min(self.max_linear_vel, linear_vel))
        angular_vel = max(-self.max_angular_vel, min(self.max_angular_vel, angular_vel))

        if self.control_mode == 'automatic':
            # Automatic mode: send velocity setpoints in rad/s
            self.send_automatic_command(linear_vel, angular_vel)
        else:
            # Manual mode: send PWM and direction commands
            self.send_manual_command(linear_vel, angular_vel)

    def send_automatic_command(self, linear_vel, angular_vel):
        """
        Send velocity setpoints for PID control

        Args:
            linear_vel: Linear velocity in m/s
            angular_vel: Angular velocity in rad/s
        """
        # Convert to wheel velocities (rad/s)
        # v_left = (linear_vel - angular_vel * wheel_separation / 2) / wheel_radius
        # v_right = (linear_vel + angular_vel * wheel_separation / 2) / wheel_radius

        v_left = (linear_vel - angular_vel * self.wheel_separation / 2.0) / self.wheel_radius
        v_right = (linear_vel + angular_vel * self.wheel_separation / 2.0) / self.wheel_radius

        cmd = {
            "velocitySetpoints": {
                "left": round(v_left, 3),
                "right": round(v_right, 3)
            }
        }

        self.send_command(cmd)
        self.get_logger().debug(f'Setpoints - L: {v_left:.3f}, R: {v_right:.3f} rad/s')

    def send_manual_command(self, linear_vel, angular_vel):
        """
        Send manual PWM and direction commands using differential drive

        TWO TURNING STRATEGIES:
        1. PURE ROTATION (linear ≈ 0, angular != 0):
           - Turn left: LEFT wheel OFF (PWM=0), RIGHT wheel ON
           - Turn right: RIGHT wheel OFF (PWM=0), LEFT wheel ON

        2. ARC TURN (linear != 0, angular != 0):
           - Both wheels ON at different speeds for smooth arc
           - Uses differential drive kinematics

        Args:
            linear_vel: Linear velocity in m/s
            angular_vel: Angular velocity in rad/s
        """
        # Define thresholds
        ANGULAR_THRESHOLD = 0.1  # rad/s
        LINEAR_THRESHOLD = 0.05   # m/s (threshold for arc vs pure rotation)

        # Check if stopped
        if abs(linear_vel) < 0.01 and abs(angular_vel) < 0.01:
            motor_status = "stop"
            left_pwm = 0
            right_pwm = 0

        # PURE ROTATION: Linear velocity is essentially zero but turning
        elif abs(linear_vel) < LINEAR_THRESHOLD and abs(angular_vel) > ANGULAR_THRESHOLD:
            if angular_vel > 0:
                # Pure rotation left: left wheel OFF, right wheel spins
                motor_status = "turnleft"
                left_pwm = 0
                right_pwm = self.velocity_to_pwm(abs(angular_vel) / self.max_angular_vel)
            else:
                # Pure rotation right: right wheel OFF, left wheel spins
                motor_status = "turnright"
                left_pwm = self.velocity_to_pwm(abs(angular_vel) / self.max_angular_vel)
                right_pwm = 0

        # ARC TURN or STRAIGHT: Linear velocity is present
        else:
            # Calculate differential wheel velocities
            # v_left = v_linear - (v_angular * wheel_separation / 2)
            # v_right = v_linear + (v_angular * wheel_separation / 2)
            left_vel = linear_vel - (angular_vel * self.wheel_separation / 2.0)
            right_vel = linear_vel + (angular_vel * self.wheel_separation / 2.0)

            # Determine motor status
            if abs(angular_vel) < ANGULAR_THRESHOLD:
                # Straight motion
                motor_status = "forward" if linear_vel >= 0 else "backward"
            elif left_vel < right_vel:
                # Arc turn left (left slower, right faster)
                motor_status = "turnleft"
            else:
                # Arc turn right (right slower, left faster)
                motor_status = "turnright"

            # Convert to PWM, using differential speeds
            left_ratio = abs(left_vel) / self.max_linear_vel
            right_ratio = abs(right_vel) / self.max_linear_vel

            # Clamp ratios to [0, 1]
            left_ratio = max(0.0, min(1.0, left_ratio))
            right_ratio = max(0.0, min(1.0, right_ratio))

            left_pwm = self.velocity_to_pwm(left_ratio)
            right_pwm = self.velocity_to_pwm(right_ratio)

        cmd = {
            "motorStatus": motor_status,
            "motorVelocity": {
                "left": int(left_pwm),
                "right": int(right_pwm)
            }
        }

        self.send_command(cmd)
        self.get_logger().info(
            f'{motor_status.upper()}: L_PWM={left_pwm}, R_PWM={right_pwm} '
            f'(lin={linear_vel:.2f}, ang={angular_vel:.2f})'
        )

    def velocity_to_pwm(self, velocity_ratio):
        """
        Convert velocity ratio (0-1) to PWM duty cycle

        Args:
            velocity_ratio: Velocity as ratio of maximum (0.0 to 1.0)

        Returns:
            PWM duty cycle (0 or pwm_min to pwm_max)
        """
        velocity_ratio = max(0.0, min(1.0, velocity_ratio))

        # Special case: if ratio is 0, return 0 (motor off)
        if velocity_ratio < 0.01:
            return 0

        # Otherwise map to pwm_min to pwm_max range
        pwm = self.pwm_min + velocity_ratio * (self.pwm_max - self.pwm_min)
        return int(pwm)

    def watchdog_callback(self):
        """Check for cmd_vel timeout and stop motors if needed"""
        if not self.serial_conn:
            return

        time_since_cmd = (self.get_clock().now() - self.last_cmd_time).nanoseconds / 1e9

        if time_since_cmd > self.cmd_vel_timeout:
            # Timeout - send stop command
            if self.control_mode == 'automatic':
                cmd = {"velocitySetpoints": {"left": 0.0, "right": 0.0}}
            else:
                cmd = {"motorStatus": "stop", "motorVelocity": {"left": self.pwm_min, "right": self.pwm_min}}

            self.send_command(cmd)

    def read_feedback_loop(self):
        """Background thread to read encoder feedback from Arduino"""
        while rclpy.ok():
            if not self.serial_conn or not self.serial_conn.is_open:
                continue

            try:
                if self.serial_conn.in_waiting:
                    line = self.serial_conn.readline().decode('utf-8', errors='ignore').strip()

                    if not line:
                        continue

                    # Try to parse JSON
                    try:
                        data = json.loads(line)

                        # Check for encoder feedback
                        if 'encoders' in data and 'velocities' in data:
                            self.publish_encoder_feedback(data)
                        elif 'status' in data or 'error' in data:
                            # Status or error message
                            status_msg = String()
                            status_msg.data = line
                            self.status_pub.publish(status_msg)
                            self.get_logger().info(f'Arduino: {line}')
                        else:
                            self.get_logger().debug(f'Arduino: {line}')

                    except json.JSONDecodeError:
                        # Not JSON, just log it
                        self.get_logger().debug(f'Arduino: {line}')

            except Exception as e:
                self.get_logger().error(f'Error reading feedback: {e}')

    def publish_encoder_feedback(self, data):
        """Publish encoder ticks and velocities"""
        try:
            # Publish encoder ticks
            ticks_msg = Int32MultiArray()
            ticks_msg.data = [data['encoders']['left'], data['encoders']['right']]
            self.encoder_ticks_pub.publish(ticks_msg)

            # Publish encoder velocities
            vel_msg = Float32MultiArray()
            vel_msg.data = [
                float(data['velocities']['left']),
                float(data['velocities']['right'])
            ]
            self.encoder_velocities_pub.publish(vel_msg)

            self.get_logger().debug(
                f"Encoders: L={data['encoders']['left']}, R={data['encoders']['right']} | "
                f"Vel: L={data['velocities']['left']:.3f}, R={data['velocities']['right']:.3f} rad/s"
            )

        except Exception as e:
            self.get_logger().error(f'Error publishing encoder feedback: {e}')

    def shutdown(self):
        """Clean shutdown"""
        self.get_logger().info('Shutting down Arduino Serial Bridge...')

        # Stop motors
        if self.control_mode == 'automatic':
            cmd = {"velocitySetpoints": {"left": 0.0, "right": 0.0}}
        else:
            cmd = {"motorStatus": "stop", "motorVelocity": {"left": self.pwm_min, "right": self.pwm_min}}

        self.send_command(cmd)

        # Close serial connection
        if self.serial_conn and self.serial_conn.is_open:
            self.serial_conn.close()
            self.get_logger().info('Serial connection closed')


def main(args=None):
    """Main entry point"""
    rclpy.init(args=args)

    node = None
    try:
        node = ArduinoSerialBridge()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node:
            node.shutdown()
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
