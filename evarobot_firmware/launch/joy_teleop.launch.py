#!/usr/bin/env python3
"""
Launch file for PS4 joystick teleoperation of EvaRobot in manual mode

This launch file starts:
1. ROS2 joy node - Publishes joystick input to /joy topic
2. Arduino serial bridge - Communicates with Arduino for motor control
3. Joy control node - Converts joystick input to cmd_vel commands

Author: Kevin Medrano Ayala
Contact: kevin.ejem18@gmail.com
License: BSD-3-Clause
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    """Generate launch description for PS4 joystick teleoperation"""

    # Declare launch arguments
    serial_port_arg = DeclareLaunchArgument(
        'serial_port',
        default_value='/dev/ttyACM0',
        description='Serial port for Arduino connection'
    )

    baud_rate_arg = DeclareLaunchArgument(
        'baud_rate',
        default_value='115200',
        description='Serial baud rate'
    )

    joy_device_arg = DeclareLaunchArgument(
        'joy_device',
        default_value='0',
        description='Joystick device ID (0 for /dev/input/js0)'
    )

    control_mode_arg = DeclareLaunchArgument(
        'control_mode',
        default_value='manual',
        description='Control mode: manual or automatic'
    )

    linear_speed_arg = DeclareLaunchArgument(
        'linear_speed',
        default_value='0.5',
        description='Maximum linear speed (m/s)'
    )

    angular_speed_arg = DeclareLaunchArgument(
        'angular_speed',
        default_value='1.0',
        description='Maximum angular speed (rad/s)'
    )

    # Get launch configurations
    serial_port = LaunchConfiguration('serial_port')
    baud_rate = LaunchConfiguration('baud_rate')
    joy_device = LaunchConfiguration('joy_device')
    control_mode = LaunchConfiguration('control_mode')
    linear_speed = LaunchConfiguration('linear_speed')
    angular_speed = LaunchConfiguration('angular_speed')

    # Node 1: ROS2 Joy Node (Joystick Driver)
    joy_node = Node(
        package='joy',
        executable='joy_node',
        name='joy_node',
        output='screen',
        parameters=[{
            'device_id': joy_device,
            'deadzone': 0.1,
            'autorepeat_rate': 20.0,  # 20 Hz
        }],
        remappings=[
            ('joy', '/joy'),
        ]
    )

    # Node 2: Arduino Serial Bridge (Motor Control)
    arduino_bridge_node = Node(
        package='evarobot_firmware',
        executable='arduino_serial_com.py',
        name='arduino_serial_bridge',
        output='screen',
        parameters=[{
            'serial_port': serial_port,
            'baud_rate': baud_rate,
            'control_mode': control_mode,
            'wheel_separation': 0.194,  # meters (distance between wheels)
            'wheel_radius': 0.042,      # meters (wheel radius)
            'max_linear_velocity': 0.5,  # m/s
            'max_angular_velocity': 2.0,  # rad/s
            'pwm_min': 38,
            'pwm_max': 95,
            'cmd_vel_timeout': 1.0,
            'publish_encoder_feedback': True,
        }]
    )

    # Node 3: Joy Teleop Node (Joystick to cmd_vel)
    joy_teleop_node = Node(
        package='evarobot_firmware',
        executable='joy_teleop_node.py',
        name='joy_teleop_node',
        output='screen',
        parameters=[{
            'linear_speed': linear_speed,
            'angular_speed': angular_speed,
            'speed_increment': 0.1,
            'joy_deadzone': 0.1,
            'cmd_vel_topic': '/cmd_vel',
            'joy_topic': '/joy',
        }]
    )

    # Log message
    log_info = LogInfo(
        msg=[
            '\n',
            '='*60, '\n',
            'EvaRobot PS4 Joystick Teleoperation Started\n',
            '='*60, '\n',
            'Control Mapping:\n',
            '  Left Stick X-axis  : Rotate Left/Right\n',
            '  Right Stick Y-axis : Forward/Backward\n',
            '  D-pad Up/Down      : Increase/Decrease Speed\n',
            '\n',
            'Parameters:\n',
            '  Serial Port    : ', serial_port, '\n',
            '  Baud Rate      : ', baud_rate, '\n',
            '  Control Mode   : ', control_mode, '\n',
            '  Linear Speed   : ', linear_speed, ' m/s\n',
            '  Angular Speed  : ', angular_speed, ' rad/s\n',
            '='*60, '\n',
        ]
    )

    return LaunchDescription([
        # Declare arguments
        serial_port_arg,
        baud_rate_arg,
        joy_device_arg,
        control_mode_arg,
        linear_speed_arg,
        angular_speed_arg,

        # Log info
        log_info,

        # Launch nodes
        joy_node,
        arduino_bridge_node,
        joy_teleop_node,
    ])
