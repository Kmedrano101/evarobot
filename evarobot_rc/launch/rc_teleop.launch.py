#!/usr/bin/env python3
"""
EvaRobot RC Teleoperation Launch File

Launches the CRSF receiver node for ExpressLRS RC control.
Reads CRSF frames from ELRS receiver on Raspberry Pi 4B UART
and publishes velocity commands to /cmd_vel.

Author: Kevin Medrano Ayala
License: BSD-3-Clause
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # ========================================================================
    # LAUNCH ARGUMENTS
    # ========================================================================

    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation clock if true'
    )

    serial_port_arg = DeclareLaunchArgument(
        'serial_port',
        default_value='/dev/ttyAMA0',
        description='Serial port for ELRS receiver (Pi 4B GPIO UART)'
    )

    baud_rate_arg = DeclareLaunchArgument(
        'baud_rate',
        default_value='420000',
        description='CRSF baud rate (420000 for ExpressLRS)'
    )

    max_linear_vel_arg = DeclareLaunchArgument(
        'max_linear_vel',
        default_value='0.5',
        description='Maximum linear velocity (m/s)'
    )

    max_angular_vel_arg = DeclareLaunchArgument(
        'max_angular_vel',
        default_value='1.5',
        description='Maximum angular velocity (rad/s)'
    )

    rc_config_arg = DeclareLaunchArgument(
        'rc_config',
        default_value=PathJoinSubstitution([
            FindPackageShare('evarobot_rc'),
            'config',
            'rc_receiver.yaml'
        ]),
        description='Path to RC receiver configuration file'
    )

    # ========================================================================
    # CONFIGURATION
    # ========================================================================

    use_sim_time = LaunchConfiguration('use_sim_time')
    serial_port = LaunchConfiguration('serial_port')
    baud_rate = LaunchConfiguration('baud_rate')
    max_linear_vel = LaunchConfiguration('max_linear_vel')
    max_angular_vel = LaunchConfiguration('max_angular_vel')
    rc_config = LaunchConfiguration('rc_config')

    # ========================================================================
    # NODES
    # ========================================================================

    # RC Receiver Node
    # Reads CRSF frames from ELRS receiver and publishes:
    #   - /cmd_vel (Twist) for robot control
    #   - rc/channels (Int32MultiArray) for raw channel data
    #   - rc/link_stats (String) for link quality info
    rc_receiver_node = Node(
        package='evarobot_rc',
        executable='rc_receiver_node',
        name='rc_receiver_node',
        parameters=[
            rc_config,
            {
                'use_sim_time': use_sim_time,
                'serial_port': serial_port,
                'baud_rate': baud_rate,
                'max_linear_vel': max_linear_vel,
                'max_angular_vel': max_angular_vel,
            }
        ],
        output='screen',
    )

    # ========================================================================
    # LAUNCH DESCRIPTION
    # ========================================================================

    return LaunchDescription([
        # Launch arguments
        use_sim_time_arg,
        serial_port_arg,
        baud_rate_arg,
        max_linear_vel_arg,
        max_angular_vel_arg,
        rc_config_arg,

        # Nodes
        rc_receiver_node,
    ])


# ============================================================================
# USAGE EXAMPLES
# ============================================================================
#
# 1. Launch with default settings (Pi 4B UART):
#    ros2 launch evarobot_rc rc_teleop.launch.py
#
# 2. Launch with custom serial port (USB-to-UART adapter):
#    ros2 launch evarobot_rc rc_teleop.launch.py serial_port:=/dev/ttyUSB0
#
# 3. Launch with custom speed limits:
#    ros2 launch evarobot_rc rc_teleop.launch.py \
#        max_linear_vel:=0.8 max_angular_vel:=2.0
#
# 4. Monitor raw RC channels:
#    ros2 topic echo /rc/channels
#
# 5. Monitor velocity commands:
#    ros2 topic echo /cmd_vel
#
# 6. Monitor link quality:
#    ros2 topic echo /rc/link_stats
#
# ============================================================================
# TROUBLESHOOTING
# ============================================================================
#
# Problem: "Failed to open serial port"
# Solution:
#   - Check wiring: ELRS RX TX pad -> Pi GPIO 15 (pin 10)
#   - Disable Bluetooth: add dtoverlay=disable-bt to /boot/firmware/config.txt
#   - Disable serial console: sudo systemctl disable serial-getty@ttyAMA0.service
#   - Check permissions: sudo usermod -a -G dialout $USER (then re-login)
#   - Verify device exists: ls -la /dev/ttyAMA0
#
# Problem: "RC failsafe triggered" immediately
# Solution:
#   - Check ELRS receiver is powered and bound to transmitter
#   - Verify receiver LED shows solid (connected) not blinking (binding)
#   - Ensure receiver is set to CRSF protocol (not SBUS or MAVLink)
#   - Check baud rate matches receiver config (default 420000)
#
# Problem: Channels read but robot doesn't move
# Solution:
#   - Check arm switch is in armed position: ros2 topic echo /rc/channels
#   - Verify channel mapping matches your transmitter (AETR vs TAER)
#   - Check if invert_throttle/invert_steering need to be toggled
#   - Increase deadzone if center noise is too high
#
# Problem: Occasional CRC errors in log
# Solution:
#   - This is normal at low rates (<1% of frames)
#   - Check wiring for loose connections
#   - Keep UART wires short and away from motors/ESCs
#   - Ensure ground is shared between Pi and receiver
#
