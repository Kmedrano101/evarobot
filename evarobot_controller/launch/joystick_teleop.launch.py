#!/usr/bin/env python3
"""
EvaRobot Joystick Teleoperation Launch File

This launch file sets up joystick control for the EvaRobot:
  1. Joy node - reads joystick input from /dev/input/js*
  2. Joy Teleop node - converts joystick to velocity commands on /cmd_vel

Simple two-node setup for direct joystick control.

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

    joy_config_arg = DeclareLaunchArgument(
        'joy_config',
        default_value=PathJoinSubstitution([
            FindPackageShare('evarobot_firmware'),
            'config',
            'joy_config.yaml'
        ]),
        description='Path to joystick configuration file'
    )

    linear_speed_arg = DeclareLaunchArgument(
        'linear_speed',
        default_value='0.5',
        description='Maximum linear speed (m/s)'
    )

    angular_speed_arg = DeclareLaunchArgument(
        'angular_speed',
        default_value='1.5',
        description='Maximum angular speed (rad/s)'
    )

    # ========================================================================
    # CONFIGURATION
    # ========================================================================

    use_sim_time = LaunchConfiguration('use_sim_time')
    joy_config = LaunchConfiguration('joy_config')
    linear_speed = LaunchConfiguration('linear_speed')
    angular_speed = LaunchConfiguration('angular_speed')

    # ========================================================================
    # NODES
    # ========================================================================

    # Joy Node
    # Reads input from the joystick device and publishes to /joy topic
    joy_node = Node(
        package='joy',
        executable='joy_node',
        name='joystick',
        parameters=[
            joy_config,
            {'use_sim_time': use_sim_time}
        ],
        output='screen',
    )

    # Joy Teleop Node
    # Converts /joy messages to velocity commands on /cmd_vel
    # Direct control - no multiplexing needed
    joy_teleop_node = Node(
        package='evarobot_controller',
        executable='joy_teleop_node.py',
        name='joy_teleop_node',
        parameters=[
            {
                'use_sim_time': use_sim_time,
                'linear_speed': linear_speed,
                'angular_speed': angular_speed,
                'speed_increment': 0.1,
                'joy_deadzone': 0.1,
                'cmd_vel_topic': '/cmd_vel',
                'joy_topic': '/joy',
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
        joy_config_arg,
        linear_speed_arg,
        angular_speed_arg,

        # Nodes
        joy_node,
        joy_teleop_node,
    ])


# ============================================================================
# USAGE EXAMPLES
# ============================================================================
#
# 1. Launch with simulation:
#    ros2 launch evarobot_controller joystick_teleop.launch.py use_sim_time:=true
#
# 2. Launch with custom speeds:
#    ros2 launch evarobot_controller joystick_teleop.launch.py \
#        linear_speed:=0.8 angular_speed:=2.0
#
# 3. Check joystick is detected:
#    ros2 topic echo /joy
#    (Move joystick and observe output)
#
# 4. Check velocity commands:
#    ros2 topic echo /cmd_vel
#    (Move joystick sticks)
#
# 5. List available joystick devices:
#    ls /dev/input/js*
#
# 6. Test joystick (without ROS):
#    jstest /dev/input/js0
#
# ============================================================================
# JOYSTICK CONTROL MAPPING (PS4 Controller)
# ============================================================================
#
# Controls (NO deadman button required):
#   - Left Stick Horizontal (Axis 0): Rotate left/right (angular velocity)
#   - Right Stick Vertical (Axis 4): Forward/backward (linear velocity)
#   - D-pad Up: Increase speed
#   - D-pad Down: Decrease speed
#
# To customize, edit parameters in launch file or:
#   evarobot_firmware/config/joy_config.yaml
#
# ============================================================================
# TROUBLESHOOTING
# ============================================================================
#
# Problem: Joystick not detected
# Solution:
#   - Check device exists: ls /dev/input/js*
#   - Check permissions: sudo chmod a+rw /dev/input/js0
#   - Add user to input group: sudo usermod -a -G input $USER
#   - Verify with jstest: jstest /dev/input/js0
#
# Problem: Robot doesn't move
# Solution:
#   - Move the joystick sticks (no deadman button needed)
#   - Check /joy topic for input: ros2 topic echo /joy
#   - Check /cmd_vel for output: ros2 topic echo /cmd_vel
#   - Verify controller is loaded: ros2 control list_controllers
#
# Problem: Axes are inverted
# Solution:
#   - Check PS4 controller is in correct mode
#   - Axes should be: 0=left stick X, 4=right stick Y
#
# Problem: QoS compatibility warnings
# Solution:
#   - Fixed: joy_teleop_node now uses RELIABLE QoS for /cmd_vel
#
# ============================================================================
