#!/usr/bin/env python3
"""
EvaRobot Joystick Teleoperation Launch File

This launch file sets up joystick control for the EvaRobot:
  1. Joy node - reads joystick input
  2. Joy Teleop - converts joystick to velocity commands
  3. Twist Mux - multiplexes multiple command sources

The joystick commands are published to /joy_vel and then multiplexed
with other command sources before being sent to the robot controller.

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

    twist_mux_config_arg = DeclareLaunchArgument(
        'twist_mux_config',
        default_value=PathJoinSubstitution([
            FindPackageShare('evarobot_controller'),
            'config',
            'twist_mux.yaml'
        ]),
        description='Path to twist mux configuration file'
    )

    joy_vel_topic_arg = DeclareLaunchArgument(
        'joy_vel_topic',
        default_value='joy_vel',
        description='Topic name for joystick velocity commands'
    )

    cmd_vel_topic_arg = DeclareLaunchArgument(
        'cmd_vel_topic',
        default_value='cmd_vel',
        description='Topic name for final multiplexed velocity commands'
    )

    # ========================================================================
    # CONFIGURATION
    # ========================================================================

    use_sim_time = LaunchConfiguration('use_sim_time')
    joy_config = LaunchConfiguration('joy_config')
    twist_mux_config = LaunchConfiguration('twist_mux_config')
    joy_vel_topic = LaunchConfiguration('joy_vel_topic')
    cmd_vel_topic = LaunchConfiguration('cmd_vel_topic')

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
    # Converts /joy messages to velocity commands (/joy_vel)
    # This uses the joy_teleop configuration from joy_config.yaml
    joy_teleop_node = Node(
        package='joy_teleop',
        executable='joy_teleop',
        name='joy_teleop',
        parameters=[
            joy_config,
            {'use_sim_time': use_sim_time}
        ],
        # Remap the output to joy_vel (will be multiplexed by twist_mux)
        remappings=[
            ('input_joy/cmd_vel', joy_vel_topic),
        ],
        output='screen',
    )

    # Twist Mux Node
    # Multiplexes velocity commands from multiple sources
    # Subscribes to: /joy_vel, /key_vel, /nav_vel, etc.
    # Publishes to: /cmd_vel (to the robot controller)
    twist_mux_node = Node(
        package='twist_mux',
        executable='twist_mux',
        name='twist_mux',
        parameters=[
            twist_mux_config,
            {'use_sim_time': use_sim_time}
        ],
        remappings=[
            ('cmd_vel_out', cmd_vel_topic),
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
        twist_mux_config_arg,
        joy_vel_topic_arg,
        cmd_vel_topic_arg,

        # Nodes
        joy_node,
        joy_teleop_node,
        twist_mux_node,
    ])


# ============================================================================
# USAGE EXAMPLES
# ============================================================================
#
# 1. Launch with default configuration:
#    ros2 launch evarobot_controller joystick_teleop.launch.py
#
# 2. Launch with custom joy config:
#    ros2 launch evarobot_controller joystick_teleop.launch.py \
#        joy_config:=/path/to/custom_joy_config.yaml
#
# 3. Check joystick is detected:
#    ros2 topic echo /joy
#    (Move joystick and observe output)
#
# 4. Check velocity commands:
#    ros2 topic echo /joy_vel
#    (Hold deadman button and move joystick)
#
# 5. Check multiplexed output:
#    ros2 topic echo /cmd_vel
#
# 6. Check active input source:
#    ros2 topic echo /cmd_vel_mux/active
#
# 7. List available joystick devices:
#    ls /dev/input/js*
#
# 8. Test joystick (without ROS):
#    jstest /dev/input/js0
#
# ============================================================================
# JOYSTICK BUTTON MAPPING (PS4 Controller)
# ============================================================================
#
# Default configuration uses:
#   - R1 (Button 5): Deadman switch (must hold to send commands)
#   - Left Stick Vertical (Axis 1): Forward/backward linear velocity
#   - Right Stick Horizontal (Axis 0): Left/right angular velocity
#
# To customize button mappings, edit:
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
#   - Hold deadman button (R1)
#   - Check /joy topic for input: ros2 topic echo /joy
#   - Check /joy_vel for output: ros2 topic echo /joy_vel
#   - Check /cmd_vel for final output: ros2 topic echo /cmd_vel
#   - Verify controller is loaded: ros2 control list_controllers
#
# Problem: Axes are inverted
# Solution:
#   - Adjust scale values in joy_config.yaml (use negative values)
#
# Problem: Low deadzone (drift)
# Solution:
#   - Increase deadzone in joy_config.yaml (try 0.1-0.2)
#
# ============================================================================
