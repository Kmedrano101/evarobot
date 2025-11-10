#!/usr/bin/env python3
"""
EvaRobot Controller Launch File

This launch file spawns the ROS2 controllers for the EvaRobot differential drive robot.
It loads the controller configuration and spawns:
  1. Joint State Broadcaster - publishes joint states to /joint_states
  2. Differential Drive Controller - controls the robot base movement

Author: Kevin Medrano Ayala
License: BSD-3-Clause
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, RegisterEventHandler
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
import os
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # ========================================================================
    # LAUNCH ARGUMENTS
    # ========================================================================

    # Use sim time (for Gazebo simulation)
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation (Gazebo) clock if true'
    )

    # Controller configuration file
    controller_config_arg = DeclareLaunchArgument(
        'controller_config_file',
        default_value=PathJoinSubstitution([
            FindPackageShare('evarobot_controller'),
            'config',
            'evarobot_controllers.yaml'
        ]),
        description='Path to controller configuration file'
    )

    # Controller manager timeout
    controller_manager_timeout_arg = DeclareLaunchArgument(
        'controller_manager_timeout',
        default_value='10',
        description='Timeout for controller manager in seconds'
    )

    # Start rviz
    start_rviz_arg = DeclareLaunchArgument(
        'start_rviz',
        default_value='false',
        description='Start RViz for visualization'
    )

    # ========================================================================
    # CONFIGURATION
    # ========================================================================

    use_sim_time = LaunchConfiguration('use_sim_time')
    controller_config_file = LaunchConfiguration('controller_config_file')
    controller_manager_timeout = LaunchConfiguration('controller_manager_timeout')
    start_rviz = LaunchConfiguration('start_rviz')

    # ========================================================================
    # CONTROLLER SPAWNERS
    # ========================================================================
    # The controller_manager spawner is used to load and start controllers
    # These nodes will wait for the controller_manager to be available

    # Joint State Broadcaster
    # This controller publishes the state of all joints to /joint_states
    # Required for robot_state_publisher to compute TF transforms
    joint_state_broadcaster_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=[
            'joint_state_broadcaster',
            '--controller-manager', '/controller_manager',
            '--controller-manager-timeout', controller_manager_timeout,
        ],
        output='screen',
    )

    # Differential Drive Controller
    # This controller handles the robot's base movement
    # Subscribes to: /cmd_vel (via relay node)
    # Publishes to: /odom, /tf
    robot_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=[
            'evarobot_controller',
            '--controller-manager', '/controller_manager',
            '--controller-manager-timeout', controller_manager_timeout,
        ],
        output='screen',
    )

    # Simple Controller: forwards /cmd_vel to /evarobot_controller/cmd_vel_unstamped
    # This allows using the standard /cmd_vel topic for compatibility
    simple_controller = Node(
        package='evarobot_controller',
        executable='simple_controller.py',
        name='simple_controller',
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen',
    )

    # ========================================================================
    # EVENT HANDLERS
    # ========================================================================
    # Delay spawning the robot controller until the joint state broadcaster is active
    # This ensures proper initialization order

    delay_robot_controller_spawner_after_joint_state_broadcaster = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[robot_controller_spawner],
        )
    )

    # ========================================================================
    # RVIZ (OPTIONAL)
    # ========================================================================

    rviz_config_file = PathJoinSubstitution([
        FindPackageShare('evarobot_controller'),
        'config',
        'controller_view.rviz'
    ])

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_file],
        condition=IfCondition(start_rviz),
        parameters=[{'use_sim_time': use_sim_time}],
    )

    # ========================================================================
    # LAUNCH DESCRIPTION
    # ========================================================================

    return LaunchDescription([
        # Launch arguments
        use_sim_time_arg,
        controller_config_arg,
        controller_manager_timeout_arg,
        start_rviz_arg,

        # Controller spawners
        joint_state_broadcaster_spawner,
        delay_robot_controller_spawner_after_joint_state_broadcaster,

        # Simple controller for /cmd_vel relay
        simple_controller,

        # Optional visualization
        rviz_node,
    ])


# ============================================================================
# USAGE EXAMPLES
# ============================================================================
#
# 1. Launch with default settings (real robot):
#    ros2 launch evarobot_controller controller.launch.py
#
# 2. Launch for Gazebo simulation:
#    ros2 launch evarobot_controller controller.launch.py use_sim_time:=true
#
# 3. Launch with RViz:
#    ros2 launch evarobot_controller controller.launch.py start_rviz:=true
#
# 4. Launch with custom config:
#    ros2 launch evarobot_controller controller.launch.py \
#        controller_config_file:=/path/to/custom_controllers.yaml
#
# 5. Check active controllers:
#    ros2 control list_controllers
#
# 6. Check controller status:
#    ros2 control list_hardware_interfaces
#
# 7. Test the robot:
#    ros2 topic pub /cmd_vel geometry_msgs/msg/Twist \
#        "{linear: {x: 0.2}, angular: {z: 0.0}}"
#
# ============================================================================
