#!/usr/bin/env python3
"""
EvaRobot Gazebo Simulation Launch File

Launches the EvaRobot in Gazebo simulation with:
  - Configurable world file
  - Robot state publisher
  - Gazebo simulation
  - Robot spawner
  - Optional RViz visualization

Author: Kevin Medrano Ayala
License: BSD-3-Clause
"""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, ExecuteProcess, RegisterEventHandler
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # ========================================================================
    # LAUNCH ARGUMENTS
    # ========================================================================

    world_arg = DeclareLaunchArgument(
        'world',
        default_value='empty',
        description='World file name (without .world extension). Options: empty, test_world'
    )

    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation time'
    )

    start_rviz_arg = DeclareLaunchArgument(
        'start_rviz',
        default_value='false',
        description='Start RViz for visualization'
    )

    x_pose_arg = DeclareLaunchArgument(
        'x_pose',
        default_value='0.0',
        description='Initial x position of the robot'
    )

    y_pose_arg = DeclareLaunchArgument(
        'y_pose',
        default_value='0.0',
        description='Initial y position of the robot'
    )

    z_pose_arg = DeclareLaunchArgument(
        'z_pose',
        default_value='0.1',
        description='Initial z position of the robot'
    )

    yaw_arg = DeclareLaunchArgument(
        'yaw',
        default_value='0.0',
        description='Initial yaw orientation of the robot (radians)'
    )

    gui_arg = DeclareLaunchArgument(
        'gui',
        default_value='true',
        description='Start Gazebo GUI'
    )

    # ========================================================================
    # CONFIGURATION
    # ========================================================================

    world_name = LaunchConfiguration('world')
    use_sim_time = LaunchConfiguration('use_sim_time')
    start_rviz = LaunchConfiguration('start_rviz')
    x_pose = LaunchConfiguration('x_pose')
    y_pose = LaunchConfiguration('y_pose')
    z_pose = LaunchConfiguration('z_pose')
    yaw = LaunchConfiguration('yaw')
    gui = LaunchConfiguration('gui')

    # Package directories
    pkg_evarobot_description = get_package_share_directory('evarobot_description')

    # World file path
    world_file = PathJoinSubstitution([
        FindPackageShare('evarobot_description'),
        'worlds',
        [world_name, '.world']
    ])

    # URDF file path
    urdf_file = os.path.join(pkg_evarobot_description, 'urdf', 'evarobot.urdf.xacro')

    # Process the URDF with xacro
    robot_description_content = Command([
        PathJoinSubstitution([FindExecutable(name='xacro')]),
        ' ',
        urdf_file,
        ' ',
        'use_sim_time:=',
        use_sim_time
    ])

    robot_description = {'robot_description': robot_description_content}

    # ========================================================================
    # NODES
    # ========================================================================

    # Robot State Publisher
    # Publishes robot transforms based on URDF
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[
            robot_description,
            {'use_sim_time': use_sim_time}
        ]
    )

    # Gazebo Server (physics simulation)
    gazebo_server = ExecuteProcess(
        cmd=['gzserver',
             '-s', 'libgazebo_ros_init.so',
             '-s', 'libgazebo_ros_factory.so',
             world_file],
        output='screen'
    )

    # Gazebo Client (GUI)
    gazebo_client = ExecuteProcess(
        cmd=['gzclient'],
        output='screen',
        condition=IfCondition(gui)
    )

    # Spawn Robot in Gazebo
    spawn_robot = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-topic', 'robot_description',
            '-entity', 'evarobot',
            '-x', x_pose,
            '-y', y_pose,
            '-z', z_pose,
            '-Y', yaw
        ],
        output='screen'
    )

    # RViz (optional)
    rviz_config_file = PathJoinSubstitution([
        FindPackageShare('evarobot_description'),
        'rviz',
        'view_robot.rviz'
    ])

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_file],
        parameters=[{'use_sim_time': use_sim_time}],
        condition=IfCondition(start_rviz),
        output='screen'
    )

    # ========================================================================
    # EVENT HANDLERS
    # ========================================================================

    # Delay spawning robot until Gazebo is ready
    delay_spawn_after_gazebo = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=gazebo_server,
            on_exit=[spawn_robot],
        )
    )

    # ========================================================================
    # LAUNCH DESCRIPTION
    # ========================================================================

    return LaunchDescription([
        # Launch arguments
        world_arg,
        use_sim_time_arg,
        start_rviz_arg,
        x_pose_arg,
        y_pose_arg,
        z_pose_arg,
        yaw_arg,
        gui_arg,

        # Nodes and processes
        robot_state_publisher_node,
        gazebo_server,
        gazebo_client,
        spawn_robot,
        rviz_node,
    ])


# ============================================================================
# USAGE EXAMPLES
# ============================================================================
#
# 1. Launch with default empty world:
#    ros2 launch evarobot_description gazebo.launch.py
#
# 2. Launch with test world:
#    ros2 launch evarobot_description gazebo.launch.py world:=test_world
#
# 3. Launch with RViz:
#    ros2 launch evarobot_description gazebo.launch.py start_rviz:=true
#
# 4. Launch at specific position:
#    ros2 launch evarobot_description gazebo.launch.py x_pose:=2.0 y_pose:=1.0
#
# 5. Launch without GUI (headless):
#    ros2 launch evarobot_description gazebo.launch.py gui:=false
#
# 6. Control the robot:
#    # First, spawn the controllers:
#    ros2 launch evarobot_controller controller.launch.py use_sim_time:=true
#
#    # Then send velocity commands:
#    ros2 topic pub /cmd_vel geometry_msgs/msg/Twist \
#        "{linear: {x: 0.2}, angular: {z: 0.5}}" --once
#
# 7. Launch with joystick control:
#    # Terminal 1: Launch Gazebo
#    ros2 launch evarobot_description gazebo.launch.py world:=test_world
#
#    # Terminal 2: Launch controllers
#    ros2 launch evarobot_controller controller.launch.py use_sim_time:=true
#
#    # Terminal 3: Launch joystick
#    ros2 launch evarobot_controller joystick_teleop.launch.py use_sim_time:=true
#
# 8. Check robot transforms:
#    ros2 run tf2_tools view_frames
#    evince frames.pdf
#
# 9. Check available topics:
#    ros2 topic list
#
# 10. Monitor joint states:
#    ros2 topic echo /joint_states
#
# ============================================================================
