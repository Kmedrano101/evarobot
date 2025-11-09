#!/usr/bin/env python3
"""
EvaRobot Gazebo Harmonic Simulation Launch File

Launches the EvaRobot in Gazebo Harmonic (gz sim) for ROS2 Jazzy.

Uses:
  - gz sim (Gazebo Harmonic) instead of gzserver/gzclient (Gazebo Classic)
  - ros_gz_sim for simulation interface
  - gz_ros2_control for robot control

Author: Kevin Medrano Ayala
License: BSD-3-Clause
"""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, RegisterEventHandler
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # ========================================================================
    # LAUNCH ARGUMENTS
    # ========================================================================

    world_arg = DeclareLaunchArgument(
        'world',
        default_value='empty.sdf',
        description='World file name (with .sdf extension). Options: empty.sdf, test_world.sdf'
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

    headless_arg = DeclareLaunchArgument(
        'headless',
        default_value='false',
        description='Run Gazebo in headless mode (no GUI)'
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
    headless = LaunchConfiguration('headless')

    # Package directories
    pkg_evarobot_description = get_package_share_directory('evarobot_description')
    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')

    # URDF file path
    urdf_file = os.path.join(pkg_evarobot_description, 'urdf', 'evarobot.urdf.xacro')

    # RViz config file
    rviz_config_file = os.path.join(pkg_evarobot_description, 'rviz', 'evarobot.rviz')

    # Process the URDF with xacro
    robot_description_content = Command([
        PathJoinSubstitution([FindExecutable(name='xacro')]),
        ' ',
        urdf_file,
        ' ',
        'use_sim_time:=',
        use_sim_time
    ])

    robot_description = {'robot_description': ParameterValue(robot_description_content, value_type=str)}

    # ========================================================================
    # NODES
    # ========================================================================

    # Robot State Publisher
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

    # Gazebo Harmonic (gz sim) Launch
    # Uses ros_gz_sim package to launch Gazebo Harmonic
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py')
        ]),
        launch_arguments={
            'gz_args': PathJoinSubstitution([
                '-r -v 4 ',
                world_name
            ]),
            'on_exit_shutdown': 'true'
        }.items()
    )

    # Spawn Robot in Gazebo
    spawn_robot = Node(
        package='ros_gz_sim',
        executable='create',
        name='spawn_evarobot',
        output='screen',
        arguments=[
            '-name', 'evarobot',
            '-topic', 'robot_description',
            '-x', x_pose,
            '-y', y_pose,
            '-z', z_pose,
        ],
    )

    # ROS-Gazebo Bridge for clock
    # Bridges /clock topic from Gazebo to ROS2
    clock_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='clock_bridge',
        output='screen',
        arguments=[
            '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock'
        ],
        parameters=[{'use_sim_time': use_sim_time}]
    )

    # RViz (optional)
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config_file],
        parameters=[{'use_sim_time': use_sim_time}],
        condition=IfCondition(start_rviz)
    )

    # ========================================================================
    # EVENT HANDLERS
    # ========================================================================

    # Load controller after robot spawns
    load_joint_state_broadcaster = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=spawn_robot,
            on_exit=[
                Node(
                    package='controller_manager',
                    executable='spawner',
                    arguments=['joint_state_broadcaster'],
                    output='screen',
                )
            ],
        )
    )

    load_diff_drive_controller = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=spawn_robot,
            on_exit=[
                Node(
                    package='controller_manager',
                    executable='spawner',
                    arguments=['evarobot_base_controller'],
                    output='screen',
                )
            ],
        )
    )

    # ========================================================================
    # LAUNCH DESCRIPTION
    # ========================================================================

    return LaunchDescription([
        # Arguments
        world_arg,
        use_sim_time_arg,
        start_rviz_arg,
        x_pose_arg,
        y_pose_arg,
        z_pose_arg,
        headless_arg,

        # Nodes
        robot_state_publisher_node,
        gazebo,
        spawn_robot,
        clock_bridge,
        rviz_node,

        # Event handlers (load controllers after spawn)
        load_joint_state_broadcaster,
        load_diff_drive_controller,
    ])


# ============================================================================
# USAGE
# ============================================================================
#
# Launch simulation:
#   ros2 launch evarobot_description gazebo_harmonic.launch.py
#
# Launch with RViz:
#   ros2 launch evarobot_description gazebo_harmonic.launch.py start_rviz:=true
#
# Launch headless (no GUI):
#   ros2 launch evarobot_description gazebo_harmonic.launch.py headless:=true
#
# Launch with custom world:
#   ros2 launch evarobot_description gazebo_harmonic.launch.py world:=my_world.sdf
#
# Check if robot spawned:
#   ros2 topic list | grep evarobot
#
# Check controllers:
#   ros2 control list_controllers
#
# Send test command:
#   ros2 topic pub /evarobot_base_controller/cmd_vel_unstamped \
#     geometry_msgs/msg/Twist "{linear: {x: 0.2}}" -r 10
#
# ============================================================================
