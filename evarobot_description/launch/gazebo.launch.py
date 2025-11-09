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
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution, TextSubstitution
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

    yaw_arg = DeclareLaunchArgument(
        'yaw',
        default_value='0.0',
        description='Initial yaw orientation of the robot (radians)'
    )

    gui_arg = DeclareLaunchArgument(
        'gui',
        default_value='true',
        description='Start Gazebo GUI (set to false for headless mode)'
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
    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')

    # World file path
    world_file = PathJoinSubstitution([
        FindPackageShare('evarobot_description'),
        'worlds',
        world_name
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

    # Gazebo Harmonic (gz sim) Launch with GUI
    gazebo_gui = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py')
        ]),
        launch_arguments={
            'gz_args': [world_file, TextSubstitution(text=' -r')],
            'on_exit_shutdown': 'true'
        }.items(),
        condition=IfCondition(gui)
    )

    # Gazebo Harmonic (gz sim) Launch headless (no GUI)
    gazebo_headless = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py')
        ]),
        launch_arguments={
            'gz_args': [world_file, TextSubstitution(text=' -r -s')],
            'on_exit_shutdown': 'true'
        }.items(),
        condition=UnlessCondition(gui)
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
            '-Y', yaw,
        ],
    )

    # ROS-Gazebo Bridge for clock
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
    rviz_config_file = PathJoinSubstitution([
        FindPackageShare('evarobot_description'),
        'rviz',
        'view_robot.rviz'
    ])

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
    # EVENT HANDLERS (Optional - Controllers loaded separately)
    # ========================================================================

    # Note: Controllers are loaded via separate launch file for better control
    # Use: ros2 launch evarobot_controller controller.launch.py use_sim_time:=true

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
        yaw_arg,
        gui_arg,

        # Nodes
        robot_state_publisher_node,
        gazebo_gui,
        gazebo_headless,
        spawn_robot,
        clock_bridge,
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
#    ros2 launch evarobot_description gazebo.launch.py world:=test_world.sdf
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
# 6. Launch controllers (in separate terminal):
#    ros2 launch evarobot_controller controller.launch.py use_sim_time:=true
#
# 7. Check controllers:
#    ros2 control list_controllers
#
# 8. Send test command:
#    ros2 topic pub /evarobot_base_controller/cmd_vel_unstamped \
#      geometry_msgs/msg/Twist "{linear: {x: 0.2}}" -r 10
#
# 9. Check robot transforms:
#    ros2 run tf2_tools view_frames
#    evince frames.pdf
#
# 10. Monitor joint states:
#    ros2 topic echo /joint_states
#
# ============================================================================
