# EvaRobot Gazebo Simulation Setup Guide

Complete guide to simulate EvaRobot with differential drive controller in Gazebo.

## System Requirements

**Desktop (Simulation Host):**
- ROS2 Jazzy Full Desktop
- Gazebo (Harmonic or newer)
- Ubuntu 24.04

**Raspberry Pi (Optional - for testing):**
- ROS2 Jazzy Base
- Ubuntu 24.04

---

## Part 1: Desktop Setup (Simulation)

### Step 1: Install Required Packages

On your **Desktop PC** with ROS2 Jazzy Full Desktop:

```bash
# Update package lists
sudo apt update

# Install Gazebo Harmonic (ROS2 Jazzy compatible)
sudo apt install ros-jazzy-ros-gz

# Install ros2_control packages
sudo apt install ros-jazzy-ros2-control \
                 ros-jazzy-ros2-controllers \
                 ros-jazzy-gz-ros2-control

# Install additional tools
sudo apt install ros-jazzy-xacro \
                 ros-jazzy-joint-state-publisher \
                 ros-jazzy-robot-state-publisher

# Install joystick support
sudo apt install ros-jazzy-joy \
                 ros-jazzy-teleop-twist-joy \
                 ros-jazzy-teleop-twist-keyboard
```

### Step 2: Verify Installation

```bash
# Source ROS2
source /opt/ros/jazzy/setup.bash

# Check if Gazebo is installed
gz sim --version

# Check ros2_control
ros2 pkg list | grep ros2_control

# You should see:
# - controller_manager
# - ros2_controllers
# - gz_ros2_control
```

---

## Part 2: Create Simulation Package

### Step 3: Create Simulation Package Structure

```bash
cd ~/evarobot_ws/src

# Create simulation package
ros2 pkg create evarobot_simulation \
  --build-type ament_cmake \
  --dependencies rclcpp std_msgs geometry_msgs sensor_msgs \
    ros2_control ros2_controllers gz_ros2_control \
    xacro robot_state_publisher

# Create directory structure
cd evarobot_simulation
mkdir -p urdf meshes worlds launch config rviz
```

### Step 4: Create Robot URDF

We'll create the robot model in stages.

**Create base URDF file:**

```bash
touch urdf/evarobot.urdf.xacro
```

**Edit `urdf/evarobot.urdf.xacro`:** (See detailed URDF in next section)

The URDF will include:
- Robot chassis
- Two differential drive wheels
- Caster wheel
- Sensors (optional IMU, lidar)
- ros2_control hardware interface
- Gazebo plugins

---

## Part 3: Configure Differential Drive Controller

### Step 5: Create Controller Configuration

```bash
touch config/controllers.yaml
```

**Edit `config/controllers.yaml`:**

```yaml
controller_manager:
  ros__parameters:
    update_rate: 50  # Hz

    # List of controllers
    joint_state_broadcaster:
      type: joint_state_broadcaster/JointStateBroadcaster

    diff_drive_controller:
      type: diff_drive_controller/DiffDriveController

# Joint State Broadcaster
joint_state_broadcaster:
  ros__parameters:
    joints:
      - left_wheel_joint
      - right_wheel_joint
    interfaces:
      - position
      - velocity

# Differential Drive Controller
diff_drive_controller:
  ros__parameters:
    # Wheel parameters
    left_wheel_names: ["left_wheel_joint"]
    right_wheel_names: ["right_wheel_joint"]

    wheel_separation: 0.194  # meters (distance between wheels)
    wheel_radius: 0.042      # meters

    # Limits
    linear:
      x:
        has_velocity_limits: true
        max_velocity: 0.5      # m/s
        has_acceleration_limits: true
        max_acceleration: 1.0  # m/s^2

    angular:
      z:
        has_velocity_limits: true
        max_velocity: 2.0      # rad/s
        has_acceleration_limits: true
        max_acceleration: 3.0  # rad/s^2

    # Topic configuration
    publish_rate: 50.0
    odom_frame_id: odom
    base_frame_id: base_link
    pose_covariance_diagonal: [0.001, 0.001, 0.0, 0.0, 0.0, 0.001]
    twist_covariance_diagonal: [0.001, 0.0, 0.0, 0.0, 0.0, 0.001]

    # Open loop odometry (no wheel encoders in simulation)
    open_loop: false

    # Enable/disable odometry
    enable_odom_tf: true

    # Command timeout
    cmd_vel_timeout: 0.5  # seconds
```

---

## Part 4: Create Launch Files

### Step 6: Simulation Launch File

```bash
touch launch/simulation.launch.py
```

**Edit `launch/simulation.launch.py`:**

```python
#!/usr/bin/env python3
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node

def generate_launch_description():
    # Get package directories
    pkg_simulation = get_package_share_directory('evarobot_simulation')

    # Paths
    urdf_file = os.path.join(pkg_simulation, 'urdf', 'evarobot.urdf.xacro')
    controller_config = os.path.join(pkg_simulation, 'config', 'controllers.yaml')
    world_file = os.path.join(pkg_simulation, 'worlds', 'empty.world')

    # Launch arguments
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')

    # Process URDF
    robot_description = Command(['xacro ', urdf_file])

    # Robot State Publisher
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robot_description,
            'use_sim_time': use_sim_time
        }]
    )

    # Gazebo
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(get_package_share_directory('ros_gz_sim'),
                        'launch', 'gz_sim.launch.py')
        ]),
        launch_arguments={'gz_args': ['-r -v4 empty.sdf']}.items()
    )

    # Spawn robot
    spawn_robot = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-name', 'evarobot',
            '-topic', 'robot_description',
            '-x', '0.0',
            '-y', '0.0',
            '-z', '0.1'
        ],
        output='screen'
    )

    # Controller Manager
    controller_manager = Node(
        package='controller_manager',
        executable='ros2_control_node',
        parameters=[controller_config],
        output='screen'
    )

    # Spawn controllers
    spawn_joint_state_broadcaster = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster'],
        output='screen'
    )

    spawn_diff_drive_controller = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['diff_drive_controller'],
        output='screen'
    )

    # Bridge for Gazebo-ROS communication
    gz_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock'
        ],
        output='screen'
    )

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        robot_state_publisher,
        gazebo,
        spawn_robot,
        gz_bridge,
        controller_manager,
        spawn_joint_state_broadcaster,
        spawn_diff_drive_controller,
    ])
```

### Step 7: Teleop Launch File

```bash
touch launch/teleop.launch.py
```

**Edit `launch/teleop.launch.py`:**

```python
#!/usr/bin/env python3
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():

    # Joy node
    joy_node = Node(
        package='joy',
        executable='joy_node',
        name='joy_node',
        parameters=[{
            'device_id': 0,
            'deadzone': 0.1,
            'autorepeat_rate': 20.0
        }]
    )

    # Teleop twist joy
    teleop_twist_joy = Node(
        package='teleop_twist_joy',
        executable='teleop_node',
        name='teleop_twist_joy',
        parameters=[{
            'axis_linear.x': 1,
            'axis_angular.yaw': 0,
            'scale_linear.x': 0.5,
            'scale_angular.yaw': 1.0,
            'enable_button': 5,  # R1 button
            'enable_turbo_button': 7  # R2 button
        }],
        remappings=[
            ('/cmd_vel', '/diff_drive_controller/cmd_vel_unstamped')
        ]
    )

    return LaunchDescription([
        joy_node,
        teleop_twist_joy
    ])
```

---

## Part 5: Build and Test

### Step 8: Build the Package

```bash
cd ~/evarobot_ws
colcon build --packages-select evarobot_simulation
source install/setup.bash
```

### Step 9: Launch Simulation

**Terminal 1 - Start Simulation:**
```bash
source ~/evarobot_ws/install/setup.bash
ros2 launch evarobot_simulation simulation.launch.py
```

**Terminal 2 - Start Teleop (with joystick):**
```bash
source ~/evarobot_ws/install/setup.bash
ros2 launch evarobot_simulation teleop.launch.py
```

**OR use keyboard teleop:**
```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard \
  --ros-args -r /cmd_vel:=/diff_drive_controller/cmd_vel_unstamped
```

### Step 10: Verify Topics and Controllers

```bash
# Check available topics
ros2 topic list

# You should see:
# /diff_drive_controller/cmd_vel_unstamped
# /diff_drive_controller/odom
# /tf
# /joint_states

# Check controller status
ros2 control list_controllers

# Check odometry
ros2 topic echo /diff_drive_controller/odom

# Send test command
ros2 topic pub /diff_drive_controller/cmd_vel_unstamped \
  geometry_msgs/msg/Twist "{linear: {x: 0.5}, angular: {z: 0.0}}"
```

---

## Part 6: Visualization with RViz2

### Step 11: Launch RViz2

```bash
rviz2
```

**In RViz2:**
1. Set Fixed Frame: `odom`
2. Add → RobotModel
3. Add → TF
4. Add → Odometry (topic: `/diff_drive_controller/odom`)
5. Save config to `evarobot_simulation/rviz/simulation.rviz`

---

## Part 7: Raspberry Pi Integration (Optional)

### Step 12: Network Configuration

**On Desktop (Simulation Host):**
```bash
# Find IP address
hostname -I
# Example: 192.168.1.100
```

**On Raspberry Pi:**
```bash
# Edit .bashrc
nano ~/.bashrc

# Add at the end:
export ROS_DOMAIN_ID=42
export ROS_LOCALHOST_ONLY=0
export ROS_MASTER_URI=http://192.168.1.100:11311

source ~/.bashrc
```

**On Desktop:**
```bash
# Edit .bashrc
nano ~/.bashrc

# Add:
export ROS_DOMAIN_ID=42
export ROS_LOCALHOST_ONLY=0

source ~/.bashrc
```

### Step 13: Test Network Connection

**On Raspberry Pi:**
```bash
# List topics from desktop
ros2 topic list

# You should see simulation topics!

# Control from Raspberry Pi
ros2 topic pub /diff_drive_controller/cmd_vel_unstamped \
  geometry_msgs/msg/Twist "{linear: {x: 0.3}, angular: {z: 0.0}}"
```

---

## Troubleshooting

### Gazebo doesn't start
```bash
# Check Gazebo installation
gz sim --version

# Try minimal world
gz sim empty.sdf
```

### Controllers not loading
```bash
# Check controller manager
ros2 control list_controllers

# Manually spawn
ros2 run controller_manager spawner diff_drive_controller

# Check configuration
ros2 param list /controller_manager
```

### Robot not moving
```bash
# Check if controller receives commands
ros2 topic echo /diff_drive_controller/cmd_vel_unstamped

# Check joint states
ros2 topic echo /joint_states

# Verify controller is active
ros2 control list_controllers
# diff_drive_controller should show "active"
```

### No odometry
```bash
# Check odom topic
ros2 topic echo /diff_drive_controller/odom

# Verify TF tree
ros2 run tf2_tools view_frames
```

---

## Next Steps

1. ✓ Install packages on desktop
2. ✓ Create simulation package
3. ✓ Build URDF (see URDF_GUIDE.md)
4. ✓ Configure controllers
5. ✓ Test in Gazebo
6. ✓ Add sensors (camera, lidar)
7. ✓ Integrate with navigation stack

---

**Author:** Kevin Medrano Ayala
**License:** BSD-3-Clause
**ROS Version:** ROS2 Jazzy
**Gazebo:** Harmonic
