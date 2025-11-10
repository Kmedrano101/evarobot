# 🎮 EvaRobot Controller

> ROS2 Control configuration and joystick teleoperation for EvaRobot differential drive platform

[![ROS2](https://img.shields.io/badge/ROS2-Jazzy-blue?logo=ros&logoColor=white)](https://docs.ros.org/en/jazzy/)
[![License](https://img.shields.io/badge/license-BSD--3--Clause-green)](LICENSE)
[![Platform](https://img.shields.io/badge/platform-Linux-lightgrey)](https://www.linux.org/)

---

## 📋 Table of Contents

- [Overview](#-overview)
- [Features](#-features)
- [Package Contents](#-package-contents)
- [Dependencies](#-dependencies)
- [Installation](#-installation)
- [Configuration](#-configuration)
- [Usage](#-usage)
- [Testing](#-testing)
- [Troubleshooting](#-troubleshooting)
- [Architecture](#-architecture)
- [Author](#-author)

---

## 🌟 Overview

The `evarobot_controller` package provides a complete ROS2 Control implementation for the EvaRobot differential drive mobile robot. It includes controller configuration, joystick teleoperation, and velocity command multiplexing to support multiple control sources.

This package uses the standard **DiffDriveController** from `ros2_controllers`, providing:
- Robust odometry computation
- Velocity/acceleration limiting
- TF publishing (odom → base_footprint)
- Hardware abstraction through ROS2 Control

---

## ✨ Features

### 🎯 Core Features

- ✅ **Differential Drive Control**: Standard ROS2 DiffDriveController implementation
- ✅ **Joint State Broadcasting**: Real-time joint position and velocity feedback
- ✅ **Odometry Publishing**: Integrated wheel odometry with TF support
- ✅ **Velocity Limiting**: Configurable linear and angular velocity/acceleration limits
- ✅ **Hardware Abstraction**: Compatible with both simulation (Gazebo) and real hardware
- ✅ **Simple Controller Relay**: Automatic Twist→TwistStamped conversion for /cmd_vel compatibility

### 🎮 PS4 Joystick Control

- ✅ **PS4 Controller Support**: Full PlayStation 4 controller integration
- ✅ **No Deadman Button Required**: Direct, intuitive control
- ✅ **Speed Adjustment**: D-pad controls for dynamic speed changes
- ✅ **Low Latency**: Optimized for responsive robot control
- ✅ **QoS Optimized**: Fixed RELIABLE QoS for simulation compatibility

### 📡 Standard ROS Integration

- ✅ **Standard /cmd_vel Topic**: Direct compatibility with teleop tools
- ✅ **Keyboard Teleop Ready**: Works with teleop_twist_keyboard out-of-box
- ✅ **Nav2 Compatible**: Ready for autonomous navigation integration
- ✅ **Message Type Conversion**: Seamless Twist/TwistStamped handling

---

## 📦 Package Contents

```
evarobot_controller/
├── evarobot_controller/             # Python package
│   ├── __init__.py
│   ├── simple_controller.py         # Twist->TwistStamped relay
│   └── joy_teleop_node.py           # PS4 joystick control
├── config/
│   ├── evarobot_controllers.yaml    # Main controller configuration
│   └── teleop_twist_joy.yaml        # Joystick configuration (reference)
├── launch/
│   ├── controller.launch.py         # Controller spawning
│   └── joystick_teleop.launch.py    # PS4 joystick integration
├── CMakeLists.txt
├── package.xml
└── README.md
```

### 📄 Configuration Files

#### `evarobot_controllers.yaml`
- Controller manager configuration
- DiffDriveController parameters (evarobot_controller)
- Joint state broadcaster setup
- Robot physical parameters (wheel radius, separation)
- Velocity/acceleration limits
- TwistStamped message configuration (use_stamped_vel: true)
- Comprehensive calibration guide

#### `teleop_twist_joy.yaml`
- PS4 controller axis mappings
- Speed configuration
- Reference configuration (not actively used)

### 🚀 Launch Files

#### `controller.launch.py`
Spawns ROS2 controllers with proper initialization sequence.

**Arguments:**
- `use_sim_time` (default: false) - Use simulation clock
- `controller_config_file` - Path to controller configuration
- `controller_manager_timeout` (default: 10) - Startup timeout
- `start_rviz` (default: false) - Launch RViz visualization

#### `joystick_teleop.launch.py`
Sets up PS4 joystick control for direct robot control.

**Arguments:**
- `use_sim_time` (default: false) - Use simulation clock
- `joy_config` - Path to joystick configuration
- `linear_speed` (default: 0.5) - Maximum linear speed (m/s)
- `angular_speed` (default: 1.5) - Maximum angular speed (rad/s)

---

## 📦 Dependencies

### Required ROS2 Packages

```bash
sudo apt-get install -y \
    ros-${ROS_DISTRO}-controller-manager \
    ros-${ROS_DISTRO}-diff-drive-controller \
    ros-${ROS_DISTRO}-joint-state-broadcaster \
    ros-${ROS_DISTRO}-ros2-control \
    ros-${ROS_DISTRO}-ros2-controllers \
    ros-${ROS_DISTRO}-joy \
    ros-${ROS_DISTRO}-teleop-twist-keyboard
```

### Build Dependencies
- `ament_cmake`
- `ament_cmake_python`
- `rclpy`
- `geometry_msgs`

---

## 🔧 Installation

### 1. Clone Repository

```bash
cd ~/evarobot_ws/src
# Repository should already be cloned as part of evarobot project
```

### 2. Install Dependencies

```bash
cd ~/evarobot_ws
rosdep install --from-paths src --ignore-src -r -y
```

### 3. Build Package

```bash
cd ~/evarobot_ws
colcon build --packages-select evarobot_controller
source install/setup.bash
```

---

## ⚙️ Configuration

### Robot Physical Parameters

The robot's physical dimensions are configured in `config/evarobot_controllers.yaml`:

```yaml
evarobot_controller:
  ros__parameters:
    wheel_separation: 0.194  # Distance between wheels (meters)
    wheel_radius: 0.042      # Wheel radius (meters)
    use_stamped_vel: true    # Use TwistStamped messages
```

**⚠️ Important:** These values must match the URDF model. The simple_controller node automatically converts standard Twist messages to TwistStamped for the controller.

### Velocity Limits

Current configuration:

| Parameter | Value | Notes |
|-----------|-------|-------|
| Max Linear Velocity | 0.5 m/s | Forward/backward speed |
| Max Angular Velocity | 2.0 rad/s | ~115°/s rotation speed |
| Max Linear Acceleration | 0.8 m/s² | Smooth starts/stops |
| Max Angular Acceleration | 3.0 rad/s² | Smooth rotation |

### PS4 Joystick Configuration

**Controller Mapping:**
- **Left Stick Horizontal (Axis 0)**: Angular velocity (rotation left/right)
- **Right Stick Vertical (Axis 4)**: Linear velocity (forward/backward)
- **D-pad Up**: Increase speed dynamically
- **D-pad Down**: Decrease speed dynamically
- **NO Deadman Button Required**: Direct, responsive control

**Key Parameters:**
```yaml
linear_speed: 0.5        # Max linear velocity (m/s)
angular_speed: 1.5       # Max angular velocity (rad/s)
joy_deadzone: 0.1        # Joystick drift threshold
speed_increment: 0.1     # Speed adjustment step
```

### simple_controller Node

The `simple_controller.py` node acts as a relay between standard ROS tools and the controller:

**Function:**
- Subscribes to `/cmd_vel` (geometry_msgs/Twist)
- Converts to TwistStamped with proper timestamps
- Publishes to `/evarobot_controller/cmd_vel` (geometry_msgs/TwistStamped)
- Uses RELIABLE QoS for simulation compatibility

**Why:**
- Enables use of standard teleop tools (keyboard, joystick)
- Maintains compatibility with Nav2 and other ROS packages
- Handles message type conversion transparently

---

## 🚀 Usage

### Basic Usage (Real Robot)

#### Step 1: Launch Controllers

```bash
ros2 launch evarobot_controller controller.launch.py
```

This spawns:
- `joint_state_broadcaster` - Publishes joint states
- `evarobot_controller` - Controls robot base
- `simple_controller` - Relays /cmd_vel to controller

#### Step 2: Verify Controllers

```bash
# Check controller status
ros2 control list_controllers

# Expected output:
# evarobot_controller[diff_drive_controller/DiffDriveController] active
# joint_state_broadcaster[joint_state_broadcaster/JointStateBroadcaster] active

# Check simple_controller is running
ros2 node list | grep simple_controller
```

#### Step 3: Test Movement

```bash
# Send velocity command
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist \
    "{linear: {x: 0.2}, angular: {z: 0.0}}" --once
```

### Joystick Control

#### Launch Complete Joystick System

```bash
# Terminal 1: Gazebo (if using simulation)
ros2 launch evarobot_description gazebo.launch.py

# Terminal 2: Controllers
ros2 launch evarobot_controller controller.launch.py use_sim_time:=true

# Terminal 3: PS4 Joystick
ros2 launch evarobot_controller joystick_teleop.launch.py use_sim_time:=true
```

**Controls:**
- Move left stick horizontal: Rotate robot
- Move right stick vertical: Drive forward/backward
- Press D-pad up/down: Adjust speed

#### Test Joystick Connection

```bash
# List joystick devices
ls /dev/input/js*

# Test joystick (no ROS)
jstest /dev/input/js0

# Monitor joystick in ROS
ros2 topic echo /joy

# Monitor velocity commands
ros2 topic echo /cmd_vel
```

### Simulation (Gazebo)

```bash
# Terminal 1: Launch Gazebo
ros2 launch evarobot_description gazebo.launch.py world:=test_world

# Terminal 2: Launch controllers (with sim time)
ros2 launch evarobot_controller controller.launch.py use_sim_time:=true

# Terminal 3: Joystick control (optional)
ros2 launch evarobot_controller joystick_teleop.launch.py use_sim_time:=true
```

---

## 🧪 Testing

### Test 1: Controller Status

```bash
# List all controllers
ros2 control list_controllers

# List hardware interfaces
ros2 control list_hardware_interfaces
```

### Test 2: Topic Communication

```bash
# Check available topics
ros2 topic list | grep -E "cmd_vel|odom|joint"

# Monitor odometry
ros2 topic echo /odom

# Monitor joint states
ros2 topic echo /joint_states
```

### Test 3: Manual Control

```bash
# Forward motion
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist \
    "{linear: {x: 0.3}}" --rate 10

# Rotation
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist \
    "{angular: {z: 0.5}}" --rate 10

# Combined motion (arc)
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist \
    "{linear: {x: 0.2}, angular: {z: 0.3}}" --rate 10
```

### Test 4: Twist Mux Priority

```bash
# Terminal 1: High priority (joystick)
ros2 topic pub /joy_vel geometry_msgs/msg/Twist \
    "{linear: {x: 0.3}}" --rate 10

# Terminal 2: Lower priority (navigation) - should be ignored
ros2 topic pub /nav_vel geometry_msgs/msg/Twist \
    "{linear: {x: 0.1}}" --rate 10

# Check active source
ros2 topic echo /cmd_vel_mux/selected
```

### Test 5: Emergency Stop

```bash
# Activate emergency stop
ros2 topic pub /emergency_stop std_msgs/msg/Bool "{data: true}" --once

# Try to move (should be blocked)
ros2 topic pub /joy_vel geometry_msgs/msg/Twist \
    "{linear: {x: 0.3}}" --once

# Release emergency stop
ros2 topic pub /emergency_stop std_msgs/msg/Bool "{data: false}" --once
```

---

## 🔧 Troubleshooting

### Issue: Controllers Not Loading

**Symptoms:** Controllers fail to spawn or show as "inactive"

**Solutions:**
```bash
# Check controller manager is running
ros2 node list | grep controller_manager

# Check for errors in controller configuration
ros2 param list /controller_manager

# Restart controllers
ros2 control load_controller evarobot_controller
ros2 control set_controller_state evarobot_controller start
```

### Issue: Robot Doesn't Move

**Possible Causes & Solutions:**

1. **No velocity commands received**
   ```bash
   ros2 topic echo /cmd_vel  # Should show messages
   ```

2. **Controllers not active**
   ```bash
   ros2 control list_controllers  # Should show "active"
   ```

3. **Hardware interface not connected**
   ```bash
   ros2 control list_hardware_interfaces
   ```

4. **Emergency stop active**
   ```bash
   ros2 topic pub /emergency_stop std_msgs/msg/Bool "{data: false}" --once
   ```

### Issue: Joystick Not Working

**Symptoms:** Robot doesn't respond to joystick input

**Solutions:**

1. **Check device permissions**
   ```bash
   ls -l /dev/input/js0
   sudo chmod a+rw /dev/input/js0
   # Or add user to input group:
   sudo usermod -a -G input $USER
   ```

2. **Verify joystick publishes**
   ```bash
   ros2 topic echo /joy
   # Move joystick - should see values changing
   ```

3. **Check node is running**
   ```bash
   ros2 node list | grep joy_teleop_node
   ```

4. **Monitor /cmd_vel output**
   ```bash
   ros2 topic echo /cmd_vel
   # Should show messages when moving joystick sticks
   ```

5. **Verify simple_controller relay**
   ```bash
   ros2 node info /simple_controller
   # Check subscriptions and publications
   ```

### Issue: Robot Moves Erratically

**Possible Causes:**

1. **High velocity limits**
   - Reduce max_velocity in `evarobot_controllers.yaml`

2. **Joystick drift**
   - Increase joy_deadzone in joy_teleop_node parameters

3. **Multiple command sources**
   - Ensure only one velocity source is publishing to /cmd_vel
   - Stop other teleop nodes if running

### Issue: Odometry Drift

**Symptoms:** Robot position in /odom doesn't match reality

**Solutions:**

1. **Check wheel parameters**
   - Measure actual wheel_radius and wheel_separation
   - Update in `evarobot_controllers.yaml`

2. **Calibrate multipliers**
   ```yaml
   wheel_separation_multiplier: 1.0  # Adjust if rotation incorrect
   left_wheel_radius_multiplier: 1.0  # Adjust if veers left
   right_wheel_radius_multiplier: 1.0 # Adjust if veers right
   ```

3. **Test straight line**
   ```bash
   ros2 topic pub /cmd_vel geometry_msgs/msg/Twist \
       "{linear: {x: 0.2}}" --rate 10
   ```
   - If robot veers left: increase left_wheel_radius_multiplier
   - If robot veers right: increase right_wheel_radius_multiplier

4. **Test rotation**
   ```bash
   ros2 topic pub /cmd_vel geometry_msgs/msg/Twist \
       "{angular: {z: 1.0}}" --rate 10
   ```
   - Command 2π rad (360°) rotation
   - If rotates too much: increase wheel_separation_multiplier
   - If rotates too little: decrease wheel_separation_multiplier

---

## 🏗️ Architecture

### ROS2 Control Architecture

```
┌─────────────────────────────────────────────────────────────┐
│                     Controller Manager                       │
│  ┌────────────────────────────────────────────────────────┐ │
│  │          Joint State Broadcaster                       │ │
│  │  (Publishes /joint_states)                            │ │
│  └────────────────────────────────────────────────────────┘ │
│  ┌────────────────────────────────────────────────────────┐ │
│  │          DiffDriveController                           │ │
│  │  Input:  /cmd_vel (Twist)                             │ │
│  │  Output: /odom (Odometry), /tf (odom→base_footprint)  │ │
│  │  Control: Wheel velocities to hardware interface       │ │
│  └────────────────────────────────────────────────────────┘ │
└─────────────────────────────────────────────────────────────┘
                              ↕
┌─────────────────────────────────────────────────────────────┐
│                   Hardware Interface                         │
│  (Gazebo or Real Hardware)                                  │
│  - Receives wheel velocity commands                         │
│  - Provides wheel position/velocity feedback                │
└─────────────────────────────────────────────────────────────┘
```

### Teleoperation Architecture

```
┌──────────┐      ┌──────────────┐      ┌─────────────┐
│ Joystick │──────>│ Joy Node     │──────>│ Joy Teleop  │
│ Device   │ /joy  │              │ /joy  │             │
└──────────┘      └──────────────┘      └─────────────┘
                                                │
                                                │ /joy_vel
                                                ↓
                  ┌────────────────────────────────────────┐
                  │         Twist Mux                      │
                  │  ┌──────────────────────────────────┐ │
                  │  │ Priority Selection:               │ │
                  │  │  100: /joy_vel                   │ │
                  │  │   90: /key_vel                   │ │
                  │  │   80: /nav_vel                   │ │
                  │  └──────────────────────────────────┘ │
                  └────────────────────────────────────────┘
                                  │
                                  │ /cmd_vel
                                  ↓
                  ┌────────────────────────────────────────┐
                  │     DiffDriveController                │
                  └────────────────────────────────────────┘
```

### Topic Flow

| Topic | Type | Publisher | Subscriber | Description |
|-------|------|-----------|------------|-------------|
| `/joy` | sensor_msgs/Joy | joy_node | joy_teleop | Raw joystick data |
| `/joy_vel` | geometry_msgs/Twist | joy_teleop | twist_mux | Joystick velocities |
| `/key_vel` | geometry_msgs/Twist | teleop_twist_keyboard | twist_mux | Keyboard velocities |
| `/nav_vel` | geometry_msgs/Twist | nav2 | twist_mux | Navigation velocities |
| `/cmd_vel` | geometry_msgs/Twist | twist_mux | diff_drive_controller | Final velocities |
| `/odom` | nav_msgs/Odometry | diff_drive_controller | localization | Wheel odometry |
| `/joint_states` | sensor_msgs/JointState | joint_state_broadcaster | robot_state_publisher | Joint feedback |
| `/tf` | tf2_msgs/TFMessage | diff_drive_controller | navigation | Transforms |

---

## 📝 Calibration Guide

### Step-by-Step Calibration

#### 1. Measure Physical Parameters

```bash
# Measure wheel diameter with ruler/caliper
# wheel_radius = diameter / 2
# Example: 84mm diameter → 0.042m radius

# Measure wheel separation (center to center)
# Use ruler to measure distance between wheel contact points
# Example: 194mm → 0.194m
```

#### 2. Update Configuration

Edit `config/evarobot_controllers.yaml`:
```yaml
wheel_separation: 0.194  # Your measurement
wheel_radius: 0.042      # Your measurement
```

#### 3. Test Straight Line Motion

```bash
# Drive forward 1 meter
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist \
    "{linear: {x: 0.2}}" --rate 10

# Observe:
# - If veers left: increase left_wheel_radius_multiplier
# - If veers right: increase right_wheel_radius_multiplier
```

#### 4. Test Rotation

```bash
# Rotate 360 degrees (2π radians)
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist \
    "{angular: {z: 1.0}}" --rate 10

# Run for approximately 6.28 seconds (2π / 1.0)

# Observe:
# - If rotates too much: increase wheel_separation_multiplier
# - If rotates too little: decrease wheel_separation_multiplier
```

#### 5. Fine-Tune

Adjust multipliers in small increments (0.01-0.02):
```yaml
wheel_separation_multiplier: 1.02  # If rotated 5% too much
left_wheel_radius_multiplier: 0.98  # If veered left slightly
```

---

## 🤝 Contributing

Contributions are welcome! Areas for improvement:

- [ ] Add IMU sensor fusion
- [ ] Implement velocity smoothing filters
- [ ] Add support for more controller types
- [ ] Create automated calibration script
- [ ] Add comprehensive unit tests

---

## 📄 License

This package is licensed under the BSD-3-Clause License. See the [LICENSE](../../LICENSE) file for details.

---

## 👤 Author

**Kevin Medrano Ayala**
- Email: kevin.ejem18@gmail.com
- GitHub: [@kmedrano](https://github.com/kmedrano)

---

## 📚 Additional Resources

### Official Documentation
- [ROS2 Control](https://control.ros.org/)
- [DiffDriveController](https://control.ros.org/master/doc/ros2_controllers/diff_drive_controller/doc/userdoc.html)
- [Twist Mux](http://wiki.ros.org/twist_mux)

### Related Packages
- [evarobot_description](../evarobot_description/README.md) - Robot URDF and Gazebo simulation
- [evarobot_firmware](../evarobot_firmware/README.md) - Arduino motor control firmware
- [evarobot_hardware](../evarobot_hardware/README.md) - Hardware interface nodes

---

## 📊 Current Status

| Feature | Status | Notes |
|---------|--------|-------|
| DiffDriveController | ✅ Complete | Fully configured and tested |
| Joint State Broadcaster | ✅ Complete | Active and publishing |
| Joystick Teleoperation | ✅ Complete | PS4/Xbox support |
| Twist Mux | ✅ Complete | Multi-source command handling |
| Gazebo Integration | ✅ Complete | Simulation ready |
| Hardware Interface | 🚧 In Progress | See evarobot_hardware package |
| Sensor Fusion | ⏳ Planned | IMU + wheel odometry |
| Navigation Integration | ⏳ Planned | Nav2 compatibility |

**Last Updated:** 2025-11-10

### Recent Changes
- ✅ **simple_controller**: Added Twist→TwistStamped relay for /cmd_vel compatibility
- ✅ **PS4 Joystick**: Direct control without deadman button, D-pad speed adjustment
- ✅ **Simplified Architecture**: Removed twist_mux dependency for streamlined control
- ✅ **Fixed QoS**: RELIABLE QoS for simulation compatibility

---

<p align="center">
  Made with ❤️ for the EvaRobot project
</p>
