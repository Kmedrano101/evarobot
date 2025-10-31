# 🎮 EvaRobot Controller

> ROS2 Control configuration and joystick teleoperation for EvaRobot differential drive platform

[![ROS2](https://img.shields.io/badge/ROS2-Humble-blue?logo=ros&logoColor=white)](https://docs.ros.org/en/humble/)
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

### 🎮 Joystick Control

- ✅ **PS4/Xbox Controller Support**: Pre-configured button/axis mappings
- ✅ **Deadman Switch**: Safety feature requiring button hold
- ✅ **Configurable Controls**: Easily adjust axes, scales, and deadzone
- ✅ **Multiple Input Sources**: Joystick, keyboard, navigation commands

### 🔀 Command Multiplexing

- ✅ **Priority-Based Selection**: Automatic switching between control sources
- ✅ **Timeout Protection**: Stops robot if commands cease
- ✅ **Emergency Stop**: Lock mechanism to disable all commands
- ✅ **Flexible Architecture**: Easy to add new command sources

---

## 📦 Package Contents

```
evarobot_controller/
├── config/
│   ├── evarobot_controllers.yaml    # Main controller configuration
│   └── twist_mux.yaml               # Command multiplexing config
├── launch/
│   ├── controller.launch.py         # Controller spawning
│   └── joystick_teleop.launch.py    # Joystick integration
├── include/
│   └── evarobot_controller/         # Header files (for future extensions)
├── src/                             # Source files (for future extensions)
├── CMakeLists.txt
├── package.xml
└── README.md
```

### 📄 Configuration Files

#### `evarobot_controllers.yaml`
- Controller manager configuration
- DiffDriveController parameters
- Joint state broadcaster setup
- Robot physical parameters (wheel radius, separation)
- Velocity/acceleration limits
- Comprehensive calibration guide

#### `twist_mux.yaml`
- Input source definitions
- Priority levels
- Timeout settings
- Emergency stop configuration

### 🚀 Launch Files

#### `controller.launch.py`
Spawns ROS2 controllers with proper initialization sequence.

**Arguments:**
- `use_sim_time` (default: false) - Use simulation clock
- `controller_config_file` - Path to controller configuration
- `controller_manager_timeout` (default: 10) - Startup timeout
- `start_rviz` (default: false) - Launch RViz visualization

#### `joystick_teleop.launch.py`
Sets up complete joystick control pipeline.

**Arguments:**
- `use_sim_time` (default: false) - Use simulation clock
- `joy_config` - Path to joystick configuration
- `twist_mux_config` - Path to twist mux configuration
- `joy_vel_topic` (default: joy_vel) - Joystick output topic
- `cmd_vel_topic` (default: cmd_vel) - Final command topic

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
    ros-${ROS_DISTRO}-joy-teleop \
    ros-${ROS_DISTRO}-twist-mux \
    ros-${ROS_DISTRO}-teleop-twist-joy
```

### Build Dependencies
- `ament_cmake`
- `rclcpp`
- `hardware_interface`
- `pluginlib`
- `rclcpp_lifecycle`

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
evarobot_base_controller:
  ros__parameters:
    wheel_separation: 0.194  # Distance between wheels (meters)
    wheel_radius: 0.042      # Wheel radius (meters)
```

**⚠️ Important:** These values are calibrated for the actual EvaRobot. Adjust if your robot has different dimensions.

### Velocity Limits

Current configuration:

| Parameter | Value | Notes |
|-----------|-------|-------|
| Max Linear Velocity | 0.5 m/s | Forward/backward speed |
| Max Angular Velocity | 2.0 rad/s | ~115°/s rotation speed |
| Max Linear Acceleration | 0.8 m/s² | Smooth starts/stops |
| Max Angular Acceleration | 3.0 rad/s² | Smooth rotation |

### Joystick Configuration

Joystick settings are in `evarobot_firmware/config/joy_config.yaml`:

**Default PS4 Controller Mapping:**
- **R1 (Button 5)**: Deadman switch (hold to enable)
- **Left Stick Vertical (Axis 1)**: Linear velocity (forward/backward)
- **Right Stick Horizontal (Axis 0)**: Angular velocity (rotation)

**Key Parameters:**
```yaml
deadzone: 0.05           # Joystick drift threshold
autorepeat_rate: 20.0    # Update frequency (Hz)
scale (linear): 0.5      # Max linear speed multiplier
scale (angular): 1.5     # Max angular speed multiplier
```

### Command Priority System

Priority levels in twist_mux (higher = more important):

| Priority | Source | Use Case |
|----------|--------|----------|
| 255 | Emergency Stop | Safety lock |
| 100 | Joystick | Manual teleoperation |
| 90 | Keyboard | Manual teleoperation |
| 85 | Web Interface | Remote control |
| 80 | Navigation | Autonomous navigation |
| 50 | Safety Override | Emergency intervention |

---

## 🚀 Usage

### Basic Usage (Real Robot)

#### Step 1: Launch Controllers

```bash
ros2 launch evarobot_controller controller.launch.py
```

This spawns:
- `joint_state_broadcaster` - Publishes joint states
- `evarobot_base_controller` - Controls robot base

#### Step 2: Verify Controllers

```bash
# Check controller status
ros2 control list_controllers

# Expected output:
# evarobot_base_controller[diff_drive_controller/DiffDriveController] active
# joint_state_broadcaster[joint_state_broadcaster/JointStateBroadcaster] active
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
# Terminal 1: Controllers
ros2 launch evarobot_controller controller.launch.py

# Terminal 2: Joystick
ros2 launch evarobot_controller joystick_teleop.launch.py
```

#### Test Joystick Connection

```bash
# List joystick devices
ls /dev/input/js*

# Test joystick (no ROS)
jstest /dev/input/js0

# Monitor joystick in ROS
ros2 topic echo /joy
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
ros2 control load_controller evarobot_base_controller
ros2 control set_controller_state evarobot_base_controller start
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

3. **Check deadman button**
   - Hold R1 (button 5) while moving sticks
   - Verify in joy_config.yaml: `deadman_buttons: [5]`

4. **Monitor joy_vel output**
   ```bash
   ros2 topic echo /joy_vel
   # Should show messages only when deadman pressed
   ```

5. **Check twist_mux**
   ```bash
   ros2 topic echo /cmd_vel_mux/selected
   # Should show "joystick" as active source
   ```

### Issue: Robot Moves Erratically

**Possible Causes:**

1. **High velocity limits**
   - Reduce max_velocity in `evarobot_controllers.yaml`

2. **Joystick drift**
   - Increase deadzone in `joy_config.yaml`

3. **Multiple command sources**
   - Check twist_mux active source
   - Ensure only one source is publishing

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

**Last Updated:** 2025-10-30

---

<p align="center">
  Made with ❤️ for the EvaRobot project
</p>
