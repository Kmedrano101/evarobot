# Gazebo Harmonic Migration Guide

**Migration from Gazebo Classic to Gazebo Harmonic for ROS2 Jazzy**

---

## Overview

ROS2 Jazzy uses **Gazebo Harmonic** (gz sim) instead of Gazebo Classic. This document details all changes made to support Gazebo Harmonic.

## Key Changes

| Component | Gazebo Classic (Humble/Iron) | Gazebo Harmonic (Jazzy) |
|-----------|------------------------------|-------------------------|
| Simulator | `gzserver` / `gzclient` | `gz sim` |
| ROS Package | `gazebo_ros` | `ros_gz_sim` |
| Control Plugin | `gazebo_ros2_control` | `gz_ros2_control` |
| Bridge | `gazebo_ros` | `ros_gz_bridge` |
| Hardware Interface | `gazebo_ros2_control/GazeboSystem` | `gz_ros2_control/GazeboSimSystem` |
| Plugin Library | `libgazebo_ros2_control.so` | `libgz_ros2_control-system.so` |

---

## Files Modified

### 1. **evarobot.ros2_control.xacro**

**Location:** `evarobot_description/urdf/evarobot.ros2_control.xacro`

**Changes:**

```xml
<!-- OLD (Gazebo Classic) -->
<hardware>
  <plugin>gazebo_ros2_control/GazeboSystem</plugin>
</hardware>

<plugin filename="libgazebo_ros2_control.so" name="gazebo_ros2_control">

<!-- NEW (Gazebo Harmonic) -->
<hardware>
  <plugin>gz_ros2_control/GazeboSimSystem</plugin>
</hardware>

<plugin filename="libgz_ros2_control-system.so" name="gz_ros2_control::GazeboSimROS2ControlPlugin">
```

### 2. **package.xml**

**Location:** `evarobot_description/package.xml`

**Changes:**

```xml
<!-- OLD -->
<exec_depend>gazebo_ros</exec_depend>
<exec_depend>gazebo_ros2_control</exec_depend>
<exec_depend>gazebo_plugins</exec_depend>

<!-- NEW -->
<exec_depend>ros_gz_sim</exec_depend>
<exec_depend>ros_gz_bridge</exec_depend>
<exec_depend>gz_ros2_control</exec_depend>
```

### 3. **Launch File**

**Location:** `evarobot_description/launch/gazebo.launch.py`

Updated for Gazebo Harmonic.

**Key features:**
- Uses `ros_gz_sim` to launch Gazebo
- Uses `gz sim` command instead of `gzserver`/`gzclient`
- Includes ros_gz_bridge for clock synchronization
- Controllers are loaded separately using `controller.launch.py`

---

## Installation

### Required Packages (Desktop - ROS2 Jazzy)

```bash
sudo apt update

# Gazebo Harmonic
sudo apt install gz-harmonic

# ROS2-Gazebo integration
sudo apt install ros-jazzy-ros-gz-sim \
                 ros-jazzy-ros-gz-bridge \
                 ros-jazzy-gz-ros2-control

# ROS2 Control
sudo apt install ros-jazzy-ros2-control \
                 ros-jazzy-ros2-controllers \
                 ros-jazzy-controller-manager

# Additional tools
sudo apt install ros-jazzy-xacro \
                 ros-jazzy-robot-state-publisher \
                 ros-jazzy-joint-state-publisher
```

### Verify Installation

```bash
# Check Gazebo Harmonic
gz sim --version
# Should show: Gazebo Sim, version 8.x.x

# Check ROS packages
ros2 pkg list | grep gz
# Should show:
# - ros_gz_bridge
# - ros_gz_sim
# - gz_ros2_control (may show as part of ros2_control)

# Check controller packages
ros2 pkg list | grep diff_drive
# Should show: diff_drive_controller
```

---

## Building the Package

```bash
cd ~/evarobot_ws

# Clean old build (recommended after migration)
rm -rf build/ install/ log/

# Build
colcon build --packages-select evarobot_description evarobot_controller

# Source
source install/setup.bash
```

---

## Testing

### Test 1: Launch Gazebo Harmonic Simulation

```bash
source ~/evarobot_ws/install/setup.bash

# Terminal 1: Launch Gazebo
ros2 launch evarobot_description gazebo.launch.py

# Terminal 2: Launch Controllers
ros2 launch evarobot_controller controller.launch.py use_sim_time:=true
```

**Expected:**
- Gazebo Harmonic GUI opens
- EvaRobot spawns in simulation
- Controllers load in separate terminal

**Verify:**

```bash
# In another terminal
source ~/evarobot_ws/install/setup.bash

# Check topics
ros2 topic list
# Should see:
# /clock
# /cmd_vel
# /evarobot_controller/cmd_vel_unstamped
# /evarobot_controller/odom
# /joint_states
# /robot_description
# /tf

# Check controllers
ros2 control list_controllers
# Should show:
# evarobot_controller[diff_drive_controller/DiffDriveController] active
# joint_state_broadcaster[joint_state_broadcaster/JointStateBroadcaster] active
```

### Test 2: Send Velocity Command

```bash
# Forward motion (use standard /cmd_vel topic)
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist \
  "{linear: {x: 0.3}, angular: {z: 0.0}}" -r 10

# Rotation
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist \
  "{linear: {x: 0.0}, angular: {z: 0.5}}" -r 10

# Stop (Ctrl+C to stop publishing)
```

**Expected:**
- Robot moves in Gazebo
- Odometry updates: `ros2 topic echo /evarobot_controller/odom`

### Test 3: Keyboard Teleop

```bash
# Standard /cmd_vel topic - no remapping needed!
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

**Controls:**
- `i` - Forward
- `,` - Backward
- `j` - Turn left
- `l` - Turn right
- `k` - Stop

### Test 4: Joystick Teleop

```bash
# Connect PS4 controller
cd ~/evarobot_ws/src/evarobot_utils/scripts/bluetooth
./ps4_controller.sh

# Launch joystick teleop
source ~/evarobot_ws/install/setup.bash
ros2 launch evarobot_controller joystick_teleop.launch.py use_sim_time:=true
```

**Controls:**
- Hold **R1** + Move **Left Stick** = Drive

### Test 5: RViz Visualization

```bash
# Launch simulation with RViz
ros2 launch evarobot_description gazebo.launch.py start_rviz:=true
```

**In RViz:**
1. Fixed Frame: `odom`
2. Add → RobotModel
3. Add → TF
4. Add → Odometry (topic: `/evarobot_controller/odom`)

---

## Troubleshooting

### Issue: Gazebo doesn't start

**Error:** `command 'gzserver' not found`

**Solution:**
```bash
# Check Gazebo version
gz sim --version

# If not installed:
sudo apt install gz-harmonic ros-jazzy-ros-gz-sim
```

### Issue: Controllers don't load

**Error:** `Failed to load controller 'evarobot_controller'`

**Check:**
```bash
# Verify gz_ros2_control is installed
ros2 pkg list | grep control

# Check URDF
ros2 topic echo /robot_description --once

# Manually spawn controller
ros2 run controller_manager spawner evarobot_controller
```

### Issue: Plugin not found

**Error:** `libgz_ros2_control-system.so: cannot open shared object`

**Solution:**
```bash
# Install gz_ros2_control
sudo apt install ros-jazzy-gz-ros2-control

# Verify plugin exists
find /opt/ros/jazzy -name "libgz_ros2_control*"
```

### Issue: Robot falls through ground

**Solution:**
```bash
# Increase z_pose
ros2 launch evarobot_description gazebo_harmonic.launch.py z_pose:=0.5

# Or check URDF collision models
```

### Issue: No odometry published

**Check:**
```bash
# Verify controller is running
ros2 control list_controllers

# Check odom topic rate
ros2 topic hz /evarobot_controller/odom

# Check for errors
ros2 topic echo /diagnostics
```

---

## Compatibility Matrix

| ROS2 Version | Gazebo Version | Control Plugin | Status |
|--------------|----------------|----------------|--------|
| Humble | Gazebo Classic 11 | `gazebo_ros2_control` | ✓ Works |
| Iron | Gazebo Classic 11 or Fortress | `gazebo_ros2_control` | ✓ Works |
| Jazzy | **Gazebo Harmonic** | **`gz_ros2_control`** | ✓ **Current** |
| Rolling | Gazebo Harmonic+ | `gz_ros2_control` | ✓ Works |

---

## Backward Compatibility

To support both Gazebo Classic and Harmonic, you can:

1. **Keep both plugins in URDF:**
```xml
<!-- Uncomment for Gazebo Classic -->
<!--
<plugin filename="libgazebo_ros2_control.so" name="gazebo_ros2_control">
  <parameters>$(find evarobot_controller)/config/evarobot_controllers.yaml</parameters>
</plugin>
-->

<!-- Use for Gazebo Harmonic -->
<plugin filename="libgz_ros2_control-system.so" name="gz_ros2_control::GazeboSimROS2ControlPlugin">
  <parameters>$(find evarobot_controller)/config/evarobot_controllers.yaml</parameters>
</plugin>
```

2. **Use separate launch files for different ROS versions:**
   - ROS2 Humble/Iron: `gazebo.launch.py` for Gazebo Classic
   - ROS2 Jazzy: `gazebo.launch.py` for Gazebo Harmonic (current)

---

## Next Steps

1. ✓ Test simulation with Gazebo Harmonic
2. ✓ Verify controllers work correctly
3. ✓ Test keyboard and joystick teleop
4. Add sensors (camera, lidar, IMU)
5. Integrate with navigation stack
6. Create custom worlds

---

## References

- [Gazebo Harmonic Documentation](https://gazebosim.org/docs/harmonic)
- [ROS2 Jazzy + Gazebo](https://docs.ros.org/en/jazzy/Tutorials/Advanced/Simulators/Gazebo/Gazebo.html)
- [gz_ros2_control](https://github.com/ros-controls/gz_ros2_control)
- [ros_gz](https://github.com/gazebosim/ros_gz)

---

**Author:** Kevin Medrano Ayala
**Date:** 2025-11-09
**ROS2 Version:** Jazzy
**Gazebo Version:** Harmonic
