# EvaRobot Differential Drive Controller Testing Guide

Complete step-by-step guide to test the differential drive controller with joystick and keyboard teleop using existing packages.

---

## Overview

Your evarobot_ws already includes:
- **evarobot_description**: Robot URDF, Gazebo simulation
- **evarobot_controller**: Differential drive controller, teleop configurations

**What we'll test:**
1. ✓ Gazebo simulation with differential drive controller
2. ✓ Joystick teleop (PS4 controller)
3. ✓ Keyboard teleop
4. ✓ Network communication between Desktop and Raspberry Pi

---

## Prerequisites

### Desktop PC (Full ROS2 Jazzy Desktop)

Install required packages:

```bash
# ROS2 control and Gazebo
sudo apt update
sudo apt install ros-jazzy-ros2-control \
                 ros-jazzy-ros2-controllers \
                 ros-jazzy-gazebo-ros2-control \
                 ros-jazzy-gazebo-ros-pkgs

# Teleop packages
sudo apt install ros-jazzy-joy \
                 ros-jazzy-joy-teleop \
                 ros-jazzy-teleop-twist-keyboard \
                 ros-jazzy-twist-mux

# Visualization
sudo apt install ros-jazzy-rviz2 \
                 ros-jazzy-robot-state-publisher \
                 ros-jazzy-joint-state-publisher-gui
```

### Raspberry Pi (ROS2 Jazzy Base)

Install joystick support:

```bash
sudo apt update
sudo apt install ros-jazzy-joy \
                 ros-jazzy-teleop-twist-keyboard \
                 jstest-gtk
```

---

## Part 1: Desktop Simulation Testing

### Step 1: Build Packages

On your Desktop:

```bash
cd ~/evarobot_ws

# Build the necessary packages
colcon build --packages-select evarobot_description \
                                evarobot_controller \
                                evarobot_firmware

# Source the workspace
source install/setup.bash
```

### Step 2: Launch Gazebo Simulation

**Terminal 1 - Start Gazebo with Robot:**

```bash
source ~/evarobot_ws/install/setup.bash

# Launch Gazebo simulation
ros2 launch evarobot_description gazebo.launch.py
```

**What to expect:**
- Gazebo opens with EvaRobot spawned
- Robot should be visible in simulation
- Check console for any errors

**Verify simulation:**

```bash
# In a new terminal
ros2 topic list

# You should see:
# /joint_states
# /robot_description
# /tf
# /tf_static
```

### Step 3: Launch Controller

**Terminal 2 - Start Controllers:**

```bash
source ~/evarobot_ws/install/setup.bash

# Launch the differential drive controller
ros2 launch evarobot_controller controller.launch.py use_sim_time:=true
```

**What to expect:**
- Controller manager starts
- `joint_state_broadcaster` loads
- `evarobot_base_controller` (diff drive) loads

**Verify controllers:**

```bash
# Check controller status
ros2 control list_controllers

# Expected output:
# evarobot_base_controller[diff_drive_controller/DiffDriveController] active
# joint_state_broadcaster[joint_state_broadcaster/JointStateBroadcaster] active
```

**Check topics:**

```bash
ros2 topic list

# You should now see additional topics:
# /evarobot_base_controller/cmd_vel_unstamped  (controller input)
# /evarobot_base_controller/odom               (odometry output)
# /cmd_vel                                     (from twist_mux)
```

---

## Part 2: Keyboard Teleop Testing

### Step 4: Test with Keyboard

**Terminal 3 - Keyboard Teleop:**

```bash
source ~/evarobot_ws/install/setup.bash

# Launch keyboard teleop
ros2 run teleop_twist_keyboard teleop_twist_keyboard \
  --ros-args -r /cmd_vel:=/evarobot_base_controller/cmd_vel_unstamped
```

**Or use twist_mux version:**

```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard
# This publishes to /cmd_vel which twist_mux forwards to controller
```

**Controls:**
```
Reading from keyboard...
---------------------------
Moving around:
   u    i    o
   j    k    l
   m    ,    .

q/z : increase/decrease max speeds by 10%
w/x : increase/decrease only linear speed by 10%
e/c : increase/decrease only angular speed by 10%
space key, k : force stop
anything else : stop smoothly

CTRL-C to quit
```

**Test sequence:**
1. Press `i` - Robot should move forward in Gazebo
2. Press `,` - Robot should move backward
3. Press `j` - Robot should turn left
4. Press `l` - Robot should turn right
5. Press `k` or `space` - Robot should stop

**Monitor odometry:**

```bash
# In another terminal
ros2 topic echo /evarobot_base_controller/odom
```

---

## Part 3: Joystick Teleop Testing

### Step 5: Connect PS4 Controller

**On Desktop or Raspberry Pi:**

```bash
# Pair controller (if not already paired)
cd ~/evarobot_ws/src/evarobot_utils/scripts/bluetooth
./ps4_controller.sh
# Select option 1 to pair

# Verify joystick is detected
ls /dev/input/js*
# Should show: /dev/input/js0

# Test joystick
jstest /dev/input/js0
# Move sticks and press buttons to verify
```

### Step 6: Launch Joystick Teleop

**Terminal 4 - Joystick Teleop:**

```bash
source ~/evarobot_ws/install/setup.bash

# Launch joystick teleop
ros2 launch evarobot_controller joystick_teleop.launch.py use_sim_time:=true
```

**What launches:**
- `joy_node` - Reads joystick input → `/joy`
- `joy_teleop` - Converts joy to velocity → `/joy_vel`
- `twist_mux` - Multiplexes inputs → `/cmd_vel` → controller

**PS4 Controller Mapping:**
- **Hold R1** (deadman switch) - Required to send commands
- **Left Stick Vertical** - Forward/Backward (linear.x)
- **Right Stick Horizontal** - Rotate Left/Right (angular.z)
- **R2** - Turbo mode (faster speeds)

**Test sequence:**
1. Hold R1 + Push left stick up → Forward
2. Hold R1 + Push left stick down → Backward
3. Hold R1 + Push right stick left → Rotate left
4. Hold R1 + Push right stick right → Rotate right
5. Release R1 → Stop

**Debug joystick:**

```bash
# Check raw joystick input
ros2 topic echo /joy

# Check joy_teleop output
ros2 topic echo /joy_vel

# Check twist_mux output
ros2 topic echo /cmd_vel

# Check which input is active
ros2 topic echo /twist_mux/diagnostics
```

---

## Part 4: Visualization with RViz2

### Step 7: Launch RViz2

**Terminal 5 - RViz:**

```bash
source ~/evarobot_ws/install/setup.bash
rviz2
```

**Configure RViz:**

1. **Set Fixed Frame:** `odom`

2. **Add displays:**
   - Add → RobotModel
     - Description Topic: `/robot_description`

   - Add → TF
     - Show Names: ✓

   - Add → Odometry
     - Topic: `/evarobot_base_controller/odom`
     - Covariance: Position ✓

   - Add → Path (optional)
     - Topic: `/evarobot_base_controller/odom` (via Odometry)

3. **Save config:**
   - File → Save Config As → `~/evarobot_ws/src/evarobot_viz/rviz/teleop_test.rviz`

**Now drive the robot** and watch:
- Robot model moves in RViz
- TF frames update
- Odometry path shows trajectory
- Gazebo shows physical simulation

---

## Part 5: Raspberry Pi Integration

### Step 8: Network Setup

**Find Desktop IP:**

```bash
# On Desktop
hostname -I
# Example output: 192.168.1.100
```

**Configure ROS2 Domain:**

**On Desktop - Edit `~/.bashrc`:**

```bash
nano ~/.bashrc

# Add at the end:
export ROS_DOMAIN_ID=42
export ROS_LOCALHOST_ONLY=0

# Save and exit (Ctrl+X, Y, Enter)
source ~/.bashrc
```

**On Raspberry Pi - Edit `~/.bashrc`:**

```bash
nano ~/.bashrc

# Add at the end:
export ROS_DOMAIN_ID=42
export ROS_LOCALHOST_ONLY=0

# Save and exit
source ~/.bashrc
```

### Step 9: Test Network Communication

**On Raspberry Pi:**

```bash
# Source workspace
source ~/evarobot_ws/install/setup.bash

# List topics from Desktop
ros2 topic list

# You should see topics from Desktop simulation!
# /evarobot_base_controller/cmd_vel_unstamped
# /evarobot_base_controller/odom
# /joint_states
# etc.

# Test: Send command from Raspberry Pi
ros2 topic pub --once /evarobot_base_controller/cmd_vel_unstamped \
  geometry_msgs/msg/Twist "{linear: {x: 0.3}, angular: {z: 0.0}}"

# Robot in Desktop Gazebo should move!
```

### Step 10: Control from Raspberry Pi

**Option A: Keyboard on Raspberry Pi controlling Desktop simulation:**

```bash
# On Raspberry Pi
source ~/evarobot_ws/install/setup.bash

ros2 run teleop_twist_keyboard teleop_twist_keyboard \
  --ros-args -r /cmd_vel:=/evarobot_base_controller/cmd_vel_unstamped
```

**Option B: Joystick on Raspberry Pi:**

```bash
# On Raspberry Pi - pair PS4 controller
cd ~/evarobot_ws/src/evarobot_utils/scripts/bluetooth
./ps4_controller.sh

# Launch joystick teleop
source ~/evarobot_ws/install/setup.bash
ros2 launch evarobot_controller joystick_teleop.launch.py use_sim_time:=true
```

**Desktop Gazebo shows robot moving from Raspberry Pi commands!**

---

## Part 6: Diagnostics and Monitoring

### Useful Commands

**Check controller status:**

```bash
ros2 control list_controllers
ros2 control list_hardware_interfaces
```

**Monitor topics:**

```bash
# Robot velocity commands
ros2 topic echo /evarobot_base_controller/cmd_vel_unstamped

# Odometry
ros2 topic echo /evarobot_base_controller/odom

# Joint states
ros2 topic echo /joint_states

# TF tree
ros2 run tf2_tools view_frames
# Generates frames.pdf
```

**Check topic rates:**

```bash
ros2 topic hz /evarobot_base_controller/odom
# Should be ~50 Hz

ros2 topic hz /joy
# Should be ~20 Hz when joystick active
```

**Remap command topics:**

```bash
# Send direct command
ros2 topic pub /evarobot_base_controller/cmd_vel_unstamped \
  geometry_msgs/msg/Twist "{linear: {x: 0.2}, angular: {z: 0.5}}" -r 10
```

---

## Part 7: Tuning and Calibration

### Adjust Controller Parameters

Edit `evarobot_controller/config/evarobot_controllers.yaml`:

**If robot doesn't drive straight:**

```yaml
evarobot_base_controller:
  ros__parameters:
    # Adjust wheel radius multipliers
    left_wheel_radius_multiplier: 1.02   # Increase if robot veers right
    right_wheel_radius_multiplier: 1.0
```

**If robot rotates incorrectly:**

```yaml
    # Adjust wheel separation
    wheel_separation_multiplier: 1.05  # Increase if rotates too little
```

**Adjust speed limits:**

```yaml
    linear:
      x:
        max_velocity: 0.8  # Increase for faster forward speed

    angular:
      z:
        max_velocity: 3.0  # Increase for faster rotation
```

**After changes:**

```bash
cd ~/evarobot_ws
colcon build --packages-select evarobot_controller
source install/setup.bash

# Restart controller
ros2 launch evarobot_controller controller.launch.py use_sim_time:=true
```

---

## Troubleshooting

### Gazebo Issues

**Robot falls through ground:**
- Check `z_pose` is positive (default 0.1)
- Verify URDF collision models

**Robot doesn't spawn:**
```bash
# Check robot_description topic
ros2 topic echo /robot_description --once

# Check spawn errors in Gazebo console
```

### Controller Issues

**Controllers don't load:**
```bash
# Check configuration
ros2 param list /controller_manager

# Manually spawn controller
ros2 run controller_manager spawner evarobot_base_controller
```

**Robot doesn't move:**
```bash
# Verify controller is active
ros2 control list_controllers

# Check if commands are received
ros2 topic echo /evarobot_base_controller/cmd_vel_unstamped

# Check joint command interfaces
ros2 control list_hardware_interfaces
```

### Joystick Issues

**Joystick not detected:**
```bash
# Check device
ls /dev/input/js*

# Test joystick
jstest /dev/input/js0

# Check permissions
sudo chmod a+rw /dev/input/js0
```

**Robot doesn't move with joystick:**
```bash
# Check joy topic
ros2 topic echo /joy
# Move joystick - you should see axis/button changes

# Check if deadman is pressed (R1 = button 5)
# buttons[5] should be 1 when pressed

# Check joy_vel output
ros2 topic echo /joy_vel
# Hold R1 + move stick - should see velocity commands
```

### Network Issues

**Raspberry Pi can't see Desktop topics:**
```bash
# Verify ROS_DOMAIN_ID matches on both
echo $ROS_DOMAIN_ID  # Should be 42 on both

# Check ROS_LOCALHOST_ONLY
echo $ROS_LOCALHOST_ONLY  # Should be 0 on both

# Ping Desktop from RPI
ping 192.168.1.100  # Use your Desktop IP

# Check firewall (on Desktop)
sudo ufw status
# If active, allow ROS2 multicast
sudo ufw allow from 192.168.1.0/24
```

---

## Complete Test Workflow

**All-in-one test sequence:**

```bash
# Terminal 1 - Gazebo
ros2 launch evarobot_description gazebo.launch.py

# Terminal 2 - Controllers
ros2 launch evarobot_controller controller.launch.py use_sim_time:=true

# Terminal 3 - RViz
rviz2

# Terminal 4 - Choose one:
# Option A: Keyboard
ros2 run teleop_twist_keyboard teleop_twist_keyboard \
  --ros-args -r /cmd_vel:=/evarobot_base_controller/cmd_vel_unstamped

# Option B: Joystick
ros2 launch evarobot_controller joystick_teleop.launch.py use_sim_time:=true
```

---

## Next Steps

After successful simulation testing:

1. ✓ Test with real robot hardware (replace Gazebo with actual Arduino)
2. ✓ Add sensors (camera, lidar, IMU)
3. ✓ Implement navigation stack
4. ✓ Add autonomous behaviors

---

**Author:** Kevin Medrano Ayala
**License:** BSD-3-Clause
**Last Updated:** 2025-11-09
