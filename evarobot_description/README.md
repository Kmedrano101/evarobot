# 🤖 EvaRobot Description

> URDF robot model and Gazebo simulation environment for EvaRobot differential drive platform

[![ROS2](https://img.shields.io/badge/ROS2-Jazzy-blue?logo=ros&logoColor=white)](https://docs.ros.org/en/jazzy/)
[![Gazebo](https://img.shields.io/badge/Gazebo-Harmonic-orange?logo=gazebo)](https://gazebosim.org/)
[![License](https://img.shields.io/badge/license-BSD--3--Clause-green)](LICENSE)

---

## 📋 Table of Contents

- [Overview](#-overview)
- [Features](#-features)
- [Package Contents](#-package-contents)
- [Dependencies](#-dependencies)
- [Installation](#-installation)
- [Robot Specifications](#-robot-specifications)
- [Usage](#-usage)
- [World Files](#-world-files)
- [Customization](#-customization)
- [Visualization](#-visualization)
- [Troubleshooting](#-troubleshooting)
- [Architecture](#-architecture)
- [Author](#-author)

---

## 🌟 Overview

The `evarobot_description` package contains the complete robot model definition for EvaRobot, including:
- **URDF/Xacro** robot description files
- **Gazebo simulation** configuration
- **ROS2 Control** hardware interface definition
- **World files** for testing environments
- **RViz** configurations for visualization

This package enables both **physical robot visualization** and **Gazebo simulation** of the EvaRobot platform, making it essential for development, testing, and debugging before deploying to real hardware.

---

## ✨ Features

### 🤖 Robot Model

- ✅ **Modular Xacro Design**: Easy to modify and extend
- ✅ **Physical Accuracy**: Based on measured robot dimensions
- ✅ **Inertia Calculations**: Proper dynamics for simulation
- ✅ **Collision Geometry**: Accurate physics interactions
- ✅ **Visual Materials**: Color-coded components for clarity

### 🎮 Gazebo Integration

- ✅ **Physics Simulation**: Realistic wheel dynamics and friction
- ✅ **ROS2 Control Plugin**: Seamless controller integration
- ✅ **Sensor Support**: Ready for IMU, LiDAR, cameras (expandable)
- ✅ **Multiple Worlds**: Empty and test environments included
- ✅ **Configurable Spawn**: Position and orientation parameters

### 🔧 ROS2 Control

- ✅ **Differential Drive**: Left/right wheel velocity control
- ✅ **State Interfaces**: Position and velocity feedback
- ✅ **Gazebo Hardware**: Simulated hardware interface
- ✅ **Real Hardware Ready**: Easy swap to physical interface

---

## 📦 Package Contents

```
evarobot_description/
├── urdf/
│   ├── evarobot.urdf.xacro           # Main robot description
│   ├── evarobot.gazebo.xacro         # Gazebo-specific elements
│   └── evarobot.ros2_control.xacro   # ROS2 Control configuration
├── worlds/
│   ├── empty.world                   # Minimal test environment
│   └── test_world.world              # Environment with obstacles
├── launch/
│   └── gazebo.launch.py              # Gazebo simulation launcher
├── rviz/
│   └── view_robot.rviz               # RViz configuration
├── meshes/                           # 3D model files (for future use)
├── CMakeLists.txt
├── package.xml
└── README.md
```

### 📄 URDF Files

#### `evarobot.urdf.xacro` - Main Robot Model

**Components:**
- **Base Link**: Rectangular chassis (30cm × 20cm × 8cm, 2kg)
- **Left Wheel**: Driven wheel (84mm diameter, 30mm width)
- **Right Wheel**: Driven wheel (84mm diameter, 30mm width)
- **Caster Wheel**: Front support wheel (30mm diameter, passive)

**Properties:**
```yaml
Base:
  Dimensions: 0.30m × 0.20m × 0.08m
  Mass: 2.0 kg
  Material: Blue (Gazebo/Blue)

Driven Wheels:
  Radius: 0.042m (84mm diameter)
  Width: 0.03m
  Mass: 0.1 kg each
  Separation: 0.194m
  Material: Dark Grey (Gazebo/DarkGrey)
  Friction: μ1=1.0, μ2=1.0

Caster Wheel:
  Radius: 0.015m (30mm diameter)
  Mass: 0.05 kg
  Position: 0.12m forward from base center
  Material: Light Grey (Gazebo/Grey)
  Friction: μ1=0.1, μ2=0.1
```

#### `evarobot.gazebo.xacro` - Gazebo Configuration

**Features:**
- Material colors and textures
- Friction coefficients for realistic physics
- Contact parameters (kp, kd) for stability
- Sensor plugin templates (IMU, LiDAR - commented for easy addition)

#### `evarobot.ros2_control.xacro` - Control Interface

**Hardware Interface:**
- Plugin: `gz_ros2_control/GazeboSimSystem`
- Joints: `left_wheel_joint`, `right_wheel_joint`
- Command Interface: Velocity (±10.0 rad/s)
- State Interfaces: Position, Velocity

---

## 📦 Dependencies

### System Dependencies

```bash
# Gazebo Harmonic (for ROS2 Jazzy)
sudo apt-get install -y \
    gz-harmonic \
    ros-${ROS_DISTRO}-ros-gz-sim \
    ros-${ROS_DISTRO}-ros-gz-bridge \
    ros-${ROS_DISTRO}-gz-ros2-control
```

### ROS2 Packages

```bash
sudo apt-get install -y \
    ros-${ROS_DISTRO}-robot-state-publisher \
    ros-${ROS_DISTRO}-joint-state-publisher \
    ros-${ROS_DISTRO}-xacro \
    ros-${ROS_DISTRO}-urdf \
    ros-${ROS_DISTRO}-rviz2
```

### Build Dependencies
- `ament_cmake`
- `urdf`
- `xacro`

---

## 🔧 Installation

### 1. Prerequisites

Ensure ROS2 Jazzy and Gazebo are installed:

```bash
# Check ROS2 installation
ros2 --version

# Check Gazebo Harmonic installation
gz sim --version
```

### 2. Build Package

```bash
cd ~/evarobot_ws
colcon build --packages-select evarobot_description
source install/setup.bash
```

### 3. Verify Installation

```bash
# Check package is found
ros2 pkg prefix evarobot_description

# List package files
ros2 pkg executables evarobot_description
```

---

## 📐 Robot Specifications

### Physical Dimensions

| Component | Measurement | Notes |
|-----------|-------------|-------|
| **Base** | | |
| Length | 0.30 m | 300 mm |
| Width | 0.20 m | 200 mm |
| Height | 0.08 m | 80 mm |
| Mass | 2.0 kg | Chassis only |
| **Wheels** | | |
| Wheel Radius | 0.042 m | 84 mm diameter |
| Wheel Width | 0.03 m | 30 mm |
| Wheel Separation | 0.194 m | Center to center |
| Wheel Mass | 0.1 kg | Each wheel |
| **Caster** | | |
| Caster Radius | 0.015 m | 30 mm diameter |
| Caster Mass | 0.05 kg | |
| Caster Position | 0.12 m forward | From base center |

### Performance Characteristics

| Parameter | Value | Unit |
|-----------|-------|------|
| Max Linear Velocity | 0.5 | m/s |
| Max Angular Velocity | 2.0 | rad/s |
| Max Linear Acceleration | 0.8 | m/s² |
| Max Angular Acceleration | 3.0 | rad/s² |
| Ground Clearance | ~0.015 | m |
| Wheelbase | 0.12 | m |
| Total Mass | ~2.3 | kg |

### Frame Hierarchy

```
base_footprint (ground projection)
    └── base_link (robot chassis)
        ├── left_wheel_link (left driven wheel)
        ├── right_wheel_link (right driven wheel)
        └── caster_wheel_link (front support wheel)
```

---

## 🚀 Usage

### Quick Start: Gazebo Simulation

#### Launch Default Empty World

```bash
ros2 launch evarobot_description gazebo.launch.py
```

This will:
1. Start Gazebo with empty world
2. Spawn robot at origin (0, 0, 0.1)
3. Load robot state publisher
4. Configure ROS2 Control hardware interface

#### Launch Test World with Obstacles

```bash
ros2 launch evarobot_description gazebo.launch.py world:=test_world
```

#### Launch with RViz Visualization

```bash
ros2 launch evarobot_description gazebo.launch.py \
    world:=test_world \
    start_rviz:=true
```

### Launch Arguments

| Argument | Default | Description | Example Values |
|----------|---------|-------------|----------------|
| `world` | `empty` | World file to load | `empty`, `test_world`, custom |
| `use_sim_time` | `true` | Use simulation clock | `true`, `false` |
| `start_rviz` | `false` | Launch RViz | `true`, `false` |
| `x_pose` | `0.0` | Initial X position (m) | `-2.0`, `0.0`, `2.0` |
| `y_pose` | `0.0` | Initial Y position (m) | `-1.0`, `0.0`, `1.0` |
| `z_pose` | `0.1` | Initial Z position (m) | `0.1` |
| `yaw` | `0.0` | Initial yaw angle (rad) | `0.0`, `1.57`, `3.14` |
| `gui` | `true` | Show Gazebo GUI | `true`, `false` |

### Complete Simulation Setup (3 Terminals)

#### Terminal 1: Launch Gazebo

```bash
source ~/evarobot_ws/install/setup.bash
ros2 launch evarobot_description gazebo.launch.py world:=test_world
```

#### Terminal 2: Launch Controllers

```bash
source ~/evarobot_ws/install/setup.bash
ros2 launch evarobot_controller controller.launch.py use_sim_time:=true
```

#### Terminal 3: Control Robot

```bash
# Option A: Manual command
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist \
    "{linear: {x: 0.2}, angular: {z: 0.0}}" --rate 10

# Option B: Joystick control
ros2 launch evarobot_controller joystick_teleop.launch.py use_sim_time:=true

# Option C: Keyboard control
ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args \
    --remap cmd_vel:=key_vel
```

---

## 🗺️ World Files

### `empty.world` - Minimal Test Environment

**Description:**
- Simple ground plane
- Basic lighting (sun)
- No obstacles
- Perfect for basic movement testing

**Use Cases:**
- Controller testing
- Odometry validation
- Basic motion testing
- Performance benchmarking

**Launch:**
```bash
ros2 launch evarobot_description gazebo.launch.py world:=empty
```

### `test_world.world` - Obstacle Course

**Description:**
- Ground plane with grid pattern
- Multiple obstacle types:
  - **Red Box** (0.5m × 0.5m × 0.5m) at (2, 0, 0.25)
  - **Green Cylinder** (r=0.3m, h=1.0m) at (-2, 2, 0.5)
  - **Grey Wall** (4m × 0.2m × 1.0m) at (0, -3, 0.5)
- Enhanced lighting

**Use Cases:**
- Navigation testing
- Collision avoidance
- Sensor integration testing
- Path planning validation

**Launch:**
```bash
ros2 launch evarobot_description gazebo.launch.py world:=test_world
```

### Creating Custom Worlds

1. **Copy template:**
```bash
cp ~/evarobot_ws/src/evarobot_description/worlds/empty.world \
   ~/evarobot_ws/src/evarobot_description/worlds/my_world.world
```

2. **Edit world file** (add obstacles, change lighting, etc.)

3. **Launch custom world:**
```bash
ros2 launch evarobot_description gazebo.launch.py world:=my_world
```

---

## 🎨 Customization

### Modifying Robot Dimensions

Edit `urdf/evarobot.urdf.xacro`:

```xml
<!-- Change base size -->
<xacro:property name="base_length" value="0.35" />  <!-- was 0.30 -->
<xacro:property name="base_width" value="0.25" />   <!-- was 0.20 -->

<!-- Change wheel parameters -->
<xacro:property name="wheel_radius" value="0.050" />  <!-- was 0.042 -->
<xacro:property name="wheel_separation" value="0.25" /> <!-- was 0.194 -->
```

**Important:** After changing dimensions, update the controller configuration:
```yaml
# In evarobot_controller/config/evarobot_controllers.yaml
wheel_radius: 0.050      # Match URDF value
wheel_separation: 0.25   # Match URDF value
```

### Adding Sensors

#### Example: Adding IMU

Uncomment in `urdf/evarobot.gazebo.xacro`:

```xml
<gazebo reference="base_link">
  <sensor name="imu_sensor" type="imu">
    <always_on>true</always_on>
    <update_rate>100</update_rate>
    <topic>imu</topic>
    <gz_frame_id>base_link</gz_frame_id>
    <enable_metrics>false</enable_metrics>
  </sensor>
</gazebo>
```

#### Example: Adding LiDAR

1. Add lidar link to main URDF:
```xml
<link name="lidar_link">
  <visual>
    <geometry>
      <cylinder radius="0.05" length="0.05"/>
    </geometry>
    <material name="dark_grey"/>
  </visual>
  <!-- Add collision and inertial -->
</link>

<joint name="lidar_joint" type="fixed">
  <parent link="base_link"/>
  <child link="lidar_link"/>
  <origin xyz="0 0 0.05" rpy="0 0 0"/>
</joint>
```

2. Uncomment LiDAR sensor in `evarobot.gazebo.xacro`

### Changing Colors

Edit material definitions in `urdf/evarobot.urdf.xacro`:

```xml
<!-- Base color -->
<material name="blue">
  <color rgba="0.8 0.2 0.2 1.0"/>  <!-- Change to red -->
</material>

<!-- Wheel color -->
<material name="dark_grey">
  <color rgba="0.1 0.1 0.1 1.0"/>  <!-- Darker wheels -->
</material>
```

---

## 👁️ Visualization

### RViz Configuration

#### Launch RViz with Robot Model

```bash
# With Gazebo simulation
ros2 launch evarobot_description gazebo.launch.py start_rviz:=true

# Standalone (no Gazebo)
rviz2 -d ~/evarobot_ws/src/evarobot_description/rviz/view_robot.rviz
```

#### RViz Displays Included

- **Grid**: Reference frame grid
- **TF**: Transform tree visualization
- **RobotModel**: 3D robot visualization
- **Odometry**: Robot path trajectory

### View Robot Model (No Simulation)

```bash
# Terminal 1: Publish robot state
ros2 launch robot_state_publisher robot_state_publisher \
    robot_description:="$(xacro ~/evarobot_ws/src/evarobot_description/urdf/evarobot.urdf.xacro)"

# Terminal 2: Publish joint states
ros2 run joint_state_publisher_gui joint_state_publisher_gui

# Terminal 3: Launch RViz
rviz2
```

### Useful RViz Commands

```bash
# Save current configuration
# File → Save Config As → my_config.rviz

# View TF tree
ros2 run tf2_tools view_frames
evince frames.pdf

# Echo transforms
ros2 run tf2_ros tf2_echo base_footprint base_link
```

---

## 🔧 Troubleshooting

### Issue: Gazebo Doesn't Launch

**Symptoms:** Gazebo fails to start or crashes immediately

**Solutions:**

1. **Check Gazebo Harmonic installation:**
```bash
gz sim --version
which gz
```

2. **Test Gazebo standalone:**
```bash
gz sim
```

3. **Check for conflicting processes:**
```bash
killall gz ruby
```

4. **Reset Gazebo cache:**
```bash
rm -rf ~/.gz/sim
```

### Issue: Robot Not Spawning

**Symptoms:** Gazebo launches but robot doesn't appear

**Solutions:**

1. **Check robot_description topic:**
```bash
ros2 topic echo /robot_description --once
# Should output URDF content
```

2. **Verify URDF is valid:**
```bash
cd ~/evarobot_ws/src/evarobot_description/urdf
check_urdf evarobot.urdf.xacro
```

3. **Process xacro manually:**
```bash
xacro evarobot.urdf.xacro > /tmp/robot.urdf
check_urdf /tmp/robot.urdf
```

4. **Check spawn command:**
```bash
ros2 run ros_gz_sim create \
    -topic robot_description \
    -name evarobot
```

### Issue: Robot Falls Through Ground

**Symptoms:** Robot drops below ground plane in simulation

**Solutions:**

1. **Increase spawn height:**
```bash
ros2 launch evarobot_description gazebo.launch.py z_pose:=0.2
```

2. **Check collision geometry:**
   - Ensure all links have `<collision>` elements
   - Verify ground plane has collision

3. **Adjust physics parameters:**
   - Check `kp` (stiffness) values in gazebo.xacro
   - Increase contact depth: `<minDepth>0.001</minDepth>`

### Issue: Robot Moves Unrealistically

**Symptoms:** Robot slides, tips over, or has poor traction

**Solutions:**

1. **Adjust wheel friction:**
```xml
<!-- In evarobot.gazebo.xacro -->
<mu1>1.5</mu1>  <!-- Increase for more grip -->
<mu2>1.5</mu2>
```

2. **Increase contact stiffness:**
```xml
<kp>1000000.0</kp>  <!-- Higher = stiffer contact -->
<kd>100.0</kd>      <!-- Higher = more damping -->
```

3. **Check center of mass:**
   - Ensure base_link mass is appropriate
   - Verify inertia calculations are correct

4. **Reduce physics timestep:**
```xml
<!-- In world file -->
<max_step_size>0.001</max_step_size>
```

### Issue: Controllers Not Working in Gazebo

**Symptoms:** Robot spawns but doesn't respond to commands

**Solutions:**

1. **Check gz_ros2_control is loaded:**
```bash
ros2 node list | grep controller_manager
# Should see: /controller_manager
```

2. **Verify controllers are active:**
```bash
ros2 control list_controllers
```

3. **Check for plugin errors:**
```bash
# Look for errors in Gazebo terminal output
# Common issues:
#   - Missing controller config file
#   - Wrong plugin name
#   - URDF/xacro parsing errors
```

4. **Reload controllers:**
```bash
ros2 launch evarobot_controller controller.launch.py use_sim_time:=true
```

### Issue: Xacro Processing Errors

**Symptoms:** Error messages about undefined properties or syntax

**Solutions:**

1. **Check xacro syntax:**
```bash
xacro --check-order ~/evarobot_ws/src/evarobot_description/urdf/evarobot.urdf.xacro
```

2. **Process and view output:**
```bash
xacro ~/evarobot_ws/src/evarobot_description/urdf/evarobot.urdf.xacro \
    > /tmp/robot.urdf
cat /tmp/robot.urdf
```

3. **Common xacro errors:**
   - Missing closing tags
   - Undefined properties
   - Incorrect macro calls
   - Wrong file paths in includes

---

## 🏗️ Architecture

### URDF Structure

```
evarobot.urdf.xacro (main file)
    ├── Properties (dimensions, masses)
    ├── Macros (inertia calculations, wheel template)
    ├── Links (base, wheels, caster)
    ├── Joints (fixed, continuous)
    └── Includes:
        ├── evarobot.gazebo.xacro (simulation config)
        └── evarobot.ros2_control.xacro (control interface)
```

### Transform Tree

```
odom (published by diff_drive_controller)
  └── base_footprint (ground projection)
      └── base_link (robot chassis)
          ├── left_wheel_link (continuous joint)
          ├── right_wheel_link (continuous joint)
          └── caster_wheel_link (fixed joint)
```

### Gazebo Plugin Flow

```
┌────────────────────────────────────────────────────┐
│        Gazebo Harmonic Physics Engine               │
│  ┌──────────────────────────────────────────────┐ │
│  │  Robot Model (from URDF)                      │ │
│  │   - Links with collision/inertia              │ │
│  │   - Joints with limits/dynamics               │ │
│  └──────────────────────────────────────────────┘ │
└────────────────────────────────────────────────────┘
                      ↕
┌────────────────────────────────────────────────────┐
│           gz_ros2_control Plugin                    │
│  ┌──────────────────────────────────────────────┐ │
│  │  GazeboSimSystem Hardware Interface           │ │
│  │   - Reads joint states from Gazebo            │ │
│  │   - Writes joint commands to Gazebo           │ │
│  └──────────────────────────────────────────────┘ │
└────────────────────────────────────────────────────┘
                      ↕
┌────────────────────────────────────────────────────┐
│            Controller Manager                       │
│  (See evarobot_controller package)                 │
└────────────────────────────────────────────────────┘
```

---

## 📚 Tips and Best Practices

### 1. Modular Design

Keep URDF files modular:
- Main robot structure in `.urdf.xacro`
- Simulation-specific in `.gazebo.xacro`
- Control configuration in `.ros2_control.xacro`

### 2. Use Properties

Define properties at the top for easy modification:
```xml
<xacro:property name="wheel_radius" value="0.042" />
<!-- Use everywhere: -->
<cylinder radius="${wheel_radius}" .../>
```

### 3. Inertia Matters

Incorrect inertia = unrealistic simulation:
- Use macros for standard shapes
- Verify with online calculators
- Test with varying masses

### 4. Collision vs Visual

- Keep collision geometry simple (boxes, cylinders, spheres)
- Visual geometry can be complex (meshes)
- Don't make collision match every detail

### 5. Testing Workflow

1. Test URDF: `check_urdf robot.urdf`
2. View in RViz: `joint_state_publisher_gui`
3. Test in Gazebo: Basic movement
4. Add controllers: Full functionality
5. Tune parameters: Realistic behavior

---

## 🔗 Integration with Other Packages

### With evarobot_controller

```bash
# Terminal 1: Simulation
ros2 launch evarobot_description gazebo.launch.py

# Terminal 2: Controllers
ros2 launch evarobot_controller controller.launch.py use_sim_time:=true
```

### With evarobot_firmware

For real hardware, the URDF dimensions should match your physical robot:
- Measure actual wheel_radius and wheel_separation
- Update URDF properties
- Keep controller config synchronized

### With Navigation (Nav2)

```bash
# Ensure robot publishes required topics:
# - /odom (from DiffDriveController)
# - /tf (odom → base_footprint)
# - /scan (if LiDAR added)

# Then launch Nav2 with evarobot config
```

---

## 📊 Current Status

| Feature | Status | Notes |
|---------|--------|-------|
| Basic URDF Model | ✅ Complete | Differential drive with caster |
| Gazebo Integration | ✅ Complete | Physics, materials, friction |
| ROS2 Control | ✅ Complete | Velocity command interface |
| Empty World | ✅ Complete | Basic testing environment |
| Test World | ✅ Complete | Obstacles for navigation |
| RViz Config | ✅ Complete | Visualization setup |
| IMU Sensor | 📝 Template | Commented, ready to enable |
| LiDAR Sensor | 📝 Template | Commented, ready to enable |
| Camera Sensor | ⏳ Planned | Future addition |
| Custom Meshes | ⏳ Planned | 3D models for visual realism |

**Last Updated:** 2025-10-30

---

## 🤝 Contributing

Ways to improve this package:

- [ ] Add high-quality 3D meshes for visual model
- [ ] Create more complex test worlds
- [ ] Add sensor configurations (IMU, LiDAR, camera)
- [ ] Improve collision geometry accuracy
- [ ] Add URDF testing framework
- [ ] Create automated calibration tools

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

### Documentation
- [URDF Tutorials](http://wiki.ros.org/urdf/Tutorials)
- [Xacro Documentation](http://wiki.ros.org/xacro)
- [Gazebo Harmonic](https://gazebosim.org/docs/harmonic)
- [gz_ros2_control](https://github.com/ros-controls/gz_ros2_control)
- [ros_gz](https://github.com/gazebosim/ros_gz)
- [Robot State Publisher](http://wiki.ros.org/robot_state_publisher)

### Tools
- [SolidWorks to URDF Exporter](http://wiki.ros.org/sw_urdf_exporter)
- [Blender to URDF](https://github.com/ros/solidworks_urdf_exporter)
- [URDF Visual Studio Code Extension](https://marketplace.visualstudio.com/items?itemName=ms-vscode.vscode-ros)

### Related Packages
- [evarobot_controller](../evarobot_controller/README.md) - ROS2 Control and teleoperation
- [evarobot_firmware](../evarobot_firmware/README.md) - Arduino motor firmware
- [evarobot_hardware](../evarobot_hardware/README.md) - Hardware interface

---

## 🎯 Quick Reference

### Most Common Commands

```bash
# Launch Gazebo with test world
ros2 launch evarobot_description gazebo.launch.py world:=test_world

# View robot in RViz (no simulation)
ros2 launch robot_state_publisher robot_state_publisher \
    robot_description:="$(xacro ~/evarobot_ws/src/evarobot_description/urdf/evarobot.urdf.xacro)"

# Check URDF validity
check_urdf ~/evarobot_ws/src/evarobot_description/urdf/evarobot.urdf.xacro

# Process xacro to URDF
xacro ~/evarobot_ws/src/evarobot_description/urdf/evarobot.urdf.xacro \
    > /tmp/evarobot.urdf

# View TF tree
ros2 run tf2_tools view_frames && evince frames.pdf
```

---

<p align="center">
  <strong>Built for simulation, tested in reality</strong><br>
  Made with ❤️ for the EvaRobot project
</p>
