# EvaRobot - ROS2 Mobile Robot Platform

<p align="center">
  <img src="media/evarobot_logo.png" alt="EvaRobot" width="450">
</p>

EvaRobot is a ROS2-based mobile robot platform designed for autonomous navigation, SLAM, and perception tasks. The project is optimized for deployment on Raspberry Pi with support for both real hardware and Gazebo simulation.

## Features

### Deployment Packages (Optimized for Raspberry Pi)
- **evarobot_bringup**: Launch files for real robot
- **evarobot_controller**: ROS2 Control configuration and hardware interface
- **evarobot_firmware**: Motor controller firmware (Arduino)
- **evarobot_hardware**: Hardware interface nodes
- **evarobot_description**: URDF robot model (minimal)
- **evarobot_localization**: Odometry and sensor fusion
- **evarobot_navigation**: Nav2 runtime configuration
- **evarobot_msgs**: Custom message definitions
- **evarobot_utils**: Utility nodes and tools

### Desktop Packages (Development & Visualization)
- **evarobot_mapping**: SLAM Toolbox integration with visualization
- **evarobot_planning**: Path planning algorithms
- **evarobot_motion**: Motion planning and controllers
- **evarobot_perception**: Camera and vision processing
- **evarobot_viz**: RViz configurations and monitoring tools
- **evarobot_cpp_examples**: C++ development templates
- **evarobot_py_examples**: Python development templates

## Prerequisites

- Ubuntu 22.04 (ROS2 Humble) or Ubuntu 24.04 (ROS2 Jazzy)
- ROS2 installed (Humble or Jazzy)
- Gazebo Garden (for simulation)

### Install Dependencies

```bash
sudo apt-get update && sudo apt-get install -y \
    ros-${ROS_DISTRO}-gazebo-ros-pkgs \
    ros-${ROS_DISTRO}-ros2-control \
    ros-${ROS_DISTRO}-ros2-controllers \
    ros-${ROS_DISTRO}-slam-toolbox \
    ros-${ROS_DISTRO}-navigation2 \
    ros-${ROS_DISTRO}-nav2-bringup \
    ros-${ROS_DISTRO}-robot-localization \
    libserial-dev \
    python3-pip

pip install pyserial
```

## Installation

### 1. Create Workspace

```bash
mkdir -p ~/evarobot_ws/src
cd ~/evarobot_ws/src
```

### 2. Clone Repository

```bash
git clone https://github.com/yourusername/evarobot.git
```

### 3. Install ROS Dependencies

```bash
cd ~/evarobot_ws
rosdep install --from-paths src --ignore-src -y
```

### 4. Build the Workspace

**For Desktop (Full Build):**
```bash
colcon build --symlink-install
```

**For Deployment (Raspberry Pi - Minimal Build):**
```bash
colcon build --symlink-install \
    --packages-skip evarobot_mapping evarobot_viz evarobot_planning \
                    evarobot_motion evarobot_perception \
                    evarobot_cpp_examples evarobot_py_examples
```

### 5. Source the Workspace

```bash
source install/setup.bash
```

## Usage

### Simulation (Desktop)

```bash
# Launch simulated robot with Gazebo and RViz
ros2 launch evarobot_bringup simulated_robot.launch.py

# Launch with SLAM
ros2 launch evarobot_bringup simulated_robot.launch.py use_slam:=true
```

### Real Robot (Raspberry Pi)

```bash
# Launch real robot
ros2 launch evarobot_bringup real_robot.launch.py
```

## Project Structure

```
evarobot/
├── evarobot_bringup/          # Launch files
├── evarobot_controller/        # ROS2 Control
├── evarobot_description/       # Robot URDF/SDF
├── evarobot_firmware/          # Arduino firmware
├── evarobot_hardware/          # Hardware interface
├── evarobot_localization/      # Odometry & localization
├── evarobot_msgs/              # Custom messages
├── evarobot_navigation/        # Nav2 configuration
├── evarobot_utils/             # Utilities
├── evarobot_mapping/           # SLAM (desktop)
├── evarobot_planning/          # Path planning (desktop)
├── evarobot_motion/            # Motion planning (desktop)
├── evarobot_perception/        # Vision (desktop)
├── evarobot_viz/               # Visualization (desktop)
├── evarobot_cpp_examples/      # C++ examples (desktop)
└── evarobot_py_examples/       # Python examples (desktop)
```

## Development

### Git Branches

- `main` - Stable release
- `development` - Active development
- `simulation` - Simulation testing
- `deploy` - Deployment tags

### Building Single Package

```bash
colcon build --packages-select evarobot_<package_name>
```

### Running Tests

```bash
colcon test --packages-select evarobot_<package_name>
colcon test-result --verbose
```

## Contributing

Contributions are welcome! Please read [CONTRIBUTING.md](CONTRIBUTING.md) for details.

## License

This project is licensed under the BSD 3-Clause License - see [LICENSE](LICENSE) file.

## Maintainer

Kevin Medrano Ayala - kevin.ejem18@gmail.com

## Acknowledgements

- Inspired by [Bumper-Bot](https://github.com/AntoBrandi/Bumper-Bot)
- Built with ROS2 and Gazebo
