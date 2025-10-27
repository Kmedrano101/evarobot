# EvaRobot Project Structure

## Overview

This document describes the complete structure of the EvaRobot project using a **hybrid architecture** approach.

## Architecture Strategy

The project uses a **single repository with selective building** to support both:
- **Deployment** (Raspberry Pi) - Minimal, optimized packages
- **Desktop Development** - Full packages with visualization and development tools

## Git Branches

- `main` - Stable production code
- `development` - Active development branch

## Package Organization

### 🚀 Deployment Packages (Core Robot Functionality)

#### evarobot_bringup
Launch files for starting the robot (real and simulated)

#### evarobot_controller
ROS2 Control configuration and hardware interface

#### evarobot_firmware
Arduino/motor controller firmware code

#### evarobot_hardware
Hardware interface nodes for sensors and actuators

#### evarobot_description
URDF robot model (minimal, includes optional Gazebo)

#### evarobot_localization
Odometry calculation and sensor fusion (robot_localization)

#### evarobot_navigation
Nav2 runtime configuration (no RViz)

#### evarobot_msgs
Custom message, service, and action definitions

#### evarobot_utils
Utility nodes and helper tools

---

### 🖥️ Desktop Packages (Development & Visualization)

#### evarobot_mapping
SLAM Toolbox integration with visualization

#### evarobot_planning
Path planning algorithms for Nav2

#### evarobot_motion
Motion planning and custom controllers

#### evarobot_perception
Vision and camera processing

#### evarobot_viz
RViz configurations and monitoring tools

#### evarobot_cpp_examples
C++ development templates and examples

#### evarobot_py_examples
Python development templates and examples

---

## Build Commands

### Desktop (Full Build)
```bash
cd ~/evarobot_ws
./src/evarobot/build_desktop.sh
# or manually:
colcon build --symlink-install
```

### Deployment (Raspberry Pi - Minimal)
```bash
cd ~/evarobot_ws
./src/evarobot/build_deploy.sh
# or manually:
colcon build --symlink-install \
    --packages-skip evarobot_mapping evarobot_viz evarobot_planning \
                    evarobot_motion evarobot_perception \
                    evarobot_cpp_examples evarobot_py_examples
```

### Single Package
```bash
colcon build --packages-select evarobot_<package_name>
```

## Directory Structure

```
evarobot/
├── .git/                          # Git repository
├── .github/                       # GitHub workflows (future)
│   └── workflows/
├── media/                         # Images and media files
├── .gitignore                     # Git ignore rules
├── README.md                      # Main documentation
├── LICENSE                        # BSD-3-Clause license
├── CONTRIBUTING.md                # Contribution guidelines
├── PROJECT_STRUCTURE.md           # This file
├── build_desktop.sh               # Desktop build script
├── build_deploy.sh                # Deployment build script
│
├── evarobot_bringup/              # [DEPLOY] Launch files
│   ├── launch/
│   ├── config/
│   ├── package.xml
│   └── CMakeLists.txt
│
├── evarobot_controller/           # [DEPLOY] ROS2 Control
│   ├── src/
│   ├── include/evarobot_controller/
│   ├── launch/
│   ├── config/
│   ├── package.xml
│   └── CMakeLists.txt
│
├── evarobot_firmware/             # [DEPLOY] Arduino firmware
│   └── firmware/
│
├── evarobot_hardware/             # [DEPLOY] Hardware interface
│   ├── src/
│   ├── include/evarobot_hardware/
│   ├── package.xml
│   └── CMakeLists.txt
│
├── evarobot_description/          # [DEPLOY] Robot model
│   ├── urdf/
│   ├── meshes/
│   ├── launch/
│   ├── rviz/
│   ├── package.xml
│   └── CMakeLists.txt
│
├── evarobot_localization/         # [DEPLOY] Odometry & localization
│   ├── launch/
│   ├── config/
│   ├── package.xml
│   └── CMakeLists.txt
│
├── evarobot_navigation/           # [DEPLOY] Nav2 config
│   ├── launch/
│   ├── config/
│   ├── package.xml
│   └── CMakeLists.txt
│
├── evarobot_msgs/                 # [DEPLOY] Message definitions
│   ├── msg/
│   ├── srv/
│   ├── action/
│   ├── package.xml
│   └── CMakeLists.txt
│
├── evarobot_utils/                # [DEPLOY] Utilities
│   ├── src/
│   ├── include/evarobot_utils/
│   ├── scripts/
│   ├── package.xml
│   └── CMakeLists.txt
│
├── evarobot_mapping/              # [DESKTOP] SLAM
│   ├── launch/
│   ├── config/
│   ├── maps/
│   ├── package.xml
│   └── CMakeLists.txt
│
├── evarobot_planning/             # [DESKTOP] Path planning
│   ├── src/
│   ├── include/evarobot_planning/
│   ├── launch/
│   ├── config/
│   ├── package.xml
│   └── CMakeLists.txt
│
├── evarobot_motion/               # [DESKTOP] Motion planning
│   ├── src/
│   ├── include/evarobot_motion/
│   ├── launch/
│   ├── config/
│   ├── package.xml
│   └── CMakeLists.txt
│
├── evarobot_perception/           # [DESKTOP] Vision
│   ├── src/
│   ├── include/evarobot_perception/
│   ├── launch/
│   ├── config/
│   ├── package.xml
│   └── CMakeLists.txt
│
├── evarobot_viz/                  # [DESKTOP] Visualization
│   ├── rviz/
│   ├── config/
│   ├── launch/
│   ├── package.xml
│   └── CMakeLists.txt
│
├── evarobot_cpp_examples/         # [DESKTOP] C++ examples
│   ├── src/
│   ├── include/evarobot_cpp_examples/
│   ├── package.xml
│   └── CMakeLists.txt
│
└── evarobot_py_examples/          # [DESKTOP] Python examples
    ├── evarobot_py_examples/
    ├── scripts/
    ├── package.xml
    └── CMakeLists.txt
```

## Workflow

### Development Phase (Desktop)
1. Checkout `development` branch
2. Build with `build_desktop.sh`
3. Test in simulation with Gazebo + RViz
4. Develop and test with full tools

### Deployment Phase (Raspberry Pi)
1. Checkout `main` branch (stable)
2. Clone to Raspberry Pi
3. Build with `build_deploy.sh`
4. Deploy minimal packages only

## Next Steps

1. Migrate existing code from `myrobot` package to appropriate evarobot packages
2. Create launch files in `evarobot_bringup`
3. Set up URDF in `evarobot_description`
4. Configure hardware interfaces
5. Set up CI/CD pipelines in `.github/workflows`

## Benefits of This Architecture

✅ **Single source of truth** - One repository for all code
✅ **Selective builds** - Deploy only what's needed
✅ **Clear separation** - Desktop vs deployment packages
✅ **ROS2 standard** - Follows ROS2 best practices
✅ **Scalable** - Easy to add new packages
✅ **Maintainable** - Clear package responsibilities
