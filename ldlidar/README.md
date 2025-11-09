# 🌀 LDLiDAR ROS2 Package

<div align="center">

![ROS2](https://img.shields.io/badge/ROS2-Jazzy-blue?logo=ros&logoColor=white)
![Platform](https://img.shields.io/badge/Platform-Linux-orange?logo=linux&logoColor=white)
![License](https://img.shields.io/badge/License-MIT-green)
![Build](https://img.shields.io/badge/Build-ament__cmake-blue)

**High-performance ROS2 driver for LDRobot LiDAR sensors**

[Features](#-features) • [Installation](#-installation) • [Quick Start](#-quick-start) • [Configuration](#-configuration) • [Troubleshooting](#-troubleshooting)

</div>

---

## 📋 Overview

This package provides a complete ROS2 integration for LDRobot's LiDAR sensors, enabling real-time 2D laser scanning capabilities for mobile robots and autonomous systems. The driver is fully integrated with all necessary headers and libraries for standalone operation.

## 🎯 Supported Models

| Model | Range | Frequency | Interface |
|-------|-------|-----------|-----------|
| **LD06** | 12m | 4500Hz | Serial/Network |
| **LD19** | 12m | 4500Hz | Serial/Network |
| **STL06P** | 12m | 4500Hz | Serial/Network |
| **STL26** | 30m | 4500Hz | Serial/Network |
| **STL27L** | 30m | 4500Hz | Serial/Network |

## ✨ Features

- 🔌 **Dual Communication**: Serial (USB) and Network (Ethernet) support
- 🎛️ **Flexible Configuration**: Runtime parameter adjustment
- 📐 **Angle Cropping**: Mask specific angular ranges
- 🔄 **Scan Direction**: Configurable clockwise/counterclockwise rotation
- 📊 **ROS2 Integration**: Standard `sensor_msgs/LaserScan` publishing
- 🎨 **RViz2 Support**: Pre-configured visualization configs
- 🚀 **High Performance**: Optimized C++14 implementation

---

## 🔧 Installation

### Prerequisites

```bash
# ROS2 Jazzy (or compatible distribution)
# Install ROS2 if not already installed
# sudo apt install ros-jazzy-desktop

# Install dependencies
sudo apt update
sudo apt install -y \
    ros-jazzy-rclcpp \
    ros-jazzy-sensor-msgs \
    ros-jazzy-tf2-ros \
    ros-jazzy-rviz2
```

### Building the Package

```bash
# Navigate to your workspace
cd ~/evarobot_ws

# Build the ldlidar package
colcon build --packages-select ldlidar

# Source the workspace
source install/setup.bash
```

### 🔐 Serial Port Permissions

For USB/Serial connection, add your user to the `dialout` group:

```bash
sudo usermod -aG dialout $USER
sudo chmod 666 /dev/ttyUSB0  # Adjust device name as needed

# Log out and back in for changes to take effect
```

---

## 🚀 Quick Start

### 1️⃣ Serial Connection (USB)

```bash
# For LD06 model
ros2 launch ldlidar ld06.launch.py

# For LD19 model
ros2 launch ldlidar ld19.launch.py

# For STL series
ros2 launch ldlidar stl06p.launch.py
ros2 launch ldlidar stl26.launch.py
ros2 launch ldlidar stl27l.launch.py
```

### 2️⃣ Network Connection (Ethernet)

Edit the launch file to enable network mode:

```python
{'enable_serial_or_network_communication': False},
{'server_ip': '192.168.1.200'},  # LiDAR IP address
{'server_port': '2000'},          # LiDAR port
```

Then launch normally:

```bash
ros2 launch ldlidar ld06.launch.py
```

### 3️⃣ Visualization with RViz2

```bash
# Launch LiDAR with RViz2 visualization
ros2 launch ldlidar viewer_ld06.launch.py

# Or launch separately
ros2 run rviz2 rviz2 -d $(ros2 pkg prefix ldlidar)/share/ldlidar/rviz2/ldlidar.rviz
```

---

## ⚙️ Configuration

### 📡 Communication Settings

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `enable_serial_or_network_communication` | bool | `True` | `True` for Serial, `False` for Network |
| `port_name` | string | `/dev/ttyUSB0` | Serial port device |
| `port_baudrate` | int | `230400` | Serial baudrate |
| `server_ip` | string | `192.168.1.200` | Network IP address |
| `server_port` | string | `2000` | Network port |

### 📊 Scan Settings

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `product_name` | string | `LDLiDAR_LD06` | LiDAR model identifier |
| `topic_name` | string | `scan` | Published topic name |
| `frame_id` | string | `base_laser` | TF frame ID |
| `laser_scan_dir` | bool | `True` | `True` for CCW, `False` for CW |
| `measure_point_freq` | int | `4500` | Measurement frequency (Hz) |

### ✂️ Angle Cropping

Mask data within a specific angular range:

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `enable_angle_crop_func` | bool | `False` | Enable angle cropping |
| `angle_crop_min` | double | `135.0` | Minimum crop angle (degrees) |
| `angle_crop_max` | double | `225.0` | Maximum crop angle (degrees) |

**Example**: To mask rear data (135° to 225°):

```python
{'enable_angle_crop_func': True},
{'angle_crop_min': 135.0},
{'angle_crop_max': 225.0}
```

---

## 📝 Usage Examples

### Custom Launch Configuration

Create your own launch file:

```python
#!/usr/bin/env python3
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    ldlidar_node = Node(
        package='ldlidar',
        executable='ldlidar',
        name='my_lidar',
        output='screen',
        parameters=[
            {'product_name': 'LDLiDAR_LD06'},
            {'topic_name': 'scan'},
            {'frame_id': 'laser_frame'},
            {'enable_serial_or_network_communication': True},
            {'port_name': '/dev/ttyUSB0'},
            {'port_baudrate': 230400},
            {'laser_scan_dir': True},
            {'enable_angle_crop_func': False},
        ]
    )

    return LaunchDescription([ldlidar_node])
```

### Subscribe to Scan Data

```bash
# View raw scan data
ros2 topic echo /scan

# Monitor publishing rate
ros2 topic hz /scan

# View topic info
ros2 topic info /scan
```

### Check TF Transforms

```bash
# View TF tree
ros2 run tf2_tools view_frames

# Echo transform
ros2 run tf2_ros tf2_echo base_link base_laser
```

---

## 🔍 Troubleshooting

### ❌ No Data Published

**Check device permissions:**
```bash
ls -l /dev/ttyUSB*
sudo chmod 666 /dev/ttyUSB0
```

**Verify device is connected:**
```bash
dmesg | grep tty
```

### ⚠️ Port Already in Use

**Find and kill blocking process:**
```bash
sudo lsof /dev/ttyUSB0
sudo kill -9 <PID>
```

### 🔌 Network Connection Issues

**Test connectivity:**
```bash
ping 192.168.1.200
telnet 192.168.1.200 2000
```

**Check firewall:**
```bash
sudo ufw status
sudo ufw allow 2000/tcp
```

### 📉 Low Performance

- Ensure baudrate matches LiDAR specification (230400 for LD06/LD19)
- Check CPU usage: `top` or `htop`
- Verify ROS2 middleware settings (DDS configuration)

### 🐛 Debug Mode

Enable detailed logging:

```bash
ros2 launch ldlidar ld06.launch.py --ros-args --log-level DEBUG
```

---

## 📂 Package Structure

```
ldlidar/
├── CMakeLists.txt          # Build configuration
├── package.xml             # Package metadata
├── README.md              # This file
├── include/               # Header files
│   ├── core/              # Core driver headers
│   ├── dataprocess/       # Data processing
│   ├── filter/            # Filtering algorithms
│   ├── logger/            # Logging utilities
│   ├── networkcom/        # Network communication
│   ├── serialcom/         # Serial communication
│   └── ros2_api.h         # ROS2 interface
├── src/                   # Source files
│   ├── core/              # Core implementation
│   ├── dataprocess/       # Data processing
│   ├── filter/            # Filters
│   ├── logger/            # Logger
│   ├── networkcom/        # Network code
│   ├── serialcom/         # Serial code
│   └── ros2_node/         # ROS2 node
├── launch/                # Launch files
│   ├── ld06.launch.py
│   ├── ld19.launch.py
│   ├── stl*.launch.py
│   └── viewer_*.launch.py
└── rviz2/                 # RViz configurations
    └── ldlidar.rviz
```

---

## 📚 Additional Resources

- 🌐 **LDRobot Official**: [www.ldrobot.com](http://www.ldrobot.com)
- 📖 **ROS2 Documentation**: [docs.ros.org](https://docs.ros.org)
- 💬 **Support**: Open an issue in the repository

---

## 📄 License

This package is released under the **MIT License**.

```
Copyright (c) LDRobot
```

---

## 🤝 Contributing

Contributions are welcome! Please feel free to:
- Report bugs
- Suggest features
- Submit pull requests

---

<div align="center">

**Made with ❤️ for the ROS2 community**

⭐ Star this repo if you find it useful!

</div>
