# 🤖 EvaRobot Firmware

> Arduino Nano RP2040 firmware for differential drive mobile robot

[![License](https://img.shields.io/badge/license-BSD--3--Clause-blue.svg)](LICENSE)
[![Platform](https://img.shields.io/badge/platform-Arduino%20RP2040-green.svg)](https://www.arduino.cc/)
[![ROS2](https://img.shields.io/badge/ROS2-Jazzy-blue.svg)](https://docs.ros.org/en/jazzy/)

---

## 📋 Table of Contents

- [Features](#-features)
- [Hardware Setup](#-hardware-setup)
- [Control Modes](#-control-modes)
- [PS4 Joystick Control](#-ps4-joystick-control)
- [Serial Protocol](#-serial-communication-protocol)
- [Arduino Libraries](#-arduino-libraries-required)
- [Upload Instructions](#-upload-instructions)
- [Testing](#-testing)
- [PID Tuning](#-pid-tuning)
- [Troubleshooting](#-troubleshooting)
- [Author](#-author)

---

## ✨ Features

- 🎮 **PS4 Controller Support (NEW!):**
  - Bluetooth wireless control
  - Differential drive with simultaneous forward/backward and rotation
  - Real-time speed adjustment via D-pad
  - One-command launch file for complete setup

- 🎮 **Dual Control Modes:**
  - **Manual Mode:** Direct PWM control for teleoperation with differential drive
  - **Automatic Mode:** PID-based velocity control for autonomous navigation

- 🚗 **Differential Drive Control:**
  - Independent left/right wheel velocity control
  - Smooth curved motion (simultaneous forward + rotation)
  - Individual PWM values calculated from combined joystick inputs

- 🔄 **Quadrature Encoders:**
  - Real-time position tracking (encoder ticks)
  - Velocity measurement (rad/s)
  - 385 ticks per revolution resolution

- 📡 **JSON Serial Protocol:**
  - Bidirectional communication
  - Mode switching on-the-fly
  - Continuous encoder feedback (100ms)

- 🎯 **PID Controller:**
  - Closed-loop velocity control
  - Independent tuning for each motor
  - Configurable gains (Kp, Ki, Kd)

- 🧪 **Interactive Testing:**
  - Real-time feedback monitoring
  - PID tuning validation
  - Multiple test modes

---

## 🔧 Hardware Setup

### 🎛️ Microcontroller
- **Board:** Arduino Nano RP2040
- **Baud Rate:** 115200
- **Communication:** USB Serial

### 📌 Pin Configuration

#### 🚗 Motor Control (H-Bridge)
| Pin | Function | Description |
|-----|----------|-------------|
| GPIO 9 | IN1 | Left motor forward/backward |
| GPIO 10 | IN2 | Left motor forward/backward |
| GPIO 11 | IN3 | Right motor forward/backward |
| GPIO 12 | IN4 | Right motor forward/backward |

#### ⚡ PWM Control
| Pin | Function | Settings |
|-----|----------|----------|
| GPIO 17 | PWM1 | Left motor speed control |
| GPIO 18 | PWM2 | Right motor speed control |

**PWM Parameters:**
- **Frequency:** 500 Hz
- **Duty Cycle Range:** 38% - 95%

#### 🔄 Encoders (Quadrature)
| Motor | Pin A | Pin B | Resolution |
|-------|-------|-------|------------|
| Left | A0 | A1 | 385 ticks/rev |
| Right | A2 | A3 | 385 ticks/rev |

---

## 🎮 Control Modes

The firmware supports two distinct control modes that can be switched dynamically:

### 🔧 Manual Mode (Default)

**Purpose:** Direct teleoperation and testing

**Characteristics:**
- Direct PWM duty cycle control (38-95%)
- Directional commands (forward, backward, turnleft, turnright, stop)
- No feedback loop - open-loop control
- Immediate response to commands

**Use Cases:**
- Manual robot control via keyboard/joystick
- Motor testing and calibration
- Simple teleoperation tasks

**Protocol:**
```json
{"motorStatus": "forward", "motorVelocity": {"left": 70, "right": 70}}
```

---

### 🤖 Automatic Mode (PID Control)

**Purpose:** Autonomous navigation with velocity control

**Characteristics:**
- Closed-loop PID velocity control
- Setpoints in rad/s (radians per second)
- Continuous encoder feedback integration
- Smooth velocity tracking

**Use Cases:**
- ROS2 navigation stack integration
- Autonomous path following
- Precise velocity control for odometry
- Coordinated motor control

**Protocol:**
```json
{"mode": "automatic", "velocitySetpoints": {"left": 1.5, "right": 1.5}}
```

**PID Parameters:**
| Motor | Kp | Ki | Kd | Sample Rate |
|-------|-----|-----|-----|-------------|
| Left  | 11.5 | 7.5 | 0.1 | 100 ms |
| Right | 12.8 | 8.3 | 0.1 | 100 ms |

*Tunable in `evarobot_config.h:70-77`*

---

### 🔄 Switching Modes

Switch between modes at any time:

```json
// Switch to automatic mode
{"mode": "automatic"}

// Switch to manual mode
{"mode": "manual"}
```

---

## 🎮 PS4 Joystick Control

Control your robot wirelessly using a PS4 DualShock 4 controller!

### 🔌 Quick Start

**Single command to launch everything:**

```bash
cd ~/evarobot_ws
source install/setup.bash
ros2 launch evarobot_firmware joy_teleop.launch.py
```

This automatically starts:
1. **joy_node** - Reads PS4 controller input from `/dev/input/js0`
2. **arduino_serial_bridge** - Communicates with Arduino via `/dev/ttyACM0`
3. **joy_teleop_node** - Converts joystick to differential drive commands

---

### 🎮 Controller Mapping

| Control | Function | Details |
|---------|----------|---------|
| **Left Stick (Left/Right)** | Rotate | ⬅️ Turn left / Turn right ➡️ |
| **Right Stick (Up/Down)** | Linear | ⬆️ Forward / Backward ⬇️ |
| **D-pad Up** | Increase Speed | Boost max velocity |
| **D-pad Down** | Decrease Speed | Reduce max velocity |

**Key Feature:** Use **both sticks simultaneously** for smooth curved motion!
- Push right stick forward + left stick right = Move forward while turning right
- Works in all combinations for natural, car-like steering

---

### 🔧 Differential Drive Algorithm

The system calculates individual wheel velocities from both joystick inputs:

```
left_velocity  = linear_velocity - (angular_velocity × wheel_separation / 2)
right_velocity = linear_velocity + (angular_velocity × wheel_separation / 2)
```

**Result:**
- Both motors can rotate forward at different speeds
- Smooth curved trajectories
- Natural, intuitive control feel

---

### 🛠️ PS4 Controller Setup

#### 1️⃣ Install Required Packages

```bash
# Install joystick support
sudo apt update
sudo apt install joystick bluez bluez-tools

# Load kernel module
sudo modprobe joydev
echo "joydev" | sudo tee -a /etc/modules
```

#### 2️⃣ Pair PS4 Controller via Bluetooth

**Put controller in pairing mode:**
- Press and hold **PS + Share buttons** for 3-5 seconds
- Light bar should flash rapidly (white)

**Connect from Raspberry Pi:**

```bash
# Make adapter pairable
bluetoothctl pairable on

# Scan for controller
bluetoothctl scan on
# Wait 5 seconds, then:
bluetoothctl scan off

# Replace MAC with your controller's address
bluetoothctl pair XX:XX:XX:XX:XX:XX
bluetoothctl trust XX:XX:XX:XX:XX:XX
bluetoothctl connect XX:XX:XX:XX:XX:XX
```

#### 3️⃣ Verify Connection

```bash
# Check for joystick device
ls /dev/input/js*
# Should show: /dev/input/js0

# Test joystick input
jstest /dev/input/js0
# Move sticks and press buttons to verify
```

---

### ⚙️ Launch File Parameters

Customize behavior with parameters:

```bash
ros2 launch evarobot_firmware joy_teleop.launch.py \
  serial_port:=/dev/ttyACM0 \
  baud_rate:=115200 \
  joy_device:=0 \
  control_mode:=manual \
  linear_speed:=0.5 \
  angular_speed:=1.0
```

**Available parameters:**
- `serial_port` - Arduino serial port (default: `/dev/ttyACM0`)
- `baud_rate` - Serial baud rate (default: `115200`)
- `joy_device` - Joystick device ID (default: `0` for `/dev/input/js0`)
- `control_mode` - `manual` or `automatic` (default: `manual`)
- `linear_speed` - Max linear velocity m/s (default: `0.5`)
- `angular_speed` - Max angular velocity rad/s (default: `1.0`)

---

### 📊 Monitoring (Optional)

Monitor system topics in separate terminals:

```bash
# Watch velocity commands
ros2 topic echo /cmd_vel

# Watch encoder feedback
ros2 topic echo /evarobot/encoder_ticks

# Watch joystick raw input
ros2 topic echo /joy
```

---

### 🔍 Troubleshooting PS4 Controller

**Controller not connecting:**
- ✅ Verify Bluetooth is enabled: `bluetoothctl power on`
- ✅ Check controller is in pairing mode (rapid white flashing)
- ✅ Try removing old pairing: `bluetoothctl remove XX:XX:XX:XX:XX:XX`
- ✅ Restart Bluetooth: `sudo systemctl restart bluetooth`

**No /dev/input/js0 device:**
- ✅ Load joydev module: `sudo modprobe joydev`
- ✅ Reconnect controller after loading module
- ✅ Check permissions: `ls -la /dev/input/js0`

**Motors not responding to joystick:**
- ✅ Verify launch file started all 3 nodes (check terminal output)
- ✅ Check `/cmd_vel` topic is publishing: `ros2 topic hz /cmd_vel`
- ✅ Verify Arduino is connected: `ls /dev/ttyACM*`
- ✅ Test with `jstest` to ensure controller works

**Controller disconnects randomly:**
- ✅ Check battery level (plug in USB cable to charge)
- ✅ Reduce distance to Raspberry Pi
- ✅ Trust the device: `bluetoothctl trust XX:XX:XX:XX:XX:XX`

---

## 📡 Serial Communication Protocol

### 📝 Format
JSON messages terminated with newline (`\n`)

### 🎮 Commands

#### 1️⃣ Control Mode Commands

**Set Control Mode:**
```json
{"mode": "manual"}      // Switch to manual mode
{"mode": "automatic"}   // Switch to automatic mode
```

---

#### 2️⃣ Manual Mode Commands

**Motor Status (Direction):**
```json
{"motorStatus": "forward"}
```

**Valid values:**
| Command | Description | Left Motor | Right Motor |
|---------|-------------|------------|-------------|
| `"forward"` | Move forward | Forward ⬆️ | Forward ⬆️ |
| `"backward"` | Move backward | Backward ⬇️ | Backward ⬇️ |
| `"turnleft"` | Rotate left | Backward ⬇️ | Forward ⬆️ |
| `"turnright"` | Rotate right | Forward ⬆️ | Backward ⬇️ |
| `"stop"` | Stop motors | Stop 🛑 | Stop 🛑 |

**Motor Velocity (PWM %):**
```json
{"motorVelocity": {"left": 60, "right": 60}}
```

**Values:** `38` - `95` (PWM duty cycle percentage)

**Combined Example:**
```json
{"motorStatus": "forward", "motorVelocity": {"left": 70, "right": 70}}
```

---

#### 3️⃣ Automatic Mode Commands

**Velocity Setpoints (rad/s):**
```json
{"velocitySetpoints": {"left": 1.5, "right": 1.5}}
```

**Values:** Velocity in radians per second (rad/s)
- Typical range: `0.0` - `3.0` rad/s (depends on motor specs)
- Positive values = forward direction
- Negative values = backward direction

**Example - Forward at 1.5 rad/s:**
```json
{"mode": "automatic", "velocitySetpoints": {"left": 1.5, "right": 1.5}}
```

**Example - Rotate in place:**
```json
{"mode": "automatic", "velocitySetpoints": {"left": -1.0, "right": 1.0}}
```

---

#### 4️⃣ Encoder Feedback (Published by Arduino)

The Arduino publishes encoder data every 100ms when `ENABLE_ENCODER_FEEDBACK` is enabled:

```json
{
  "encoders": {
    "left": 1234,
    "right": 5678
  },
  "velocities": {
    "left": 0.523,
    "right": 0.518
  }
}
```

**Fields:**
- `encoders`: Cumulative encoder ticks (position)
- `velocities`: Current wheel velocities in rad/s

---

### 📤 Response Format

**Acknowledgment (DEBUG mode):**
```json
{"ack":true,"status":"forward","left":70,"right":70}
```
*(Only when `DEBUG_ENABLED` is set in config)*

**Startup Message:**
```json
{"status":"ready","mode":"manual"}
```

---

## 📚 Arduino Libraries Required

> ⚠️ **CRITICAL NOTE:** Use **Arduino IDE Desktop** (NOT Arduino Cloud IDE)
> The cloud IDE has compatibility issues with these specific library versions!

### Required Libraries & Versions

| Library | Version | Purpose | Installation |
|---------|---------|---------|--------------|
| 📦 **ArduinoJson** | `>= 6.x` | JSON protocol | Arduino Library Manager |
| ⚡ **RP2040_PWM** | `1.5.0` | PWM control | [GitHub](https://github.com/khoih-prog/RP2040_PWM) |
| 🔄 **Encoder** | `1.4.4` | Quadrature encoders | [GitHub](https://github.com/PaulStoffregen/Encoder) |
| 🎯 **PID by Brett Beauregard** | `1.2.0` | Closed-loop velocity control | Arduino Library Manager |

### 📥 Installation Steps

1. **Open Arduino IDE Desktop** (version 2.x recommended)

2. **Install ArduinoJson:**
   - Go to `Tools` → `Manage Libraries...`
   - Search for "ArduinoJson"
   - Install version 6.x or higher

3. **Install RP2040_PWM v1.5.0:**
   - Go to `Tools` → `Manage Libraries...`
   - Search for "RP2040_PWM"
   - Select version **1.5.0** (very important!)
   - Click Install

4. **Install Encoder v1.4.4:**
   - Go to `Tools` → `Manage Libraries...`
   - Search for "Encoder"
   - Select version **1.4.4** (very important!)
   - Click Install

5. **Install PID_v1:**
   - Go to `Tools` → `Manage Libraries...`
   - Search for "PID"
   - Select **"PID by Brett Beauregard"**
   - Install version 1.2.0 or higher

### ⚠️ Troubleshooting Library Errors

If you encounter compilation errors:
- ✅ Verify you're using **Arduino IDE Desktop** (NOT Cloud IDE)
- ✅ Check library versions match exactly: `RP2040_PWM 1.5.0`, `Encoder 1.4.4`
- ✅ Restart Arduino IDE after installing libraries
- ✅ Check board manager has RP2040 support installed

---

## 📤 Upload Instructions

1. **Open** `firmware/evarobot_control.ino` in Arduino IDE Desktop
2. **Select Board:** `Tools` → `Board` → `Raspberry Pi Pico/RP2040` → `Arduino Nano RP2040 Connect`
3. **Select Port:** `Tools` → `Port` → (your RP2040 port, e.g., `/dev/ttyACM0`)
4. **Upload:** Click the upload button ➡️

### First Upload (Bootloader Mode)
If the board is not detected:
1. Hold the **BOOT** button on the RP2040
2. Press and release **RESET** while holding BOOT
3. Release **BOOT** button
4. Board should appear as USB storage device
5. Select port and upload

---

## 🧪 Testing

The firmware includes comprehensive test scripts for validation and debugging.

### Test Scripts

#### 1️⃣ Manual Control Test (`test_motor_serial.py`)

Test basic motor control without PID:

```bash
cd evarobot_firmware
python3 test_motor_serial.py /dev/ttyACM0
```

**Features:**
- Basic movement commands
- Speed testing (38-95%)
- Interactive keyboard control

**Test Modes:**
- `--test basic` - Automated movement sequence
- `--test speed` - Test different PWM values
- `--test interactive` - Manual keyboard control (default)

---

#### 2️⃣ PID & Encoder Feedback Test (`test_pid_feedback.py`)

Test PID control and encoder feedback:

```bash
cd evarobot_firmware
python3 test_pid_feedback.py /dev/ttyACM0
```

**Features:**
- Real-time encoder position monitoring
- Velocity measurement validation
- PID setpoint testing
- Mode switching verification
- Live feedback display

**Test Modes:**

```bash
# Test encoder feedback only
python3 test_pid_feedback.py /dev/ttyACM0 --test encoder

# Test PID control with different velocities
python3 test_pid_feedback.py /dev/ttyACM0 --test pid

# Test mode switching (manual ↔ automatic)
python3 test_pid_feedback.py /dev/ttyACM0 --test switch

# Run all tests
python3 test_pid_feedback.py /dev/ttyACM0 --test all

# Interactive mode (default)
python3 test_pid_feedback.py /dev/ttyACM0
```

**Interactive Commands:**
- `a` - Switch to automatic mode
- `m` - Switch to manual mode
- `s 1.5 1.5` - Set PID velocity setpoints (left, right in rad/s)
- `f 70` - Manual forward at 70% PWM
- `x` - Stop motors
- `q` - Quit

**Example Output:**
```
Time         Left Enc     Right Enc    Left Vel (rad/s)   Right Vel (rad/s)
================================================================================
08:45:12.123 1234         5678         1.523              1.518
08:45:12.323 1289         5745         1.531              1.524
...
```

---

## 🎯 PID Tuning

The PID controller requires tuning to match your specific motors and load.

### Default Parameters

Located in `evarobot_config.h:70-77`:

```cpp
// Left Motor PID Gains
#define PID_KP_LEFT 11.5   // Proportional gain
#define PID_KI_LEFT 7.5    // Integral gain
#define PID_KD_LEFT 0.1    // Derivative gain

// Right Motor PID Gains
#define PID_KP_RIGHT 12.8  // Proportional gain
#define PID_KI_RIGHT 8.3   // Integral gain
#define PID_KD_RIGHT 0.1   // Derivative gain
```

*These values are based on Bumper-Bot reference design and may need adjustment.*

---

### Tuning Procedure

#### Step 1: Test with Default Values

```bash
# Start interactive test
python3 test_pid_feedback.py /dev/ttyACM0

# In the test script:
> a                    # Switch to automatic mode
> s 1.0 1.0           # Set low velocity setpoint
```

**Observe the behavior:**
- Does velocity reach setpoint?
- Is there overshoot?
- Does it oscillate?
- How long to stabilize?

---

#### Step 2: Adjust Proportional Gain (Kp)

**Effect:** Controls responsiveness

- **Too low:** Slow response, doesn't reach setpoint
- **Too high:** Overshoots, oscillates
- **Good value:** Quick response with minimal overshoot

**Tuning:**
1. Start with `Kp = 1.0`, `Ki = 0`, `Kd = 0`
2. Increase Kp until system responds quickly
3. If oscillations occur, reduce Kp by 20-30%

---

#### Step 3: Add Integral Gain (Ki)

**Effect:** Eliminates steady-state error

- **Too low:** Setpoint never quite reached
- **Too high:** Overshoot and oscillations
- **Good value:** Reaches setpoint with minimal overshoot

**Tuning:**
1. With Kp tuned, start `Ki = 0.1`
2. Increase until steady-state error is eliminated
3. Reduce if oscillations appear

---

#### Step 4: Add Derivative Gain (Kd)

**Effect:** Dampens oscillations

- **Too low:** System oscillates
- **Too high:** Sluggish response, noise sensitive
- **Good value:** Smooth approach to setpoint

**Tuning:**
1. Usually small values: `0.01` - `0.5`
2. Increase if oscillations persist
3. Reduce if response becomes sluggish

---

#### Step 5: Fine-tune Each Motor

Motors may have different characteristics. Tune left and right independently:

```bash
# Test left motor only
> s 1.5 0.0

# Test right motor only
> s 0.0 1.5

# Test both together
> s 1.5 1.5
```

---

### Validation Tests

After tuning, verify performance:

```bash
python3 test_pid_feedback.py /dev/ttyACM0 --test pid
```

**Check:**
- ✅ Setpoint tracking accuracy
- ✅ Response time (< 1 second ideal)
- ✅ Overshoot (< 10% ideal)
- ✅ Steady-state error (< 2% ideal)
- ✅ Both motors track similarly

---

### Debug Options

Enable detailed PID debugging in `evarobot_config.h`:

```cpp
#define DEBUG_ENABLED 1         // General debug output
#define PID_DEBUG_ENABLED 2     // Verbose PID debug (1=basic, 2=verbose)
```

**Debug output includes:**
- Measured velocities
- PID setpoints
- PID outputs (PWM values)
- Computation timing

---

## 📊 PID Test Results

### Motor Characterization Summary

Comprehensive testing was conducted to characterize motor behavior and validate PID control performance.

#### 🔑 Key Findings

**Motor Dependency:**
- ⚠️ **Critical:** Motors must run together - they do not work independently
- Single motor operation is not supported by the current hardware configuration
- Both motors must receive power for proper operation

**Motor Balance:**
- Left/Right motor velocity ratio: **0.97** (well-matched)
- Motors exhibit nearly identical performance characteristics
- Minimal compensation needed for straight-line motion

**Controllable Velocity Range:**
- Minimum: **63 rad/s** at 39% PWM (lowest controllable speed)
- Maximum: **103 rad/s** at 95% PWM (hardware limit)
- Effective range: **63-103 rad/s** (40 rad/s span)

#### 📈 PWM to Velocity Mapping

Measured velocity characteristics for both motors running simultaneously:

| PWM Duty Cycle | Left Motor (rad/s) | Right Motor (rad/s) | Notes |
|----------------|-------------------|-------------------|--------|
| **39%** | 60-63 | 60-63 | Minimum controllable speed |
| **50%** | 72-75 | 72-75 | Low speed operation |
| **70%** | 95-99 | 95-99 | Medium speed (optimal for navigation) |
| **85%** | 100-104 | 100-104 | High speed |
| **95%** | 103-109 | 103-109 | Maximum speed (hardware limit) |

**Velocity Constraints Added to Configuration:**
```cpp
// evarobot_config.h
#define MOTOR_MIN_VELOCITY 63.0f    // rad/s - Minimum controllable
#define MOTOR_MAX_VELOCITY 103.0f   // rad/s - Maximum at 95% PWM
```

#### ✅ PID Implementation Improvements

Based on testing results, the following enhancements were implemented:

1. **Direction Handling:**
   - PID now properly handles positive/negative velocity setpoints
   - Motor direction set based on setpoint sign
   - PID works with absolute velocity values

2. **Zero-Velocity Detection:**
   - Clean motor stops when setpoints are zero
   - Prevents PID windup at zero velocity
   - Immediate response to stop commands

3. **Debug Separation:**
   - PID debug macros independent of general debug
   - Allows targeted debugging without verbose output
   - Levels: 0=Off, 1=Basic, 2=Verbose

#### 🎯 Recommended Operating Points

Based on test results:

| Use Case | Velocity (rad/s) | PWM (%) | Characteristics |
|----------|-----------------|---------|-----------------|
| **Slow Navigation** | 63-75 | 39-50 | Precise positioning, tight spaces |
| **Normal Operation** | 75-99 | 50-70 | Optimal for SLAM and navigation |
| **Fast Transit** | 99-103 | 70-95 | Quick movement between waypoints |

#### 📝 Test Configuration

**Hardware:**
- Arduino Nano RP2040 Connect
- Dual DC motors with quadrature encoders (385 ticks/rev)
- H-bridge motor driver
- PWM frequency: 500 Hz

**PID Parameters (Tuned):**
- Left Motor: Kp=11.5, Ki=7.5, Kd=0.1
- Right Motor: Kp=12.8, Ki=8.3, Kd=0.1
- Sample rate: 100ms

**Measurement Method:**
- Encoder-based velocity calculation
- Multiple test runs for each PWM level
- Both motors running simultaneously (required)

---

## 🔍 Troubleshooting

### 🚫 Motors not moving
- ✅ Check H-bridge connections (IN1-IN4 pins: 9, 10, 11, 12)
- ✅ Verify PWM pins (17-18) are connected correctly
- ✅ Check power supply to motors (ensure adequate current)
- ✅ Test with different PWM values (try 70-80%)
- ✅ Verify motor driver is powered separately from Arduino

### 📡 Serial communication issues
- ✅ Verify baud rate is **115200**
- ✅ Check USB cable connection (use quality cable with data lines)
- ✅ Ensure correct board is selected in Arduino IDE
- ✅ Try resetting the Arduino (BOOT + RESET)
- ✅ Check that Serial Monitor is closed when using ROS2 node

### 🔄 Encoder not counting
- ✅ Verify encoder connections to A0, A1 (left) and A2, A3 (right)
- ✅ Check encoder power supply (typically 3.3V or 5V)
- ✅ Test encoder manually by rotating wheels
- ✅ Enable debug mode: set `DEBUG_ENABLED 1` in `evarobot_config.h`

### 💡 Compilation errors
- ✅ **Use Arduino IDE Desktop** (NOT Cloud IDE)
- ✅ Verify library versions: `RP2040_PWM 1.5.0`, `Encoder 1.4.4`, `PID_v1 >= 1.2.0`
- ✅ Install RP2040 board support: `Tools` → `Board Manager` → "Raspberry Pi Pico/RP2040"
- ✅ Clear build cache: Close IDE, delete build folder, restart
- ✅ Check all libraries are installed: ArduinoJson, RP2040_PWM, Encoder, PID_v1

### 🎯 PID Control issues

**Motors don't move in automatic mode:**
- ✅ Verify you switched to automatic mode: `{"mode": "automatic"}`
- ✅ Set non-zero velocity setpoints: `{"velocitySetpoints": {"left": 1.0, "right": 1.0}}`
- ✅ Check encoders are connected and counting (test in manual mode first)
- ✅ Enable PID debug: `PID_DEBUG_ENABLED 2` in config
- ✅ Verify PID output limits match PWM range (38-95%)

**Velocity doesn't reach setpoint:**
- ✅ Increase Kp gain (proportional)
- ✅ Add Ki gain (integral) to eliminate steady-state error
- ✅ Check encoder feedback is updating (enable `ENABLE_ENCODER_FEEDBACK`)
- ✅ Verify velocity calculation: wheel should move when setpoint > 0
- ✅ Check motor power supply is adequate

**Motors oscillate or overshoot:**
- ✅ Reduce Kp gain (proportional)
- ✅ Reduce Ki gain (integral)
- ✅ Add small Kd gain (derivative) for damping
- ✅ Check PID sample time (100ms default in config)
- ✅ Verify encoders are noise-free (check wiring, add capacitors)

**No encoder feedback received:**
- ✅ Verify `ENABLE_ENCODER_FEEDBACK 1` in config
- ✅ Check `FEEDBACK_INTERVAL` setting (100ms default)
- ✅ Test encoders manually: rotate wheels and check position changes
- ✅ Enable debug mode and watch Serial Monitor
- ✅ Verify USB serial connection (close Arduino Serial Monitor)

---

## ⚠️ Known Issues

| Issue | Status | Solution |
|-------|--------|----------|
| ~~Backward + turning combinations may behave incorrectly~~ | ✅ **FIXED** | Differential drive implemented (v1.1.0) |
| ~~Forward/backward + left/right combinations~~ | ✅ **FIXED** | Both sticks now work simultaneously |

**Recent Updates:**
- **2025-10-25:** Implemented true differential drive control - both joystick sticks work together for smooth curved motion

---

## 🧪 Testing

Use the provided test script to verify motor control:

```bash
cd evarobot_firmware
python3 test_motor_serial.py /dev/ttyACM0
```

**Test modes:**
- `--test basic` - Run basic movement test
- `--test speed` - Test different speed values
- `--test interactive` - Manual keyboard control (default)

---

## 📄 License

This project is licensed under the BSD-3-Clause License.

---

## 👤 Author

**Kevin Medrano Ayala**
- 📧 Email: kevin.ejem18@gmail.com
- 🔗 GitHub: [EvaRobot Project](https://github.com)

---

## 🙏 Acknowledgments

- Arduino RP2040 community
- ROS2 community
- Library authors:
  - khoih-prog (RP2040_PWM)
  - PaulStoffregen (Encoder)
  - Brett Beauregard (PID_v1)

---

*Last updated: 2025-10-25*
*Version: 1.1.0 - PS4 Joystick + Differential Drive*
