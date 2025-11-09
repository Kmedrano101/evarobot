# PS4 Controller Bluetooth Setup

Automated Bluetooth connection system for PS4 DualShock 4 controllers with EvaRobot.

## Quick Start

### 1. Pair or Connect Your Controller

Use the unified management script:

```bash
cd ~/evarobot_ws/src/evarobot_utils/scripts/bluetooth
./ps4_controller.sh
```

The script will guide you through:
- **Option 1:** Pair controller (first time setup)
- **Option 2:** Connect controller (already paired)
- **Option 3:** Show controller info

**Pairing Instructions:**
- Press and hold **PS + Share** buttons for 3-5 seconds
- Light bar should flash **WHITE** rapidly
- Follow the on-screen prompts

**Default MAC Address:** AC:36:1B:92:0F:C3
The script will ask if you want to use a different MAC address.

### 2. Install Auto-Connection (Optional)

For automatic connection on boot:

```bash
sudo ./install_bt_autoconnect.sh
```

This installs:
- Systemd service for auto-pairing on boot
- Udev rules for device detection
- Logging system for troubleshooting

### 3. Use with EvaRobot

Once connected:

```bash
cd ~/evarobot_ws
source install/setup.bash
ros2 launch evarobot_firmware joy_teleop.launch.py
```

**Controller Layout:**
- **Hold R1** + **Left Stick Vertical** = Forward/Backward
- **Hold R1** + **Right Stick Horizontal** = Rotate Left/Right
- **D-pad Up** = Increase speed
- **D-pad Down** = Decrease speed

## File Structure

```
bluetooth/
├── ps4_controller.sh           # Main script - pair and connect controllers
├── install_bt_autoconnect.sh   # Install auto-connection system
├── bt_autopair.sh              # Auto-pair service (runs on boot)
├── bt_connect_ps4.sh           # Auto-connect script (triggered by udev)
├── bt-autopair.service         # Systemd service configuration
├── 99-ps4-controller.rules     # Udev rules for device detection
└── README.md                   # This file
```

## Features

- **Interactive Menu:** Easy-to-use interface for pairing and connecting
- **Flexible MAC Address:** Use default or specify custom controller MAC
- **Auto-Connection:** Optional automatic connection on system boot
- **Status Information:** View controller connection details
- **Validation:** MAC address format validation and error checking
- **Joystick Detection:** Automatic verification of /dev/input/js* devices

## Verification

After connecting your controller, verify it's working:

```bash
# Check Bluetooth connection
bluetoothctl devices
bluetoothctl info XX:XX:XX:XX:XX:XX

# Check joystick device
ls -l /dev/input/js*

# Test joystick input
jstest /dev/input/js0
```

## Auto-Connection System

The auto-connection system provides:
- **Automatic pairing** on system boot
- **Auto-reconnection** when controller is powered on
- **Systemd service** for reliable startup
- **Udev rules** for device detection
- **Logging** for troubleshooting

### Check Service Status

```bash
# View service status
sudo systemctl status bt-autopair.service

# View logs
tail -f /var/log/bt_autopair.log
tail -f /var/log/bt_ps4_connect.log

# Restart service
sudo systemctl restart bt-autopair.service
```

## Troubleshooting

### Controller Won't Pair

```bash
# Remove old pairing
bluetoothctl remove XX:XX:XX:XX:XX:XX

# Run the pairing script again
./ps4_controller.sh
# Select option 1
```

### Controller Paired but Won't Connect

```bash
# Make sure controller is on (press PS button)
# Then run:
./ps4_controller.sh
# Select option 2
```

### No /dev/input/js0 Device

```bash
# Load joystick module
sudo modprobe joydev

# Check if loaded
lsmod | grep joydev

# Make it permanent
echo "joydev" | sudo tee -a /etc/modules
```

### Bluetooth Adapter Issues

```bash
# Check Bluetooth adapter status
hciconfig -a

# Power on adapter
sudo hciconfig hci0 up

# Restart Bluetooth service
sudo systemctl restart bluetooth
```

### Permission Issues

```bash
# Add user to bluetooth and input groups
sudo usermod -aG bluetooth $USER
sudo usermod -aG input $USER

# Log out and back in for changes to take effect
```

### Controller Disconnects Randomly

```bash
# Make sure it's trusted
bluetoothctl trust XX:XX:XX:XX:XX:XX

# Check battery level (charge via USB if low)
```

## Manual Bluetooth Commands

If you prefer manual control:

```bash
# Start bluetoothctl
bluetoothctl

# In bluetoothctl, run these commands:
power on
agent on
default-agent
scan on

# Wait to see "Wireless Controller" with MAC address
# Then pair (replace XX:XX:XX:XX:XX:XX with your MAC):
pair XX:XX:XX:XX:XX:XX
trust XX:XX:XX:XX:XX:XX
connect XX:XX:XX:XX:XX:XX
scan off
exit
```

### Quick Manual Commands

```bash
# Connect
bluetoothctl connect XX:XX:XX:XX:XX:XX

# Disconnect
bluetoothctl disconnect XX:XX:XX:XX:XX:XX

# Check status
bluetoothctl info XX:XX:XX:XX:XX:XX

# Remove pairing
bluetoothctl remove XX:XX:XX:XX:XX:XX

# List all devices
bluetoothctl devices
```

## Uninstall Auto-Connection

To remove the auto-connection system:

```bash
# Stop and disable service
sudo systemctl stop bt-autopair.service
sudo systemctl disable bt-autopair.service

# Remove service file
sudo rm /etc/systemd/system/bt-autopair.service

# Remove udev rules
sudo rm /etc/udev/rules.d/99-ps4-controller.rules

# Reload systemd and udev
sudo systemctl daemon-reload
sudo udevadm control --reload-rules

# Remove log files (optional)
sudo rm /var/log/bt_autopair.log
sudo rm /var/log/bt_ps4_connect.log
sudo rm /var/log/bt_ps4_pair.log
```

## Requirements

- Ubuntu 24.04 or compatible Linux distribution
- ROS2 Jazzy (for EvaRobot integration)
- Bluetooth adapter
- Packages: bluez, bluetooth, joystick

The installation script will handle package installation automatically.

## Additional Resources

- [Arch Linux Bluetooth Wiki](https://wiki.archlinux.org/title/Bluetooth)
- [Ubuntu Bluetooth Documentation](https://help.ubuntu.com/community/BluetoothSetup)
- [PS4 Controller on Linux](https://github.com/chrippa/ds4drv)

## Reporting Issues

If you encounter issues:

1. Check logs: `tail -f /var/log/bt_autopair.log`
2. Check service: `sudo systemctl status bt-autopair.service`
3. Check Bluetooth: `bluetoothctl show`
4. Run controller info: `./ps4_controller.sh` (option 3)
5. Provide error messages when reporting

## License

BSD-3-Clause License (same as EvaRobot project)

---

**Last Updated:** 2025-11-09
**Compatible with:** Ubuntu 24.04, ROS2 Jazzy
