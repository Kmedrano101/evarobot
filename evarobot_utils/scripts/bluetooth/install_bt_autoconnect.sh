#!/bin/bash
# Installation script for PS4 Controller Bluetooth Auto-Connection
# Run this script with sudo to install the auto-connection system

set -e

# Color codes for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m' # No Color

echo -e "${GREEN}========================================${NC}"
echo -e "${GREEN}PS4 Controller Bluetooth Auto-Connect${NC}"
echo -e "${GREEN}Installation Script${NC}"
echo -e "${GREEN}========================================${NC}"
echo ""

# Check if running as root
if [ "$EUID" -ne 0 ]; then
    echo -e "${RED}Error: Please run as root (use sudo)${NC}"
    exit 1
fi

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

echo -e "${YELLOW}Installing Bluetooth auto-connection system...${NC}"
echo ""

# 1. Install required packages
echo -e "${GREEN}[1/6]${NC} Installing required packages..."
apt-get update -qq
apt-get install -y bluetooth bluez bluez-tools joystick

# 2. Enable joydev kernel module
echo -e "${GREEN}[2/6]${NC} Enabling joydev kernel module..."
modprobe joydev
if ! grep -q "joydev" /etc/modules; then
    echo "joydev" >> /etc/modules
    echo "  → Added joydev to /etc/modules"
fi

# 3. Copy and set permissions for scripts
echo -e "${GREEN}[3/6]${NC} Installing Bluetooth scripts..."
chmod +x "$SCRIPT_DIR/bt_autopair.sh"
chmod +x "$SCRIPT_DIR/bt_connect_ps4.sh"
chmod +x "$SCRIPT_DIR/bt_pair_usb_ps4.sh"
echo "  → Script permissions set"

# 4. Install systemd service
echo -e "${GREEN}[4/6]${NC} Installing systemd service..."
cp "$SCRIPT_DIR/bt-autopair.service" /etc/systemd/system/
systemctl daemon-reload
systemctl enable bt-autopair.service
echo "  → Systemd service installed and enabled"

# 5. Install udev rules
echo -e "${GREEN}[5/6]${NC} Installing udev rules..."
cp "$SCRIPT_DIR/99-ps4-controller.rules" /etc/udev/rules.d/
udevadm control --reload-rules
udevadm trigger
echo "  → Udev rules installed"

# 6. Create log files
echo -e "${GREEN}[6/6]${NC} Creating log files..."
touch /var/log/bt_autopair.log
touch /var/log/bt_ps4_connect.log
touch /var/log/bt_ps4_pair.log
chmod 666 /var/log/bt_autopair.log
chmod 666 /var/log/bt_ps4_connect.log
chmod 666 /var/log/bt_ps4_pair.log
echo "  → Log files created"

echo ""
echo -e "${GREEN}========================================${NC}"
echo -e "${GREEN}Installation Complete!${NC}"
echo -e "${GREEN}========================================${NC}"
echo ""
echo -e "${YELLOW}Next Steps:${NC}"
echo ""
echo "1. Pair your PS4 controller (first time only):"
echo "   - Press and hold PS + Share buttons for 3-5 seconds"
echo "   - Run: bluetoothctl"
echo "   - In bluetoothctl: scan on"
echo "   - Wait to see your controller MAC address"
echo "   - In bluetoothctl: pair <MAC_ADDRESS>"
echo "   - In bluetoothctl: trust <MAC_ADDRESS>"
echo "   - In bluetoothctl: connect <MAC_ADDRESS>"
echo ""
echo "2. Start the auto-connection service:"
echo "   sudo systemctl start bt-autopair.service"
echo ""
echo "3. Reboot to test auto-connection on boot:"
echo "   sudo reboot"
echo ""
echo "4. View logs:"
echo "   tail -f /var/log/bt_autopair.log"
echo ""
echo -e "${GREEN}The controller will now auto-connect on boot!${NC}"
echo ""
