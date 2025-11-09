#!/bin/bash
# Unified PS4 Controller Management Script
# Handles pairing and connecting to PS4 DualShock 4 controllers

# Color codes
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
CYAN='\033[0;36m'
NC='\033[0m' # No Color

# Default controller MAC address
DEFAULT_MAC="AC:36:1B:92:0F:C3"

# Functions
show_header() {
    echo -e "${GREEN}========================================${NC}"
    echo -e "${GREEN}PS4 Controller Manager${NC}"
    echo -e "${GREEN}========================================${NC}"
    echo ""
}

show_menu() {
    echo -e "${CYAN}What would you like to do?${NC}"
    echo ""
    echo "  1) Pair controller (first time setup)"
    echo "  2) Connect controller (already paired)"
    echo "  3) Show controller info"
    echo "  4) Exit"
    echo ""
}

get_mac_address() {
    echo -e "${YELLOW}Controller MAC Address${NC}"
    echo ""
    echo "Default: ${BLUE}${DEFAULT_MAC}${NC}"
    echo ""
    read -p "Press Enter to use default, or type a different MAC address: " INPUT_MAC

    if [ -z "$INPUT_MAC" ]; then
        PS4_MAC="$DEFAULT_MAC"
        echo -e "Using default: ${BLUE}${PS4_MAC}${NC}"
    else
        # Validate MAC address format
        if [[ $INPUT_MAC =~ ^([0-9A-Fa-f]{2}:){5}[0-9A-Fa-f]{2}$ ]]; then
            PS4_MAC=$(echo "$INPUT_MAC" | tr '[:lower:]' '[:upper:]')
            echo -e "Using custom: ${BLUE}${PS4_MAC}${NC}"
        else
            echo -e "${RED}Invalid MAC address format!${NC}"
            echo "Expected format: XX:XX:XX:XX:XX:XX"
            return 1
        fi
    fi
    echo ""
    return 0
}

check_bluetooth() {
    if ! command -v bluetoothctl &> /dev/null; then
        echo -e "${RED}Error: bluetoothctl not found!${NC}"
        echo "Please install bluez: sudo apt-get install bluez"
        return 1
    fi

    # Power on Bluetooth
    bluetoothctl power on &>/dev/null
    return 0
}

pair_controller() {
    echo -e "${GREEN}========================================${NC}"
    echo -e "${GREEN}Pairing PS4 Controller${NC}"
    echo -e "${GREEN}========================================${NC}"
    echo ""

    if ! get_mac_address; then
        return 1
    fi

    # Check if already paired
    if bluetoothctl info "$PS4_MAC" 2>&1 | grep -q "Paired: yes"; then
        echo -e "${GREEN}✓ Controller is already paired!${NC}"
        echo ""
        read -p "Remove and re-pair? (y/N): " REPAIR
        if [[ $REPAIR =~ ^[Yy]$ ]]; then
            echo "Removing existing pairing..."
            bluetoothctl remove "$PS4_MAC" &>/dev/null
        else
            echo "Keeping existing pairing."
            return 0
        fi
    fi

    echo -e "${YELLOW}Instructions:${NC}"
    echo "  1. Press and hold ${BLUE}PS + Share${NC} buttons for 3-5 seconds"
    echo "  2. Light bar should flash ${BLUE}WHITE${NC} rapidly"
    echo ""
    read -p "Press Enter when controller is in pairing mode..."
    echo ""

    echo -e "${YELLOW}[1/5]${NC} Initializing Bluetooth..."
    (
        echo -e "power on\nagent NoInputNoOutput\ndefault-agent"
        sleep 2
    ) | bluetoothctl &>/dev/null
    sleep 1

    echo -e "${YELLOW}[2/5]${NC} Scanning for device (15 seconds)..."
    echo "  Keep controller in pairing mode (light flashing white)..."
    (
        echo "scan on"
        sleep 15
        echo "scan off"
    ) | bluetoothctl > /tmp/bt_scan.log 2>&1

    # Check if controller was found
    if grep -q "$PS4_MAC" /tmp/bt_scan.log; then
        echo -e "${GREEN}✓ Controller found in scan!${NC}"
    else
        echo -e "${YELLOW}⚠ Specific MAC not seen in scan, but attempting pairing anyway...${NC}"
    fi

    echo -e "${YELLOW}[3/5]${NC} Pairing with controller..."
    if bluetoothctl pair "$PS4_MAC" 2>&1 | grep -q "AlreadyExists\|Failed"; then
        if bluetoothctl info "$PS4_MAC" | grep -q "Paired: yes"; then
            echo -e "${GREEN}✓ Controller is already paired${NC}"
        else
            echo -e "${RED}✗ Pairing failed${NC}"
            rm -f /tmp/bt_scan.log
            return 1
        fi
    else
        echo -e "${GREEN}✓ Pairing completed${NC}"
    fi

    echo -e "${YELLOW}[4/5]${NC} Setting device as trusted..."
    bluetoothctl trust "$PS4_MAC" &>/dev/null
    echo -e "${GREEN}✓ Device trusted${NC}"
    sleep 1

    echo -e "${YELLOW}[5/5]${NC} Connecting to controller..."
    if bluetoothctl connect "$PS4_MAC"; then
        echo ""
        echo -e "${GREEN}========================================${NC}"
        echo -e "${GREEN}✓ Controller Paired and Connected!${NC}"
        echo -e "${GREEN}========================================${NC}"
        echo ""

        # Show device info
        echo -e "${BLUE}Device Information:${NC}"
        bluetoothctl info "$PS4_MAC" | grep -E "Name|Alias|Connected|Paired|Trusted"
        echo ""

        # Check for joystick device
        sleep 2
        check_joystick

        echo ""
        echo -e "${GREEN}Setup Complete!${NC}"
        echo ""
    else
        echo ""
        echo -e "${YELLOW}⚠ Connection failed${NC}"
        echo ""
        echo "The controller is paired but not connected."
        echo "Try pressing the PS button and run option 2 (Connect)"
        echo ""
    fi

    # Cleanup
    rm -f /tmp/bt_scan.log
}

connect_controller() {
    echo -e "${GREEN}========================================${NC}"
    echo -e "${GREEN}Connecting PS4 Controller${NC}"
    echo -e "${GREEN}========================================${NC}"
    echo ""

    if ! get_mac_address; then
        return 1
    fi

    # Check if device is paired
    echo -e "${YELLOW}[1/3]${NC} Checking if device is paired..."
    if bluetoothctl info "$PS4_MAC" &>/dev/null; then
        echo -e "${GREEN}✓ Device is paired${NC}"
    else
        echo -e "${RED}✗ Device is not paired yet${NC}"
        echo ""
        echo "Please pair the controller first (option 1)"
        return 1
    fi

    # Trust the device
    echo -e "${YELLOW}[2/3]${NC} Setting device as trusted..."
    bluetoothctl trust "$PS4_MAC" &>/dev/null || true

    # Connect to the device
    echo -e "${YELLOW}[3/3]${NC} Connecting to controller..."
    echo "  (Press PS button if not already on)"
    if bluetoothctl connect "$PS4_MAC"; then
        echo ""
        echo -e "${GREEN}========================================${NC}"
        echo -e "${GREEN}✓ Controller Connected Successfully!${NC}"
        echo -e "${GREEN}========================================${NC}"
        echo ""

        # Check for joystick device
        sleep 2
        check_joystick
        echo ""
    else
        echo ""
        echo -e "${RED}========================================${NC}"
        echo -e "${RED}✗ Connection Failed${NC}"
        echo -e "${RED}========================================${NC}"
        echo ""
        echo "Troubleshooting:"
        echo "  1. Make sure controller is on (press PS button)"
        echo "  2. Try removing and re-pairing (option 1)"
        echo "  3. Check Bluetooth service: systemctl status bluetooth"
        echo ""
        return 1
    fi
}

show_info() {
    echo -e "${GREEN}========================================${NC}"
    echo -e "${GREEN}Controller Information${NC}"
    echo -e "${GREEN}========================================${NC}"
    echo ""

    if ! get_mac_address; then
        return 1
    fi

    if bluetoothctl info "$PS4_MAC" &>/dev/null; then
        bluetoothctl info "$PS4_MAC"
        echo ""
        check_joystick
    else
        echo -e "${RED}Controller not found or not paired${NC}"
        echo ""
        echo "Available controllers:"
        bluetoothctl devices | grep -i "wireless controller\|dualshock"
    fi
    echo ""
}

check_joystick() {
    if ls /dev/input/js* &> /dev/null; then
        echo -e "${GREEN}✓ Joystick device detected:${NC}"
        ls -l /dev/input/js*
        echo ""
        echo "Test with: jstest /dev/input/js0"
        echo ""
        echo "Use with EvaRobot:"
        echo "  ros2 launch evarobot_firmware joy_teleop.launch.py"
    else
        echo -e "${YELLOW}⚠ Joystick device not found yet${NC}"
        echo ""
        echo "Try loading the joydev module:"
        echo "  sudo modprobe joydev"
    fi
}

# Main script
main() {
    show_header

    if ! check_bluetooth; then
        exit 1
    fi

    while true; do
        show_menu
        read -p "Select option (1-4): " choice
        echo ""

        case $choice in
            1)
                pair_controller
                ;;
            2)
                connect_controller
                ;;
            3)
                show_info
                ;;
            4)
                echo "Goodbye!"
                exit 0
                ;;
            *)
                echo -e "${RED}Invalid option. Please select 1-4.${NC}"
                echo ""
                ;;
        esac

        echo ""
        read -p "Press Enter to continue..."
        clear
        show_header
    done
}

# Run main function
main
