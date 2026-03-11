# Jumper T-Pro V2 + ELRS Receiver Setup for Raspberry Pi 4B

Complete guide for connecting a Jumper T-Pro V2 transmitter with an ExpressLRS
receiver to a Raspberry Pi 4B for RC teleop control via the `evarobot_rc` package.

**Tested Hardware:**
- Transmitter: Jumper T-Pro V2 (EdgeTX, internal ELRS 2.4GHz module)
- Receiver: Jumper AION Mini 2.4GHz RX (ELRS firmware 3.3.1)
- SBC: Raspberry Pi 4B (Ubuntu, Linux 6.8.0-raspi)

---

## Table of Contents

1. [Raspberry Pi UART Configuration](#1-raspberry-pi-uart-configuration)
2. [Wiring the Receiver](#2-wiring-the-receiver)
3. [ELRS Receiver Settings](#3-elrs-receiver-settings)
4. [Transmitter Settings](#4-transmitter-settings)
5. [Building and Running](#5-building-and-running)
6. [Verification](#6-verification)
7. [Key Lessons Learned](#7-key-lessons-learned)
8. [Troubleshooting](#8-troubleshooting)

---

## 1. Raspberry Pi UART Configuration

The Pi 4B has a PL011 UART on GPIO 14 (TXD) and GPIO 15 (RXD), exposed as
`/dev/ttyAMA0`. By default, Bluetooth occupies this UART and a serial console
may be active on it. Both must be disabled.

### 1.1 Disable Bluetooth overlay

Add to `/boot/firmware/config.txt` under the `[all]` section (NOT under `[cm4]`
or any other board-specific section):

```ini
[all]
dtoverlay=disable-bt
```

> **Keynote:** If placed under `[cm4]`, this overlay is ignored on Pi 4B and
> `/dev/ttyAMA0` will not appear.

### 1.2 Remove serial console from kernel command line

Edit `/boot/firmware/cmdline.txt` and remove any `console=serial0,115200` entry.
The remaining contents should look something like:

```
multipath=off dwc_otg.lpm_enable=0 ... root=PARTUUID=...
```

> **Keynote:** If the serial console is left active, the Pi interprets incoming
> binary CRSF data as shell commands, causing erratic behavior.

### 1.3 Disable serial getty service

```bash
sudo systemctl stop serial-getty@ttyAMA0.service
sudo systemctl disable serial-getty@ttyAMA0.service
```

### 1.4 Create udev rule for permissions

Create `/etc/udev/rules.d/99-ttyAMA0.rules`:

```
KERNEL=="ttyAMA0", GROUP="dialout", MODE="0660"
```

Then reload:

```bash
sudo udevadm control --reload-rules
sudo udevadm trigger
```

Add your user to the `dialout` group if not already:

```bash
sudo usermod -aG dialout $USER
```

### 1.5 Reboot and verify

```bash
sudo reboot
# After reboot:
ls -la /dev/ttyAMA0
# Should show: crw-rw---- 1 root dialout ... /dev/ttyAMA0
```

---

## 2. Wiring the Receiver

### Connections

| ELRS Receiver | Raspberry Pi 4B        | Notes                        |
|---------------|------------------------|------------------------------|
| TX pad        | Pin 10 (GPIO 15 / RXD) | Receiver transmits to Pi     |
| RX pad        | Pin 8 (GPIO 14 / TXD)  | Optional (for telemetry)     |
| GND           | Any GND pin (e.g. pin 6) | **Must share ground with Pi** |
| 5V / VCC      | **External 5V supply**  | See power warning below      |

### Power Warning

> **CRITICAL: Do NOT power the ELRS receiver from the Pi GPIO 5V/3.3V pins.**
>
> The receiver can draw enough current during transmit bursts to cause the Pi to
> brownout and restart. Use a separate regulated 5V power source for the receiver
> and connect the grounds together.

### Diagram

```
                    Raspberry Pi 4B
                   ┌──────────────────┐
                   │  Pin 8  (GPIO 14 TXD) ◄── RX pad (optional)
  ELRS Receiver    │  Pin 10 (GPIO 15 RXD) ◄── TX pad
                   │  Pin 6  (GND)         ◄── GND ──┐
                   └──────────────────┘               │
                                                      │
  External 5V ──► VCC                                 │
  External GND ──► GND ──────────────────────────────┘
```

---

## 3. ELRS Receiver Settings

Access the ELRS receiver web UI (connect via WiFi when the receiver is not bound,
or use ExpressLRS Configurator) and verify these settings:

| Setting        | Value     |
|----------------|-----------|
| Protocol       | **CRSF**  |
| UART Baud      | **420000** |
| Firmware       | 3.3.1+    |

> **Keynote:** The receiver web UI shows "UART Baud: 420000". The transmitter
> (EdgeTX) shows "Baudrate: 400K" — these refer to different links. The UART
> baud between receiver and Pi is 420000.

> **Keynote:** "Inverted CRSF" is for flight controllers with inverted UARTs
> (e.g. some F4 boards). The Pi 4B PL011 uses normal (non-inverted) logic levels,
> so use plain **CRSF**.

---

## 4. Transmitter Settings

On the Jumper T-Pro V2 (EdgeTX), configure the external/internal module:

| Setting        | Value      |
|----------------|------------|
| Mode           | CRSF       |
| Baudrate       | 400K       |
| Channel Range  | CH1-16     |
| Channel Order  | AETR       |

Channel mapping with AETR order:

| Channel | Function  | Stick              | Robot Use |
|---------|-----------|--------------------|-----------|
| CH0     | Aileron   | Right stick horiz. | Steering  |
| CH1     | Elevator  | Right stick vert.  | Throttle  |
| CH2     | Throttle  | Left stick vert.   | Unused    |
| CH3     | Rudder    | Left stick horiz.  | Unused    |
| CH4     | AUX1      | Switch SA (2-pos)  | Arm switch|
| CH5+    | AUX2+     | Additional switches| Available |

---

## 5. Building and Running

### Build

```bash
cd ~/evarobot_ws
source /opt/ros/jazzy/setup.bash
colcon build --packages-select evarobot_rc
source install/setup.bash
```

### Run

```bash
ros2 launch evarobot_rc rc_teleop.launch.py
```

Or with custom parameters:

```bash
ros2 launch evarobot_rc rc_teleop.launch.py \
    serial_port:=/dev/ttyAMA0 \
    baud_rate:=420000 \
    max_linear_vel:=0.5 \
    max_angular_vel:=1.5
```

---

## 6. Verification

### Check RC channel values

```bash
ros2 topic echo /rc/channels
# Expected: 16 values, center ~992, switches at 191 or 1811
```

### Check velocity commands

```bash
ros2 topic echo /cmd_vel
# Expected: zero when sticks centered or arm switch off
# Move right stick: linear.x and angular.z change
```

### Check link statistics

```bash
ros2 topic echo /rc/link_stats
# Expected: RSSI, link quality, SNR, TX power
```

### Expected channel values (CRSF 11-bit)

| Position     | Value |
|--------------|-------|
| Stick min    | 172   |
| Stick center | 992   |
| Stick max    | 1811  |
| Switch pos 1 | 191   |
| Switch pos 2 | 1811  |

---

## 7. Key Lessons Learned

These are critical findings discovered during development and testing:

### 7.1 Pi 4B PL011 termios2 BOTHER Warmup Requirement

**The single most important discovery.** On the Raspberry Pi 4B, the PL011 UART
driver does not correctly apply a custom baud rate via `termios2` BOTHER unless
the UART is first initialized with a standard baud rate using regular `termios`.

**Symptom:** `ioctl(TCSETS2)` succeeds but the actual baud rate is wrong. Data
appears garbled — recognizable patterns but with CRC failures.

**Solution (two-step initialization):**

1. First, configure the UART with standard termios at a nearby standard rate
   (B460800):
   ```c
   struct termios tty;
   tcgetattr(fd, &tty);
   cfmakeraw(&tty);
   cfsetispeed(&tty, B460800);
   cfsetospeed(&tty, B460800);
   tcsetattr(fd, TCSANOW, &tty);
   ```

2. Then, apply the exact custom baud rate via termios2 BOTHER:
   ```c
   struct termios2 tty2;
   ioctl(fd, TCGETS2, &tty2);
   tty2.c_cflag &= ~CBAUD;
   tty2.c_cflag |= BOTHER;
   tty2.c_ispeed = 420000;
   tty2.c_ospeed = 420000;
   ioctl(fd, TCSETS2, &tty2);
   ```

Without step 1, BOTHER silently fails. At 460800 alone (~10% baud error from
420000), frames are partially recognizable but CRC checks fail. The two-step
approach gives perfect decoding.

See: `evarobot_rc/src/serial_port.cpp`

### 7.2 termios2 Header Conflicts on aarch64

Including `<linux/termios.h>` for the `termios2` struct causes redefinition
errors with `<sys/ioctl.h>` (`struct winsize`, `struct termio`). The solution
is to manually define the `termios2` struct and ioctl constants:

```cpp
#define BOTHER  0010000
#define TCGETS2 0x802C542A  // aarch64 value
#define TCSETS2 0x402C542B  // aarch64 value

struct termios2 {
  tcflag_t c_iflag, c_oflag, c_cflag, c_lflag;
  cc_t c_line;
  cc_t c_cc[19];
  speed_t c_ispeed, c_ospeed;
};
```

> **Note:** These ioctl values are architecture-specific. The values above are
> for aarch64 (ARM64). x86_64 uses different values.

### 7.3 Serial Console Interference

If `/boot/firmware/cmdline.txt` contains `console=serial0,115200` or the
`serial-getty@ttyAMA0` service is running, the Pi's login shell will consume
the incoming CRSF bytes and attempt to interpret them as terminal input. This
causes the Pi to become unresponsive or behave erratically when the receiver
is connected.

### 7.4 Receiver Power Supply

Powering the ELRS receiver from the Pi's GPIO 5V pin can draw enough current
to cause the Pi to brownout and reboot. Always use an independent 5V power
supply for the receiver, sharing only the ground connection with the Pi.

### 7.5 CRSF vs Inverted CRSF

The "Inverted CRSF" protocol option in the ELRS receiver settings is intended
for flight controllers with hardware-inverted UARTs (common on F4-based FCs).
The Raspberry Pi PL011 uses standard (non-inverted) UART logic, so always use
plain "CRSF". The Pi 4B `uart0` overlay does not support software signal
inversion (`rxinv`/`txinv` are not valid parameters).

### 7.6 Transmitter vs Receiver Baud Rate Confusion

The Jumper T-Pro V2 (EdgeTX) shows "Baudrate: 400K" — this is the serial link
between the transmitter's MCU and its internal ELRS module. The ELRS receiver's
UART baud (shown in its web UI as 420000) is the rate for the serial output to
the connected device (Pi). These are independent settings. The Pi must match the
**receiver's** UART baud: 420000.

---

## 8. Troubleshooting

### No data received (`/dev/ttyAMA0` exists but no bytes)

1. Check wiring: receiver TX pad must go to Pi GPIO 15 (pin 10, RXD)
2. Verify receiver is bound (solid LED) and transmitter is on
3. Confirm CRSF protocol (not Inverted CRSF) in receiver settings
4. Check `serial-getty` is disabled: `systemctl status serial-getty@ttyAMA0`

### Garbled data / CRC failures

1. Verify two-step baud rate initialization is in use (see section 7.1)
2. Confirm receiver UART baud is 420000 via ELRS web UI
3. Check for loose wiring connections

### Pi reboots when receiver is connected

1. Do NOT power receiver from Pi GPIO pins
2. Use separate 5V supply with shared ground

### `/dev/ttyAMA0` does not exist

1. Check `dtoverlay=disable-bt` is under `[all]` in `/boot/firmware/config.txt`
2. Reboot after changes
3. Verify with `ls -la /dev/ttyAMA0`

### Permission denied on `/dev/ttyAMA0`

1. Check udev rule: `cat /etc/udev/rules.d/99-ttyAMA0.rules`
2. Verify user is in dialout group: `groups $USER`
3. Log out and back in after adding to group

### Diagnostic scripts

Two Python scripts are included for low-level debugging:

- **`test_baud.py`**: Scans multiple baud rates and counts CRSF sync bytes
  (0xC8) at each rate. The correct rate will have the most sync bytes.

- **`test_decode.py`**: Captures raw UART data at 420000 baud for 2 seconds
  and attempts to find/decode valid CRSF frames with CRC validation.

Run with:
```bash
sudo python3 test_baud.py
sudo python3 test_decode.py
```
