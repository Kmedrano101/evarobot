# EvaRobot Motor Test Guide

This guide explains how to test your EvaRobot motors to diagnose issues before running ROS2 control.

## Files

- **test_motors.ino** - Arduino sketch for direct motor testing
- **test_serial_com.py** - Python script to test serial communication
- **test_motor_serial.py** - Full motor control test via serial

## Quick Start - Arduino Direct Test

### Step 1: Upload Arduino Sketch

1. Open Arduino IDE
2. Open `test_motors.ino`
3. Select your Arduino board (Tools → Board)
4. Select the correct port (Tools → Port → /dev/ttyACM0)
5. Click Upload

### Step 2: Open Serial Monitor

1. Tools → Serial Monitor
2. Set baud rate to **115200**
3. You should see the startup message

### Step 3: Run Tests

**Quick test:** Type `t` and press Enter to run full automated test sequence

**Manual control:**
- `w` - Forward
- `s` - Backward
- `a` - Turn Left
- `d` - Turn Right
- `q` - Spin Left
- `e` - Spin Right
- `x` - Stop
- `+` - Increase speed
- `-` - Decrease speed
- `h` - Help

**Individual tests:**
- `l` - Test left motor only
- `r` - Test right motor only
- `b` - Test basic movements
- `v` - Test speed range

## Troubleshooting Motor Issues

### Motors Don't Move

**Check:**
1. ✓ Motor driver is powered (external power, not USB)
2. ✓ Battery/power supply voltage is sufficient (7.4V - 12V typical)
3. ✓ Motor driver ground is connected to Arduino ground
4. ✓ Motor wires are connected correctly
5. ✓ H-bridge enable pin is HIGH (if applicable)

### Only One Motor Works

**Test:**
1. Run `l` to test left motor only
2. Run `r` to test right motor only
3. Compare which motor responds

**If left motor doesn't work:**
- Check connections on pins 9 and 10
- Check left motor driver channel
- Swap motor connections to verify motor is good

**If right motor doesn't work:**
- Check connections on pins 11 and 12
- Check right motor driver channel
- Swap motor connections to verify motor is good

### Motors Work in Test but Not in ROS2

This indicates the motors are fine, but communication is the issue.

**Next steps:**
1. Run serial communication test:
   ```bash
   python3 test_serial_com.py /dev/ttyACM0 --verbose
   ```

2. Verify control mode matches:
   - Arduino firmware control mode (manual vs automatic)
   - ROS2 launch parameter control mode
   - Default is 'manual' for both

3. Check if Arduino is receiving commands:
   - Add `Serial.println()` in Arduino firmware command handler
   - Monitor serial output while running ROS2

### Motors Jitter or Don't Start

**Cause:** PWM too low

**Solution:**
- Increase `PWM_MIN` value (currently 38)
- Test with higher speeds: type `+` several times
- Typical range: 30-50 for minimum PWM

### Motors Run Too Fast

**Solution:**
- Decrease `PWM_MAX` value (currently 255)
- Reduce speed: type `-` several times
- Or modify speed in test: `currentSpeed = 50;`

## Pin Configuration

```
Left Motor H-Bridge:
  IN1 = Pin 9  (PWM)
  IN2 = Pin 10 (PWM)

Right Motor H-Bridge:
  IN1 = Pin 11 (PWM)
  IN2 = Pin 12 (PWM)
```

## Motor Control Logic

**Forward:**
- Left: IN1=PWM, IN2=0
- Right: IN1=PWM, IN2=0

**Backward:**
- Left: IN1=0, IN2=PWM
- Right: IN1=0, IN2=PWM

**Turn Left (Right motor only):**
- Left: IN1=0, IN2=0
- Right: IN1=PWM, IN2=0

**Turn Right (Left motor only):**
- Left: IN1=PWM, IN2=0
- Right: IN1=0, IN2=0

**Stop:**
- Both: IN1=0, IN2=0

## Expected Test Sequence

When you type `t`, the test will run:

1. **Left Motor Test** (forward 2s, backward 2s)
2. **Right Motor Test** (forward 2s, backward 2s)
3. **Basic Movements:**
   - Forward 2s
   - Backward 2s
   - Turn Left 2s
   - Turn Right 2s
   - Spin Left 2s
   - Spin Right 2s
4. **Speed Range Test:** Tests PWM values from 38 to 255

**Total test time:** ~3 minutes

## Serial Communication Test

After verifying motors work with the Arduino test:

```bash
cd /home/kmedrano/evarobot_ws/src/evarobot_firmware/tests

# Test manual mode (same as ROS2 default)
python3 test_serial_com.py /dev/ttyACM0 --verbose

# Test automatic mode (PID control)
python3 test_serial_com.py /dev/ttyACM0 --mode automatic --verbose

# Simulate ROS2 bridge behavior
python3 test_serial_com.py /dev/ttyACM0 --mode ros2 --verbose
```

This tests the serial JSON protocol used by ROS2.

## Advanced Motor Test (with Encoders)

```bash
python3 test_motor_serial.py /dev/ttyACM0 --encoders -t basic
```

This tests motors AND verifies encoder feedback.

## Common Issues Summary

| Issue | Likely Cause | Solution |
|-------|-------------|----------|
| No motors move | Power issue | Check battery, motor driver power |
| One motor missing | Wiring/pin issue | Check pins, swap to test |
| Works in test, not ROS2 | Communication | Run test_serial_com.py |
| Motors jitter | PWM too low | Increase PWM_MIN |
| Encoders not working | Wrong pins/wiring | Check encoder connections |

## Next Steps

1. ✓ Upload and run `test_motors.ino`
2. ✓ Verify both motors work in all directions
3. ✓ Run `test_serial_com.py` to verify JSON protocol
4. ✓ Check ROS2 control mode matches Arduino firmware
5. ✓ Monitor serial output during ROS2 operation

---

**Author:** Kevin Medrano Ayala
**License:** BSD-3-Clause
