#!/usr/bin/env python3
"""
Test script for EvaRobot PID control and encoder feedback
Author: Kevin Medrano Ayala
Contact: kevin.ejem18@gmail.com
License: BSD-3-Clause

Usage:
    python3 test_pid_feedback.py /dev/ttyACM0

Features:
    - Test encoder feedback (position and velocity)
    - Test PID control with velocity setpoints
    - Switch between manual and automatic modes
    - Monitor real-time encoder data
"""

import serial
import json
import time
import sys
import argparse
from datetime import datetime


class PIDTester:
    def __init__(self, port='/dev/ttyACM0', baudrate=115200):
        """Initialize serial connection to Arduino"""
        self.port = port
        self.baudrate = baudrate
        self.ser = None
        self.encoder_data = {'left': 0, 'right': 0}
        self.velocity_data = {'left': 0.0, 'right': 0.0}

    def connect(self):
        """Connect to Arduino"""
        try:
            self.ser = serial.Serial(self.port, self.baudrate, timeout=0.1)
            time.sleep(2)  # Wait for Arduino to reset
            print(f"✓ Connected to {self.port} at {self.baudrate} baud")

            # Read startup message
            time.sleep(0.5)
            while self.ser.in_waiting:
                msg = self.ser.readline().decode('utf-8').strip()
                print(f"Arduino says: {msg}")
            return True
        except serial.SerialException as e:
            print(f"✗ Failed to connect: {e}")
            return False

    def send_command(self, cmd_dict):
        """Send JSON command to Arduino"""
        if not self.ser or not self.ser.is_open:
            print("✗ Serial port not open")
            return False

        # Send command
        cmd_str = json.dumps(cmd_dict) + '\n'
        print(f"→ Sending: {cmd_str.strip()}")
        self.ser.write(cmd_str.encode('utf-8'))

        return True

    def read_feedback(self, timeout=1.0):
        """Read encoder feedback from Arduino"""
        start_time = time.time()
        feedback_list = []

        while time.time() - start_time < timeout:
            if self.ser.in_waiting:
                try:
                    line = self.ser.readline().decode('utf-8').strip()
                    if line:
                        # Try to parse as JSON
                        try:
                            data = json.loads(line)

                            # Check if this is encoder feedback
                            if 'encoders' in data and 'velocities' in data:
                                self.encoder_data = data['encoders']
                                self.velocity_data = data['velocities']
                                feedback_list.append(data)
                            else:
                                # Other message (ack, status, etc.)
                                print(f"← {line}")
                        except json.JSONDecodeError:
                            # Not JSON, just print it
                            print(f"← {line}")
                except Exception as e:
                    print(f"Error reading: {e}")
            time.sleep(0.01)

        return feedback_list

    def set_mode(self, mode):
        """Set control mode (manual or automatic)"""
        print(f"\n🔄 Switching to {mode.upper()} mode")
        cmd = {"mode": mode}
        self.send_command(cmd)
        time.sleep(0.2)
        self.read_feedback(timeout=0.5)

    def set_pid_velocity(self, left_vel, right_vel):
        """Set PID velocity setpoints in rad/s"""
        print(f"\n🎯 Setting velocity setpoints - Left: {left_vel} rad/s, Right: {right_vel} rad/s")
        cmd = {
            "velocitySetpoints": {
                "left": left_vel,
                "right": right_vel
            }
        }
        self.send_command(cmd)

    def send_manual_command(self, status, left_speed=60, right_speed=60):
        """Send manual motor command"""
        print(f"\n🎮 Manual command: {status}, L:{left_speed}%, R:{right_speed}%")
        cmd = {
            "motorStatus": status,
            "motorVelocity": {
                "left": left_speed,
                "right": right_speed
            }
        }
        self.send_command(cmd)

    def monitor_feedback(self, duration=10):
        """Monitor encoder feedback for specified duration"""
        print(f"\n📊 Monitoring encoder feedback for {duration} seconds...")
        print("=" * 80)
        print(f"{'Time':<12} {'Left Enc':<12} {'Right Enc':<12} {'Left Vel (rad/s)':<18} {'Right Vel (rad/s)':<18}")
        print("=" * 80)

        start_time = time.time()
        last_print_time = 0

        while time.time() - start_time < duration:
            feedback_list = self.read_feedback(timeout=0.1)

            # Print feedback at reasonable rate
            current_time = time.time() - start_time
            if feedback_list and (current_time - last_print_time >= 0.2):
                timestamp = datetime.now().strftime('%H:%M:%S.%f')[:-3]
                print(f"{timestamp:<12} {self.encoder_data['left']:<12} "
                      f"{self.encoder_data['right']:<12} "
                      f"{self.velocity_data['left']:<18.3f} "
                      f"{self.velocity_data['right']:<18.3f}")
                last_print_time = current_time

        print("=" * 80)

    def test_encoder_feedback(self):
        """Test encoder feedback in manual mode"""
        print("\n" + "=" * 80)
        print("TEST 1: Encoder Feedback (Manual Mode)")
        print("=" * 80)

        # Ensure manual mode
        self.set_mode("manual")
        time.sleep(0.5)

        # Move forward
        self.send_manual_command("forward", 70, 70)
        self.monitor_feedback(duration=5)

        # Stop
        self.send_manual_command("stop", 38, 38)
        time.sleep(1)

    def test_pid_control(self):
        """Test PID control in automatic mode"""
        print("\n" + "=" * 80)
        print("TEST 2: PID Control (Automatic Mode)")
        print("=" * 80)

        # Switch to automatic mode
        self.set_mode("automatic")
        time.sleep(0.5)

        # Test different velocity setpoints
        test_velocities = [
            (1.0, 1.0),   # Both motors 1 rad/s
            (2.0, 2.0),   # Both motors 2 rad/s
            (1.5, 1.0),   # Different velocities
            (0.0, 0.0),   # Stop
        ]

        for left_vel, right_vel in test_velocities:
            print(f"\n--- Testing velocity: L={left_vel} rad/s, R={right_vel} rad/s ---")
            self.set_pid_velocity(left_vel, right_vel)
            self.monitor_feedback(duration=5)
            time.sleep(1)

    def test_mode_switching(self):
        """Test switching between manual and automatic modes"""
        print("\n" + "=" * 80)
        print("TEST 3: Mode Switching")
        print("=" * 80)

        # Start in manual
        self.set_mode("manual")
        self.send_manual_command("forward", 60, 60)
        print("Running in MANUAL mode...")
        self.monitor_feedback(duration=3)

        # Switch to automatic
        self.set_mode("automatic")
        self.set_pid_velocity(1.5, 1.5)
        print("Running in AUTOMATIC mode...")
        self.monitor_feedback(duration=3)

        # Switch back to manual
        self.set_mode("manual")
        self.send_manual_command("stop", 38, 38)
        print("Back to MANUAL mode - stopped")
        time.sleep(1)

    def test_velocity_ramp(self):
        """Test gradual velocity ramp from min to max"""
        print("\n" + "=" * 80)
        print("TEST 4: Velocity Ramp (Visual Speed Increase)")
        print("=" * 80)
        print("\nThis test will gradually increase motor speed from low to high,")
        print("allowing you to visually observe the velocity control.\n")

        # Switch to automatic mode
        self.set_mode("automatic")
        time.sleep(0.5)

        # Define velocity steps (rad/s)
        # Starting from realistic minimum for this hardware (~10 rad/s)
        velocity_steps = [
            (10.0, 10.0),   # Low speed
            (15.0, 15.0),   #
            (20.0, 20.0),   # Medium-low
            (25.0, 25.0),   #
            (30.0, 30.0),   # Medium
            (35.0, 35.0),   #
            (40.0, 40.0),   # Medium-high
            (45.0, 45.0),   #
            (50.0, 50.0),   # High speed
            (55.0, 55.0),   # Maximum
        ]

        print("🚀 RAMPING UP: Increasing velocity gradually...")
        print("=" * 80)

        for left_vel, right_vel in velocity_steps:
            print(f"\n→ Target: {left_vel} rad/s")
            self.set_pid_velocity(left_vel, right_vel)

            # Monitor for enough time to see the change
            time.sleep(0.5)  # Brief pause for command to take effect

            # Show real-time feedback for 3 seconds at this velocity
            start_time = time.time()
            while time.time() - start_time < 3.0:
                feedback_list = self.read_feedback(timeout=0.1)
                if feedback_list:
                    # Print single line update (overwrite previous)
                    print(f"  Encoders: L={self.encoder_data['left']:6d}, R={self.encoder_data['right']:6d} | "
                          f"Velocity: L={self.velocity_data['left']:5.1f}, R={self.velocity_data['right']:5.1f} rad/s",
                          end='\r', flush=True)
            print()  # New line after this velocity step

        print("\n" + "=" * 80)
        print("🛑 RAMPING DOWN: Decreasing velocity gradually...")
        print("=" * 80)

        # Ramp down
        for left_vel, right_vel in reversed(velocity_steps):
            print(f"\n→ Target: {left_vel} rad/s")
            self.set_pid_velocity(left_vel, right_vel)

            time.sleep(0.5)

            start_time = time.time()
            while time.time() - start_time < 3.0:
                feedback_list = self.read_feedback(timeout=0.1)
                if feedback_list:
                    print(f"  Encoders: L={self.encoder_data['left']:6d}, R={self.encoder_data['right']:6d} | "
                          f"Velocity: L={self.velocity_data['left']:5.1f}, R={self.velocity_data['right']:5.1f} rad/s",
                          end='\r', flush=True)
            print()

        # Stop
        print("\n→ Target: 0.0 rad/s (STOP)")
        self.set_pid_velocity(0.0, 0.0)
        time.sleep(2)

        print("\n" + "=" * 80)
        print("✓ Velocity ramp test completed!")
        print("=" * 80)

    def interactive_pid_mode(self):
        """Interactive PID testing mode"""
        print("\n" + "=" * 80)
        print("INTERACTIVE PID MODE")
        print("=" * 80)
        print("\nCommands:")
        print("  a - Switch to AUTOMATIC mode")
        print("  m - Switch to MANUAL mode")
        print("  s <left> <right> - Set PID velocity setpoints (rad/s)")
        print("  f <speed> - Forward (manual)")
        print("  x - Stop")
        print("  q - Quit")
        print("\nExample: 's 1.5 1.5' sets both wheels to 1.5 rad/s")

        try:
            while True:
                # Read and display feedback
                feedback_list = self.read_feedback(timeout=0.1)
                if feedback_list:
                    print(f"Encoders: L={self.encoder_data['left']}, R={self.encoder_data['right']} | "
                          f"Vel: L={self.velocity_data['left']:.3f}, R={self.velocity_data['right']:.3f} rad/s")

                # Check for user input (non-blocking)
                import select
                if select.select([sys.stdin], [], [], 0)[0]:
                    cmd = input("\n> ").strip().lower()

                    if cmd == 'q':
                        break
                    elif cmd == 'a':
                        self.set_mode("automatic")
                    elif cmd == 'm':
                        self.set_mode("manual")
                    elif cmd.startswith('s '):
                        parts = cmd.split()
                        if len(parts) == 3:
                            try:
                                left = float(parts[1])
                                right = float(parts[2])
                                self.set_pid_velocity(left, right)
                            except ValueError:
                                print("Invalid velocity values")
                    elif cmd.startswith('f '):
                        parts = cmd.split()
                        if len(parts) == 2:
                            try:
                                speed = int(parts[1])
                                self.send_manual_command("forward", speed, speed)
                            except ValueError:
                                print("Invalid speed value")
                    elif cmd == 'x':
                        self.set_mode("manual")
                        self.send_manual_command("stop", 38, 38)

        except KeyboardInterrupt:
            print("\n\nInterrupted by user")

    def stop(self):
        """Stop all motors"""
        print("\n🛑 STOPPING MOTORS")
        self.set_mode("manual")
        time.sleep(0.2)
        self.send_manual_command("stop", 38, 38)
        time.sleep(0.5)

    def close(self):
        """Close serial connection"""
        if self.ser and self.ser.is_open:
            self.stop()
            time.sleep(0.5)
            self.ser.close()
            print("\n✓ Serial port closed")


def main():
    parser = argparse.ArgumentParser(description='Test EvaRobot PID control and encoder feedback')
    parser.add_argument('port', nargs='?', default='/dev/ttyACM0',
                       help='Serial port (default: /dev/ttyACM0)')
    parser.add_argument('-b', '--baudrate', type=int, default=115200,
                       help='Baud rate (default: 115200)')
    parser.add_argument('-t', '--test',
                       choices=['encoder', 'pid', 'switch', 'ramp', 'interactive', 'all'],
                       default='interactive',
                       help='Test mode')

    args = parser.parse_args()

    print("=" * 80)
    print("EvaRobot PID Control & Encoder Feedback Test")
    print("=" * 80)

    tester = PIDTester(args.port, args.baudrate)

    if not tester.connect():
        sys.exit(1)

    try:
        if args.test == 'encoder':
            tester.test_encoder_feedback()
        elif args.test == 'pid':
            tester.test_pid_control()
        elif args.test == 'switch':
            tester.test_mode_switching()
        elif args.test == 'ramp':
            tester.test_velocity_ramp()
        elif args.test == 'all':
            tester.test_encoder_feedback()
            tester.test_pid_control()
            tester.test_mode_switching()
            tester.test_velocity_ramp()
        else:
            tester.interactive_pid_mode()
    except KeyboardInterrupt:
        print("\n\n⚠️  Interrupted by user")
    finally:
        tester.close()


if __name__ == '__main__':
    main()
