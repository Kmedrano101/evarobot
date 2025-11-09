#!/usr/bin/env python3
"""
Serial Communication Test for EvaRobot Arduino Bridge
Tests the communication protocol between ROS2 bridge and Arduino

Author: Kevin Medrano Ayala
Contact: kevin.ejem18@gmail.com
License: BSD-3-Clause

Usage:
    python3 test_serial_com.py /dev/ttyACM0
    python3 test_serial_com.py /dev/ttyACM0 --mode automatic
    python3 test_serial_com.py /dev/ttyACM0 --verbose
"""

import serial
import json
import time
import sys
import argparse
import threading


class SerialComTester:
    """Test serial communication with Arduino"""

    def __init__(self, port='/dev/ttyACM0', baudrate=115200, verbose=False):
        self.port = port
        self.baudrate = baudrate
        self.verbose = verbose
        self.ser = None
        self.running = False
        self.feedback_thread = None
        self.responses = []

    def connect(self):
        """Connect to Arduino with same logic as ROS2 bridge"""
        max_retries = 3
        retry_delay = 1.0

        for attempt in range(max_retries):
            try:
                print(f"[{attempt + 1}/{max_retries}] Connecting to {self.port} at {self.baudrate} baud...")

                # Close any existing connection
                if self.ser and self.ser.is_open:
                    self.ser.close()
                    time.sleep(0.5)

                # Open serial connection
                self.ser = serial.Serial(
                    port=self.port,
                    baudrate=self.baudrate,
                    timeout=0.1
                )

                print(f"✓ Port opened, waiting for Arduino to reset...")

                # Wait for Arduino to reset after serial connection
                time.sleep(2.5)

                # Flush any stale data
                self.ser.reset_input_buffer()
                self.ser.reset_output_buffer()

                # Read startup message
                if self.ser.in_waiting:
                    msg = self.ser.readline().decode('utf-8', errors='ignore').strip()
                    print(f"Arduino says: {msg}")

                print(f"✓ Successfully connected to Arduino on {self.port}")

                # Start feedback thread
                self.running = True
                self.feedback_thread = threading.Thread(target=self._read_feedback_loop, daemon=True)
                self.feedback_thread.start()

                return True

            except serial.SerialException as e:
                print(f"✗ Connection attempt {attempt + 1} failed: {e}")
                self.ser = None

                if attempt < max_retries - 1:
                    print(f"Retrying in {retry_delay} seconds...")
                    time.sleep(retry_delay)
                else:
                    print(f"✗ Failed to connect after {max_retries} attempts")
                    return False

        return False

    def _read_feedback_loop(self):
        """Background thread to read Arduino responses"""
        while self.running:
            try:
                if self.ser and self.ser.is_open and self.ser.in_waiting:
                    line = self.ser.readline().decode('utf-8', errors='ignore').strip()

                    if not line:
                        continue

                    # Store response
                    self.responses.append(line)

                    # Print if verbose
                    if self.verbose:
                        print(f"← {line}")

                    # Try to parse as JSON
                    try:
                        data = json.loads(line)

                        if 'encoders' in data and 'velocities' in data:
                            if self.verbose:
                                print(f"📊 Encoders: L={data['encoders']['left']}, R={data['encoders']['right']} | "
                                      f"Velocities: L={data['velocities']['left']:.2f}, R={data['velocities']['right']:.2f} rad/s")
                    except json.JSONDecodeError:
                        pass

            except Exception as e:
                if self.running and self.verbose:
                    print(f"⚠️  Read error: {e}")

            time.sleep(0.01)

    def send_command(self, cmd_dict, wait_response=0.5):
        """Send JSON command to Arduino"""
        if not self.ser or not self.ser.is_open:
            print("✗ Serial port not open")
            return False

        try:
            cmd_str = json.dumps(cmd_dict) + '\n'
            print(f"→ Sending: {cmd_str.strip()}")
            self.ser.write(cmd_str.encode('utf-8'))

            if wait_response > 0:
                time.sleep(wait_response)

            return True
        except Exception as e:
            print(f"✗ Send error: {e}")
            return False

    def test_mode_switch(self, mode):
        """Test switching control mode"""
        print(f"\n{'='*60}")
        print(f"TEST: Switch to {mode.upper()} mode")
        print('='*60)

        cmd = {"mode": mode}
        self.send_command(cmd, wait_response=1.0)

        print(f"✓ Mode command sent")

    def test_manual_commands(self):
        """Test manual mode commands (PWM-based)"""
        print(f"\n{'='*60}")
        print("TEST: Manual Mode Commands")
        print('='*60)

        duration = 2.0

        # Test 1: Forward
        print("\n[1/4] Forward (PWM=60)")
        cmd = {
            "motorStatus": "forward",
            "motorVelocity": {"left": 60, "right": 60}
        }
        self.send_command(cmd)
        time.sleep(duration)

        # Stop
        self._send_stop()
        time.sleep(1)

        # Test 2: Backward
        print("\n[2/4] Backward (PWM=60)")
        cmd = {
            "motorStatus": "backward",
            "motorVelocity": {"left": 60, "right": 60}
        }
        self.send_command(cmd)
        time.sleep(duration)

        # Stop
        self._send_stop()
        time.sleep(1)

        # Test 3: Turn Left
        print("\n[3/4] Turn Left (PWM=60)")
        cmd = {
            "motorStatus": "turnleft",
            "motorVelocity": {"left": 0, "right": 60}
        }
        self.send_command(cmd)
        time.sleep(duration)

        # Stop
        self._send_stop()
        time.sleep(1)

        # Test 4: Turn Right
        print("\n[4/4] Turn Right (PWM=60)")
        cmd = {
            "motorStatus": "turnright",
            "motorVelocity": {"left": 60, "right": 0}
        }
        self.send_command(cmd)
        time.sleep(duration)

        # Stop
        self._send_stop()

        print("\n✓ Manual mode tests completed")

    def test_automatic_commands(self):
        """Test automatic mode commands (velocity setpoints)"""
        print(f"\n{'='*60}")
        print("TEST: Automatic Mode Commands (PID Velocity Control)")
        print('='*60)

        duration = 2.0

        # Test 1: Forward (same velocity both wheels)
        print("\n[1/4] Forward (2.0 rad/s)")
        cmd = {
            "velocitySetpoints": {
                "left": 2.0,
                "right": 2.0
            }
        }
        self.send_command(cmd)
        time.sleep(duration)

        # Stop
        self._send_stop_automatic()
        time.sleep(1)

        # Test 2: Backward
        print("\n[2/4] Backward (-2.0 rad/s)")
        cmd = {
            "velocitySetpoints": {
                "left": -2.0,
                "right": -2.0
            }
        }
        self.send_command(cmd)
        time.sleep(duration)

        # Stop
        self._send_stop_automatic()
        time.sleep(1)

        # Test 3: Turn Left (differential)
        print("\n[3/4] Turn Left (L=1.0, R=3.0 rad/s)")
        cmd = {
            "velocitySetpoints": {
                "left": 1.0,
                "right": 3.0
            }
        }
        self.send_command(cmd)
        time.sleep(duration)

        # Stop
        self._send_stop_automatic()
        time.sleep(1)

        # Test 4: Turn Right (differential)
        print("\n[4/4] Turn Right (L=3.0, R=1.0 rad/s)")
        cmd = {
            "velocitySetpoints": {
                "left": 3.0,
                "right": 1.0
            }
        }
        self.send_command(cmd)
        time.sleep(duration)

        # Stop
        self._send_stop_automatic()

        print("\n✓ Automatic mode tests completed")

    def _send_stop(self):
        """Send stop command (manual mode)"""
        print("→ STOP")
        cmd = {
            "motorStatus": "stop",
            "motorVelocity": {"left": 38, "right": 38}
        }
        self.send_command(cmd, wait_response=0.5)

    def _send_stop_automatic(self):
        """Send stop command (automatic mode)"""
        print("→ STOP")
        cmd = {
            "velocitySetpoints": {
                "left": 0.0,
                "right": 0.0
            }
        }
        self.send_command(cmd, wait_response=0.5)

    def test_ros2_bridge_simulation(self):
        """Simulate ROS2 bridge behavior"""
        print(f"\n{'='*60}")
        print("TEST: Simulate ROS2 Bridge Behavior")
        print('='*60)

        print("\nSimulating ROS2 joy_teleop launch sequence...")

        # Step 1: Set control mode (from launch param)
        print("\n[Step 1] Set control mode to MANUAL")
        self.test_mode_switch("manual")

        # Step 2: Simulate joystick input -> cmd_vel -> bridge
        print("\n[Step 2] Simulate joystick forward input")
        print("  (Joystick: Right stick pushed up)")
        print("  (cmd_vel: linear.x=0.5, angular.z=0.0)")
        print("  (Bridge converts to: Forward PWM=60)")

        cmd = {
            "motorStatus": "forward",
            "motorVelocity": {"left": 60, "right": 60}
        }
        self.send_command(cmd)
        time.sleep(2)

        self._send_stop()

        print("\n✓ ROS2 bridge simulation completed")

    def run_all_tests(self, mode='manual'):
        """Run complete test suite"""
        print("\n" + "="*60)
        print("SERIAL COMMUNICATION TEST SUITE")
        print("="*60)
        print(f"Port: {self.port}")
        print(f"Baudrate: {self.baudrate}")
        print(f"Test Mode: {mode}")
        print("="*60)

        if mode == 'manual':
            self.test_mode_switch('manual')
            time.sleep(1)
            self.test_manual_commands()
        elif mode == 'automatic':
            self.test_mode_switch('automatic')
            time.sleep(1)
            self.test_automatic_commands()
        elif mode == 'both':
            # Test manual
            self.test_mode_switch('manual')
            time.sleep(1)
            self.test_manual_commands()
            time.sleep(2)

            # Test automatic
            self.test_mode_switch('automatic')
            time.sleep(1)
            self.test_automatic_commands()
        elif mode == 'ros2':
            self.test_ros2_bridge_simulation()

        print(f"\n{'='*60}")
        print("TEST SUMMARY")
        print('='*60)
        print(f"Total responses received: {len(self.responses)}")

        if self.responses:
            print("\nSample responses from Arduino:")
            for resp in self.responses[:10]:
                print(f"  {resp}")
        else:
            print("\n⚠️  No responses received from Arduino!")
            print("Possible issues:")
            print("  1. Arduino firmware not sending feedback")
            print("  2. Baud rate mismatch")
            print("  3. Arduino Serial.println() not working")

        print('='*60)

    def close(self):
        """Close connection"""
        # Stop feedback thread
        if self.feedback_thread:
            self.running = False
            time.sleep(0.2)

        # Stop motors and close
        if self.ser and self.ser.is_open:
            print("\n🛑 Stopping motors...")
            self._send_stop()
            time.sleep(0.5)
            self.ser.close()
            print("✓ Serial port closed")


def main():
    parser = argparse.ArgumentParser(
        description='Test serial communication with EvaRobot Arduino',
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Examples:
  %(prog)s /dev/ttyACM0                        # Test manual mode
  %(prog)s /dev/ttyACM0 --mode automatic       # Test automatic mode
  %(prog)s /dev/ttyACM0 --mode both            # Test both modes
  %(prog)s /dev/ttyACM0 --mode ros2            # Simulate ROS2 bridge
  %(prog)s /dev/ttyACM0 --verbose              # Show all responses
        """
    )
    parser.add_argument('port', nargs='?', default='/dev/ttyACM0',
                       help='Serial port (default: /dev/ttyACM0)')
    parser.add_argument('-b', '--baudrate', type=int, default=115200,
                       help='Baud rate (default: 115200)')
    parser.add_argument('-m', '--mode', choices=['manual', 'automatic', 'both', 'ros2'],
                       default='manual',
                       help='Test mode (default: manual)')
    parser.add_argument('-v', '--verbose', action='store_true',
                       help='Show all Arduino responses in real-time')

    args = parser.parse_args()

    tester = SerialComTester(args.port, args.baudrate, verbose=args.verbose)

    if not tester.connect():
        sys.exit(1)

    try:
        tester.run_all_tests(mode=args.mode)
    except KeyboardInterrupt:
        print("\n\n⚠️  Interrupted by user")
    finally:
        tester.close()


if __name__ == '__main__':
    main()
