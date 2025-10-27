#!/usr/bin/env python3
"""
Test script for EvaRobot motor control via serial port
Author: Kevin Medrano Ayala
Contact: kevin.ejem18@gmail.com
License: BSD-3-Clause

Usage:
    python3 test_motor_serial.py /dev/ttyACM0
    python3 test_motor_serial.py /dev/ttyACM0 --encoders  # Show encoder feedback
"""

import serial
import json
import time
import sys
import argparse
import threading


class MotorTester:
    def __init__(self, port='/dev/ttyACM0', baudrate=115200, show_encoders=False):
        """Initialize serial connection to Arduino"""
        self.port = port
        self.baudrate = baudrate
        self.ser = None
        self.show_encoders = show_encoders
        self.encoder_data = {
            'left_ticks': 0,
            'right_ticks': 0,
            'left_velocity': 0.0,
            'right_velocity': 0.0,
            'timestamp': time.time()
        }
        self.running = False
        self.feedback_thread = None

    def connect(self):
        """Connect to Arduino"""
        try:
            self.ser = serial.Serial(self.port, self.baudrate, timeout=1)
            time.sleep(2)  # Wait for Arduino to reset
            print(f"✓ Connected to {self.port} at {self.baudrate} baud")

            # Read startup message
            if self.ser.in_waiting:
                msg = self.ser.readline().decode('utf-8').strip()
                print(f"Arduino says: {msg}")

            # Start encoder feedback thread if enabled
            if self.show_encoders:
                self.running = True
                self.feedback_thread = threading.Thread(target=self._read_feedback_loop, daemon=True)
                self.feedback_thread.start()
                print("✓ Encoder feedback monitoring started")

            return True
        except serial.SerialException as e:
            print(f"✗ Failed to connect: {e}")
            return False

    def _read_feedback_loop(self):
        """Background thread to continuously read encoder feedback"""
        while self.running:
            try:
                if self.ser and self.ser.is_open and self.ser.in_waiting:
                    line = self.ser.readline().decode('utf-8', errors='ignore').strip()

                    if not line:
                        continue

                    # Try to parse as JSON
                    try:
                        data = json.loads(line)

                        # Check if it contains encoder data
                        if 'encoders' in data and 'velocities' in data:
                            self.encoder_data['left_ticks'] = data['encoders']['left']
                            self.encoder_data['right_ticks'] = data['encoders']['right']
                            self.encoder_data['left_velocity'] = data['velocities']['left']
                            self.encoder_data['right_velocity'] = data['velocities']['right']
                            self.encoder_data['timestamp'] = time.time()
                    except json.JSONDecodeError:
                        # Not JSON, might be debug message
                        pass

            except Exception as e:
                if self.running:
                    print(f"\n⚠️  Feedback read error: {e}")

            time.sleep(0.01)  # 100 Hz polling rate

    def get_encoder_string(self):
        """Get formatted encoder feedback string"""
        age = time.time() - self.encoder_data['timestamp']
        if age > 2.0:
            return "⚠️  No recent encoder data"

        return (f"Encoders: L={self.encoder_data['left_ticks']:6d} ticks, "
                f"R={self.encoder_data['right_ticks']:6d} ticks | "
                f"Velocities: L={self.encoder_data['left_velocity']:+6.2f} rad/s, "
                f"R={self.encoder_data['right_velocity']:+6.2f} rad/s")

    def send_command(self, status, left_speed=60, right_speed=60):
        """Send motor command to Arduino"""
        if not self.ser or not self.ser.is_open:
            print("✗ Serial port not open")
            return False

        # Build JSON command
        cmd = {
            "motorStatus": status,
            "motorVelocity": {
                "left": left_speed,
                "right": right_speed
            }
        }

        # Send command
        cmd_str = json.dumps(cmd) + '\n'
        print(f"→ Sending: {cmd_str.strip()}")
        self.ser.write(cmd_str.encode('utf-8'))

        # Show encoder feedback if enabled
        if self.show_encoders:
            time.sleep(0.2)  # Wait a bit for encoder data
            print(f"📊 {self.get_encoder_string()}")

        return True

    def stop(self):
        """Stop all motors"""
        print("\n🛑 STOPPING MOTORS")
        return self.send_command("stop", 38, 38)

    def forward(self, speed=60):
        """Move forward"""
        print(f"\n⬆️  FORWARD at speed {speed}")
        return self.send_command("forward", speed, speed)

    def backward(self, speed=60):
        """Move backward"""
        print(f"\n⬇️  BACKWARD at speed {speed}")
        return self.send_command("backward", speed, speed)

    def turn_left(self, speed=60):
        """Turn left - Left wheel OFF, Right wheel ON"""
        print(f"\n↶  TURN LEFT - Left wheel OFF, Right wheel at speed {speed}")
        return self.send_command("turnleft", 0, speed)

    def turn_right(self, speed=60):
        """Turn right - Right wheel OFF, Left wheel ON"""
        print(f"\n↷  TURN RIGHT - Left wheel at speed {speed}, Right wheel OFF")
        return self.send_command("turnright", speed, 0)

    def run_basic_test(self):
        """Run basic movement test"""
        print("\n" + "="*50)
        print("BASIC MOVEMENT TEST")
        print("="*50)

        duration = 2  # seconds per movement

        # Forward
        self.forward(60)
        time.sleep(duration)

        # Stop
        self.stop()
        time.sleep(1)

        # Backward
        self.backward(60)
        time.sleep(duration)

        # Stop
        self.stop()
        time.sleep(1)

        # Turn left
        self.turn_left(60)
        time.sleep(duration)

        # Stop
        self.stop()
        time.sleep(1)

        # Turn right
        self.turn_right(60)
        time.sleep(duration)

        # Stop
        self.stop()

        print("\n✓ Basic test completed!")

    def run_speed_test(self):
        """Test different speeds"""
        print("\n" + "="*50)
        print("SPEED TEST (Forward)")
        print("="*50)

        speeds = [38, 50, 60, 70, 80, 90, 95]

        for speed in speeds:
            print(f"\nTesting speed: {speed}%")
            self.forward(speed)
            time.sleep(2)
            self.stop()
            time.sleep(1)

        print("\n✓ Speed test completed!")

    def monitor_encoders(self, duration=10):
        """Monitor encoder feedback in real-time"""
        if not self.show_encoders:
            print("✗ Encoder feedback not enabled. Use --encoders flag.")
            return

        print("\n" + "="*50)
        print("ENCODER MONITOR MODE")
        print("="*50)
        print(f"Monitoring for {duration} seconds (Ctrl+C to stop early)")
        print("\n")

        start_time = time.time()
        try:
            while (time.time() - start_time) < duration:
                # Clear line and print encoder data
                print(f"\r📊 {self.get_encoder_string()}", end='', flush=True)
                time.sleep(0.1)
        except KeyboardInterrupt:
            pass

        print("\n\n✓ Encoder monitoring completed!")

    def interactive_mode(self):
        """Interactive control mode"""
        print("\n" + "="*50)
        print("INTERACTIVE MODE")
        print("="*50)
        print("\nCommands:")
        print("  w - Forward")
        print("  s - Backward")
        print("  a - Turn Left")
        print("  d - Turn Right")
        print("  x - Stop")
        print("  q - Quit")
        print("  + - Increase speed")
        print("  - - Decrease speed")
        if self.show_encoders:
            print("  e - Show encoder values")
        print("\nCurrent speed: 60%")

        speed = 60

        try:
            import sys, tty, termios

            # Get terminal settings
            fd = sys.stdin.fileno()
            old_settings = termios.tcgetattr(fd)

            try:
                tty.setraw(sys.stdin.fileno())

                while True:
                    char = sys.stdin.read(1)

                    if char == 'q':
                        break
                    elif char == 'w':
                        self.forward(speed)
                    elif char == 's':
                        self.backward(speed)
                    elif char == 'a':
                        self.turn_left(speed)
                    elif char == 'd':
                        self.turn_right(speed)
                    elif char == 'x':
                        self.stop()
                    elif char == 'e' and self.show_encoders:
                        print(f"\n📊 {self.get_encoder_string()}")
                    elif char == '+':
                        speed = min(95, speed + 5)
                        print(f"\nSpeed: {speed}%")
                    elif char == '-':
                        speed = max(38, speed - 5)
                        print(f"\nSpeed: {speed}%")

            finally:
                termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)

        except ImportError:
            print("✗ Interactive mode requires termios (Linux/Mac only)")
            print("Use simple interactive mode instead:")

            while True:
                prompt = "\nEnter command (w/a/s/d/x"
                if self.show_encoders:
                    prompt += "/e"
                prompt += "/+/-/q): "
                cmd = input(prompt).strip().lower()

                if cmd == 'q':
                    break
                elif cmd == 'w':
                    self.forward(speed)
                elif cmd == 's':
                    self.backward(speed)
                elif cmd == 'a':
                    self.turn_left(speed)
                elif cmd == 'd':
                    self.turn_right(speed)
                elif cmd == 'x':
                    self.stop()
                elif cmd == 'e' and self.show_encoders:
                    print(f"📊 {self.get_encoder_string()}")
                elif cmd == '+':
                    speed = min(95, speed + 5)
                    print(f"Speed: {speed}%")
                elif cmd == '-':
                    speed = max(38, speed - 5)
                    print(f"Speed: {speed}%")

        self.stop()

    def close(self):
        """Close serial connection"""
        # Stop feedback thread
        if self.feedback_thread:
            self.running = False
            time.sleep(0.2)

        # Stop motors and close port
        if self.ser and self.ser.is_open:
            self.stop()
            time.sleep(0.5)
            self.ser.close()
            print("\n✓ Serial port closed")


def main():
    parser = argparse.ArgumentParser(
        description='Test EvaRobot motors via serial',
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Examples:
  %(prog)s /dev/ttyACM0                    # Interactive mode
  %(prog)s /dev/ttyACM0 --encoders         # Interactive with encoder feedback
  %(prog)s /dev/ttyACM0 -t monitor -e      # Monitor encoders only
  %(prog)s /dev/ttyACM0 -t basic -e        # Basic test with encoder feedback
        """
    )
    parser.add_argument('port', nargs='?', default='/dev/ttyACM0',
                       help='Serial port (default: /dev/ttyACM0)')
    parser.add_argument('-b', '--baudrate', type=int, default=115200,
                       help='Baud rate (default: 115200)')
    parser.add_argument('-t', '--test', choices=['basic', 'speed', 'interactive', 'monitor'],
                       default='interactive', help='Test mode (default: interactive)')
    parser.add_argument('-e', '--encoders', action='store_true',
                       help='Enable encoder feedback display')
    parser.add_argument('-d', '--duration', type=int, default=10,
                       help='Duration for monitor mode in seconds (default: 10)')

    args = parser.parse_args()

    print("="*50)
    print("EvaRobot Motor Serial Test")
    print("="*50)

    tester = MotorTester(args.port, args.baudrate, show_encoders=args.encoders)

    if not tester.connect():
        sys.exit(1)

    try:
        if args.test == 'basic':
            tester.run_basic_test()
        elif args.test == 'speed':
            tester.run_speed_test()
        elif args.test == 'monitor':
            tester.monitor_encoders(args.duration)
        else:
            tester.interactive_mode()
    except KeyboardInterrupt:
        print("\n\n⚠️  Interrupted by user")
    finally:
        tester.close()


if __name__ == '__main__':
    main()
