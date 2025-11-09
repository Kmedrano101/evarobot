/**
 * Motor Test Sketch for EvaRobot
 *
 * Simple Arduino sketch to test motor functionality without ROS2 communication.
 * Tests both motors independently and together at different speeds.
 *
 * Author: Kevin Medrano Ayala
 * Contact: kevin.ejem18@gmail.com
 * License: BSD-3-Clause
 *
 * HARDWARE SETUP:
 * - Left Motor:  IN1=Pin 9,  IN2=Pin 10
 * - Right Motor: IN1=Pin 11, IN2=Pin 12
 * - Motor driver should be powered externally
 * - Arduino should share ground with motor driver
 *
 * USAGE:
 * 1. Upload this sketch to Arduino
 * 2. Open Serial Monitor (115200 baud)
 * 3. Watch the test sequence
 * 4. Verify motors move as expected
 *
 * SERIAL COMMANDS (send via Serial Monitor):
 * - 'w' = Forward
 * - 's' = Backward
 * - 'a' = Turn Left
 * - 'd' = Turn Right
 * - 'x' = Stop
 * - 't' = Run full test sequence
 * - 'l' = Test left motor only
 * - 'r' = Test right motor only
 * - '+' = Increase speed
 * - '-' = Decrease speed
 */

// ============================================================================
// MOTOR CONTROL PINS (H-Bridge)
// ============================================================================
#define MOTOR_LEFT_IN1 9
#define MOTOR_LEFT_IN2 10
#define MOTOR_RIGHT_IN1 11
#define MOTOR_RIGHT_IN2 12

// ============================================================================
// CONFIGURATION
// ============================================================================
#define SERIAL_BAUD 115200
#define PWM_MIN 38      // Minimum PWM to overcome motor friction
#define PWM_MAX 255     // Maximum PWM (full speed)
#define TEST_DURATION 2000  // Duration for each test (milliseconds)

// ============================================================================
// GLOBAL VARIABLES
// ============================================================================
int currentSpeed = 60;  // Default speed (0-255)

// ============================================================================
// MOTOR CONTROL FUNCTIONS
// ============================================================================

void setupMotors() {
  // Configure motor pins as outputs
  pinMode(MOTOR_LEFT_IN1, OUTPUT);
  pinMode(MOTOR_LEFT_IN2, OUTPUT);
  pinMode(MOTOR_RIGHT_IN1, OUTPUT);
  pinMode(MOTOR_RIGHT_IN2, OUTPUT);

  // Initialize all motors stopped
  stopMotors();

  Serial.println("✓ Motor pins configured");
}

void stopMotors() {
  analogWrite(MOTOR_LEFT_IN1, 0);
  analogWrite(MOTOR_LEFT_IN2, 0);
  analogWrite(MOTOR_RIGHT_IN1, 0);
  analogWrite(MOTOR_RIGHT_IN2, 0);
  Serial.println("STOP - All motors off");
}

void setLeftMotor(int speed) {
  /**
   * Control left motor
   * speed: -255 to +255 (negative = backward, positive = forward)
   */
  if (speed > 0) {
    // Forward
    analogWrite(MOTOR_LEFT_IN1, speed);
    analogWrite(MOTOR_LEFT_IN2, 0);
  } else if (speed < 0) {
    // Backward
    analogWrite(MOTOR_LEFT_IN1, 0);
    analogWrite(MOTOR_LEFT_IN2, -speed);
  } else {
    // Stop
    analogWrite(MOTOR_LEFT_IN1, 0);
    analogWrite(MOTOR_LEFT_IN2, 0);
  }
}

void setRightMotor(int speed) {
  /**
   * Control right motor
   * speed: -255 to +255 (negative = backward, positive = forward)
   */
  if (speed > 0) {
    // Forward
    analogWrite(MOTOR_RIGHT_IN1, speed);
    analogWrite(MOTOR_RIGHT_IN2, 0);
  } else if (speed < 0) {
    // Backward
    analogWrite(MOTOR_RIGHT_IN1, 0);
    analogWrite(MOTOR_RIGHT_IN2, -speed);
  } else {
    // Stop
    analogWrite(MOTOR_RIGHT_IN1, 0);
    analogWrite(MOTOR_RIGHT_IN2, 0);
  }
}

void moveForward(int speed) {
  Serial.print("FORWARD - Speed: ");
  Serial.println(speed);
  setLeftMotor(speed);
  setRightMotor(speed);
}

void moveBackward(int speed) {
  Serial.print("BACKWARD - Speed: ");
  Serial.println(speed);
  setLeftMotor(-speed);
  setRightMotor(-speed);
}

void turnLeft(int speed) {
  Serial.print("TURN LEFT - Speed: ");
  Serial.println(speed);
  // Left motor off, right motor forward
  setLeftMotor(0);
  setRightMotor(speed);
}

void turnRight(int speed) {
  Serial.print("TURN RIGHT - Speed: ");
  Serial.println(speed);
  // Right motor off, left motor forward
  setLeftMotor(speed);
  setRightMotor(0);
}

void spinLeft(int speed) {
  Serial.print("SPIN LEFT - Speed: ");
  Serial.println(speed);
  // Left motor backward, right motor forward
  setLeftMotor(-speed);
  setRightMotor(speed);
}

void spinRight(int speed) {
  Serial.print("SPIN RIGHT - Speed: ");
  Serial.println(speed);
  // Left motor forward, right motor backward
  setLeftMotor(speed);
  setRightMotor(-speed);
}

// ============================================================================
// TEST SEQUENCES
// ============================================================================

void testLeftMotorOnly() {
  Serial.println("\n========================================");
  Serial.println("TEST: Left Motor Only");
  Serial.println("========================================");

  Serial.println("\n[1/2] Left motor forward...");
  setLeftMotor(currentSpeed);
  delay(TEST_DURATION);

  stopMotors();
  delay(1000);

  Serial.println("\n[2/2] Left motor backward...");
  setLeftMotor(-currentSpeed);
  delay(TEST_DURATION);

  stopMotors();
  Serial.println("✓ Left motor test completed\n");
}

void testRightMotorOnly() {
  Serial.println("\n========================================");
  Serial.println("TEST: Right Motor Only");
  Serial.println("========================================");

  Serial.println("\n[1/2] Right motor forward...");
  setRightMotor(currentSpeed);
  delay(TEST_DURATION);

  stopMotors();
  delay(1000);

  Serial.println("\n[2/2] Right motor backward...");
  setRightMotor(-currentSpeed);
  delay(TEST_DURATION);

  stopMotors();
  Serial.println("✓ Right motor test completed\n");
}

void testBasicMovements() {
  Serial.println("\n========================================");
  Serial.println("TEST: Basic Movements");
  Serial.println("========================================");
  Serial.print("Speed: ");
  Serial.println(currentSpeed);

  // Test 1: Forward
  Serial.println("\n[1/6] Moving forward...");
  moveForward(currentSpeed);
  delay(TEST_DURATION);
  stopMotors();
  delay(1000);

  // Test 2: Backward
  Serial.println("\n[2/6] Moving backward...");
  moveBackward(currentSpeed);
  delay(TEST_DURATION);
  stopMotors();
  delay(1000);

  // Test 3: Turn Left
  Serial.println("\n[3/6] Turning left...");
  turnLeft(currentSpeed);
  delay(TEST_DURATION);
  stopMotors();
  delay(1000);

  // Test 4: Turn Right
  Serial.println("\n[4/6] Turning right...");
  turnRight(currentSpeed);
  delay(TEST_DURATION);
  stopMotors();
  delay(1000);

  // Test 5: Spin Left
  Serial.println("\n[5/6] Spinning left...");
  spinLeft(currentSpeed);
  delay(TEST_DURATION);
  stopMotors();
  delay(1000);

  // Test 6: Spin Right
  Serial.println("\n[6/6] Spinning right...");
  spinRight(currentSpeed);
  delay(TEST_DURATION);
  stopMotors();

  Serial.println("\n✓ Basic movement test completed\n");
}

void testSpeedRange() {
  Serial.println("\n========================================");
  Serial.println("TEST: Speed Range (Forward)");
  Serial.println("========================================");

  int speeds[] = {PWM_MIN, 60, 80, 100, 120, 150, 180, 200, PWM_MAX};
  int numSpeeds = sizeof(speeds) / sizeof(speeds[0]);

  for (int i = 0; i < numSpeeds; i++) {
    Serial.print("\nTesting speed: ");
    Serial.print(speeds[i]);
    Serial.println("/255");

    moveForward(speeds[i]);
    delay(1500);
    stopMotors();
    delay(800);
  }

  Serial.println("\n✓ Speed range test completed\n");
}

void runFullTestSequence() {
  Serial.println("\n╔════════════════════════════════════════╗");
  Serial.println("║   EVAROBOT MOTOR TEST SEQUENCE         ║");
  Serial.println("╚════════════════════════════════════════╝");

  delay(2000);  // Give user time to prepare

  // Test individual motors
  testLeftMotorOnly();
  delay(2000);

  testRightMotorOnly();
  delay(2000);

  // Test basic movements
  testBasicMovements();
  delay(2000);

  // Test speed range
  testSpeedRange();

  Serial.println("\n╔════════════════════════════════════════╗");
  Serial.println("║   ALL TESTS COMPLETED!                 ║");
  Serial.println("╚════════════════════════════════════════╝");
}

// ============================================================================
// SERIAL COMMAND PROCESSING
// ============================================================================

void processSerialCommand(char cmd) {
  switch (cmd) {
    case 'w':
      moveForward(currentSpeed);
      break;

    case 's':
      moveBackward(currentSpeed);
      break;

    case 'a':
      turnLeft(currentSpeed);
      break;

    case 'd':
      turnRight(currentSpeed);
      break;

    case 'q':
      spinLeft(currentSpeed);
      break;

    case 'e':
      spinRight(currentSpeed);
      break;

    case 'x':
      stopMotors();
      break;

    case 't':
      runFullTestSequence();
      break;

    case 'l':
      testLeftMotorOnly();
      break;

    case 'r':
      testRightMotorOnly();
      break;

    case 'b':
      testBasicMovements();
      break;

    case 'v':
      testSpeedRange();
      break;

    case '+':
      currentSpeed = min(PWM_MAX, currentSpeed + 10);
      Serial.print("Speed increased to: ");
      Serial.println(currentSpeed);
      break;

    case '-':
      currentSpeed = max(PWM_MIN, currentSpeed - 10);
      Serial.print("Speed decreased to: ");
      Serial.println(currentSpeed);
      break;

    case 'h':
    case '?':
      printHelp();
      break;

    default:
      Serial.print("Unknown command: ");
      Serial.println(cmd);
      Serial.println("Type 'h' for help");
      break;
  }
}

void printHelp() {
  Serial.println("\n========================================");
  Serial.println("EVAROBOT MOTOR TEST - HELP");
  Serial.println("========================================");
  Serial.println("\nMovement Commands:");
  Serial.println("  w - Move Forward");
  Serial.println("  s - Move Backward");
  Serial.println("  a - Turn Left (right motor only)");
  Serial.println("  d - Turn Right (left motor only)");
  Serial.println("  q - Spin Left (counter-rotate)");
  Serial.println("  e - Spin Right (counter-rotate)");
  Serial.println("  x - Stop all motors");
  Serial.println("\nTest Commands:");
  Serial.println("  t - Run full test sequence");
  Serial.println("  l - Test left motor only");
  Serial.println("  r - Test right motor only");
  Serial.println("  b - Test basic movements");
  Serial.println("  v - Test speed range");
  Serial.println("\nSpeed Control:");
  Serial.println("  + - Increase speed");
  Serial.println("  - - Decrease speed");
  Serial.println("\nOther:");
  Serial.println("  h - Show this help");
  Serial.print("\nCurrent Speed: ");
  Serial.println(currentSpeed);
  Serial.println("========================================\n");
}

// ============================================================================
// ARDUINO SETUP AND LOOP
// ============================================================================

void setup() {
  // Initialize serial communication
  Serial.begin(SERIAL_BAUD);
  delay(100);

  // Print startup message
  Serial.println("\n╔════════════════════════════════════════╗");
  Serial.println("║   EVAROBOT MOTOR TEST                  ║");
  Serial.println("║   Author: Kevin Medrano Ayala         ║");
  Serial.println("╚════════════════════════════════════════╝");
  Serial.println();
  Serial.print("Serial Baud Rate: ");
  Serial.println(SERIAL_BAUD);

  // Setup motors
  setupMotors();

  // Print pin configuration
  Serial.println("\nMotor Pin Configuration:");
  Serial.print("  Left Motor:  IN1=Pin ");
  Serial.print(MOTOR_LEFT_IN1);
  Serial.print(", IN2=Pin ");
  Serial.println(MOTOR_LEFT_IN2);
  Serial.print("  Right Motor: IN1=Pin ");
  Serial.print(MOTOR_RIGHT_IN1);
  Serial.print(", IN2=Pin ");
  Serial.println(MOTOR_RIGHT_IN2);

  Serial.print("\nPWM Range: ");
  Serial.print(PWM_MIN);
  Serial.print(" - ");
  Serial.println(PWM_MAX);
  Serial.print("Default Speed: ");
  Serial.println(currentSpeed);

  Serial.println("\n✓ Ready! Type 'h' for help or 't' to run full test");
  Serial.println("========================================\n");
}

void loop() {
  // Check for serial commands
  if (Serial.available() > 0) {
    char cmd = Serial.read();

    // Ignore newline and carriage return
    if (cmd != '\n' && cmd != '\r') {
      processSerialCommand(cmd);
    }
  }

  // Small delay to prevent overwhelming the serial port
  delay(10);
}
