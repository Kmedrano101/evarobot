/**
 * @file evarobot_motors.h
 * @brief Motor control functions for EvaRobot H-bridge driver
 *
 * @author Kevin Medrano Ayala
 * @contact kevin.ejem18@gmail.com
 * @license BSD-3-Clause
 *
 * @description
 * Controls two DC motors using an H-bridge driver.
 * Motor directions are set via digital pins (IN1-IN4).
 * Motor speeds are controlled separately via PWM (see evarobot_pwm.h).
 */

#ifndef EVAROBOT_MOTORS_H
#define EVAROBOT_MOTORS_H

#include <Arduino.h>
#include "evarobot_config.h"

// ============================================================================
// MOTOR DIRECTION ENUM
// ============================================================================
enum MotorDirection {
  MOTOR_STOP = 0,
  MOTOR_FORWARD = 1,
  MOTOR_BACKWARD = 2
};

// ============================================================================
// FUNCTION PROTOTYPES
// ============================================================================

/**
 * @brief Initialize motor control pins
 *
 * Sets all H-bridge control pins as outputs and stops motors
 */
void setupMotors();

/**
 * @brief Stop all motors immediately
 *
 * Sets all H-bridge pins to LOW
 */
void stopMotors();

/**
 * @brief Set direction for both motors
 *
 * @param leftForward True for forward, false for backward
 * @param rightForward True for forward, false for backward
 */
void setMotorDirections(bool leftForward, bool rightForward);

/**
 * @brief Set direction for a single motor
 *
 * @param motor Motor index (MOTOR_LEFT or MOTOR_RIGHT)
 * @param direction MotorDirection enum value
 */
void setMotorDirection(uint8_t motor, MotorDirection direction);

/**
 * @brief Execute movement command
 *
 * @param command Movement command string
 * Supported commands: forward, backward, turnleft, turnright, stop
 */
void handleMovement(const String& command);

// ============================================================================
// IMPLEMENTATION
// ============================================================================

void setupMotors()
{
  // Configure all motor control pins as outputs
  pinMode(MOTOR_LEFT_IN1, OUTPUT);
  pinMode(MOTOR_LEFT_IN2, OUTPUT);
  pinMode(MOTOR_RIGHT_IN1, OUTPUT);
  pinMode(MOTOR_RIGHT_IN2, OUTPUT);

  // Initialize in stopped state
  stopMotors();

  DEBUG_PRINTLN("Motors initialized");
}

void stopMotors()
{
  digitalWrite(MOTOR_LEFT_IN1, LOW);
  digitalWrite(MOTOR_LEFT_IN2, LOW);
  digitalWrite(MOTOR_RIGHT_IN1, LOW);
  digitalWrite(MOTOR_RIGHT_IN2, LOW);
}

void setMotorDirection(uint8_t motor, MotorDirection direction)
{
  uint8_t pin1, pin2;

  // Select appropriate pins based on motor
  if (motor == MOTOR_LEFT) {
    pin1 = MOTOR_LEFT_IN1;
    pin2 = MOTOR_LEFT_IN2;
  } else {
    pin1 = MOTOR_RIGHT_IN1;
    pin2 = MOTOR_RIGHT_IN2;
  }

  // Set direction
  switch (direction) {
    case MOTOR_FORWARD:
      digitalWrite(pin1, LOW);
      digitalWrite(pin2, HIGH);
      break;

    case MOTOR_BACKWARD:
      digitalWrite(pin1, HIGH);
      digitalWrite(pin2, LOW);
      break;

    case MOTOR_STOP:
    default:
      digitalWrite(pin1, LOW);
      digitalWrite(pin2, LOW);
      break;
  }
}

void setMotorDirections(bool leftForward, bool rightForward)
{
  // Left motor direction
  digitalWrite(MOTOR_LEFT_IN1, leftForward ? LOW : HIGH);
  digitalWrite(MOTOR_LEFT_IN2, leftForward ? HIGH : LOW);

  // Right motor direction
  digitalWrite(MOTOR_RIGHT_IN1, rightForward ? LOW : HIGH);
  digitalWrite(MOTOR_RIGHT_IN2, rightForward ? HIGH : LOW);
}

void handleMovement(const String& command)
{
  // Parse movement command and set motor directions
  if (command == CMD_FORWARD) {
    setMotorDirections(true, true);
    DEBUG_PRINTLN("Moving forward");
  }
  else if (command == CMD_BACKWARD) {
    setMotorDirections(false, false);
    DEBUG_PRINTLN("Moving backward");
  }
  else if (command == CMD_TURN_LEFT) {
    // Both wheels forward - speed difference creates the turn
    // Left wheel: minimum speed, Right wheel: higher speed
    setMotorDirections(true, true);
    DEBUG_PRINTLN("Turning left");
  }
  else if (command == CMD_TURN_RIGHT) {
    // Both wheels forward - speed difference creates the turn
    // Left wheel: higher speed, Right wheel: minimum speed
    setMotorDirections(true, true);
    DEBUG_PRINTLN("Turning right");
  }
  else if (command == CMD_FORWARD_LEFT) {
    // TODO: Implement differential speeds for smooth curves
    setMotorDirections(true, true);
    DEBUG_PRINTLN("Forward left");
  }
  else if (command == CMD_FORWARD_RIGHT) {
    // TODO: Implement differential speeds for smooth curves
    setMotorDirections(true, true);
    DEBUG_PRINTLN("Forward right");
  }
  else if (command == CMD_BACKWARD_LEFT) {
    // TODO: Implement differential speeds for smooth curves
    setMotorDirections(false, false);
    DEBUG_PRINTLN("Backward left");
  }
  else if (command == CMD_BACKWARD_RIGHT) {
    // TODO: Implement differential speeds for smooth curves
    setMotorDirections(false, false);
    DEBUG_PRINTLN("Backward right");
  }
  else {
    stopMotors();
    DEBUG_PRINTLN("Motors stopped");
  }
}

#endif // EVAROBOT_MOTORS_H
