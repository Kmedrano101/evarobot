/**
 * @file evarobot_control.ino
 * @brief Main control firmware for EvaRobot mobile platform
 *
 * @author Kevin Medrano Ayala
 * @contact kevin.ejem18@gmail.com
 * @license BSD-3-Clause
 * @date 2025
 *
 * @description
 * This firmware controls a differential drive mobile robot with:
 * - Two DC motors with PWM speed control
 * - H-bridge motor driver
 * - Quadrature encoders for odometry
 * - JSON-based serial communication protocol
 *
 * Hardware: Arduino Nano RP2040
 */

// ============================================================================
// INCLUDES
// ============================================================================
#include "evarobot_config.h"       // Configuration constants
#include "evarobot_encoders.h"     // Encoder reading (no dependencies)
#include "evarobot_pid.h"          // PID control (depends on encoders)
#include "evarobot_pwm.h"          // PWM control
#include "evarobot_motors.h"       // Motor control
#include "evarobot_serial_com.h"   // Serial communication (depends on encoders & PID)

// ============================================================================
// SETUP - Runs once at startup
// ============================================================================
void setup()
{
  // Initialize hardware subsystems
  setupSerialComm();    // Must be first for debug output
  setupMotors();        // Initialize motor control pins
  setupMotorsPWM();     // Initialize PWM for speed control
  setupEncoders();      // Initialize encoder interrupts
  setupPID();           // Initialize PID controllers

  // Send ready signal
  Serial.println("{\"status\":\"ready\",\"mode\":\"manual\"}");
}

// ============================================================================
// LOOP - Main control loop
// ============================================================================
void loop()
{
  // Process incoming serial commands
  processIncomingData();

  // Read encoder values
  readEncoders();

  // Check control mode and update motors accordingly
  if (getControlMode() == MODE_AUTOMATIC) {
    // Automatic mode: PID control
    updateVelocityMeasurements();

    if (computePID()) {
      // Check if setpoints are zero - stop motors completely
      if (abs(g_leftMotorSetpoint) < 0.01 && abs(g_rightMotorSetpoint) < 0.01) {
        stopMotors();
        setMotorVelocity(MOTOR_LEFT, 0.0);
        setMotorVelocity(MOTOR_RIGHT, 0.0);
      } else {
        // Set motor directions based on setpoint signs
        setMotorDirection(MOTOR_LEFT, g_leftMotorSetpoint >= 0 ? MOTOR_FORWARD : MOTOR_BACKWARD);
        setMotorDirection(MOTOR_RIGHT, g_rightMotorSetpoint >= 0 ? MOTOR_FORWARD : MOTOR_BACKWARD);

        // Apply PID outputs to motors (absolute values)
        setMotorVelocity(MOTOR_LEFT, abs(g_leftMotorOutput));
        setMotorVelocity(MOTOR_RIGHT, abs(g_rightMotorOutput));
      }
    }
  } else {
    // Manual mode: Direct PWM control
    updateMotorSpeeds();
    executeMovement();
  }

  // Publish encoder feedback
  publishEncoderFeedback();
}

// ============================================================================
// HELPER FUNCTIONS
// ============================================================================

/**
 * @brief Update motor PWM speeds (manual mode)
 */
void updateMotorSpeeds()
{
  setMotorVelocity(MOTOR_LEFT, g_motorVelocityLeft);
  setMotorVelocity(MOTOR_RIGHT, g_motorVelocityRight);
}

/**
 * @brief Execute current movement command (manual mode)
 */
void executeMovement()
{
  handleMovement(g_motorStatus);
}
