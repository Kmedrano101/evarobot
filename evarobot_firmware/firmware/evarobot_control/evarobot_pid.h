/**
 * @file evarobot_pid.h
 * @brief PID controller for automatic velocity control
 *
 * @author Kevin Medrano Ayala
 * @contact kevin.ejem18@gmail.com
 * @license BSD-3-Clause
 *
 * @description
 * Implements PID control for closed-loop velocity tracking of both motors.
 * Based on the PID_v1 library architecture with custom tuning for EvaRobot.
 */

#ifndef EVAROBOT_PID_H
#define EVAROBOT_PID_H

#include <Arduino.h>
#include <PID_v1.h>
#include "evarobot_config.h"

// ============================================================================
// PID CONTROL VARIABLES
// ============================================================================

/**
 * @brief Control mode enumeration
 */
enum ControlMode {
  MODE_MANUAL = 0,      // Direct PWM control (current implementation)
  MODE_AUTOMATIC = 1    // PID-based velocity control
};

/**
 * @brief Current control mode
 */
ControlMode g_controlMode = MODE_MANUAL;

// Left Motor PID Variables
double g_leftMotorSetpoint = 0.0;      // Desired velocity (rad/s)
double g_leftMotorInput = 0.0;         // Measured velocity (rad/s)
double g_leftMotorOutput = 0.0;        // PID output (PWM duty cycle)

// Right Motor PID Variables
double g_rightMotorSetpoint = 0.0;     // Desired velocity (rad/s)
double g_rightMotorInput = 0.0;        // Measured velocity (rad/s)
double g_rightMotorOutput = 0.0;       // PID output (PWM duty cycle)

// PID Controller Instances
PID* g_pidLeft = nullptr;
PID* g_pidRight = nullptr;

// Velocity measurement timing
unsigned long g_lastVelocityUpdate = 0;
long g_lastLeftPosition = 0;
long g_lastRightPosition = 0;

// ============================================================================
// FUNCTION PROTOTYPES
// ============================================================================

/**
 * @brief Initialize PID controllers
 */
void setupPID();

/**
 * @brief Update velocity measurements from encoders
 *
 * Calculates velocity in rad/s based on encoder tick changes
 * Should be called at regular intervals (VELOCITY_UPDATE_INTERVAL)
 */
void updateVelocityMeasurements();

/**
 * @brief Compute PID outputs
 *
 * @return true if PID was computed, false otherwise
 */
bool computePID();

/**
 * @brief Set PID setpoints for both motors
 *
 * @param leftVelocity Desired left motor velocity (rad/s)
 * @param rightVelocity Desired right motor velocity (rad/s)
 */
void setPIDSetpoints(double leftVelocity, double rightVelocity);

/**
 * @brief Set control mode
 *
 * @param mode MODE_MANUAL or MODE_AUTOMATIC
 */
void setControlMode(ControlMode mode);

/**
 * @brief Get current control mode
 *
 * @return Current control mode
 */
ControlMode getControlMode();

/**
 * @brief Update PID tuning parameters
 *
 * @param motor MOTOR_LEFT or MOTOR_RIGHT
 * @param kp Proportional gain
 * @param ki Integral gain
 * @param kd Derivative gain
 */
void updatePIDTuning(uint8_t motor, double kp, double ki, double kd);

/**
 * @brief Get current velocity measurements
 *
 * @param leftVel Output: left motor velocity (rad/s)
 * @param rightVel Output: right motor velocity (rad/s)
 */
void getVelocities(double& leftVel, double& rightVel);

/**
 * @brief Reset PID controllers
 */
void resetPID();

// ============================================================================
// IMPLEMENTATION
// ============================================================================

void setupPID()
{
  // Create PID controller instances
  g_pidLeft = new PID(
    &g_leftMotorInput,
    &g_leftMotorOutput,
    &g_leftMotorSetpoint,
    PID_KP_LEFT,
    PID_KI_LEFT,
    PID_KD_LEFT,
    DIRECT
  );

  g_pidRight = new PID(
    &g_rightMotorInput,
    &g_rightMotorOutput,
    &g_rightMotorSetpoint,
    PID_KP_RIGHT,
    PID_KI_RIGHT,
    PID_KD_RIGHT,
    DIRECT
  );

  // Configure PID output limits (PWM duty cycle range)
  g_pidLeft->SetOutputLimits(PWM_MIN_DUTY_CYCLE, PWM_MAX_DUTY_CYCLE);
  g_pidRight->SetOutputLimits(PWM_MIN_DUTY_CYCLE, PWM_MAX_DUTY_CYCLE);

  // Set sample time (ms)
  g_pidLeft->SetSampleTime(VELOCITY_UPDATE_INTERVAL);
  g_pidRight->SetSampleTime(VELOCITY_UPDATE_INTERVAL);

  // Start in manual mode
  g_pidLeft->SetMode(MANUAL);
  g_pidRight->SetMode(MANUAL);

  // Initialize timing
  g_lastVelocityUpdate = millis();

  DEBUG_PRINTLN(F("PID controllers initialized"));

  #if PID_DEBUG_ENABLED
    DEBUG_PRINT(F("Left PID - Kp: "));
    DEBUG_PRINT(PID_KP_LEFT);
    DEBUG_PRINT(F(", Ki: "));
    DEBUG_PRINT(PID_KI_LEFT);
    DEBUG_PRINT(F(", Kd: "));
    DEBUG_PRINTLN(PID_KD_LEFT);

    DEBUG_PRINT(F("Right PID - Kp: "));
    DEBUG_PRINT(PID_KP_RIGHT);
    DEBUG_PRINT(F(", Ki: "));
    DEBUG_PRINT(PID_KI_RIGHT);
    DEBUG_PRINT(F(", Kd: "));
    DEBUG_PRINTLN(PID_KD_RIGHT);
  #endif
}

void updateVelocityMeasurements()
{
  unsigned long currentTime = millis();
  unsigned long deltaTime = currentTime - g_lastVelocityUpdate;

  // Only update at specified interval
  if (deltaTime < VELOCITY_UPDATE_INTERVAL) {
    return;
  }

  // Get current encoder positions
  long currentLeftPos = g_positionLeft;
  long currentRightPos = g_positionRight;

  // Calculate tick differences
  long leftTicks = currentLeftPos - g_lastLeftPosition;
  long rightTicks = currentRightPos - g_lastRightPosition;

  // Convert to velocity (rad/s)
  // Formula: vel = (ticks / ticks_per_rev) * (2 * PI) / (deltaTime / 1000)
  // Simplified: vel = ticks * (2 * PI * 1000) / (ticks_per_rev * deltaTime)
  float dt_seconds = deltaTime / 1000.0;

  g_leftMotorInput = (leftTicks / (float)ENCODER_TICKS_PER_REV) * TWO_PI / dt_seconds;
  g_rightMotorInput = (rightTicks / (float)ENCODER_TICKS_PER_REV) * TWO_PI / dt_seconds;

  // Store current positions for next calculation
  g_lastLeftPosition = currentLeftPos;
  g_lastRightPosition = currentRightPos;
  g_lastVelocityUpdate = currentTime;

  #if PID_DEBUG_ENABLED >= 2
    DEBUG_PRINT(F("Vel L: "));
    DEBUG_PRINT(g_leftMotorInput, 3);
    DEBUG_PRINT(F(" R: "));
    DEBUG_PRINT(g_rightMotorInput, 3);
    DEBUG_PRINT(F(" rad/s (dt: "));
    DEBUG_PRINT(deltaTime);
    DEBUG_PRINTLN(F(" ms)"));
  #endif
}

bool computePID()
{
  if (g_controlMode != MODE_AUTOMATIC) {
    return false;
  }

  // Update velocity measurements
  updateVelocityMeasurements();

  // Compute PID outputs
  bool leftComputed = g_pidLeft->Compute();
  bool rightComputed = g_pidRight->Compute();

  #if PID_DEBUG_ENABLED >= 2
    if (leftComputed || rightComputed) {
      DEBUG_PRINT(F("PID Out - L: "));
      DEBUG_PRINT(g_leftMotorOutput, 1);
      DEBUG_PRINT(F(" R: "));
      DEBUG_PRINT(g_rightMotorOutput, 1);
      DEBUG_PRINT(F(" (SP L: "));
      DEBUG_PRINT(g_leftMotorSetpoint, 2);
      DEBUG_PRINT(F(" R: "));
      DEBUG_PRINT(g_rightMotorSetpoint, 2);
      DEBUG_PRINTLN(F(")"));
    }
  #endif

  return (leftComputed || rightComputed);
}

void setPIDSetpoints(double leftVelocity, double rightVelocity)
{
  g_leftMotorSetpoint = leftVelocity;
  g_rightMotorSetpoint = rightVelocity;

  #if PID_DEBUG_ENABLED
    DEBUG_PRINT(F("PID Setpoints - L: "));
    DEBUG_PRINT(leftVelocity, 3);
    DEBUG_PRINT(F(" R: "));
    DEBUG_PRINT(rightVelocity, 3);
    DEBUG_PRINTLN(F(" rad/s"));
  #endif
}

void setControlMode(ControlMode mode)
{
  if (g_controlMode == mode) {
    return;  // No change
  }

  g_controlMode = mode;

  if (mode == MODE_AUTOMATIC) {
    // Switch to automatic mode
    g_pidLeft->SetMode(AUTOMATIC);
    g_pidRight->SetMode(AUTOMATIC);

    // Reset velocity measurements
    g_lastVelocityUpdate = millis();
    g_lastLeftPosition = g_positionLeft;
    g_lastRightPosition = g_positionRight;

    DEBUG_PRINTLN(F("Control mode: AUTOMATIC (PID)"));
  } else {
    // Switch to manual mode
    g_pidLeft->SetMode(MANUAL);
    g_pidRight->SetMode(MANUAL);

    // Reset setpoints
    g_leftMotorSetpoint = 0.0;
    g_rightMotorSetpoint = 0.0;

    DEBUG_PRINTLN(F("Control mode: MANUAL (Direct PWM)"));
  }
}

ControlMode getControlMode()
{
  return g_controlMode;
}

void updatePIDTuning(uint8_t motor, double kp, double ki, double kd)
{
  if (motor == MOTOR_LEFT) {
    g_pidLeft->SetTunings(kp, ki, kd);

    #if PID_DEBUG_ENABLED
      DEBUG_PRINT(F("Left PID tuning updated - Kp: "));
      DEBUG_PRINT(kp);
      DEBUG_PRINT(F(", Ki: "));
      DEBUG_PRINT(ki);
      DEBUG_PRINT(F(", Kd: "));
      DEBUG_PRINTLN(kd);
    #endif
  } else if (motor == MOTOR_RIGHT) {
    g_pidRight->SetTunings(kp, ki, kd);

    #if PID_DEBUG_ENABLED
      DEBUG_PRINT(F("Right PID tuning updated - Kp: "));
      DEBUG_PRINT(kp);
      DEBUG_PRINT(F(", Ki: "));
      DEBUG_PRINT(ki);
      DEBUG_PRINT(F(", Kd: "));
      DEBUG_PRINTLN(kd);
    #endif
  }
}

void getVelocities(double& leftVel, double& rightVel)
{
  leftVel = g_leftMotorInput;
  rightVel = g_rightMotorInput;
}

void resetPID()
{
  // Reset setpoints
  g_leftMotorSetpoint = 0.0;
  g_rightMotorSetpoint = 0.0;

  // Reset inputs
  g_leftMotorInput = 0.0;
  g_rightMotorInput = 0.0;

  // Reset outputs
  g_leftMotorOutput = 0.0;
  g_rightMotorOutput = 0.0;

  // Reset timing
  g_lastVelocityUpdate = millis();
  g_lastLeftPosition = g_positionLeft;
  g_lastRightPosition = g_positionRight;

  DEBUG_PRINTLN(F("PID reset"));
}

#endif // EVAROBOT_PID_H
