/**
 * @file evarobot_serial_com.h
 * @brief Serial communication handler for EvaRobot
 *
 * @author Kevin Medrano Ayala
 * @contact kevin.ejem18@gmail.com
 * @license BSD-3-Clause
 *
 * @description
 * Handles JSON-based serial communication protocol for receiving
 * motor commands from ROS2 via USB serial connection.
 *
 * Protocol Format:
 * {"motorStatus":"forward","motorVelocity":{"left":60,"right":60}}
 */

#ifndef EVAROBOT_SERIAL_COM_H
#define EVAROBOT_SERIAL_COM_H

#include <Arduino.h>
#include <ArduinoJson.h>
#include "evarobot_config.h"
#include "evarobot_encoders.h"
#include "evarobot_pid.h"

// ============================================================================
// GLOBAL VARIABLES (accessed by other modules)
// ============================================================================

/**
 * @brief Current motor movement command
 * Valid values: stop, forward, backward, turnleft, turnright
 */
String g_motorStatus = CMD_STOP;

/**
 * @brief Left motor velocity (PWM duty cycle percentage)
 * Range: PWM_MIN_DUTY_CYCLE to PWM_MAX_DUTY_CYCLE
 */
unsigned int g_motorVelocityLeft = static_cast<unsigned int>(PWM_MIN_DUTY_CYCLE);

/**
 * @brief Right motor velocity (PWM duty cycle percentage)
 * Range: PWM_MIN_DUTY_CYCLE to PWM_MAX_DUTY_CYCLE
 */
unsigned int g_motorVelocityRight = static_cast<unsigned int>(PWM_MIN_DUTY_CYCLE);

// Feedback publishing timing
unsigned long g_lastFeedbackTime = 0;

// ============================================================================
// FUNCTION PROTOTYPES
// ============================================================================

/**
 * @brief Initialize serial communication
 *
 * Starts serial port at configured baud rate and sets timeout
 */
void setupSerialComm();

/**
 * @brief Process incoming serial data
 *
 * Reads and parses JSON commands from serial port.
 * Updates global motor status and velocity variables.
 */
void processIncomingData();

/**
 * @brief Validate motor velocity value
 *
 * @param velocity Raw velocity value from JSON
 * @return Clamped velocity within valid range
 */
unsigned int validateVelocity(int velocity);

/**
 * @brief Publish encoder feedback
 *
 * Sends encoder positions and velocities via serial in JSON format
 */
void publishEncoderFeedback();

// ============================================================================
// IMPLEMENTATION
// ============================================================================

void setupSerialComm()
{
  Serial.begin(SERIAL_BAUD_RATE);

  // Wait for serial port to connect (necessary for native USB boards)
  unsigned long startTime = millis();
  const unsigned long SERIAL_WAIT_TIMEOUT = 3000; // 3 seconds

  while (!Serial && (millis() - startTime < SERIAL_WAIT_TIMEOUT)) {
    // Wait for serial connection or timeout
  }

  Serial.setTimeout(SERIAL_TIMEOUT_MS);

  // Send startup message
  Serial.print(F("{\"firmware\":\"EvaRobot v"));
  Serial.print(FIRMWARE_VERSION);
  Serial.println(F("\",\"status\":\"initialized\"}"));
}

unsigned int validateVelocity(int velocity)
{
  // SPECIAL CASE: Allow 0 for turning (motor completely off)
  if (velocity == 0) {
    return 0;
  }

  // Clamp velocity to valid PWM range
  if (velocity < PWM_MIN_DUTY_CYCLE) {
    return static_cast<unsigned int>(PWM_MIN_DUTY_CYCLE);
  }
  if (velocity > PWM_MAX_DUTY_CYCLE) {
    return static_cast<unsigned int>(PWM_MAX_DUTY_CYCLE);
  }
  return static_cast<unsigned int>(velocity);
}

void processIncomingData()
{
  // Check if data is available
  if (Serial.available() <= 0) {
    return;
  }

  // Read incoming JSON string
  String jsonStr = Serial.readStringUntil('\n');
  jsonStr.trim();

  // Ignore empty strings
  if (jsonStr.length() == 0) {
    return;
  }

  DEBUG_PRINT(F("Received: "));
  DEBUG_PRINTLN(jsonStr);

  // Parse JSON document
  StaticJsonDocument<JSON_BUFFER_SIZE> doc;
  DeserializationError error = deserializeJson(doc, jsonStr);

  if (error) {
    // Send error response
    Serial.print(F("{\"error\":\"JSON parse failed: "));
    Serial.print(error.c_str());
    Serial.println(F("\"}"));
    return;
  }

  // Process control mode command
  if (doc.containsKey("mode")) {
    const char* mode = doc["mode"];
    if (mode != nullptr) {
      String modeStr = String(mode);
      if (modeStr == CMD_MODE_AUTOMATIC) {
        setControlMode(MODE_AUTOMATIC);
      } else if (modeStr == CMD_MODE_MANUAL) {
        setControlMode(MODE_MANUAL);
      }
    }
  }

  // Process automatic mode velocity setpoints (rad/s)
  if (doc.containsKey("velocitySetpoints")) {
    JsonObject setpoints = doc["velocitySetpoints"];

    if (setpoints.containsKey("left") && setpoints.containsKey("right")) {
      double leftVel = setpoints["left"];
      double rightVel = setpoints["right"];

      // Set PID setpoints
      setPIDSetpoints(leftVel, rightVel);

      DEBUG_PRINT(F("Velocity setpoints - L: "));
      DEBUG_PRINT(leftVel);
      DEBUG_PRINT(F(" R: "));
      DEBUG_PRINT(rightVel);
      DEBUG_PRINTLN(F(" rad/s"));
    }
  }

  // Process motor status command (manual mode)
  if (doc.containsKey("motorStatus")) {
    const char* status = doc["motorStatus"];
    if (status != nullptr) {
      g_motorStatus = String(status);
      DEBUG_PRINT(F("Motor status: "));
      DEBUG_PRINTLN(g_motorStatus);
    }
  }

  // Process motor velocity command (manual mode PWM %)
  if (doc.containsKey("motorVelocity")) {
    JsonObject velocities = doc["motorVelocity"];

    // Update left motor velocity
    if (velocities.containsKey("left")) {
      int leftVel = velocities["left"];
      g_motorVelocityLeft = validateVelocity(leftVel);
      DEBUG_PRINT(F("Left velocity: "));
      DEBUG_PRINTLN(g_motorVelocityLeft);
    }

    // Update right motor velocity
    if (velocities.containsKey("right")) {
      int rightVel = velocities["right"];
      g_motorVelocityRight = validateVelocity(rightVel);
      DEBUG_PRINT(F("Right velocity: "));
      DEBUG_PRINTLN(g_motorVelocityRight);
    }
  }

  // Send acknowledgment
  #if DEBUG_ENABLED
  Serial.print(F("{\"ack\":true,\"status\":\""));
  Serial.print(g_motorStatus);
  Serial.print(F("\",\"left\":"));
  Serial.print(g_motorVelocityLeft);
  Serial.print(F(",\"right\":"));
  Serial.print(g_motorVelocityRight);
  Serial.println(F("}"));
  #endif
}

void publishEncoderFeedback()
{
  #if !ENABLE_ENCODER_FEEDBACK
    return;  // Feedback disabled
  #endif

  unsigned long currentTime = millis();

  // Check if it's time to publish
  if (currentTime - g_lastFeedbackTime < FEEDBACK_INTERVAL) {
    return;
  }

  g_lastFeedbackTime = currentTime;

  // Get encoder positions
  long leftPos = getLeftEncoderPosition();
  long rightPos = getRightEncoderPosition();

  // Get velocities (declared in evarobot_pid.h when PID is enabled)
  // For manual mode, calculate simple velocities
  long leftVelTicks, rightVelTicks;
  getEncoderVelocities(leftVelTicks, rightVelTicks);

  // Convert tick velocities to rad/s
  // vel (rad/s) = (ticks / ticks_per_rev) * 2*PI / (interval / 1000)
  float dt_seconds = FEEDBACK_INTERVAL / 1000.0;
  float leftVelRadS = (leftVelTicks / (float)ENCODER_TICKS_PER_REV) * TWO_PI / dt_seconds;
  float rightVelRadS = (rightVelTicks / (float)ENCODER_TICKS_PER_REV) * TWO_PI / dt_seconds;

  // Build JSON feedback message
  // Format: {"encoders":{"left":1234,"right":5678},"velocities":{"left":0.5,"right":0.48}}
  Serial.print(F("{\"encoders\":{\"left\":"));
  Serial.print(leftPos);
  Serial.print(F(",\"right\":"));
  Serial.print(rightPos);
  Serial.print(F("},\"velocities\":{\"left\":"));
  Serial.print(leftVelRadS, 3);  // 3 decimal places
  Serial.print(F(",\"right\":"));
  Serial.print(rightVelRadS, 3);
  Serial.println(F("}}"));

  #if DEBUG_ENABLED >= 2
    DEBUG_PRINT(F("Feedback published - Enc L:"));
    DEBUG_PRINT(leftPos);
    DEBUG_PRINT(F(" R:"));
    DEBUG_PRINT(rightPos);
    DEBUG_PRINT(F(" Vel L:"));
    DEBUG_PRINT(leftVelRadS, 3);
    DEBUG_PRINT(F(" R:"));
    DEBUG_PRINTLN(rightVelRadS, 3);
  #endif
}

#endif // EVAROBOT_SERIAL_COM_H
