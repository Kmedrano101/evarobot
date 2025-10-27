/**
 * @file evarobot_encoders.h
 * @brief Quadrature encoder reading for EvaRobot odometry
 *
 * @author Kevin Medrano Ayala
 * @contact kevin.ejem18@gmail.com
 * @license BSD-3-Clause
 *
 * @description
 * Reads quadrature encoders attached to both motors for odometry feedback.
 * Uses the Encoder library for interrupt-based encoder counting.
 */

#ifndef EVAROBOT_ENCODERS_H
#define EVAROBOT_ENCODERS_H

#include <Arduino.h>
#include <Encoder.h>
#include "evarobot_config.h"

// ============================================================================
// ENCODER OBJECTS
// ============================================================================

/**
 * @brief Left motor encoder instance
 * Connected to pins A0 (A) and A1 (B)
 */
Encoder g_encoderLeft(ENCODER_LEFT_A, ENCODER_LEFT_B);

/**
 * @brief Right motor encoder instance
 * Connected to pins A2 (A) and A3 (B)
 */
Encoder g_encoderRight(ENCODER_RIGHT_A, ENCODER_RIGHT_B);

// ============================================================================
// ENCODER STATE VARIABLES
// ============================================================================

/**
 * @brief Left encoder position (cumulative ticks)
 */
long g_positionLeft = 0;

/**
 * @brief Right encoder position (cumulative ticks)
 */
long g_positionRight = 0;

/**
 * @brief Previous left encoder reading
 */
long g_prevPositionLeft = 0;

/**
 * @brief Previous right encoder reading
 */
long g_prevPositionRight = 0;

// ============================================================================
// FUNCTION PROTOTYPES
// ============================================================================

/**
 * @brief Initialize encoder subsystem
 *
 * Resets encoder counters to zero
 */
void setupEncoders();

/**
 * @brief Read current encoder values
 *
 * Updates position variables with latest encoder counts
 */
void readEncoders();

/**
 * @brief Get left encoder position
 *
 * @return Current left encoder count
 */
long getLeftEncoderPosition();

/**
 * @brief Get right encoder position
 *
 * @return Current right encoder count
 */
long getRightEncoderPosition();

/**
 * @brief Reset encoder positions to zero
 */
void resetEncoders();

/**
 * @brief Get encoder velocity (ticks since last read)
 *
 * @param left Output: left encoder velocity
 * @param right Output: right encoder velocity
 */
void getEncoderVelocities(long& left, long& right);

// ============================================================================
// IMPLEMENTATION
// ============================================================================

void setupEncoders()
{
  // Reset encoder positions
  resetEncoders();

  DEBUG_PRINTLN("Encoders initialized");
}

void readEncoders()
{
  // Store previous positions
  g_prevPositionLeft = g_positionLeft;
  g_prevPositionRight = g_positionRight;

  // Read current encoder positions
  g_positionLeft = g_encoderLeft.read();
  g_positionRight = g_encoderRight.read();

  #if DEBUG_ENABLED >= 2
    // Only log if values changed
    if (g_positionLeft != g_prevPositionLeft || g_positionRight != g_prevPositionRight) {
      DEBUG_PRINT(F("Encoders - L: "));
      DEBUG_PRINT(g_positionLeft);
      DEBUG_PRINT(F(" R: "));
      DEBUG_PRINTLN(g_positionRight);
    }
  #endif
}

long getLeftEncoderPosition()
{
  return g_positionLeft;
}

long getRightEncoderPosition()
{
  return g_positionRight;
}

void resetEncoders()
{
  g_encoderLeft.write(0);
  g_encoderRight.write(0);
  g_positionLeft = 0;
  g_positionRight = 0;
  g_prevPositionLeft = 0;
  g_prevPositionRight = 0;

  DEBUG_PRINTLN("Encoders reset");
}

void getEncoderVelocities(long& left, long& right)
{
  // Calculate velocity as difference from previous reading
  left = g_positionLeft - g_prevPositionLeft;
  right = g_positionRight - g_prevPositionRight;
}

#endif // EVAROBOT_ENCODERS_H
