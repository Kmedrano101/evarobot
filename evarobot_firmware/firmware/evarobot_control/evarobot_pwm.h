/**
 * @file evarobot_pwm.h
 * @brief PWM control for EvaRobot motor speed regulation
 *
 * @author Kevin Medrano Ayala
 * @contact kevin.ejem18@gmail.com
 * @license BSD-3-Clause
 *
 * @description
 * Provides PWM speed control for two DC motors using RP2040 hardware PWM.
 * Uses the RP2040_PWM library for precise PWM generation.
 */

#ifndef EVAROBOT_PWM_H
#define EVAROBOT_PWM_H

#include <Arduino.h>
#include "evarobot_config.h"

// ============================================================================
// PWM LIBRARY CONFIGURATION
// ============================================================================

// Set PWM library log level from config
#define _PWM_LOGLEVEL_ PWM_LOG_LEVEL

// Platform validation for RP2040
#if ( defined(ARDUINO_NANO_RP2040_CONNECT) || defined(ARDUINO_RASPBERRY_PI_PICO) || \
      defined(ARDUINO_ADAFRUIT_FEATHER_RP2040) || defined(ARDUINO_GENERIC_RP2040) ) && \
      defined(ARDUINO_ARCH_MBED)

  #if (_PWM_LOGLEVEL_ > 3)
    #warning "Using MBED RP2040 PWM"
  #endif

#elif ( defined(ARDUINO_ARCH_RP2040) || defined(ARDUINO_RASPBERRY_PI_PICO) || \
        defined(ARDUINO_ADAFRUIT_FEATHER_RP2040) || defined(ARDUINO_GENERIC_RP2040) ) && \
        !defined(ARDUINO_ARCH_MBED)

  #if (_PWM_LOGLEVEL_ > 3)
    #warning "Using Arduino-Pico RP2040 PWM"
  #endif

#else
  #error "This firmware requires an RP2040-based board (Nano RP2040, Pico, etc.)"
#endif

// Include RP2040 PWM library
#include "RP2040_PWM.h"

// ============================================================================
// PWM CONFIGURATION
// ============================================================================

// Number of PWM channels (one per motor)
#define NUM_PWM_CHANNELS NUM_MOTORS

// GPIO pins for PWM output
const uint32_t g_pwmPins[NUM_PWM_CHANNELS] = {
  PWM_PIN_LEFT,
  PWM_PIN_RIGHT
};

// PWM duty cycle storage
float g_pwmDutyCycle[NUM_PWM_CHANNELS] = {
  PWM_DEFAULT_DUTY_CYCLE,
  PWM_DEFAULT_DUTY_CYCLE
};

// PWM instance pointers
RP2040_PWM* g_pwmInstance[NUM_PWM_CHANNELS] = { nullptr, nullptr };

// ============================================================================
// FUNCTION PROTOTYPES
// ============================================================================

/**
 * @brief Initialize PWM for all motor channels
 *
 * Creates PWM instances and configures hardware PWM on RP2040
 */
void setupMotorsPWM();

/**
 * @brief Set motor velocity via PWM duty cycle
 *
 * @param motorIndex Motor index (MOTOR_LEFT or MOTOR_RIGHT)
 * @param dutyCycle PWM duty cycle percentage (will be clamped to valid range)
 */
void setMotorVelocity(uint8_t motorIndex, float dutyCycle);

/**
 * @brief Get current PWM duty cycle for a motor
 *
 * @param motorIndex Motor index
 * @return Current duty cycle percentage
 */
float getMotorVelocity(uint8_t motorIndex);

// ============================================================================
// IMPLEMENTATION
// ============================================================================

void setupMotorsPWM()
{
  DEBUG_PRINTLN("Initializing PWM...");

  for (uint8_t i = 0; i < NUM_PWM_CHANNELS; i++)
  {
    // Create PWM instance for this motor
    g_pwmInstance[i] = new RP2040_PWM(
      g_pwmPins[i],
      PWM_FREQUENCY,
      g_pwmDutyCycle[i]
    );

    if (g_pwmInstance[i] != nullptr)
    {
      // Configure and start PWM
      g_pwmInstance[i]->setPWM();

      // Log PWM configuration
      #if (_PWM_LOGLEVEL_ > 2)
        uint32_t div = g_pwmInstance[i]->get_DIV();
        uint32_t top = g_pwmInstance[i]->get_TOP();
        uint32_t cpuFreq = g_pwmInstance[i]->get_freq_CPU();

        PWM_LOGDEBUG5(
          "Motor ", i,
          " PWM initialized - TOP=", top,
          ", DIV=", div,
          ", CPU_freq=", cpuFreq
        );
      #endif

      DEBUG_PRINT("PWM Channel ");
      DEBUG_PRINT(i);
      DEBUG_PRINT(" (GPIO ");
      DEBUG_PRINT(g_pwmPins[i]);
      DEBUG_PRINTLN(") initialized");
    }
    else
    {
      // PWM instance creation failed
      #if (_PWM_LOGLEVEL_ > 0)
        PWM_LOGERROR1("Failed to create PWM instance for GPIO", g_pwmPins[i]);
      #endif

      Serial.print(F("{\"error\":\"PWM init failed for motor "));
      Serial.print(i);
      Serial.println(F("\"}"));
    }
  }
}

void setMotorVelocity(uint8_t motorIndex, float dutyCycle)
{
  // Validate motor index
  if (motorIndex >= NUM_PWM_CHANNELS) {
    #if (_PWM_LOGLEVEL_ > 0)
      PWM_LOGERROR1("Invalid motor index:", motorIndex);
    #endif
    return;
  }

  // Clamp duty cycle to valid range
  // SPECIAL CASE: Allow 0 for completely stopping the motor during turns
  if (dutyCycle > PWM_MAX_DUTY_CYCLE) {
    dutyCycle = PWM_MAX_DUTY_CYCLE;
  }
  else if (dutyCycle > 0.0f && dutyCycle < PWM_MIN_DUTY_CYCLE) {
    // Only apply minimum if value is not zero
    dutyCycle = PWM_MIN_DUTY_CYCLE;
  }
  // If dutyCycle == 0, leave it as 0 (motor off)

  // Update duty cycle storage
  g_pwmDutyCycle[motorIndex] = dutyCycle;

  // Update hardware PWM
  if (g_pwmInstance[motorIndex] != nullptr) {
    g_pwmInstance[motorIndex]->setPWM(
      g_pwmPins[motorIndex],
      PWM_FREQUENCY,
      dutyCycle
    );

    #if (_PWM_LOGLEVEL_ > 3)
      PWM_LOGDEBUG3("Motor", motorIndex, "velocity set to", dutyCycle);
    #endif
  }
  else {
    #if (_PWM_LOGLEVEL_ > 0)
      PWM_LOGERROR1("PWM instance null for motor", motorIndex);
    #endif
  }
}

float getMotorVelocity(uint8_t motorIndex)
{
  if (motorIndex >= NUM_PWM_CHANNELS) {
    return 0.0f;
  }
  return g_pwmDutyCycle[motorIndex];
}

#endif // EVAROBOT_PWM_H
