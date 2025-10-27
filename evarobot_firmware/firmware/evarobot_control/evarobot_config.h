/**
 * @file evarobot_config.h
 * @brief Configuration parameters for EvaRobot firmware
 *
 * @author Kevin Medrano Ayala
 * @contact kevin.ejem18@gmail.com
 * @license BSD-3-Clause
 */

#ifndef EVAROBOT_CONFIG_H
#define EVAROBOT_CONFIG_H

// ============================================================================
// FIRMWARE VERSION
// ============================================================================
#define FIRMWARE_VERSION "1.0.0"
#define FIRMWARE_DATE "2025-01-23"

// ============================================================================
// SERIAL COMMUNICATION
// ============================================================================
#define SERIAL_BAUD_RATE 115200
#define SERIAL_TIMEOUT_MS 1000
#define JSON_BUFFER_SIZE 200

// ============================================================================
// MOTOR INDICES
// ============================================================================
#define MOTOR_LEFT 0
#define MOTOR_RIGHT 1
#define NUM_MOTORS 2

// ============================================================================
// MOTOR CONTROL PINS (H-Bridge)
// ============================================================================
#define MOTOR_LEFT_IN1 9
#define MOTOR_LEFT_IN2 10
#define MOTOR_RIGHT_IN1 11
#define MOTOR_RIGHT_IN2 12

// ============================================================================
// PWM CONFIGURATION
// ============================================================================
#define PWM_PIN_LEFT 17
#define PWM_PIN_RIGHT 18
#define PWM_FREQUENCY 500.0f  // Hz
#define PWM_MIN_DUTY_CYCLE 38.0f
#define PWM_MAX_DUTY_CYCLE 95.0f
#define PWM_DEFAULT_DUTY_CYCLE 75.0f

// ============================================================================
// ENCODER PINS
// ============================================================================
#define ENCODER_LEFT_A A0
#define ENCODER_LEFT_B A1
#define ENCODER_RIGHT_A A2
#define ENCODER_RIGHT_B A3

// ============================================================================
// ENCODER SPECIFICATIONS
// ============================================================================
#define ENCODER_TICKS_PER_REV 385    // Encoder ticks per motor revolution
#define VELOCITY_UPDATE_INTERVAL 100 // Velocity update interval (ms)

// ============================================================================
// PID CONTROL PARAMETERS
// ============================================================================

// Left Motor PID Gains (tune these for your robot)
#define PID_KP_LEFT 11.5   // Proportional gain
#define PID_KI_LEFT 7.5    // Integral gain
#define PID_KD_LEFT 0.1    // Derivative gain

// Right Motor PID Gains (tune these for your robot)
#define PID_KP_RIGHT 12.8  // Proportional gain
#define PID_KI_RIGHT 8.3   // Integral gain
#define PID_KD_RIGHT 0.1   // Derivative gain

// PID Output Limits (already defined in PWM section)
// PWM_MIN_DUTY_CYCLE and PWM_MAX_DUTY_CYCLE are used as output limits

// ============================================================================
// FEEDBACK PUBLISHING
// ============================================================================
#define FEEDBACK_INTERVAL 100         // Encoder feedback publish interval (ms)
#define ENABLE_ENCODER_FEEDBACK 1     // 0=Disabled, 1=Enabled

// ============================================================================
// MOTION COMMANDS
// ============================================================================
#define CMD_STOP "stop"
#define CMD_FORWARD "forward"
#define CMD_BACKWARD "backward"
#define CMD_TURN_LEFT "turnleft"
#define CMD_TURN_RIGHT "turnright"
#define CMD_FORWARD_LEFT "forward_left"
#define CMD_FORWARD_RIGHT "forward_right"
#define CMD_BACKWARD_LEFT "backward_left"
#define CMD_BACKWARD_RIGHT "backward_right"

// Control mode commands
#define CMD_MODE_MANUAL "manual"
#define CMD_MODE_AUTOMATIC "automatic"

// ============================================================================
// DEBUG OPTIONS
// ============================================================================
#define DEBUG_ENABLED 0       // Set to 1 to enable debug output
#define PID_DEBUG_ENABLED 0   // Set to 1 for PID debug, 2 for verbose PID debug
#define PWM_LOG_LEVEL 1       // 0=None, 1=Error, 2=Warn, 3=Info, 4=Debug, 5=Verbose

#if DEBUG_ENABLED
  #define DEBUG_PRINT(x) Serial.print(x)
  #define DEBUG_PRINTLN(x) Serial.println(x)
#else
  #define DEBUG_PRINT(x)
  #define DEBUG_PRINTLN(x)
#endif

#endif // EVAROBOT_CONFIG_H
