#ifndef CONFIG_H
#define CONFIG_H

// ============================================================================
// PIPE CRAWLER ROBOT - CONFIGURATION FILE
// ============================================================================
// All tunable parameters in one place for easy adjustment
// Last updated: November 12, 2025
// ============================================================================

// ----------------------------------------------------------------------------
// MOTOR CONFIGURATION
// ----------------------------------------------------------------------------

// Motor I2C addresses (from MOTOR_ENCODER_MAPPING.md)
#define MOTOR1_I2C_ADDR     0x2F  // Shield 1, Motor A (RED wire)
#define MOTOR2_I2C_ADDR     0x2F  // Shield 1, Motor B (BLACK wire)
#define MOTOR3_I2C_ADDR     0x30  // Shield 2, Motor A

// Motor channels on shields
#define MOTOR1_CHANNEL      _MOTOR_A
#define MOTOR2_CHANNEL      _MOTOR_B
#define MOTOR3_CHANNEL      _MOTOR_A

// Motor PWM frequency (Hz)
#define MOTOR_PWM_FREQ      1000

// ----------------------------------------------------------------------------
// ENCODER CONFIGURATION
// ----------------------------------------------------------------------------

// Encoder pin assignments (hardware quadrature decoder pins)
// Motor 2 and 3 pins are swapped to match motor direction reversals
#define ENCODER1_PIN_A      0
#define ENCODER1_PIN_B      1

#define ENCODER2_PIN_A      3    // Swapped - motor direction reversed
#define ENCODER2_PIN_B      2    // Swapped - motor direction reversed

#define ENCODER3_PIN_A      6    // Swapped for correct polarity
#define ENCODER3_PIN_B      5    // Swapped for correct polarity

// Encoder counts per revolution (depends on your encoders)
// Adjust based on your specific encoder specs
#define ENCODER_CPR         600  // Typical value, measure/adjust as needed

// ----------------------------------------------------------------------------
// PID CONTROLLER PARAMETERS
// ----------------------------------------------------------------------------

// PID gains for Motor 1 (start conservative, tune during testing)
#define MOTOR1_KP           0.8f
#define MOTOR1_KI           0.15f
#define MOTOR1_KD           0.05f

// PID gains for Motor 2
#define MOTOR2_KP           0.8f
#define MOTOR2_KI           0.15f
#define MOTOR2_KD           0.05f

// PID gains for Motor 3
#define MOTOR3_KP           0.8f
#define MOTOR3_KI           0.15f
#define MOTOR3_KD           0.05f

// PID output limits (PWM duty cycle: -100 to +100)
// Sign indicates direction, magnitude is PWM percentage
#define PID_OUTPUT_MIN      -100.0f
#define PID_OUTPUT_MAX      100.0f

// Anti-windup: Maximum integral accumulation
#define PID_INTEGRAL_MAX    50.0f

// ----------------------------------------------------------------------------
// SPEED AND MOTION PARAMETERS
// ----------------------------------------------------------------------------

// Speed limits (0-100 percentage)
#define MIN_SPEED           0
#define MAX_SPEED           100
#define DEFAULT_SPEED       50

// Velocity setpoint scaling factor
// Converts speed percentage to target encoder counts/second
// Formula: target_velocity = (speed_percent / 100) * MAX_VELOCITY_CPS
// Note: Encoder reads MOTOR SHAFT speed (before gearbox), not wheel speed
// Measured at ~4000 counts/sec at full power
#define MAX_VELOCITY_CPS    4500.0f  // Max encoder counts per second at 100% speed

// Acceleration ramping
#define RAMP_TIME_MS        1500    // Time to ramp from 0 to 100% (milliseconds)
#define RAMP_STEP_MS        20      // Time between ramp steps (matches control loop)

// Calculate ramp increment per step
// This gives the velocity change per control loop iteration
#define RAMP_INCREMENT      (MAX_VELOCITY_CPS / (RAMP_TIME_MS / RAMP_STEP_MS))

// ----------------------------------------------------------------------------
// CONTROL LOOP TIMING
// ----------------------------------------------------------------------------

// Main control loop frequency
#define CONTROL_LOOP_HZ     50      // 50 Hz = 20ms period
#define CONTROL_LOOP_MS     (1000 / CONTROL_LOOP_HZ)

// Velocity measurement window
#define VELOCITY_CALC_MS    CONTROL_LOOP_MS  // Calculate velocity every loop

// ----------------------------------------------------------------------------
// SAFETY PARAMETERS
// ----------------------------------------------------------------------------

// Watchdog timeout (auto-stop if no command received)
#define WATCHDOG_TIMEOUT_MS 8000    // 2 seconds

// Emergency stop conditions
#define ENABLE_WATCHDOG     true
#define ENABLE_SPEED_LIMITS true

// Maximum allowed velocity error before warning
#define MAX_VELOCITY_ERROR  100.0f  // counts/sec deviation threshold

// ----------------------------------------------------------------------------
// SERIAL COMMUNICATION
// ----------------------------------------------------------------------------

#define SERIAL_BAUD         115200
#define SERIAL_TIMEOUT_MS   100

// Status reporting interval
#define STATUS_REPORT_MS    500     // Print status every 500ms

// Command buffer size
#define CMD_BUFFER_SIZE     64

// ----------------------------------------------------------------------------
// MOTOR DIRECTION MAPPINGS
// ----------------------------------------------------------------------------

// Define forward direction for each motor
// Adjust these if motors spin backwards when commanded forward
#define MOTOR1_FWD_DIR      _CW     // Clockwise = forward
#define MOTOR1_BACK_DIR     _CCW    // Counter-clockwise = backward

#define MOTOR2_FWD_DIR      _CCW    // Reversed - motor wired opposite
#define MOTOR2_BACK_DIR     _CW

#define MOTOR3_FWD_DIR      _CW
#define MOTOR3_BACK_DIR     _CCW

// ----------------------------------------------------------------------------
// DEBUG AND LOGGING
// ----------------------------------------------------------------------------

// Enable debug output (set to false for production)
#define DEBUG_ENABLED       true

// Debug output flags
#define DEBUG_PID           false   // Print PID calculations
#define DEBUG_VELOCITY      false   // Print velocity measurements
#define DEBUG_COMMANDS      true    // Print received commands
#define DEBUG_MOTORS        false   // Print motor setpoints

// ----------------------------------------------------------------------------
// ROS2 PREPARATION
// ----------------------------------------------------------------------------

// Robot name for ROS2 namespace (future use)
#define ROBOT_NAME          "pipe_crawler"

// Velocity topic names (future ROS2 integration)
#define VEL_CMD_TOPIC       "/cmd_vel"
#define VEL_STATUS_TOPIC    "/wheel_vel"

#endif // CONFIG_H
