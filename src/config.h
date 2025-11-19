#ifndef CONFIG_H
#define CONFIG_H

// ============================================================================
// DUAL PIPE CRAWLER ROBOT - CONFIGURATION FILE
// ============================================================================
// All tunable parameters in one place for easy adjustment
// Supports 2 independent 3-wheeled crawlers operating in unison
// Last updated: November 16, 2025
// ============================================================================

// ----------------------------------------------------------------------------
// CRAWLER SYSTEM CONFIGURATION
// ----------------------------------------------------------------------------

// Number of crawlers supported
#define NUM_CRAWLERS        2

// Motors per crawler
#define MOTORS_PER_CRAWLER  3

// Total motors in system
#define TOTAL_MOTORS        (NUM_CRAWLERS * MOTORS_PER_CRAWLER)  // 6 motors

// Shield I2C addresses for detection
#define SHIELD1_ADDR        0x2F  // Motors 1, 2
#define SHIELD2_ADDR        0x30  // Motors 3, 4
#define SHIELD3_ADDR        0x2E  // Motors 5, 6

// ----------------------------------------------------------------------------
// MOTOR CONFIGURATION - CRAWLER 1 (Motors 1, 2, 3)
// ----------------------------------------------------------------------------

// Motor I2C addresses (from MOTOR_ENCODER_MAPPING.md)
#define MOTOR1_I2C_ADDR     0x2F  // Shield 1, Motor A
#define MOTOR2_I2C_ADDR     0x2F  // Shield 1, Motor B
#define MOTOR3_I2C_ADDR     0x30  // Shield 2, Motor A

// Motor channels on shields
#define MOTOR1_CHANNEL      _MOTOR_A
#define MOTOR2_CHANNEL      _MOTOR_B
#define MOTOR3_CHANNEL      _MOTOR_A

// ----------------------------------------------------------------------------
// MOTOR CONFIGURATION - CRAWLER 2 (Motors 4, 5, 6)
// ----------------------------------------------------------------------------

// Motor I2C addresses
#define MOTOR4_I2C_ADDR     0x30  // Shield 2, Motor B
#define MOTOR5_I2C_ADDR     0x2E  // Shield 3, Motor A
#define MOTOR6_I2C_ADDR     0x2E  // Shield 3, Motor B

// Motor channels on shields
#define MOTOR4_CHANNEL      _MOTOR_B
#define MOTOR5_CHANNEL      _MOTOR_A
#define MOTOR6_CHANNEL      _MOTOR_B

// ----------------------------------------------------------------------------
// COMMON MOTOR PARAMETERS
// ----------------------------------------------------------------------------

// Motor PWM frequency (Hz)
#define MOTOR_PWM_FREQ      1000

// ----------------------------------------------------------------------------
// ENCODER CONFIGURATION - CRAWLER 1 (Motors 1, 2, 3)
// ----------------------------------------------------------------------------

// Encoder pin assignments (hardware quadrature decoder pins)
// Some pins are swapped to correct for negative polarity
#define ENCODER1_PIN_A      0
#define ENCODER1_PIN_B      1

#define ENCODER2_PIN_A      3    // Swapped for correct polarity
#define ENCODER2_PIN_B      2    // Swapped for correct polarity

#define ENCODER3_PIN_A      6    // Positive polarity
#define ENCODER3_PIN_B      5    // Positive polarity

// ----------------------------------------------------------------------------
// ENCODER CONFIGURATION - CRAWLER 2 (Motors 4, 5, 6)
// ----------------------------------------------------------------------------

// Encoder pin assignments (verified November 16, 2025)
#define ENCODER4_PIN_A      9    // Correct polarity (with reversed motor direction)
#define ENCODER4_PIN_B      10   // Correct polarity (with reversed motor direction)

#define ENCODER5_PIN_A      11   // Positive polarity
#define ENCODER5_PIN_B      12   // Positive polarity

#define ENCODER6_PIN_A      7    // Correct polarity (with reversed motor direction)
#define ENCODER6_PIN_B      8    // Correct polarity (with reversed motor direction)

// ----------------------------------------------------------------------------
// COMMON ENCODER PARAMETERS
// ----------------------------------------------------------------------------

// Encoder counts per revolution (depends on your encoders)
// Adjust based on your specific encoder specs
#define ENCODER_CPR         600  // Typical value, measure/adjust as needed

// ----------------------------------------------------------------------------
// PID CONTROLLER PARAMETERS - CRAWLER 1 (Motors 1, 2, 3)
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

// ----------------------------------------------------------------------------
// PID CONTROLLER PARAMETERS - CRAWLER 2 (Motors 4, 5, 6)
// ----------------------------------------------------------------------------

// PID gains for Motor 4 (start with same as Crawler 1)
#define MOTOR4_KP           0.8f
#define MOTOR4_KI           0.15f
#define MOTOR4_KD           0.05f

// PID gains for Motor 5
#define MOTOR5_KP           0.8f
#define MOTOR5_KI           0.15f
#define MOTOR5_KD           0.05f

// PID gains for Motor 6
#define MOTOR6_KP           0.8f
#define MOTOR6_KI           0.15f
#define MOTOR6_KD           0.05f

// ----------------------------------------------------------------------------
// COMMON PID PARAMETERS
// ----------------------------------------------------------------------------

// PID output limits (PWM duty cycle: -100 to +100)
// Sign indicates direction, magnitude is PWM percentage
#define PID_OUTPUT_MIN      -100.0f
#define PID_OUTPUT_MAX      100.0f

// Anti-windup: Maximum integral accumulation
#define PID_INTEGRAL_MAX    50.0f

// ----------------------------------------------------------------------------
// POSITION SYNCHRONIZATION
// ----------------------------------------------------------------------------

// Enable position-based synchronization to prevent robot tilt in pipe
// Keeps all three motors at the same encoder position during movement
#define ENABLE_POSITION_SYNC    true

// Position synchronization gain (proportional control)
// Higher values = stronger correction, but may cause oscillation
// Start conservative and increase if drift persists
#define POSITION_SYNC_KP        0.2f

// Maximum position error before warning (encoder counts)
#define MAX_POSITION_ERROR      500

// Maximum velocity correction allowed (counts/sec)
// Limits how much sync can adjust individual motor speeds
#define MAX_SYNC_CORRECTION     800.0f

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
#define RAMP_TIME_MS        2500    // Time to ramp from 0 to 100% (milliseconds)
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
#define WATCHDOG_TIMEOUT_MS 4000    // 2 seconds

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
// MOTOR DIRECTION MAPPINGS - CRAWLER 1 (Motors 1, 2, 3)
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
// MOTOR DIRECTION MAPPINGS - CRAWLER 2 (Motors 4, 5, 6)
// ----------------------------------------------------------------------------

// Motor 4-6 directions (match encoder polarity corrections)
#define MOTOR4_FWD_DIR      _CCW    // Reversed: Motor wired backward
#define MOTOR4_BACK_DIR     _CW

#define MOTOR5_FWD_DIR      _CW     // Correct direction
#define MOTOR5_BACK_DIR     _CCW

#define MOTOR6_FWD_DIR      _CCW    // Reversed: Motor wired backward
#define MOTOR6_BACK_DIR     _CW

// ----------------------------------------------------------------------------
// DEBUG AND LOGGING
// ----------------------------------------------------------------------------

// Enable debug output (set to false for production)
#define DEBUG_ENABLED       true

// Print received commands to serial
#define DEBUG_COMMANDS      true

// ----------------------------------------------------------------------------
// ROS2 PREPARATION
// ----------------------------------------------------------------------------

// Robot name for ROS2 namespace (future use)
#define ROBOT_NAME          "pipe_crawler"

// Velocity topic names (future ROS2 integration)
#define VEL_CMD_TOPIC       "/cmd_vel"
#define VEL_STATUS_TOPIC    "/wheel_vel"

#endif // CONFIG_H
