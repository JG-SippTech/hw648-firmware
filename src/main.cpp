/**
 * @file main.cpp
 * @brief Pipe Crawler Robot - Main Control Firmware
 *
 * Three-wheeled pipe crawler robot with closed-loop velocity control.
 * Uses PID controllers with encoder feedback to maintain synchronized
 * wheel speeds for smooth forward/backward motion.
 *
 * Hardware:
 * - Teensy 4.0 microcontroller
 * - 2x HW-648 motor shields (I2C addresses 0x2F, 0x30)
 * - 3x DC motors with quadrature encoders
 *
 * Architecture:
 * - config.h: All tunable parameters
 * - PIDController: Generic PID implementation
 * - SpeedRamp: Smooth acceleration/deceleration
 * - MotorController: Motor + encoder + PID for single wheel
 * - RobotMotion: Coordinates 3 motors for robot movement
 * - CommandParser: Serial command interface
 *
 * @date November 12, 2025
 * @author Claude Code
 */

#include <Arduino.h>
#include <Wire.h>
#include <Encoder.h>
#include "WEMOS_Motor.h"

// Project includes
#include "config.h"
#include "PIDController.h"
#include "SpeedRamp.h"
#include "MotorController.h"
#include "RobotMotion.h"
#include "CommandParser.h"

// ============================================================================
// HARDWARE OBJECTS
// ============================================================================

// Motor shield objects (3 motors on 2 shields)
Motor motor1_hw(MOTOR1_I2C_ADDR, MOTOR1_CHANNEL, MOTOR_PWM_FREQ);
Motor motor2_hw(MOTOR2_I2C_ADDR, MOTOR2_CHANNEL, MOTOR_PWM_FREQ);
Motor motor3_hw(MOTOR3_I2C_ADDR, MOTOR3_CHANNEL, MOTOR_PWM_FREQ);

// Encoder objects (note Motor 3 pins swapped for correct polarity)
Encoder encoder1(ENCODER1_PIN_A, ENCODER1_PIN_B);
Encoder encoder2(ENCODER2_PIN_A, ENCODER2_PIN_B);
Encoder encoder3(ENCODER3_PIN_A, ENCODER3_PIN_B);  // Pins 6, 5 (reversed)

// ============================================================================
// CONTROL OBJECTS
// ============================================================================

// PID controllers (one per motor)
PIDController pid1(MOTOR1_KP, MOTOR1_KI, MOTOR1_KD,
                   PID_OUTPUT_MIN, PID_OUTPUT_MAX, PID_INTEGRAL_MAX);
PIDController pid2(MOTOR2_KP, MOTOR2_KI, MOTOR2_KD,
                   PID_OUTPUT_MIN, PID_OUTPUT_MAX, PID_INTEGRAL_MAX);
PIDController pid3(MOTOR3_KP, MOTOR3_KI, MOTOR3_KD,
                   PID_OUTPUT_MIN, PID_OUTPUT_MAX, PID_INTEGRAL_MAX);

// Motor controllers (combine motor + encoder + PID)
MotorController motorCtrl1(&motor1_hw, &encoder1, &pid1,
                           MOTOR1_FWD_DIR, MOTOR1_BACK_DIR);
MotorController motorCtrl2(&motor2_hw, &encoder2, &pid2,
                           MOTOR2_FWD_DIR, MOTOR2_BACK_DIR);
MotorController motorCtrl3(&motor3_hw, &encoder3, &pid3,
                           MOTOR3_FWD_DIR, MOTOR3_BACK_DIR);

// Speed ramp for smooth acceleration
SpeedRamp speedRamp(RAMP_TIME_MS, MAX_VELOCITY_CPS);

// Robot motion coordinator
RobotMotion robot(&motorCtrl1, &motorCtrl2, &motorCtrl3, &speedRamp);

// Command parser for serial interface
CommandParser cmdParser(&robot, SERIAL_BAUD);

// ============================================================================
// TIMING VARIABLES
// ============================================================================

unsigned long lastControlUpdate = 0;
const unsigned long controlPeriodUs = CONTROL_LOOP_MS * 1000;  // Convert to microseconds

// ============================================================================
// SETUP
// ============================================================================

void setup() {
    // Initialize I2C for motor shields
    Wire.begin();
    Wire.setClock(100000);  // 100 kHz I2C

    // Small delay for I2C to stabilize
    delay(100);

    // Initialize command parser (opens serial)
    cmdParser.begin();

    Serial.println("Initializing robot motion controller...");

    // Initialize robot (sets up motors, encoders, PIDs)
    robot.begin();

    Serial.println("Robot ready!");
    Serial.println("Waiting for commands...\n");

    // Initialize control loop timing
    lastControlUpdate = micros();
}

// ============================================================================
// MAIN LOOP
// ============================================================================

void loop() {
    // Get current time
    unsigned long currentTime = micros();

    // Check if it's time for control loop update
    if (currentTime - lastControlUpdate >= controlPeriodUs) {
        // Calculate actual time delta (seconds)
        float dt = (currentTime - lastControlUpdate) / 1000000.0f;

        // Update robot motion control (runs PID, updates motors)
        robot.update(dt);

        // Update timing
        lastControlUpdate = currentTime;
    }

    // Process serial commands (non-blocking)
    cmdParser.update();

    // Small yield to prevent watchdog issues
    yield();
}
