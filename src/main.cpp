/**
 * @file main.cpp
 * @brief Dual Pipe Crawler Robot - Main Control Firmware
 *
 * Two 3-wheeled pipe crawler robots with closed-loop velocity control.
 * Supports auto-detection and graceful degradation (1 or 2 crawlers).
 * Uses PID controllers with encoder feedback to maintain synchronized
 * wheel speeds within each crawler for smooth coordinated motion.
 *
 * Hardware:
 * - Teensy 4.0 microcontroller
 * - 3x HW-648 motor shields (I2C addresses 0x2E, 0x2F, 0x30)
 * - 6x DC motors with quadrature encoders
 *
 * Crawler Configuration:
 * - Crawler 1: Motors 1-3 (Shields 0x2F + 0x30)
 * - Crawler 2: Motors 4-6 (Shields 0x30 + 0x2E)
 *
 * @date November 16, 2025
 * @author Claude Code
 */

#include <Arduino.h>
#include <Wire.h>
#include <Encoder.h>
#include "WEMOS_Motor.h"

// Project includes
#include "config.h"

// IMU support (after config.h for ENABLE_IMU)
#if ENABLE_IMU
#include <Adafruit_LSM9DS1.h>
#include <Adafruit_Sensor.h>
#endif
#include "PIDController.h"
#include "SpeedRamp.h"
#include "MotorController.h"
#include "RobotMotion.h"
#include "CommandParser.h"

// ============================================================================
// HARDWARE OBJECTS - CRAWLER 1 (Motors 1, 2, 3)
// ============================================================================

// Motor shield objects
Motor motor1_hw(MOTOR1_I2C_ADDR, MOTOR1_CHANNEL, MOTOR_PWM_FREQ);
Motor motor2_hw(MOTOR2_I2C_ADDR, MOTOR2_CHANNEL, MOTOR_PWM_FREQ);
Motor motor3_hw(MOTOR3_I2C_ADDR, MOTOR3_CHANNEL, MOTOR_PWM_FREQ);

// Encoder objects (with corrected polarity)
Encoder encoder1(ENCODER1_PIN_A, ENCODER1_PIN_B);
Encoder encoder2(ENCODER2_PIN_A, ENCODER2_PIN_B);
Encoder encoder3(ENCODER3_PIN_A, ENCODER3_PIN_B);

// ============================================================================
// HARDWARE OBJECTS - CRAWLER 2 (Motors 4, 5, 6)
// ============================================================================

// Motor shield objects
Motor motor4_hw(MOTOR4_I2C_ADDR, MOTOR4_CHANNEL, MOTOR_PWM_FREQ);
Motor motor5_hw(MOTOR5_I2C_ADDR, MOTOR5_CHANNEL, MOTOR_PWM_FREQ);
Motor motor6_hw(MOTOR6_I2C_ADDR, MOTOR6_CHANNEL, MOTOR_PWM_FREQ);

// Encoder objects (with corrected polarity)
Encoder encoder4(ENCODER4_PIN_A, ENCODER4_PIN_B);
Encoder encoder5(ENCODER5_PIN_A, ENCODER5_PIN_B);
Encoder encoder6(ENCODER6_PIN_A, ENCODER6_PIN_B);

// ============================================================================
// CONTROL OBJECTS
// ============================================================================

// PID controllers - Crawler 1
PIDController pid1(MOTOR1_KP, MOTOR1_KI, MOTOR1_KD,
                   PID_OUTPUT_MIN, PID_OUTPUT_MAX, PID_INTEGRAL_MAX);
PIDController pid2(MOTOR2_KP, MOTOR2_KI, MOTOR2_KD,
                   PID_OUTPUT_MIN, PID_OUTPUT_MAX, PID_INTEGRAL_MAX);
PIDController pid3(MOTOR3_KP, MOTOR3_KI, MOTOR3_KD,
                   PID_OUTPUT_MIN, PID_OUTPUT_MAX, PID_INTEGRAL_MAX);

// PID controllers - Crawler 2
PIDController pid4(MOTOR4_KP, MOTOR4_KI, MOTOR4_KD,
                   PID_OUTPUT_MIN, PID_OUTPUT_MAX, PID_INTEGRAL_MAX);
PIDController pid5(MOTOR5_KP, MOTOR5_KI, MOTOR5_KD,
                   PID_OUTPUT_MIN, PID_OUTPUT_MAX, PID_INTEGRAL_MAX);
PIDController pid6(MOTOR6_KP, MOTOR6_KI, MOTOR6_KD,
                   PID_OUTPUT_MIN, PID_OUTPUT_MAX, PID_INTEGRAL_MAX);

// Motor controllers - Crawler 1
MotorController motorCtrl1(&motor1_hw, &encoder1, &pid1,
                           MOTOR1_FWD_DIR, MOTOR1_BACK_DIR);
MotorController motorCtrl2(&motor2_hw, &encoder2, &pid2,
                           MOTOR2_FWD_DIR, MOTOR2_BACK_DIR);
MotorController motorCtrl3(&motor3_hw, &encoder3, &pid3,
                           MOTOR3_FWD_DIR, MOTOR3_BACK_DIR);

// Motor controllers - Crawler 2
MotorController motorCtrl4(&motor4_hw, &encoder4, &pid4,
                           MOTOR4_FWD_DIR, MOTOR4_BACK_DIR);
MotorController motorCtrl5(&motor5_hw, &encoder5, &pid5,
                           MOTOR5_FWD_DIR, MOTOR5_BACK_DIR);
MotorController motorCtrl6(&motor6_hw, &encoder6, &pid6,
                           MOTOR6_FWD_DIR, MOTOR6_BACK_DIR);

// Speed ramp for smooth acceleration
SpeedRamp speedRamp(RAMP_TIME_MS, MAX_VELOCITY_CPS);

// Robot motion coordinator (supports both crawlers)
RobotMotion robot(&motorCtrl1, &motorCtrl2, &motorCtrl3,
                  &motorCtrl4, &motorCtrl5, &motorCtrl6,
                  &speedRamp);

// Command parser for serial interface
CommandParser cmdParser(&robot, SERIAL_BAUD);

// ============================================================================
// IMU (LSM9DS1 on Wire1)
// ============================================================================

#if ENABLE_IMU
// IMU object on Wire1 (pins 16/17) - separate from motor shields
Adafruit_LSM9DS1 lsm = Adafruit_LSM9DS1(&IMU_WIRE);

// IMU state
bool imuAvailable = false;
float currentPitch = 0.0f;    // Pitch angle in degrees (nose up/down)
float currentRoll = 0.0f;     // Roll angle in degrees (tilt left/right)
unsigned long lastImuUpdate = 0;
const unsigned long imuPeriodMs = 1000 / IMU_UPDATE_HZ;

// Tilt correction state
bool imuCalibrated = false;
float baselinePitch = 0.0f;   // Calibrated "level" pitch
unsigned long lastTiltWarning = 0;
const unsigned long tiltWarningInterval = 2000;  // Warn every 2 seconds max

/**
 * @brief Initialize IMU on Wire1
 * @return true if IMU found and configured
 */
bool initIMU() {
    // Initialize Wire1 for IMU
    IMU_WIRE.begin();
    IMU_WIRE.setClock(400000);  // 400 kHz I2C for IMU

    Serial.print("Initializing IMU on Wire1... ");

    if (!lsm.begin()) {
        Serial.println("NOT FOUND");
        return false;
    }

    // Configure sensor ranges
    lsm.setupAccel(lsm.LSM9DS1_ACCELRANGE_2G);
    lsm.setupMag(lsm.LSM9DS1_MAGGAIN_4GAUSS);
    lsm.setupGyro(lsm.LSM9DS1_GYROSCALE_245DPS);

    Serial.println("OK");
    return true;
}

/**
 * @brief Read IMU and calculate pitch/roll
 */
void updateIMU() {
    if (!imuAvailable) return;

    // Read all sensors
    lsm.read();

    // Get sensor events
    sensors_event_t accel, mag, gyro, temp;
    lsm.getEvent(&accel, &mag, &gyro, &temp);

    // Calculate pitch and roll from accelerometer
    // Assuming IMU is mounted with X forward, Y left, Z up
    float accelPitch = atan2(-accel.acceleration.x,
                             sqrt(accel.acceleration.y * accel.acceleration.y +
                                  accel.acceleration.z * accel.acceleration.z)) * 180.0f / PI;
    float accelRoll = atan2(accel.acceleration.y, accel.acceleration.z) * 180.0f / PI;

    // Simple complementary filter (combine gyro and accel)
    // For now, just use accelerometer (no gyro integration yet)
    currentPitch = accelPitch;
    currentRoll = accelRoll;
}

/**
 * @brief Get current pitch angle
 */
float getIMUPitch() { return currentPitch; }

/**
 * @brief Get current roll angle
 */
float getIMURoll() { return currentRoll; }

/**
 * @brief Check if IMU is available
 */
bool isIMUAvailable() { return imuAvailable; }

/**
 * @brief Calibrate IMU by setting current pitch as baseline
 */
void calibrateIMU() {
    if (!imuAvailable) {
        Serial.println("ERROR: IMU not available for calibration");
        return;
    }

    // Take a few readings to stabilize
    for (int i = 0; i < 5; i++) {
        updateIMU();
        delay(50);
    }

    baselinePitch = currentPitch;
    imuCalibrated = true;

    Serial.print("IMU calibrated - Baseline pitch: ");
    Serial.print(baselinePitch, 1);
    Serial.println("°");
}

/**
 * @brief Get tilt error from calibrated baseline
 * @return Tilt error in degrees (positive = tilting forward)
 */
float getTiltError() {
    if (!imuCalibrated) return 0.0f;
    return currentPitch - baselinePitch;
}

/**
 * @brief Check if IMU is calibrated
 */
bool isIMUCalibrated() { return imuCalibrated; }

/**
 * @brief Check tilt and print warning if threshold exceeded
 */
void checkTiltWarning() {
    #if ENABLE_TILT_CORRECTION
    if (!imuAvailable || !imuCalibrated) return;

    float tiltError = getTiltError();

    if (fabs(tiltError) > TILT_WARNING_THRESHOLD) {
        // Rate-limit warnings
        if (millis() - lastTiltWarning >= tiltWarningInterval) {
            Serial.print("[TILT WARNING] ");
            Serial.print(tiltError > 0 ? "Forward" : "Backward");
            Serial.print(" tilt: ");
            Serial.print(fabs(tiltError), 1);
            Serial.println("°");
            lastTiltWarning = millis();
        }
    }
    #endif
}
#endif

// ============================================================================
// SHIELD DETECTION
// ============================================================================

struct CrawlerStatus {
    bool shield1_present;      // 0x2F - Motors 1, 2
    bool shield2_present;      // 0x30 - Motors 3, 4
    bool shield3_present;      // 0x2E - Motors 5, 6

    bool crawler1_available;   // Shields 1 AND 2
    bool crawler2_available;   // Shields 2 AND 3

    int total_motors;          // 3 or 6
    int active_crawlers;       // 1 or 2
};

/**
 * @brief Test if I2C device responds at given address
 */
bool testI2CAddress(uint8_t address) {
    Wire.beginTransmission(address);
    uint8_t error = Wire.endTransmission();
    return (error == 0);
}

/**
 * @brief Detect connected shields and determine crawler availability
 */
CrawlerStatus detectCrawlers() {
    CrawlerStatus status = {false, false, false, false, false, 0, 0};

    // Scan for each shield
    status.shield1_present = testI2CAddress(SHIELD1_ADDR);
    status.shield2_present = testI2CAddress(SHIELD2_ADDR);
    status.shield3_present = testI2CAddress(SHIELD3_ADDR);

    // Determine crawler availability
    // Crawler 1 needs shields 1 AND 2
    status.crawler1_available = status.shield1_present && status.shield2_present;

    // Crawler 2 needs shields 2 AND 3
    status.crawler2_available = status.shield2_present && status.shield3_present;

    // Count active crawlers and motors
    status.active_crawlers = (status.crawler1_available ? 1 : 0) +
                            (status.crawler2_available ? 1 : 0);
    status.total_motors = status.active_crawlers * MOTORS_PER_CRAWLER;

    return status;
}

/**
 * @brief Print shield detection results
 */
void printDetectionResults(const CrawlerStatus& status) {
    Serial.println("\n========================================");
    Serial.println("     DUAL CRAWLER DETECTION");
    Serial.println("========================================\n");

    Serial.println("I2C Shield Scan:");

    Serial.print("  0x2F (Shield 1) - ");
    Serial.print(status.shield1_present ? "✓ FOUND" : "✗ NOT FOUND");
    Serial.println(" - Motors 1, 2");

    Serial.print("  0x30 (Shield 2) - ");
    Serial.print(status.shield2_present ? "✓ FOUND" : "✗ NOT FOUND");
    Serial.println(" - Motors 3, 4");

    Serial.print("  0x2E (Shield 3) - ");
    Serial.print(status.shield3_present ? "✓ FOUND" : "✗ NOT FOUND");
    Serial.println(" - Motors 5, 6");

    Serial.println("\nCrawler Status:");

    Serial.print("  Crawler 1 (Motors 1-3) - ");
    if (status.crawler1_available) {
        Serial.println("✓ Available");
    } else {
        Serial.print("✗ Unavailable (");
        if (!status.shield1_present) Serial.print("Shield 1 missing");
        if (!status.shield1_present && !status.shield2_present) Serial.print(", ");
        if (!status.shield2_present) Serial.print("Shield 2 missing");
        Serial.println(")");
    }

    Serial.print("  Crawler 2 (Motors 4-6) - ");
    if (status.crawler2_available) {
        Serial.println("✓ Available");
    } else {
        Serial.print("✗ Unavailable (");
        if (!status.shield2_present) Serial.print("Shield 2 missing");
        if (!status.shield2_present && !status.shield3_present) Serial.print(", ");
        if (!status.shield3_present) Serial.print("Shield 3 missing");
        Serial.println(")");
    }

    Serial.print("\nTotal Operational Crawlers: ");
    Serial.println(status.active_crawlers);
    Serial.print("Total Motors: ");
    Serial.println(status.total_motors);
    Serial.println("========================================\n");
}

/**
 * @brief Wait for user confirmation to proceed
 * @return true if user confirmed, false if cancelled
 */
bool waitForUserConfirmation() {
    Serial.println("Continue with detected configuration? (Y/N): ");
    Serial.flush();

    // Wait for user input (with 30 second timeout)
    unsigned long startTime = millis();
    while (millis() - startTime < 30000) {
        if (Serial.available() > 0) {
            char response = Serial.read();

            // Clear rest of input buffer
            while (Serial.available()) Serial.read();

            if (response == 'Y' || response == 'y') {
                Serial.println("✓ User confirmed - starting system\n");
                return true;
            } else if (response == 'N' || response == 'n') {
                Serial.println("✗ User cancelled - system halted\n");
                return false;
            }
        }
        delay(10);
    }

    Serial.println("✗ Timeout - system halted\n");
    return false;
}

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
    delay(100);  // Allow I2C to stabilize

    // Initialize serial
    Serial.begin(SERIAL_BAUD);
    while (!Serial && millis() < 3000) {
        // Wait for serial connection (max 3 seconds)
    }
    delay(500);

    Serial.println("\n\n╔════════════════════════════════════════════╗");
    Serial.println("║   DUAL PIPE CRAWLER CONTROL SYSTEM         ║");
    Serial.println("║   Teensy 4.0 + 3x HW-648 Motor Shields     ║");
    Serial.println("╚════════════════════════════════════════════╝\n");

    // Detect connected crawlers
    CrawlerStatus status = detectCrawlers();
    printDetectionResults(status);

    // Check if any crawlers are available
    if (status.active_crawlers == 0) {
        Serial.println("ERROR: No complete crawlers detected!");
        Serial.println("Each crawler requires specific shields:");
        Serial.println("  Crawler 1: Shields 0x2F + 0x30");
        Serial.println("  Crawler 2: Shields 0x30 + 0x2E");
        Serial.println("\nSystem halted. Check connections and restart.\n");
        while(1) {
            delay(1000);  // Halt indefinitely
        }
    }

    // Wait for user confirmation
    if (!waitForUserConfirmation()) {
        Serial.println("System halted by user. Reset to restart.\n");
        while(1) {
            delay(1000);  // Halt indefinitely
        }
    }

    // Configure robot with detected crawlers
    robot.setActiveCrawlers(status.crawler1_available, status.crawler2_available);

    // Initialize command parser
    cmdParser.begin();

    Serial.println("Initializing dual crawler motion controller...");

    // Initialize robot (sets up active motors, encoders, PIDs)
    robot.begin();

    // Initialize IMU
    #if ENABLE_IMU
    imuAvailable = initIMU();

    // Auto-calibrate IMU baseline if enabled
    #if AUTO_CALIBRATE_ON_START
    if (imuAvailable) {
        Serial.print("Auto-calibrating IMU... ");
        calibrateIMU();
    }
    #endif
    #endif

    Serial.println("✓ Robot ready!\n");
    Serial.print("Active: ");
    if (status.crawler1_available) Serial.print("Crawler 1 ");
    if (status.crawler1_available && status.crawler2_available) Serial.print("+ ");
    if (status.crawler2_available) Serial.print("Crawler 2");
    #if ENABLE_IMU
    if (imuAvailable) Serial.print(" + IMU");
    #endif
    Serial.println("\n");
    Serial.println("Type HELP for available commands\n");

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

    // Update IMU at configured rate
    #if ENABLE_IMU
    if (imuAvailable && (millis() - lastImuUpdate >= imuPeriodMs)) {
        updateIMU();
        checkTiltWarning();
        lastImuUpdate = millis();
    }
    #endif

    // Process serial commands (non-blocking)
    cmdParser.update();

    // Small yield to prevent watchdog issues
    yield();
}
