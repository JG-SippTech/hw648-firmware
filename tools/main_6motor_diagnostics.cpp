/**
 * @file main_6motor_diagnostics.cpp
 * @brief 6-Motor Encoder Diagnostic Tool
 *
 * Tests all 6 motors across 3 HW-648 shields and identifies encoder mappings.
 * Spins each motor sequentially at 30% speed for 5 seconds while displaying
 * all encoder readings to verify correct motor-encoder pairings.
 *
 * Hardware Setup:
 * - 3x HW-648 Motor Shields (I2C addresses: 0x2F, 0x30, 0x2E)
 * - 6x DC Motors with quadrature encoders
 * - Teensy 4.0 microcontroller
 *
 * @date November 12, 2025
 */

#include <Arduino.h>
#include <Wire.h>
#include <Encoder.h>
#include "WEMOS_Motor.h"

// ============================================================================
// MOTOR SHIELD CONFIGURATION
// ============================================================================

// Shield 1 (0x2F) - Motors 1 and 2
Motor Shield1_MotorA(0x2F, _MOTOR_A, 1000);  // Motor 1
Motor Shield1_MotorB(0x2F, _MOTOR_B, 1000);  // Motor 2

// Shield 2 (0x30) - Motors 3 and 4
Motor Shield2_MotorA(0x30, _MOTOR_A, 1000);  // Motor 3
Motor Shield2_MotorB(0x30, _MOTOR_B, 1000);  // Motor 4

// Shield 3 (0x2E) - Motors 5 and 6
Motor Shield3_MotorA(0x2E, _MOTOR_A, 1000);  // Motor 5
Motor Shield3_MotorB(0x2E, _MOTOR_B, 1000);  // Motor 6

// ============================================================================
// ENCODER CONFIGURATION
// ============================================================================

// Existing encoders (Motors 1-3)
Encoder encoder_01(0, 1);     // Motor 1 - Positive polarity
Encoder encoder_23(3, 2);     // Motor 2 - Swapped for correct polarity
Encoder encoder_65(6, 5);     // Motor 3 - Swapped for correct polarity

// New encoders (Motors 4-6)
Encoder encoder_78(7, 8);     // Motor 4 - Hardware QuadTimer 3
Encoder encoder_910(9, 10);   // Motor 5 - Hardware QuadTimer 4
Encoder encoder_1112(11, 12); // Motor 6 - Software interrupts

// ============================================================================
// TEST CONFIGURATION
// ============================================================================

const int TEST_SPEED = 30;           // Motor speed (0-100%)
const unsigned long TEST_DURATION = 5000;  // Test duration per motor (ms)
const unsigned long PRINT_INTERVAL = 200;  // Status print interval (ms)

// ============================================================================
// HELPER FUNCTIONS
// ============================================================================

void printEncoderReadings() {
    Serial.print("Encoders: [0-1]=");
    Serial.print(encoder_01.read());
    Serial.print("  [3-2]=");
    Serial.print(encoder_23.read());
    Serial.print("  [6-5]=");
    Serial.print(encoder_65.read());
    Serial.print("  [7-8]=");
    Serial.print(encoder_78.read());
    Serial.print("  [9-10]=");
    Serial.print(encoder_910.read());
    Serial.print("  [11-12]=");
    Serial.println(encoder_1112.read());
}

void resetAllEncoders() {
    encoder_01.write(0);
    encoder_23.write(0);
    encoder_65.write(0);
    encoder_78.write(0);
    encoder_910.write(0);
    encoder_1112.write(0);
}

void stopAllMotors() {
    Shield1_MotorA.setmotor(_STOP);
    Shield1_MotorB.setmotor(_STOP);
    Shield2_MotorA.setmotor(_STOP);
    Shield2_MotorB.setmotor(_STOP);
    Shield3_MotorA.setmotor(_STOP);
    Shield3_MotorB.setmotor(_STOP);
}

void scanI2CBus() {
    Serial.println("\n========================================");
    Serial.println("        I2C BUS SCAN");
    Serial.println("========================================");
    Serial.println("Scanning addresses 0x2D to 0x30...\n");

    int devicesFound = 0;

    for (uint8_t addr = 0x2D; addr <= 0x30; addr++) {
        Wire.beginTransmission(addr);
        uint8_t error = Wire.endTransmission();

        Serial.print("  0x");
        Serial.print(addr, HEX);
        Serial.print(" (");
        Serial.print(addr);
        Serial.print(") - ");

        if (error == 0) {
            Serial.print("✓ FOUND");
            devicesFound++;

            // Identify which shield
            if (addr == 0x2D) Serial.println(" - Shield with AD0+AD1 soldered");
            else if (addr == 0x2E) Serial.println(" - Shield 3 (AD1 soldered)");
            else if (addr == 0x2F) Serial.println(" - Shield 1 (AD0 soldered)");
            else if (addr == 0x30) Serial.println(" - Shield 2 (no pads)");
        } else {
            Serial.println("  Not detected");
        }
    }

    Serial.print("\nTotal shields found: ");
    Serial.println(devicesFound);
    Serial.println("========================================\n");
}

void testMotor(const char* name, Motor& motor, const char* address, const char* channel) {
    Serial.println("\n========================================");
    Serial.print("Testing: ");
    Serial.print(name);
    Serial.print(" (Shield ");
    Serial.print(address);
    Serial.print(", Motor ");
    Serial.print(channel);
    Serial.println(")");
    Serial.println("========================================");
    Serial.println("Motor spinning at 30% for 5 seconds...");
    Serial.println("Watch which encoder counts increase:");
    Serial.println();

    // Reset encoders
    resetAllEncoders();

    // Spin motor
    motor.setmotor(_CW, TEST_SPEED);

    unsigned long startTime = millis();
    unsigned long lastPrint = 0;

    while (millis() - startTime < TEST_DURATION) {
        if (millis() - lastPrint >= PRINT_INTERVAL) {
            printEncoderReadings();
            lastPrint = millis();
        }
    }

    // Stop motor
    motor.setmotor(_STOP);

    Serial.println("\n--- Final Encoder Counts ---");
    printEncoderReadings();
    Serial.println();
    Serial.println("Press ENTER to continue (or type 'SCAN' for I2C scan)...");

    // Wait for user input
    String input = "";
    while (true) {
        if (Serial.available()) {
            char c = Serial.read();
            if (c == '\n' || c == '\r') {
                if (input.equalsIgnoreCase("SCAN")) {
                    scanI2CBus();
                    Serial.println("Press ENTER to continue to next motor...");
                    input = "";
                } else {
                    break;  // Continue to next motor
                }
            } else {
                input += c;
            }
        }
        delay(10);
    }

    // Clear any remaining input
    while (Serial.available()) {
        Serial.read();
    }
}

// ============================================================================
// SETUP
// ============================================================================

void setup() {
    // Initialize serial
    Serial.begin(115200);
    while (!Serial && millis() < 3000) {
        // Wait for serial connection (max 3 seconds)
    }

    delay(500);

    Serial.println("\n\n");
    Serial.println("╔════════════════════════════════════════════╗");
    Serial.println("║   6-MOTOR ENCODER DIAGNOSTIC TOOL          ║");
    Serial.println("║   Pipe Crawler Robot - Full System Test   ║");
    Serial.println("╚════════════════════════════════════════════╝");
    Serial.println();

    // Initialize I2C
    Wire.begin();
    Wire.setClock(100000);  // 100 kHz
    delay(100);

    Serial.println("Initializing I2C motor shields...");

    // Scan I2C bus
    scanI2CBus();

    Serial.println("\nMotor Shield Configuration:");
    Serial.println("  Shield 1 (0x2F): Motors 1, 2");
    Serial.println("  Shield 2 (0x30): Motors 3, 4");
    Serial.println("  Shield 3 (0x2E): Motors 5, 6");

    Serial.println("\nEncoder Pin Assignments:");
    Serial.println("  Motor 1: Pins 0, 1   (Hardware QuadTimer)");
    Serial.println("  Motor 2: Pins 3, 2   (Hardware QuadTimer - swapped)");
    Serial.println("  Motor 3: Pins 6, 5   (Hardware QuadTimer - swapped)");
    Serial.println("  Motor 4: Pins 7, 8   (Hardware QuadTimer)");
    Serial.println("  Motor 5: Pins 9, 10  (Hardware QuadTimer)");
    Serial.println("  Motor 6: Pins 11, 12 (Software interrupts)");

    // Ensure all motors stopped
    stopAllMotors();
    resetAllEncoders();

    Serial.println("\n\n");
    Serial.println("════════════════════════════════════════════");
    Serial.println("  READY TO BEGIN TESTING");
    Serial.println("════════════════════════════════════════════");
    Serial.println();
    Serial.println("This test will spin each motor individually");
    Serial.println("and show which encoder responds.");
    Serial.println();
    Serial.println("Press ENTER to start...");

    // Wait for user
    while (!Serial.available()) {
        delay(10);
    }
    while (Serial.available()) {
        Serial.read();
    }
}

// ============================================================================
// MAIN LOOP
// ============================================================================

void loop() {
    // Test Motor 1 (Shield 1, Motor A)
    testMotor("MOTOR 1", Shield1_MotorA, "0x2F", "A");

    // Test Motor 2 (Shield 1, Motor B)
    testMotor("MOTOR 2", Shield1_MotorB, "0x2F", "B");

    // Test Motor 3 (Shield 2, Motor A)
    testMotor("MOTOR 3", Shield2_MotorA, "0x30", "A");

    // Test Motor 4 (Shield 2, Motor B)
    testMotor("MOTOR 4", Shield2_MotorB, "0x30", "B");

    // Test Motor 5 (Shield 3, Motor A)
    testMotor("MOTOR 5", Shield3_MotorA, "0x2E", "A");

    // Test Motor 6 (Shield 3, Motor B)
    testMotor("MOTOR 6", Shield3_MotorB, "0x2E", "B");

    // All tests complete
    Serial.println("\n\n");
    Serial.println("╔════════════════════════════════════════════╗");
    Serial.println("║          ALL TESTS COMPLETE!               ║");
    Serial.println("╚════════════════════════════════════════════╝");
    Serial.println();
    Serial.println("Summary of expected results:");
    Serial.println("  Motor 1 → Encoder [0-1]   should increase");
    Serial.println("  Motor 2 → Encoder [3-2]   should increase");
    Serial.println("  Motor 3 → Encoder [6-5]   should increase");
    Serial.println("  Motor 4 → Encoder [7-8]   should increase");
    Serial.println("  Motor 5 → Encoder [9-10]  should increase");
    Serial.println("  Motor 6 → Encoder [11-12] should increase");
    Serial.println();
    Serial.println("If any encoder shows negative counts,");
    Serial.println("swap the pins in code for that encoder.");
    Serial.println();
    Serial.println("Press ENTER to run tests again...");

    // Wait for user
    while (!Serial.available()) {
        delay(10);
    }
    while (Serial.available()) {
        Serial.read();
    }

    stopAllMotors();
    resetAllEncoders();
}
