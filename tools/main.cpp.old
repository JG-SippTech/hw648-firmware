#include <Arduino.h>
#include <Wire.h>
#include "WEMOS_Motor.h"
#include <Encoder.h>

// Motor shield objects
Motor Shield1_MotorA(0x2F, _MOTOR_A, 1000);
Motor Shield1_MotorB(0x2F, _MOTOR_B, 1000);
Motor Shield2_MotorA(0x30, _MOTOR_A, 1000);

// Encoder objects (all three pairs)
Encoder encoder_01(0, 1);   // Encoder on pins 0-1
Encoder encoder_23(2, 3);   // Encoder on pins 2-3
Encoder encoder_56(5, 6);   // Encoder on pins 5-6

// Test state
int currentTest = 0;
unsigned long lastPrintTime = 0;
unsigned long testStartTime = 0;
const int TEST_DURATION = 5000; // 5 seconds per test
const int MOTOR_SPEED = 30;     // 30% speed for safe testing

void printEncoderReadings() {
    long pos01 = encoder_01.read();
    long pos23 = encoder_23.read();
    long pos56 = encoder_56.read();

    Serial.print("Encoders -> [0-1]: ");
    Serial.print(pos01);
    Serial.print("  |  [2-3]: ");
    Serial.print(pos23);
    Serial.print("  |  [5-6]: ");
    Serial.println(pos56);
}

void resetAllEncoders() {
    encoder_01.write(0);
    encoder_23.write(0);
    encoder_56.write(0);
}

void stopAllMotors() {
    Shield1_MotorA.setmotor(_STOP);
    Shield1_MotorB.setmotor(_STOP);
    Shield2_MotorA.setmotor(_STOP);
}

void setup() {
    Serial.begin(115200);
    while (!Serial && millis() < 3000); // Wait up to 3 seconds for serial

    delay(1000); // Give everything time to stabilize

    Serial.println("\n\n╔════════════════════════════════════════════════════════╗");
    Serial.println("║   ENCODER IDENTIFICATION TEST - Pipe Crawler Robot    ║");
    Serial.println("╚════════════════════════════════════════════════════════╝");
    Serial.println();
    Serial.println("This test will spin each motor one at a time.");
    Serial.println("Watch which encoder count changes to identify the pairing!");
    Serial.println();
    Serial.println("Wire color scheme:");
    Serial.println("  Red    = Motor A");
    Serial.println("  Black  = Motor B");
    Serial.println("  Blue   = Encoder V+ (3.3V)");
    Serial.println("  Brown  = Encoder V- (GND)");
    Serial.println("  Green  = Encoder A");
    Serial.println("  Yellow = Encoder B");
    Serial.println();
    Serial.println("Press ENTER to start the test...");

    // Wait for user input
    while (!Serial.available()) {
        delay(100);
    }
    while (Serial.available()) Serial.read(); // Clear buffer

    Serial.println("\nStarting test sequence in 2 seconds...\n");
    delay(2000);

    testStartTime = millis();
    currentTest = 1;
}

void loop() {
    unsigned long currentTime = millis();

    // Print encoder readings every 200ms
    if (currentTime - lastPrintTime >= 200) {
        lastPrintTime = currentTime;
        printEncoderReadings();
    }

    // Test sequence
    switch (currentTest) {
        case 0:
            // Idle - waiting to start
            break;

        case 1:
            // Test Shield 1, Motor A
            if (currentTime - testStartTime < TEST_DURATION) {
                if (currentTime - testStartTime == 0) {
                    Serial.println("\n╔═══════════════════════════════════════╗");
                    Serial.println("║  TEST 1: Shield 1, Motor A (0x2F-A)  ║");
                    Serial.println("║  Motor Color: RED                     ║");
                    Serial.println("╚═══════════════════════════════════════╝");
                    Serial.println("Spinning clockwise at 30% speed...\n");
                    resetAllEncoders();
                    Shield1_MotorA.setmotor(_CW, MOTOR_SPEED);
                }
            } else {
                stopAllMotors();
                delay(500);
                Serial.println("\n>>> RESULT: The encoder that changed belongs to Shield1_MotorA (RED)\n");
                Serial.println("Press ENTER for next test...");
                while (!Serial.available()) delay(100);
                while (Serial.available()) Serial.read();
                testStartTime = millis();
                currentTest = 2;
            }
            break;

        case 2:
            // Test Shield 1, Motor B
            if (currentTime - testStartTime < TEST_DURATION) {
                if (currentTime - testStartTime == 0) {
                    Serial.println("\n╔═══════════════════════════════════════╗");
                    Serial.println("║  TEST 2: Shield 1, Motor B (0x2F-B)  ║");
                    Serial.println("║  Motor Color: BLACK                   ║");
                    Serial.println("╚═══════════════════════════════════════╝");
                    Serial.println("Spinning clockwise at 30% speed...\n");
                    resetAllEncoders();
                    Shield1_MotorB.setmotor(_CW, MOTOR_SPEED);
                }
            } else {
                stopAllMotors();
                delay(500);
                Serial.println("\n>>> RESULT: The encoder that changed belongs to Shield1_MotorB (BLACK)\n");
                Serial.println("Press ENTER for next test...");
                while (!Serial.available()) delay(100);
                while (Serial.available()) Serial.read();
                testStartTime = millis();
                currentTest = 3;
            }
            break;

        case 3:
            // Test Shield 2, Motor A
            if (currentTime - testStartTime < TEST_DURATION) {
                if (currentTime - testStartTime == 0) {
                    Serial.println("\n╔═══════════════════════════════════════╗");
                    Serial.println("║  TEST 3: Shield 2, Motor A (0x30-A)  ║");
                    Serial.println("║  Motor Color: (3rd motor wire)        ║");
                    Serial.println("╚═══════════════════════════════════════╝");
                    Serial.println("Spinning clockwise at 30% speed...\n");
                    resetAllEncoders();
                    Shield2_MotorA.setmotor(_CW, MOTOR_SPEED);
                }
            } else {
                stopAllMotors();
                delay(500);
                Serial.println("\n>>> RESULT: The encoder that changed belongs to Shield2_MotorA\n");
                Serial.println("\n╔════════════════════════════════════════════════════════╗");
                Serial.println("║              TEST SEQUENCE COMPLETE!                   ║");
                Serial.println("╚════════════════════════════════════════════════════════╝");
                Serial.println("\nYou should now know which encoder pins go with which motor.");
                Serial.println("Write down your findings and update your code accordingly!");
                Serial.println("\nPress ENTER to run all tests again...");
                while (!Serial.available()) delay(100);
                while (Serial.available()) Serial.read();
                testStartTime = millis();
                currentTest = 1; // Restart from test 1
            }
            break;
    }
}
