#include <Arduino.h>
#include <Wire.h>
#include "WEMOS_Motor.h"

// Shield 1 at address 0x2F (AD0=soldered, AD1=open)
Motor Shield1_MotorA(0x2F, _MOTOR_A, 1000);
Motor Shield1_MotorB(0x2F, _MOTOR_B, 1000);

// Shield 2 at address 0x30 (AD0=open, AD1=open)
Motor Shield2_MotorA(0x30, _MOTOR_A, 1000);
Motor Shield2_MotorB(0x30, _MOTOR_B, 1000);

void setup() {
    Serial.begin(9600);
    while (!Serial);

    Serial.println("=== Dual HW-648 Motor Shield Test ===");
    Serial.println("Shield 1: 0x2F");
    Serial.println("Shield 2: 0x30");
    Serial.println();
}

void loop() {
    // Test Shield 1 - Motor A
    Serial.println("Shield 1 Motor A - Clockwise");
    for (int i = 0; i <= 100; i++) {
        Shield1_MotorA.setmotor(_CW, i);
        delay(20);
    }
    delay(500);
    Shield1_MotorA.setmotor(_STOP);
    delay(500);

    // Test Shield 1 - Motor B
    Serial.println("Shield 1 Motor B - Counter-clockwise");
    for (int i = 0; i <= 100; i++) {
        Shield1_MotorB.setmotor(_CCW, i);
        delay(20);
    }
    delay(500);
    Shield1_MotorB.setmotor(_STOP);
    delay(500);

    // Test Shield 2 - Motor A
    Serial.println("Shield 2 Motor A - Clockwise");
    for (int i = 0; i <= 100; i++) {
        Shield2_MotorA.setmotor(_CW, i);
        delay(20);
    }
    delay(500);
    Shield2_MotorA.setmotor(_STOP);
    delay(500);

    // Test Shield 2 - Motor B
    Serial.println("Shield 2 Motor B - Counter-clockwise");
    for (int i = 0; i <= 100; i++) {
        Shield2_MotorB.setmotor(_CCW, i);
        delay(20);
    }
    delay(500);
    Shield2_MotorB.setmotor(_STOP);
    delay(1000);

    // All motors together
    Serial.println("All motors forward!");
    for (int i = 0; i <= 50; i++) {
        Shield1_MotorA.setmotor(_CW, i);
        Shield1_MotorB.setmotor(_CW, i);
        Shield2_MotorA.setmotor(_CW, i);
        Shield2_MotorB.setmotor(_CW, i);
        delay(20);
    }
    delay(1000);

    // Stop all
    Serial.println("Stop all motors");
    Shield1_MotorA.setmotor(_STOP);
    Shield1_MotorB.setmotor(_STOP);
    Shield2_MotorA.setmotor(_STOP);
    Shield2_MotorB.setmotor(_STOP);

    delay(2000);
    Serial.println();
}
