# HW-648 Motor Shield Setup Guide for Teensy 4.0

This document describes the complete setup process for interfacing two HW-648 motor shields with a Teensy 4.0 using I2C communication.

## Hardware Configuration

### I2C Wiring
- **Teensy Pin 18** (SDA) ← D2 on both HW-648 shields
- **Teensy Pin 19** (SCL) ← D1 on both HW-648 shields
- **Note**: The D1/D2 labels on HW-648 may appear reversed from standard I2C conventions, but this configuration works correctly.

### I2C Addresses
Both shields are configured with different I2C addresses using solder pads:

**Shield 1:**
- Address: **0x2F (47 decimal)**
- Configuration: AD0 soldered, AD1 open

**Shield 2:**
- Address: **0x30 (48 decimal)**
- Configuration: AD0 open, AD1 open

### Address Selection Table
The STM32 firmware calculates addresses based on GPIO pins PF0 (AD0) and PF1 (AD1):

| AD0 | AD1 | I2C Address | Decimal |
|-----|-----|-------------|---------|
| LOW | LOW | 0x2D        | 45      |
| HIGH| LOW | 0x2E        | 46      |
| LOW | HIGH| 0x2F        | 47      |
| HIGH| HIGH| 0x30        | 48      |

Solder pads connect to **ground** (LOW), open pads have **pull-ups** (HIGH).

## Firmware Building and Flashing

### Why Rebuild Firmware?
The prebuilt binary (`motor_shield.bin`) did not support address configuration via AD0/AD1 pads. Building from source was necessary to enable this feature.

### Build Requirements
- **ARM GCC Toolchain**: `gcc-arm-none-eabi`
- **Make**: Standard build tool
- **stm32flash**: For flashing via serial bootloader

### Installation (Ubuntu/WSL)
```bash
sudo apt-get update
sudo apt-get install -y gcc-arm-none-eabi make
```

### Building the Firmware
```bash
cd stm32flash-0.7-binaries/wemos_motor_shield-master/wemos_motor_shield-master
make clean
make
```

**Output**: `motor_shield.bin` (3008 bytes)

### Flashing Process (Windows COM Port via WSL)

**For each shield:**

1. Connect shield to FTDI USB adapter (TX, RX, GND, VCC)
2. Put shield in bootloader mode (BOOT0 button/jumper to high during power-up)
3. Flash firmware (assuming FTDI is on COM7):

```bash
/mnt/c/path/to/stm32flash.exe -w motor_shield.bin -v -g 0x0 COM7
```

**Note**: COM7 in Windows = `/dev/ttyS6` in WSL, but the Windows executable works better.

4. Remove BOOT0 connection and power cycle the shield

## PlatformIO Project Setup

### platformio.ini
```ini
[env:teensy40]
platform = teensy
board = teensy40
framework = arduino
upload_protocol = teensy-cli
build_flags = -D USB_SERIAL
lib_deps =
    https://github.com/thomasfredericks/wemos_motor_shield.git
```

### Motor Control Library
Uses the WEMOS Motor Shield library which provides:
- `Motor` class for DC motor control
- Configurable I2C addresses
- PWM speed control (0-100)
- Direction control: `_CW`, `_CCW`, `_STOP`, `_STANDBY`, `_SHORT_BRAKE`

## Example Code

### Basic Motor Test
```cpp
#include <Arduino.h>
#include <Wire.h>
#include "WEMOS_Motor.h"

// Shield 1 at address 0x2F
Motor Shield1_MotorA(0x2F, _MOTOR_A, 1000);
Motor Shield1_MotorB(0x2F, _MOTOR_B, 1000);

// Shield 2 at address 0x30
Motor Shield2_MotorA(0x30, _MOTOR_A, 1000);
Motor Shield2_MotorB(0x30, _MOTOR_B, 1000);

void setup() {
    Serial.begin(9600);
}

void loop() {
    // Smooth acceleration
    for (int i = 0; i <= 100; i++) {
        Shield1_MotorA.setmotor(_CW, i);
        delay(20);
    }
    delay(500);
    Shield1_MotorA.setmotor(_STOP);
    delay(500);
}
```

## Quadrature Encoder Integration

### Recommendation
Connect encoders **directly to Teensy GPIO pins**, not to the HW-648 shields.

**Advantages:**
- Hardware quadrature decoder support on Teensy 4.0
- No I2C latency
- High-resolution counting
- Fast interrupt handling
- Easy integration with `Encoder` library

**Suggested Pin Mapping:**
```
Shield 1 Motor A: Pin 2 (A), Pin 3 (B)
Shield 1 Motor B: Pin 4 (A), Pin 5 (B)
Shield 2 Motor A: Pin 6 (A), Pin 7 (B)
Shield 2 Motor B: Pin 8 (A), Pin 9 (B)
```

All these pins support hardware interrupts and work with the Teensy Encoder library.

## Troubleshooting

### Only One Shield Detected
- **Cause**: Both shields configured with same I2C address
- **Solution**: Verify AD0/AD1 solder pad configuration, ensure fresh firmware is flashed

### I2C Timeout Errors During Scanning
- **Symptom**: Occasional "Timed out" messages at low addresses
- **Impact**: Cosmetic only, doesn't affect motor control
- **Solution (if problematic)**: Add 2.2kΩ - 4.7kΩ pull-up resistors on SDA/SCL to 3.3V

### Shield Not Responding After Firmware Flash
- **Cause**: Still in bootloader mode
- **Solution**: Remove BOOT0 connection and power cycle the shield

### Windows COM Port Access from WSL
- **Issue**: Linux stm32flash doesn't work with `/dev/ttySx` in WSL
- **Solution**: Use Windows `.exe` version directly from WSL: `stm32flash.exe COM7`

## Firmware Source Code Reference

The address configuration is in `main.c` line 77:
```c
#define I2C_BASE_ADDR 0x2D
...
I2C1->OAR1 = I2C_OAR1_OA1EN | ((I2C_BASE_ADDR + (GPIOF->IDR & 3)) << 1);
```

This reads PF0 and PF1 GPIO pins and adds the 2-bit value (0-3) to the base address.

## Summary

**Working Configuration:**
- ✅ Teensy 4.0 I2C master
- ✅ Two HW-648 shields at addresses 0x2F and 0x30
- ✅ Fresh firmware with address configuration support
- ✅ Smooth motor control via I2C
- ✅ All 4 motors independently controllable
- ✅ Encoder support via Teensy GPIO (recommended)

**Date**: November 2, 2025
**Tested with**: Teensy 4.0, PlatformIO, WSL2
