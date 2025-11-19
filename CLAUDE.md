# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project Overview

Dual pipe crawler robot firmware for Teensy 4.0 controlling 6 DC motors (two independent 3-wheeled crawlers) via 3 HW-648 motor shields over I2C. Features closed-loop PID velocity control with quadrature encoder feedback and per-crawler position synchronization.

**Current Status (November 17, 2025):** Fully operational dual crawler system with crawler selection feature, all motors verified with correct direction and encoder polarity.

## Hardware Configuration

### Motor Shields (HW-648 with STM32F030)
- **Shield 1 (0x2F):** Motors 1-2 - AD0 soldered, AD1 open
- **Shield 2 (0x30):** Motors 3-4 - Both pads open
- **Shield 3 (0x2E):** Motors 5-6 - AD0 open, AD1 soldered

**I2C:** Teensy Pin 18 (SDA), Pin 19 (SCL) to all shields

### Crawler Assignment
- **Crawler 1:** Motors 1-3 (Shields 0x2F + 0x30)
- **Crawler 2:** Motors 4-6 (Shields 0x30 + 0x2E)

### Encoder Pin Mappings (Verified November 17, 2025)

All encoders connect directly to Teensy GPIO (hardware QuadTimer support):

| Motor | Shield | Channel | Encoder Pins | Polarity Correction |
|-------|--------|---------|--------------|---------------------|
| 1 | 0x2F | A | 0, 1 | None (positive) |
| 2 | 0x2F | B | 3, 2 | Pins swapped in code |
| 3 | 0x30 | A | 6, 5 | None (positive) |
| 4 | 0x30 | B | 9, 10 | Motor direction reversed |
| 5 | 0x2E | A | 11, 12 | None (positive) |
| 6 | 0x2E | B | 7, 8 | Motor direction reversed |

See `MOTOR_ENCODER_MAPPING.md` for complete wiring details and test results.

## Build and Development

### PlatformIO Commands

```bash
# Build firmware
pio run

# Upload to Teensy
pio run --target upload

# Monitor serial output (115200 baud)
pio device monitor

# Build and upload in one step
pio run --target upload && pio device monitor
```

### Project Structure

```
hw648-firmware/
├── src/
│   ├── main.cpp              # Main firmware entry point
│   ├── config.h              # All tunable parameters (PID, pins, speeds)
│   ├── RobotMotion.{h,cpp}   # High-level dual crawler coordinator
│   ├── MotorController.{h,cpp} # Single motor PID control
│   ├── CommandParser.{h,cpp} # Serial command interface
│   ├── PIDController.{h,cpp} # PID implementation
│   └── SpeedRamp.{h,cpp}     # Smooth acceleration ramping
├── tools/
│   ├── main_6motor_diagnostics.cpp  # Test all 6 motors individually
│   ├── main_encoder_diagnostics.cpp # Original 3-motor diagnostic
│   └── README.md             # How to use diagnostic tools
├── platformio.ini            # PlatformIO configuration
├── MOTOR_ENCODER_MAPPING.md  # Complete hardware documentation
└── README.md                 # Hardware setup guide
```

### Architecture Overview

**Control Hierarchy:**
1. `main.cpp` - Setup, main loop, shield detection
2. `RobotMotion` - Dual crawler coordinator, position sync, watchdog
3. `MotorController` - Individual motor PID control (6 instances)
4. `PIDController` - Low-level PID math (6 instances)
5. `SpeedRamp` - Smooth velocity transitions
6. `CommandParser` - Serial command processing

**Control Loop:** Runs at 50 Hz (20ms period), calculates velocity from encoders, runs PID, applies PWM commands to motors.

**Position Synchronization:** Each crawler independently syncs its 3 motors to prevent tilting. Motors track average position and apply proportional velocity corrections.

### Configuration Changes

**All tunable parameters are in `src/config.h`:**
- Motor I2C addresses and channels
- Encoder pin assignments
- PID gains (Kp, Ki, Kd) for each motor
- Speed limits, ramp times, max velocity
- Position sync parameters
- Control loop frequency
- Debug flags

When tuning PID or changing behavior, edit `config.h` first before modifying code.

## Serial Command Interface

**Baud rate:** 115200

**Crawler Selection (Persistent):**
```
SELECT 1       # Control only Crawler 1 (motors 1-3)
SELECT 2       # Control only Crawler 2 (motors 4-6)
SELECT BOTH    # Control both crawlers (default)
```

**Movement Commands (Apply to Selected Crawler):**
```
FORWARD <speed>    # Move forward (0-100%)
BACKWARD <speed>   # Move backward (0-100%)
STOP               # Gradual stop with ramping
ESTOP              # Emergency stop (immediate)
```

**Individual Motor Control (Bypass Selection):**
```
M1 FWD 50     # Motor 1 forward at 50%
M2 BACK 30    # Motor 2 backward at 30%
M3-M6         # Same syntax
```

**Status and Utility:**
```
STATUS        # Display detailed system status
RESET         # Reset all encoder positions to zero
HELP          # Show all commands
```

## HW-648 Shield Firmware

The motor shields require custom wemos_motor_shield firmware for I2C motor control and AD0/AD1 address configuration.

### Building Shield Firmware

```bash
cd stm32flash-0.7-binaries/wemos_motor_shield-master/wemos_motor_shield-master/
make clean
make
# Output: motor_shield.bin (3008 bytes)
```

**Prerequisites:** `sudo apt-get install gcc-arm-none-eabi make`

### Flashing Shields (from WSL)

```bash
# 1. Connect shield to FTDI (TX, RX, GND, VCC)
# 2. Hold BOOT0 high during power-up
# 3. Flash (use Windows .exe from WSL):
/mnt/c/path/to/stm32flash.exe -w motor_shield.bin -v -g 0x0 COM7

# 4. Remove BOOT0 and power cycle
```

**Critical:** Use the wemos_motor_shield firmware, not generic STM32 firmware. Generic firmware will not respond on I2C.

## Diagnostic Tools

### 6-Motor Diagnostic Tool

Tests all motors individually with real-time encoder feedback and I2C scanner.

```bash
# Use diagnostic firmware
cp tools/main_6motor_diagnostics.cpp src/main.cpp
pio run --target upload
pio device monitor

# During test: Press ENTER to cycle through motors
# During test: Type "SCAN" + ENTER for I2C bus scan

# Restore control firmware
cp tools/main_control.cpp src/main.cpp  # or restore from backup
pio run --target upload
```

**Features:**
- Sequential motor testing at 30% speed for 5 seconds each
- Live encoder readings for all 6 motors
- I2C address scanner (identifies which shields are present)
- Polarity detection (positive/negative encoder counts)

## Common Issues and Solutions

### Shield Not Detected on I2C

**Symptom:** Missing shield during detection scan

**Possible Causes:**
1. **Wrong firmware** - Most common issue
   - Solution: Flash correct wemos_motor_shield firmware
2. **Incorrect I2C address** - Wrong solder pads configured
   - Diagnostic: Use "SCAN" command in diagnostic firmware
   - Solution: Check/fix AD0/AD1 solder pads with continuity tester
3. **Wiring issue** - SDA/SCL not connected
   - Solution: Verify pins 18/19 connected to all shields

**Expected addresses:**
- 0x2E: AD0=open, AD1=soldered → Shield 3
- 0x2F: AD0=soldered, AD1=open → Shield 1
- 0x30: AD0=open, AD1=open → Shield 2

### Motor Spins Wrong Direction

**Symptom:** Motor moves backward when commanded forward

**Solution:** Edit `src/config.h` and swap `_CW` ↔ `_CCW` for that motor's direction defines:
```cpp
#define MOTOR4_FWD_DIR      _CCW    // Was _CW
#define MOTOR4_BACK_DIR     _CW     // Was _CCW
```

### Encoder Counts Negative

**Symptom:** Encoder counts decrease when motor moves forward

**Solutions (pick one):**
1. **Swap pins in config.h** (preferred):
   ```cpp
   #define ENCODER2_PIN_A      3    // Was 2
   #define ENCODER2_PIN_B      2    // Was 3
   ```
2. **Negate in software:**
   ```cpp
   long pos = -encoder.read();
   ```

### Encoders Don't Count

**Diagnostics:**
1. Check encoder power: Should be 3.3V (measure blue wire to ground)
2. Check ground connection (brown wire)
3. Verify signal wires connected to correct Teensy pins (green=A, yellow=B)
4. Use diagnostic firmware to isolate which encoder

### Motors Not Synchronized / Robot Tilts

**Symptom:** Motors drift apart, robot tilts in pipe

**Solutions:**
1. Check `ENABLE_POSITION_SYNC` is `true` in config.h
2. Increase `POSITION_SYNC_KP` (e.g., 0.2 → 0.3) for stronger correction
3. Verify all encoder polarities are correct (all positive when moving forward)
4. Check STATUS output for position error values

**Expected:** Position error should be < 500 counts across all motors

## Development Workflow

### Typical Development Session

```bash
# 1. Make code changes (edit src/config.h or src/*.cpp)

# 2. Build and upload
pio run --target upload

# 3. Monitor serial output
pio device monitor
# Press Ctrl+C to exit monitor

# 4. Test with serial commands
# Type commands like: FORWARD 50, STATUS, STOP
```

### Adding a New Feature

1. **Plan:** Check if parameters need to be added to `config.h`
2. **Implement:**
   - For motion behavior: Edit `RobotMotion.cpp`
   - For commands: Edit `CommandParser.cpp`
   - For single motor: Edit `MotorController.cpp`
3. **Test:** Use diagnostic tools or individual motor commands first
4. **Document:** Update relevant sections in MOTOR_ENCODER_MAPPING.md

### Tuning PID Controllers

1. Start conservative: Kp=0.5, Ki=0.1, Kd=0.05
2. Test with `FORWARD 30` command (low speed)
3. Monitor STATUS output for velocity error
4. Adjust in `config.h`:
   - Increase Kp if motor responds too slowly
   - Increase Ki if steady-state error remains
   - Increase Kd if oscillating/overshooting
5. Rebuild and test: `pio run --target upload`

**Note:** Each motor has independent PID gains - tune individually if needed.

## Important Constraints

- **Do not use pins 18-19** - Reserved for I2C (motor shields)
- **Encoder pins 0-12** - All allocated to 6 motor encoders
- **Control loop timing** - Do not add delays in main loop (breaks PID)
- **I2C communication** - Motor shield library is blocking, keep calls minimal in control loop
- **Serial buffer** - Keep command length < 64 characters

## Git and Version Control

This is a git repository. The working firmware is in `src/main.cpp`. Old versions and diagnostic tools are preserved in `tools/`.

When making significant changes, backup the working main.cpp:
```bash
cp src/main.cpp tools/main.cpp.backup_$(date +%Y%m%d)
```
