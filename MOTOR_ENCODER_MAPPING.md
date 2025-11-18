# Motor-Encoder Pin Mapping

**Last Updated:** November 16, 2025
**Hardware:** Pipe Crawler Robot - 6 Motors with Quadrature Encoders
**MCU:** Teensy 4.0
**Motor Shields:** 3x HW-648 (STM32F030)

---

## Verified Motor-Encoder Assignments (6-Motor System)

| Motor # | I2C Shield | Motor Channel | Encoder Pins | Direction | QuadTimer | Status |
|---------|------------|---------------|--------------|-----------|-----------|--------|
| **Motor 1** | 0x2F | A | **0, 1** | ✅ Positive | QT3 | ✅ Working |
| **Motor 2** | 0x2F | B | **3, 2** | ✅ Positive | QT1 | ✅ Working (corrected) |
| **Motor 3** | 0x30 | A | **6, 5** | ✅ Positive | QT2 | ✅ Working |
| **Motor 4** | 0x30 | B | **9, 10** | ✅ Positive | QT4 | ✅ Working (motor reversed) |
| **Motor 5** | 0x2E | A | **11, 12** | ✅ Positive | QT4 | ✅ Working |
| **Motor 6** | 0x2E | B | **7, 8** | ✅ Positive | QT3 | ✅ Working (motor reversed) |

### Hardware Notes - November 16, 2025

**Shield 3 Configuration:**
- Initial issue: Shield 3 not detected on I2C bus
- Root cause: Wrong firmware (generic STM32 instead of wemos_motor_shield)
- Solution: Flashed correct wemos_motor_shield firmware
- Secondary issue: Shield appeared at 0x2D instead of 0x2E
- Root cause: Both AD0 and AD1 pads were soldered (should be AD1 only)
- Solution: Desoldered AD0 pad → Shield now correctly at 0x2E

**Motor 4 Issue - RESOLVED:**
- Initial issue: Motor did not spin during diagnostic test
- Root cause: Loose power plug connection
- Solution: Reconnected motor power plug
- Status: ✅ Now working with encoder on pins 9-10 (requires pin swap for positive polarity)

**Encoder Pin Wiring (Actual vs. Planned):**
- Motor 4: Connected to pins 9-10 (as planned) ✅
- Motor 5: Connected to pins 11-12 (planned for 9-10) - Uses QuadTimer 4
- Motor 6: Connected to pins 7-8 (planned for 11-12) - Uses QuadTimer 3
- All 6 encoders functional with hardware QuadTimer support

**Polarity Corrections Applied (November 17, 2025):**
- Motor 2: Encoder pins corrected to (3,2) ✅
- Motor 4: Motor direction reversed (_CW↔_CCW), encoder pins (9,10) ✅
- Motor 6: Motor direction reversed (_CW↔_CCW), encoder pins (7,8) ✅
- All motors now show correct polarity (negative when moving backward)

---

## Wiring Details

### Wire Color Scheme (Used During Assembly):
- **Red** = Motor A power
- **Black** = Motor B power
- **Blue** = Encoder V+ (3.3V)
- **Brown** = Encoder V- (GND)
- **Green** = Encoder Phase A
- **Yellow** = Encoder Phase B

### Power Connections:
- All encoders powered from **Teensy 3.3V** (not 5V - compatible with Teensy GPIO)
- Common ground between Teensy and encoders

---

## Code Implementation

### 6-Motor Encoder Setup (Corrected Polarity - November 17, 2025):
```cpp
#include <Encoder.h>
#include "WEMOS_Motor.h"

// Motor shield objects (3 shields, 6 motors)
Motor Shield1_MotorA(0x2F, _MOTOR_A, 1000);  // Motor 1
Motor Shield1_MotorB(0x2F, _MOTOR_B, 1000);  // Motor 2
Motor Shield2_MotorA(0x30, _MOTOR_A, 1000);  // Motor 3
Motor Shield2_MotorB(0x30, _MOTOR_B, 1000);  // Motor 4 - Direction reversed in config
Motor Shield3_MotorA(0x2E, _MOTOR_A, 1000);  // Motor 5
Motor Shield3_MotorB(0x2E, _MOTOR_B, 1000);  // Motor 6 - Direction reversed in config

// Encoder objects (with corrected polarity - all positive)
Encoder encoder_01(0, 1);      // Motor 1 - Positive polarity
Encoder encoder_32(3, 2);      // Motor 2 - SWAPPED for correct polarity
Encoder encoder_65(6, 5);      // Motor 3 - Positive polarity
Encoder encoder_910(9, 10);    // Motor 4 - Positive polarity (motor reversed)
Encoder encoder_1112(11, 12);  // Motor 5 - Positive polarity
Encoder encoder_78(7, 8);      // Motor 6 - Positive polarity (motor reversed)
```

**Note:**
- Motor 2: Encoder pins swapped to (3,2) for correct polarity
- Motors 4 & 6: Motor direction reversed in config.h (_CW↔_CCW), encoder pins correct
- All motors now show negative counts when moving backward, positive when moving forward

### 6-Motor Encoder Setup (As-Wired, Before Correction):
```cpp
// Encoder objects showing actual physical wiring (some with negative polarity)
Encoder encoder_01(0, 1);      // Motor 1 - Positive
Encoder encoder_32(3, 2);      // Motor 2 - Negative (needs swap)
Encoder encoder_65(6, 5);      // Motor 3 - Positive
Encoder encoder_109(10, 9);    // Motor 4 - Negative (needs swap)
Encoder encoder_1112(11, 12);  // Motor 5 - Positive
Encoder encoder_87(8, 7);      // Motor 6 - Negative (needs swap)
```

### 3-Motor Legacy Setup:
```cpp
// Original 3-motor configuration (Motors 1-3)
Motor motor1(0x2F, _MOTOR_A, 1000);  // Motor 1 - Shield 1, Channel A
Motor motor2(0x2F, _MOTOR_B, 1000);  // Motor 2 - Shield 1, Channel B
Motor motor3(0x30, _MOTOR_A, 1000);  // Motor 3 - Shield 2, Channel A

Encoder encoder1(0, 1);   // Motor 1 - Positive direction
Encoder encoder2(3, 2);   // Motor 2 - Swapped for correct polarity
Encoder encoder3(6, 5);   // Motor 3 - Swapped for correct polarity
```

### Alternative: Software Correction
If you prefer to keep hardware pins as-is and correct in software:
```cpp
Encoder encoder2(2, 3);  // Hardware order
Encoder encoder6(8, 7);  // Hardware order

// In your code:
long pos2 = -encoder2.read();  // Negate the reading
long pos6 = -encoder6.read();  // Negate the reading
```

---

## Hardware Quadrature Decoder Channels

All six encoders use **hardware quadrature decoding** on Teensy 4.0:

| Encoder | Pins (Corrected) | QuadTimer Channel | Performance | Status |
|---------|------------------|-------------------|-------------|--------|
| Motor 1 | 0, 1 | QuadTimer 3 | Hardware | ✅ Working |
| Motor 2 | 2, 3 | QuadTimer 1 | Hardware | ✅ Working |
| Motor 3 | 6, 5 | QuadTimer 2 | Hardware | ✅ Working |
| Motor 4 | 9, 10 | QuadTimer 4 | Hardware | ✅ Working |
| Motor 5 | 11, 12 | QuadTimer 4 | Hardware | ✅ Working |
| Motor 6 | 7, 8 | QuadTimer 3 | Hardware | ✅ Working |

**Benefits:**
- ✅ No CPU interrupts needed
- ✅ High-speed counting (up to 150 MHz)
- ✅ No missed counts at high RPM
- ✅ Low CPU overhead

**Note:** Pins 7-8 and 11-12 both support hardware quadrature decoding on Teensy 4.0, making them ideal for high-speed motor control.

---

## Testing Results

### November 16, 2025 - 6-Motor System Test (COMPLETE)

**Test Method:** Each motor spun individually at 30% speed for 5 seconds while monitoring all encoder counts.

**Final Test Results (All Motors Working):**

| Motor | Shield | Channel | Encoder Pins | Final Count | Count Rate | Polarity |
|-------|--------|---------|--------------|-------------|------------|----------|
| Motor 1 | 0x2F | A | [0-1] | +7,236 | ~1,447/sec | ✅ Positive |
| Motor 2 | 0x2F | B | [3-2] | -6,881 | ~1,376/sec | ⚠️ Negative |
| Motor 3 | 0x30 | A | [6-5] | +7,124 | ~1,425/sec | ✅ Positive |
| Motor 4 | 0x30 | B | [9-10] | -6,726 | ~1,345/sec | ⚠️ Negative |
| Motor 5 | 0x2E | A | [11-12] | +7,417 | ~1,483/sec | ✅ Positive |
| Motor 6 | 0x2E | B | [7-8] | -7,067 | ~1,413/sec | ⚠️ Negative |

**Interpretation:**
- ✅ All 6 motors working with encoder feedback
- ✅ Motor 4 fixed (loose power plug)
- ⚠️ Motors 2, 4, and 6 have negative polarity (swap encoder pins in code)
- ✅ All motors achieve similar speeds (~1,400 counts/sec at 30% PWM)
- ✅ Motor 4 encoder on pins 9-10 (as originally planned)
- ⚠️ Motors 5 and 6 on different pins than planned (11-12 and 7-8)
- ✅ All encoders using hardware QuadTimer channels (zero CPU overhead)

### November 9, 2025 - Original 3-Motor Test

**Results:**
- **Motor 1**: Encoder [0-1] counted positive (~1200 counts in 5 sec)
- **Motor 2**: Encoder [2-3] counted positive (~1150 counts in 5 sec)
- **Motor 3**: Encoder [5-6] counted **negative** (~-1180 counts in 5 sec)

**Interpretation:**
- All three encoders working correctly
- Motor 3 encoder wired in reverse polarity (corrected in code)
- Approximate encoder resolution: ~240 counts/second at 30% speed

---

## System Expansion - ✅ COMPLETE (November 16, 2025)

**Successfully expanded to 6-motor system with third HW-648 shield:**
- ✅ Shield 3 added at I2C address 0x2E (AD1 soldered)
- ✅ All 6 motors operational with encoder feedback
- ✅ Motors 4, 5, 6 added (Shield 2 Motor B, Shield 3 Motors A & B)
- ✅ All encoder pins utilize hardware QuadTimer support
- ✅ Motor 4 power connection issue resolved

**Actual Encoder Pin Assignments:**
- Motor 1: Pins 0, 1 (QuadTimer 3) ✅
- Motor 2: Pins 2, 3 (QuadTimer 1) ✅
- Motor 3: Pins 6, 5 (QuadTimer 2) ✅
- Motor 4: Pins 9, 10 (QuadTimer 4) ✅
- Motor 5: Pins 11, 12 (QuadTimer 4) ✅
- Motor 6: Pins 7, 8 (QuadTimer 3) ✅

**Reserved/Unavailable Pins:**
- 18, 19 (I2C - SDA/SCL) - In use by motor shields
- 13 (LED) - Avoid for encoder use
- 0-12 (All allocated to motor encoders)

**System Status:**
- All 6 motors operational
- All 6 encoders providing hardware quadrature feedback
- 3 HW-648 shields on I2C bus (0x2E, 0x2F, 0x30)
- Zero CPU overhead for encoder counting
- Ready for closed-loop velocity control implementation

---

## Diagnostic Tools

### 6-Motor Diagnostic Firmware
Located in `tools/main_6motor_diagnostics.cpp`, this tool sequentially tests all 6 motors and displays real-time encoder feedback.

**Features:**
- Tests each motor individually at 30% speed for 5 seconds
- Displays all 6 encoder readings simultaneously
- Built-in I2C scanner (type "SCAN" during any motor test)
- Identifies motor-encoder pairings
- Detects polarity issues

**Usage:**
```bash
# Copy diagnostic firmware to main
cp tools/main_6motor_diagnostics.cpp src/main.cpp
pio run --target upload
pio device monitor

# During test: Press ENTER to advance to next motor
# During test: Type "SCAN" + ENTER to run I2C bus scan
```

**I2C Scanner Output Example:**
```
  0x2E (46) - ✓ FOUND - Shield 3 (AD1 soldered)
  0x2F (47) - ✓ FOUND - Shield 1 (AD0 soldered)
  0x30 (48) - ✓ FOUND - Shield 2 (no pads)
```

### Restoring Control Firmware
```bash
# Restore main control firmware
cp tools/main_control.cpp src/main.cpp
pio run --target upload
```

---

## Troubleshooting Notes

### If motor shield not detected on I2C:
1. **Check firmware**: Ensure wemos_motor_shield firmware is flashed (not generic STM32 firmware)
2. **Verify address pads**: Use continuity tester to confirm AD0/AD1 solder state
3. **Run I2C scan**: Type "SCAN" in diagnostic firmware to identify active addresses
4. **Expected addresses**:
   - 0x2E: AD0=open, AD1=soldered
   - 0x2F: AD0=soldered, AD1=open
   - 0x30: AD0=open, AD1=open
   - 0x2D: Both pads soldered (incorrect for 3-shield setup)

### If encoders don't count:
1. Check encoder power: Should be exactly 3.3V (measure Blue wire)
2. Check ground connection (Brown wire)
3. Verify signal wires connected to correct pins (Green=A, Yellow=B)
4. Use diagnostic firmware to test each motor individually

### If counts are erratic:
1. Check for loose connections
2. Ensure encoder wires aren't too close to motor power wires (EMI)
3. Add 0.1µF capacitors across encoder power if needed

### If direction is wrong:
1. Swap encoder pins in code: `Encoder(B, A)` instead of `Encoder(A, B)`
2. Or negate reading in software: `pos = -encoder.read()`

### If motor spins but no encoder response:
1. Check encoder power with multimeter (should be 3.3V)
2. Verify encoder pins are actually connected to Teensy
3. Try different encoder pins to isolate hardware vs. wiring issue
4. Swap encoder with known-working motor to test encoder functionality

---

## References

- **Encoder Library**: https://github.com/PaulStoffregen/Encoder
- **PJRC Teensy 4.0**: https://www.pjrc.com/store/teensy40.html
- **WEMOS Motor Shield Library**: https://github.com/thomasfredericks/wemos_motor_shield
- **Test Firmware**:
  - 6-motor diagnostic: `hw648-firmware/tools/main_6motor_diagnostics.cpp`
  - 3-motor control: `hw648-firmware/tools/main_control.cpp`
  - Original 3-motor test: `hw648-firmware/tools/main_encoder_diagnostics.cpp`

## Firmware Change Log

### November 17, 2025 - Crawler Selection & Direction Fixes ✅
- ✅ **IMPLEMENTED CRAWLER SELECTION FEATURE:**
  - New SELECT command (SELECT 1, SELECT 2, SELECT BOTH)
  - Persistent crawler selection throughout session
  - Movement commands (FORWARD, BACKWARD, STOP, ESTOP) respect selection
  - Individual motor commands (M1-M6) always work regardless of selection
  - Enhanced STATUS display shows current crawler selection
  - Updated HELP command with new functionality
- ✅ **FIXED ALL MOTOR DIRECTION AND ENCODER POLARITY ISSUES:**
  - Motor 2 (Crawler 1): Corrected encoder polarity - pins swapped to (3,2)
  - Motor 4 (Crawler 2): Reversed motor direction (_CW↔_CCW) - encoder pins (9,10)
  - Motor 6 (Crawler 2): Reversed motor direction (_CW↔_CCW) - encoder pins (7,8)
  - All 6 motors now move in correct direction with proper encoder feedback
  - Verified: All motors show negative encoder counts when moving backward
  - Position synchronization working perfectly (max error ~542 counts)
- ✅ **TESTING COMPLETED:**
  - Both crawlers tested independently using SELECT feature
  - All motors confirmed moving in sync with correct polarity
  - Ready for field deployment

### November 16, 2025 - Dual Crawler Control System COMPLETE ✅
- ✅ Expanded system to 6 motors across 3 HW-648 shields
- ✅ Added Shield 3 at I2C address 0x2E (AD1 soldered)
- ✅ Created 6-motor diagnostic tool with I2C scanner
- ✅ Identified all encoder pin mappings
- ✅ Resolved Motor 4 power connection issue - all motors working
- ✅ Verified all 6 encoders use hardware QuadTimer channels
- ✅ Documented wemos_motor_shield firmware requirement (critical)
- ✅ Documented I2C address troubleshooting and desoldering process
- ✅ Identified polarity corrections needed (Motors 2, 4, 6)
- ✅ Measured encoder count rates: ~1,400 counts/sec at 30% PWM
- ✅ **IMPLEMENTED DUAL CRAWLER CONTROL FIRMWARE:**
  - Two independent 3-wheeled crawlers (Crawler 1: Motors 1-3, Crawler 2: Motors 4-6)
  - Automatic I2C shield detection on startup
  - Graceful degradation (operates with 1 or 2 crawlers)
  - User confirmation prompt before starting
  - Per-crawler position synchronization (independent sync within each crawler)
  - Enhanced STATUS command showing both crawlers
  - Individual motor control (M1-M6)
  - All existing PID/velocity control features maintained

### November 9, 2025
- Initial 3-motor encoder identification test
- Verified encoder pins 0-1, 2-3, 5-6
- Identified Motor 3 polarity reversal
- Confirmed hardware QuadTimer support
