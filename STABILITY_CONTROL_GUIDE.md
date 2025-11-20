# Stability Control Guide - Dual Pipe Crawler Robot

**Date:** November 2025
**Purpose:** Comprehensive guide to understanding and solving stability issues in 3-wheeled pipe crawler systems

---

## Table of Contents

1. [Understanding the Stability Problem](#understanding-the-stability-problem)
2. [Current Control System Architecture](#current-control-system-architecture)
3. [Position Sync vs Slip Detection](#position-sync-vs-slip-detection)
4. [Implemented Solutions](#implemented-solutions)
5. [IMU Integration Guide](#imu-integration-guide)
6. [Manual Control Tools](#manual-control-tools)
7. [Troubleshooting Drift and Tipping](#troubleshooting-drift-and-tipping)

---

## Understanding the Stability Problem

### The 3-Wheel Crawler Challenge

A 3-wheeled pipe crawler is essentially a **tripod** pressing outward against the pipe walls:

```
        Pipe Wall (5" diameter)
    ╔═══════════════════════════╗
    ║                           ║
    ║    ●  Motor 1 (front)     ║
    ║   ╱ ╲                     ║
    ║  ●   ●  Motors 2&3        ║
    ║  (120° apart)             ║
    ╚═══════════════════════════╝
```

**The Problem: Positive Feedback Loop**

1. **Initial condition:** One motor slips or gets slightly ahead
2. **Tilt begins:** Crawler pitches forward/backward
3. **Loss of tension:** Wheel(s) lose contact with pipe wall
4. **More slip:** Wheel with less contact slips more
5. **Severe tilt:** Crawler "nose dives" and loses all stability

**Why position sync alone isn't enough:**
- Position sync corrects for encoder position differences
- It can't distinguish between: **actual movement** vs **wheel slip**
- If a wheel is slipping (spinning but not advancing), position sync sees it as "ahead" and slows it down
- But the wheel is still slipping! The crawler continues to tilt

---

## Current Control System Architecture

### Control Hierarchy (Cascaded Loops)

```
┌─────────────────────────────────────────────────┐
│ User Command: "FORWARD 50"                      │
└─────────────────┬───────────────────────────────┘
                  ↓
┌─────────────────────────────────────────────────┐
│ Speed Ramp (RobotMotion.cpp)                    │
│ - Smoothly ramps: 0 → 2250 counts/sec          │
│ - Prevents sudden accelerations                 │
└─────────────────┬───────────────────────────────┘
                  ↓
          baseVelocity = 2250 c/s × direction
                  ↓
┌─────────────────────────────────────────────────┐
│ Position Sync (RobotMotion.cpp:297-378)        │
│ - Calculates average position per crawler       │
│ - Computes corrections for motors ahead/behind  │
│ - correction = positionError × Kp (0.2)         │
│ - Clamped to ±800 c/s max                       │
└─────────────────┬───────────────────────────────┘
                  ↓
    targetVelocity = baseVelocity + correction
                  ↓
┌─────────────────────────────────────────────────┐
│ PID Velocity Control (MotorController.cpp)     │
│ - Measures actual velocity from encoders        │
│ - Computes PID: error = target - actual         │
│ - Outputs PWM percentage (0-100%)               │
└─────────────────┬───────────────────────────────┘
                  ↓
              Motor PWM Commands
                  ↓
           ┌──────┴──────┐
           │  DC Motors  │
           └─────────────┘
```

### Key Parameters (config.h)

| Parameter | Value | Purpose |
|-----------|-------|---------|
| `POSITION_SYNC_KP` | 0.2 | Proportional gain for position sync |
| `MAX_SYNC_CORRECTION` | ±800 c/s | Max velocity adjustment for sync |
| `MAX_POSITION_ERROR` | 500 counts | Warning threshold for drift |
| `CONTROL_LOOP_HZ` | 50 Hz | Control loop frequency (20ms) |
| `MAX_VELOCITY_CPS` | 4500 c/s | Maximum encoder counts/second |

---

## Position Sync vs Slip Detection

### What Each System Does

#### **Position Synchronization** (Currently Implemented)
- **Measures:** Encoder position differences between motors
- **Goal:** Keep all motors at the same position
- **Action:** Adjust velocity setpoints to converge positions

**Example:**
```
Motor 1: 12,000 counts
Motor 2: 11,800 counts  ← 200 counts behind
Motor 3: 12,100 counts  ← 100 counts ahead

Position sync:
- Motor 1: baseVel + 6.6 c/s correction
- Motor 2: baseVel + 40 c/s correction  (speed up to catch up)
- Motor 3: baseVel - 20 c/s correction  (slow down)
```

**Limitation:** Can't detect if a motor is **spinning but not advancing** (slip)

#### **Slip Detection** (New Feature)
- **Measures:** Velocity tracking error (target vs actual)
- **Goal:** Detect when a wheel loses traction
- **Action:** Alert or stop crawler before severe tipping

**Example:**
```
Motor 1:
- Target velocity: 1500 c/s
- Actual velocity: 800 c/s
- Velocity error: 700 c/s (46% tracking error!)

Slip detection: "Motor 1 is slipping! Wheel lost contact!"
```

**Why it helps:** Catches **loss of traction** before position drift becomes severe

### How They Work Together

```
Position Sync:  Keeps motors aligned (outer loop, slow correction)
       +
Slip Detection: Catches traction loss (safety net, fast response)
       ↓
Better stability and earlier warning of problems
```

---

## Implemented Solutions

### 1. Slip Detection (MotorController.cpp)

**How it works:**
- Monitors velocity tracking error: `|targetVelocity - actualVelocity|`
- Calculates error percentage: `error / targetVelocity`
- Flags slip if error exceeds threshold for sustained period

**Configuration (config.h):**
```cpp
#define ENABLE_SLIP_DETECTION   true
#define SLIP_ERROR_THRESHOLD    0.35    // 35% velocity error
#define SLIP_DETECTION_TIME_MS  500     // Must persist 500ms
```

**Indicators:**
- ✅ Normal operation: Velocity error < 35%
- ⚠️  Warning: Velocity error 35-50% (likely slip starting)
- 🛑 Critical: Velocity error > 50% (severe slip or loss of contact)

**Actions:**
- `SLIP_ACTION_WARN`: Print warning to serial (default)
- `SLIP_ACTION_STOP`: Emergency stop crawler
- `SLIP_ACTION_REDUCE`: Reduce overall speed by 30%

**Considerations for High Gearing:**
- High gear ratios (e.g., 100:1) reduce motor load during wheel slip
- Motor may maintain speed even when wheel slips
- System also monitors:
  - **Position drift rate** (rapid divergence indicates slip)
  - **Acceleration mismatch** (motor accelerates too easily = low load)
  - **PID integral accumulation** (persistent error)

### 2. NUDGE Command (CommandParser.cpp)

**Purpose:** Fine motor positioning for manual realignment

**Syntax:**
```
NUDGE M<id> <+/- counts>

Examples:
NUDGE M2 +50     # Move motor 2 forward 50 encoder counts
NUDGE M5 -30     # Move motor 5 backward 30 counts
NUDGE M1 +200    # Move motor 1 forward 200 counts
```

**How it works:**
- Records current encoder position
- Sets target position = current + offset
- Motor runs at low speed (20% PWM) until target reached
- Auto-stops when position achieved (no watchdog timeout!)
- Much more precise than timed individual motor commands

**Benefits:**
- ✅ No watchdog timeout issues
- ✅ Precise positioning (exact encoder counts)
- ✅ Auto-stop when target reached (no overshooting)
- ✅ Ideal for manual realignment of tilted crawler

**Limitations:**
- Only one NUDGE active at a time
- Cancel with `STOP` or `ESTOP` command

### 3. Enhanced Drift Detection

**Tighter Position Limits:**
```cpp
// config.h
#define MAX_POSITION_ERROR      200   // Reduced from 500
#define AUTO_STOP_ON_DRIFT      true  // Stop before severe tipping
```

**Early Warning System:**
```
Position error 200-350 counts:  ⚠️  Warning printed
Position error 350-500 counts:  🟠 Slow down crawler 30%
Position error > 500 counts:    🛑 Emergency stop
```

**Prevents crawler from reaching tipping point.**

### 4. Manual Control Tools

See [Manual Control Tools](#manual-control-tools) section below.

---

## IMU Integration Guide

### Why Add an IMU?

An IMU (Inertial Measurement Unit) directly measures **physical tilt**, not inferred from encoders.

**Benefits:**
- ✅ Detect pitch/roll **before** severe position drift
- ✅ Close the loop on actual physical orientation
- ✅ Enable active tilt correction (keep crawler level)
- ✅ Catch slip that encoders miss (wheel spins, robot doesn't move)

### Recommended IMU: WT901 or BNO055

| Specification | WT901 | BNO055 |
|---------------|-------|--------|
| Interface | UART, I2C | UART, I2C |
| Output | Roll, Pitch, Yaw | Quaternion, Euler |
| Update rate | 200 Hz | 100 Hz |
| Size | 15mm × 15mm | 20mm × 27mm |
| Voltage | 3.3V - 5V | 3.3V |
| Cost | ~$25-35 | ~$20-30 |

**Recommended:** WT901 (simpler UART output, higher update rate)

### Wiring

**Option 1: UART (Recommended for 4-foot umbilical)**

```
WT901 IMU          Teensy 4.0
──────────         ──────────
VCC (3.3V)  ───────  3.3V
GND         ───────  GND
TX          ───────  Pin 15 (Serial3 RX)
RX          ───────  Pin 14 (Serial3 TX)
```

**Umbilical wires:**
- Add one twisted pair for TX/RX (2 wires total)
- UART easily handles 4 feet at 115200 baud
- No special drivers needed

**Option 2: I2C with Extender (If using I2C IMU)**

```
PCA9615 Extender   4-foot Twisted Pair   PCA9615 Extender
(Teensy Side)                             (Crawler Side)
────────────       ──────────────         ────────────
SDA  ──────────────  SDA+ / SDA-  ────────  SDA ── IMU
SCL  ──────────────  SCL+ / SCL-  ────────  SCL ── IMU
```

- PCA9615 converts I2C to differential signaling
- Can go 50+ feet reliably
- Costs ~$5-10 per extender (need 2)

### Firmware Integration

**Step 1: Add IMU reading (main.cpp)**

```cpp
#include <HardwareSerial.h>

// IMU on Serial3 (pins 14, 15)
HardwareSerial& imuSerial = Serial3;

float currentPitch = 0.0f;
float currentRoll = 0.0f;

void setup() {
    // ... existing setup ...

    // Initialize IMU serial
    imuSerial.begin(115200);
}

void loop() {
    // ... existing control loop ...

    // Read IMU data
    if (imuSerial.available() >= 11) {  // WT901 packet size
        readIMU(currentPitch, currentRoll);
    }

    // Apply attitude corrections
    robot.applyAttitudeCorrection(currentPitch, currentRoll);
}
```

**Step 2: Add attitude correction (RobotMotion.cpp)**

```cpp
void RobotMotion::applyAttitudeCorrection(float pitch, float roll) {
    // If pitched forward (positive pitch), leading motor is ahead
    // Apply negative correction to leading motor

    float pitchCorrection = -pitch * ATTITUDE_KP;  // e.g., Kp = 50

    // Project pitch onto each motor's position (120° apart)
    float correction1 = pitchCorrection * cos(0° * DEG_TO_RAD);      // Front
    float correction2 = pitchCorrection * cos(120° * DEG_TO_RAD);    // Left
    float correction3 = pitchCorrection * cos(240° * DEG_TO_RAD);    // Right

    // Add to existing position sync corrections
    // (This becomes the outer-most control loop)
}
```

**Step 3: Add config parameters (config.h)**

```cpp
// IMU Attitude Control
#define ENABLE_ATTITUDE_CONTROL  true
#define ATTITUDE_KP             50.0f   // Velocity correction per degree tilt
#define MAX_ATTITUDE_CORRECTION 500.0f  // Max correction (c/s)
#define PITCH_WARNING_THRESHOLD 5.0f    // Warn if pitch > 5°
#define PITCH_STOP_THRESHOLD    15.0f   // Emergency stop if > 15°
```

### Control Loop with IMU (Future)

```
User Command → Speed Ramp → Base Velocity
                               ↓
    IMU Attitude Sensor → Attitude Correction (outer loop)
                               ↓
           Position Sync → Position Correction (middle loop)
                               ↓
          PID Controller → Velocity Tracking (inner loop)
                               ↓
                          Motor PWM
```

**Three-level cascade:**
1. **Attitude control** (slowest, 1-2 Hz): Keep crawler level
2. **Position sync** (medium, 10-20 Hz): Keep motors aligned
3. **Velocity PID** (fastest, 50 Hz): Track velocity setpoints

---

## Manual Control Tools

### Python Keyboard Controller

**Location:** `tools/manual_control.py`

**Purpose:** Manual control of individual motors for realignment

Controls one crawler at a time (select at launch) with 3 motors per session.

**Key Mapping:**

```
Q = Motor Forward     A = Motor Backward   (Motor 1 or 4)
W = Motor Forward     S = Motor Backward   (Motor 2 or 5)
E = Motor Forward     D = Motor Backward   (Motor 3 or 6)

SPACE = Stop All Motors
ESC = Emergency Stop & Exit
```

**Why single-crawler design?**
- Avoids Windows keyboard ghosting (6+ simultaneous keys)
- Only 3 motors to control = simpler key mapping
- Can switch between crawlers by restarting with different argument

**Requirements:**
```bash
pip install pyserial keyboard
```

**Usage:**

```bash
# Control Crawler 1 (motors 1-3)
python tools/manual_control.py COM8 1

# Control Crawler 2 (motors 4-6)
python tools/manual_control.py COM8 2
```

**Important Notes:**
- Requires admin privileges on Windows (for keyboard library)
- Close PIO device monitor before running (only one program can use COM port)
- Hold key to move motor, release to stop

**Features:**
- Real-time status display showing motor states
- Hold-to-move, release-to-stop control
- SPACE key stops all motors without exiting
- ESC for emergency stop and exit

### NUDGE Command (Built into Firmware)

See [Implemented Solutions](#2-nudge-command-commandparsercpp) above.

---

## Troubleshooting Drift and Tipping

### Problem: Crawler gradually tilts forward during movement

**Possible Causes:**
1. **Motor speed mismatch** - One motor consistently faster/slower
2. **Wheel diameter differences** - Worn wheels or manufacturing variance
3. **Encoder polarity incorrect** - Motor moving opposite direction from encoder reading
4. **Position sync not aggressive enough** - Kp too low

**Solutions:**
1. Check STATUS output during movement:
   ```
   Motor 1 - Pos: 12000  Vel: 1450 c/s  PWM: 52%
   Motor 2 - Pos: 11800  Vel: 1420 c/s  PWM: 50%  ← Lagging
   Motor 3 - Pos: 12100  Vel: 1480 c/s  PWM: 54%  ← Leading
   ```

2. Verify all encoders show **negative counts when moving backward**:
   ```
   BACKWARD 30
   # Check STATUS - all positions should decrease
   ```

3. Increase position sync gain:
   ```cpp
   // config.h
   #define POSITION_SYNC_KP    0.3f  // Was 0.2, try higher
   ```

4. Use NUDGE to manually realign:
   ```
   STOP
   STATUS  # Check which motor is ahead/behind
   NUDGE M2 +100  # Move lagging motor forward
   ```

### Problem: Crawler nose-dives suddenly

**Cause:** One wheel lost contact with pipe wall → rapid slip → severe tilt

**Immediate Action:**
```
ESTOP           # Emergency stop
STATUS          # Check motor positions
NUDGE M1 -200   # Move leading motor back
NUDGE M2 +100   # Move lagging motor forward
# Repeat until level
RESET           # Reset encoder positions
```

**Prevention:**
1. Enable slip detection:
   ```cpp
   // config.h
   #define ENABLE_SLIP_DETECTION   true
   #define SLIP_ACTION             SLIP_ACTION_STOP
   ```

2. Tighten drift limits:
   ```cpp
   #define MAX_POSITION_ERROR      200  // Stop earlier
   ```

3. Monitor STATUS regularly during movement
4. Consider adding IMU for early tilt detection

### Problem: Position error keeps increasing despite sync

**Cause:** Likely wheel slip or severe motor speed mismatch

**Diagnosis:**
1. Check if slip detection is triggering:
   ```
   # Look for warnings in serial output:
   [SLIP] Motor 2: 45% velocity error - possible slip!
   ```

2. Check velocity tracking:
   ```
   STATUS
   # Look for large PWM differences:
   Motor 1 - PWM: 52%
   Motor 2 - PWM: 75%  ← Much higher PWM to achieve same speed = problem
   ```

3. Inspect hardware:
   - Check for loose encoder connections
   - Verify all wheels have good contact with pipe
   - Check for debris on wheels
   - Verify encoder polarity (all should count positive when moving forward)

**Solutions:**
1. Reduce overall speed:
   ```
   FORWARD 30  # Instead of FORWARD 50
   ```

2. Increase sync correction limit:
   ```cpp
   // config.h
   #define MAX_SYNC_CORRECTION  1200.0f  // Was 800
   ```

3. Check PID tuning - may need adjustment per motor:
   ```cpp
   // If Motor 2 is consistently slow:
   #define MOTOR2_KP    1.0f  // Increase from 0.8
   ```

### Problem: Crawler vibrates or oscillates

**Cause:** PID gains too aggressive or position sync fighting with PID

**Solutions:**
1. Reduce PID Kd (derivative term):
   ```cpp
   #define MOTOR1_KD    0.02f  // Reduce from 0.05
   ```

2. Reduce position sync gain:
   ```cpp
   #define POSITION_SYNC_KP    0.15f  // Reduce from 0.2
   ```

3. Check control loop timing:
   ```cpp
   // Make sure control loop is running at 50 Hz consistently
   // Add timing diagnostics to main.cpp if needed
   ```

---

## Testing Recommendations

### Initial Testing (Without IMU)

1. **Verify encoder polarity:**
   ```
   SELECT 1
   FORWARD 30
   # Wait 3 seconds
   STOP
   STATUS
   # All motor positions should be positive and similar

   BACKWARD 30
   # Wait 3 seconds
   STOP
   STATUS
   # All motor positions should be near zero (returned to start)
   ```

2. **Test position sync:**
   ```
   RESET
   FORWARD 50
   # Watch STATUS output - max error should stay < 200 counts
   ```

3. **Test slip detection:**
   ```
   # Manually lift one wheel off pipe wall during movement
   # Should see slip detection warning
   # System should respond based on SLIP_ACTION setting
   ```

4. **Test manual realignment:**
   ```
   # After some drift occurs:
   STOP
   STATUS  # Note position errors
   NUDGE M2 +150  # Realign lagging motor
   STATUS  # Verify improvement
   ```

### Future Testing (With IMU)

1. **Verify IMU readings:**
   ```
   # Add IMU output to STATUS command
   # Tilt crawler manually - verify pitch/roll values change
   ```

2. **Test attitude correction:**
   ```
   # Start level, move forward
   # Manually tilt crawler slightly
   # System should apply corrections to level out
   ```

---

## Summary

### Current System (Implemented)
- ✅ Position synchronization (keeps motors aligned)
- ✅ PID velocity control (tracks speed setpoints)
- ✅ Slip detection (warns of traction loss)
- ✅ NUDGE command (precise manual positioning)
- ✅ Manual control tools (Python keyboard controller)
- ✅ Enhanced drift detection (earlier warnings)

### Future Enhancements
- ⏳ IMU integration (direct tilt measurement)
- ⏳ Attitude control loop (active leveling)
- ⏳ Predictive slip compensation
- ⏳ Automated realignment procedures

### Key Takeaway

**The stability problem is solvable through:**
1. **Better sensing** - Detect slip and tilt earlier (slip detection + IMU)
2. **Faster correction** - React before tipping point (aggressive sync + attitude control)
3. **Manual tools** - Easily recover from drift (NUDGE command + keyboard control)

With these tools, you can maintain stability and quickly recover when issues occur.

---

**For questions or improvements, refer to:**
- `MOTOR_ENCODER_MAPPING.md` - Hardware details and encoder mapping
- `README.md` - Hardware setup and troubleshooting
- `CLAUDE.md` - Development guide and architecture
- `config.h` - All tunable parameters
