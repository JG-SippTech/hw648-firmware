# Motor-Encoder Pin Mapping

**Date Verified:** November 9, 2025
**Hardware:** Pipe Crawler Robot - 3 Motors with Quadrature Encoders
**MCU:** Teensy 4.0

---

## Verified Motor-Encoder Assignments

| Motor # | I2C Shield | Motor Channel | Encoder Pins | Direction | Wire Color | Notes |
|---------|------------|---------------|--------------|-----------|------------|-------|
| **Motor 1** | 0x2F | A | **0, 1** | ✅ Positive | Red | Correct polarity |
| **Motor 2** | 0x2F | B | **2, 3** | ✅ Positive | Black | Correct polarity |
| **Motor 3** | 0x30 | A | **5, 6** | ⚠️ Negative | (3rd motor) | Reverse polarity |

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

### Basic Encoder Setup:
```cpp
#include <Encoder.h>

// Motor objects
Motor motor1(0x2F, _MOTOR_A, 1000);  // Motor 1 - Shield 1, Channel A
Motor motor2(0x2F, _MOTOR_B, 1000);  // Motor 2 - Shield 1, Channel B
Motor motor3(0x30, _MOTOR_A, 1000);  // Motor 3 - Shield 2, Channel A

// Encoder objects (with correct polarity)
Encoder encoder1(0, 1);   // Motor 1 - Positive direction
Encoder encoder2(2, 3);   // Motor 2 - Positive direction
Encoder encoder3(6, 5);   // Motor 3 - SWAPPED for correct polarity (was 5,6)
```

**Note:** Motor 3 encoder pins are **swapped (6, 5 instead of 5, 6)** to correct the negative polarity. This makes all three motors report positive counts when spinning in the same direction.

### Alternative: Software Correction
If you prefer to keep hardware pins as-is and correct in software:
```cpp
Encoder encoder3(5, 6);  // Hardware order

// In your code:
long pos3 = -encoder3.read();  // Negate the reading
```

---

## Hardware Quadrature Decoder Channels

All three encoders use **hardware quadrature decoding** on Teensy 4.0:

| Encoder | Pins | QuadTimer Channel | Performance |
|---------|------|-------------------|-------------|
| Motor 1 | 0, 1 | QuadTimer 3 | Hardware |
| Motor 2 | 2, 3 | QuadTimer 1 | Hardware |
| Motor 3 | 5, 6 | QuadTimer 2 | Hardware |

**Benefits:**
- ✅ No CPU interrupts needed
- ✅ High-speed counting (up to 150 MHz)
- ✅ No missed counts at high RPM
- ✅ Low CPU overhead

---

## Testing Results

**Test Method:** Each motor was spun individually at 30% speed for 5 seconds while monitoring encoder counts.

**Results:**
- **Motor 1 (Red)**: Encoder [0-1] counted positive (~1200 counts in 5 sec)
- **Motor 2 (Black)**: Encoder [2-3] counted positive (~1150 counts in 5 sec)
- **Motor 3**: Encoder [5-6] counted **negative** (~-1180 counts in 5 sec)

**Interpretation:**
- All three encoders working correctly
- Motor 3 encoder wired in reverse polarity (easily corrected in code)
- Approximate encoder resolution: ~240 counts/second at 30% speed

---

## Future Expansion

**Available pins for 3 additional motors:**
- Motor 4: Pins 7, 8 (QuadTimer 3 - hardware)
- Motor 5: Pins 9, 10 (QuadTimer 4 - hardware)
- Motor 6: Pins 11, 12 (software interrupts)

**Reserved pins:**
- 18, 19 (I2C - SDA/SCL) - In use by motor shields
- 13 (LED) - Avoid

---

## Troubleshooting Notes

### If encoders don't count:
1. Check encoder power: Should be exactly 3.3V (measure Blue wire)
2. Check ground connection (Brown wire)
3. Verify signal wires connected to correct pins (Green=A, Yellow=B)

### If counts are erratic:
1. Check for loose connections
2. Ensure encoder wires aren't too close to motor power wires (EMI)
3. Add 0.1µF capacitors across encoder power if needed

### If direction is wrong:
1. Swap encoder pins in code: `Encoder(B, A)` instead of `Encoder(A, B)`
2. Or negate reading in software: `pos = -encoder.read()`

---

## References

- **Encoder Library**: https://github.com/PaulStoffregen/Encoder
- **PJRC Teensy 4.0**: https://www.pjrc.com/store/teensy40.html
- **Test Firmware**: `hw648-firmware/src/main.cpp.old` (encoder identification test)
