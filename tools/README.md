# Tools and Backup Files

This folder contains diagnostic tools, control utilities, and backup files.

## Python Control Script

### manual_control.py - Keyboard Manual Controller

Real-time keyboard control for individual motors. Controls one crawler at a time.

**Requirements:**
```bash
pip install pyserial keyboard
```

**Usage:**
```bash
python manual_control.py <COM_PORT> <CRAWLER>

# Examples:
python manual_control.py COM8 1   # Control Crawler 1 (motors 1-3)
python manual_control.py COM8 2   # Control Crawler 2 (motors 4-6)
```

**Key Mapping:**
- Q/A = Motor 1 (or 4) Forward/Backward
- W/S = Motor 2 (or 5) Forward/Backward
- E/D = Motor 3 (or 6) Forward/Backward
- SPACE = Stop all motors
- ESC = Exit

**Notes:**
- Requires `keyboard` library which needs admin privileges on Windows
- Close PIO device monitor before running (only one program can use COM port)
- Hold key to move motor, release to stop

## Diagnostic Firmware

### main_encoder_diagnostics.cpp - Motor-Encoder Diagnostic Tool

Tests individual motors and verifies encoder connections.

- Use this to test individual motors and verify encoder connections
- To use: copy to src/main.cpp (replacing current main.cpp), then build and upload
- Tests each motor sequentially and displays encoder readings

### main_6motor_diagnostics.cpp - 6-Motor Diagnostic Tool

Tests all 6 motors with I2C scanner functionality.

- Sequential testing of all motors at 30% speed
- Real-time encoder feedback for all 6 motors
- Built-in I2C scanner (type "SCAN" during test)

## Backup Files

- **main.cpp.old** - Previous version of main.cpp
- **main.cpp.backup** - Backup copy

## Using the Diagnostic Tool

If you need to verify motor-encoder connections or test hardware:

1. Backup current main.cpp:
   ```bash
   cp src/main.cpp src/main.cpp.working
   ```

2. Copy diagnostic tool:
   ```bash
   cp tools/main_encoder_diagnostics.cpp src/main.cpp
   ```

3. Build and upload:
   ```bash
   pio run --target upload
   pio device monitor
   ```

4. Restore working firmware:
   ```bash
   cp src/main.cpp.working src/main.cpp
   pio run --target upload
   ```
