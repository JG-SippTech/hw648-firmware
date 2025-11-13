# Tools and Backup Files

This folder contains diagnostic tools and backup files that are not part of the main firmware build.

## Files

- **main_encoder_diagnostics.cpp** - Motor-encoder mapping diagnostic tool
  - Use this to test individual motors and verify encoder connections
  - To use: copy to src/main.cpp (replacing current main.cpp), then build and upload
  - Tests each motor sequentially and displays encoder readings

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
