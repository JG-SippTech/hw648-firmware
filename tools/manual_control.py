#!/usr/bin/env python3
"""
Dual Crawler Robot - Manual Keyboard Controller
================================================

Two-key design to avoid Windows keyboard ghosting (6+ key limitation).
Each motor has two keys: one for forward, one for backward.

Key Mapping:
-----------
Motor 1:  Q = Forward    A = Backward
Motor 2:  W = Forward    S = Backward
Motor 3:  E = Forward    D = Backward
Motor 4:  R = Forward    F = Backward
Motor 5:  T = Forward    G = Backward
Motor 6:  Y = Forward    H = Backward

ESC = Emergency Stop All Motors

Usage:
------
python manual_control.py [COM_PORT] [BAUD_RATE]

Examples:
    python manual_control.py COM7
    python manual_control.py COM7 115200
    python manual_control.py /dev/ttyACM0  (Linux)

Requirements:
-------------
pip install pyserial keyboard

Author: Claude Code
Date: November 2025
"""

import serial
import keyboard
import time
import sys
import threading

class CrawlerController:
    def __init__(self, port, baud=115200):
        """Initialize crawler controller"""
        self.port = port
        self.baud = baud
        self.ser = None
        self.running = False

        # Motor states (which motors are currently running)
        self.motor_states = {i: 0 for i in range(1, 7)}  # 0=stopped, 1=forward, -1=backward

        # Motor speed (percentage)
        self.default_speed = 30

        # Key mapping: (key, motor_id, direction)
        self.key_map = {
            'q': (1, 'fwd'),  'a': (1, 'back'),
            'w': (2, 'fwd'),  's': (2, 'back'),
            'e': (3, 'fwd'),  'd': (3, 'back'),
            'r': (4, 'fwd'),  'f': (4, 'back'),
            't': (5, 'fwd'),  'g': (5, 'back'),
            'y': (6, 'fwd'),  'h': (6, 'back'),
        }

    def connect(self):
        """Connect to Teensy via serial"""
        try:
            print(f"Connecting to {self.port} at {self.baud} baud...")
            self.ser = serial.Serial(self.port, self.baud, timeout=0.1)
            time.sleep(2)  # Wait for connection to establish

            # Clear any startup messages
            while self.ser.in_waiting:
                self.ser.readline()

            print("✓ Connected successfully!")
            return True
        except serial.SerialException as e:
            print(f"✗ Failed to connect: {e}")
            return False

    def send_command(self, cmd):
        """Send command to Teensy"""
        if self.ser and self.ser.is_open:
            self.ser.write(f"{cmd}\n".encode())
            time.sleep(0.01)  # Small delay for command processing

    def control_motor(self, motor_id, direction, speed=None):
        """Control a single motor"""
        if speed is None:
            speed = self.default_speed

        if direction == 'fwd':
            cmd = f"M{motor_id} FWD {speed}"
            self.motor_states[motor_id] = 1
        elif direction == 'back':
            cmd = f"M{motor_id} BACK {speed}"
            self.motor_states[motor_id] = -1
        else:
            cmd = f"M{motor_id} FWD 0"
            self.motor_states[motor_id] = 0

        self.send_command(cmd)

    def stop_motor(self, motor_id):
        """Stop a specific motor"""
        self.send_command(f"M{motor_id} FWD 0")
        self.motor_states[motor_id] = 0

    def emergency_stop(self):
        """Emergency stop all motors"""
        self.send_command("ESTOP")
        for i in range(1, 7):
            self.motor_states[i] = 0
        print("\n🛑 EMERGENCY STOP!")

    def print_status(self):
        """Print current motor states"""
        status_line = "Motors: "
        for i in range(1, 7):
            state = self.motor_states[i]
            if state == 1:
                symbol = "↑"
            elif state == -1:
                symbol = "↓"
            else:
                symbol = "⦁"
            status_line += f"M{i}:{symbol} "

        # Print on same line (overwrite previous)
        print(f"\r{status_line}", end='', flush=True)

    def handle_key_press(self, key_event):
        """Handle keyboard events"""
        key = key_event.name.lower()

        # Emergency stop
        if key == 'esc':
            self.emergency_stop()
            self.running = False
            return

        # Motor control
        if key in self.key_map:
            motor_id, direction = self.key_map[key]
            self.control_motor(motor_id, direction)
            self.print_status()

    def handle_key_release(self, key_event):
        """Handle key release - stop motor"""
        key = key_event.name.lower()

        if key in self.key_map:
            motor_id, _ = self.key_map[key]
            self.stop_motor(motor_id)
            self.print_status()

    def print_instructions(self):
        """Print control instructions"""
        print("\n" + "="*60)
        print(" DUAL CRAWLER MANUAL CONTROLLER ".center(60))
        print("="*60)
        print("\nKEY MAPPING (Two-key design):")
        print("  Crawler 1:")
        print("    Motor 1:  Q = Forward    A = Backward")
        print("    Motor 2:  W = Forward    S = Backward")
        print("    Motor 3:  E = Forward    D = Backward")
        print("\n  Crawler 2:")
        print("    Motor 4:  R = Forward    F = Backward")
        print("    Motor 5:  T = Forward    G = Backward")
        print("    Motor 6:  Y = Forward    H = Backward")
        print("\n  EMERGENCY:  ESC = Stop All & Exit")
        print("\nUSAGE:")
        print("  - Press and HOLD key to move motor")
        print("  - Release key to stop motor")
        print("  - Can control multiple motors simultaneously")
        print(f"  - Speed: {self.default_speed}% (adjustable in code)")
        print("\n" + "="*60 + "\n")
        print("Press any motor key to start. Press ESC to exit.\n")

    def run(self):
        """Main control loop"""
        if not self.connect():
            return

        self.print_instructions()
        self.running = True

        # Register keyboard events
        keyboard.on_press(self.handle_key_press)
        keyboard.on_release(self.handle_key_release)

        try:
            # Keep running until ESC is pressed
            while self.running:
                time.sleep(0.1)

                # Read any responses from Teensy (for debugging)
                if self.ser.in_waiting:
                    response = self.ser.readline().decode('utf-8', errors='ignore').strip()
                    if response and not response.startswith("[CMD]"):
                        print(f"\n{response}")
                        self.print_status()

        except KeyboardInterrupt:
            print("\n\nInterrupted by user")

        finally:
            # Cleanup
            print("\n\nShutting down...")
            keyboard.unhook_all()
            self.emergency_stop()
            if self.ser and self.ser.is_open:
                self.ser.close()
            print("✓ Disconnected")


def main():
    """Main entry point"""
    # Parse command line arguments
    if len(sys.argv) < 2:
        print("Usage: python manual_control.py <COM_PORT> [BAUD_RATE]")
        print("\nExamples:")
        print("  Windows: python manual_control.py COM7")
        print("  Linux:   python manual_control.py /dev/ttyACM0")
        print("\nDefault baud rate: 115200")
        sys.exit(1)

    port = sys.argv[1]
    baud = int(sys.argv[2]) if len(sys.argv) > 2 else 115200

    # Create and run controller
    controller = CrawlerController(port, baud)
    controller.run()


if __name__ == "__main__":
    main()
