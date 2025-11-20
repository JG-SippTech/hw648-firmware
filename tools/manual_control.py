#!/usr/bin/env python3
"""
Pipe Crawler Robot - Manual Keyboard Controller
================================================

Controls ONE crawler at a time (3 motors).
Select which crawler (1 or 2) at launch.

Key Mapping:
-----------
Motor 1/4:  Q = Forward    A = Backward
Motor 2/5:  W = Forward    S = Backward
Motor 3/6:  E = Forward    D = Backward

SPACE = Stop All Motors (without exiting)
ESC   = Emergency Stop & Exit

Usage:
------
python manual_control.py <COM_PORT> <CRAWLER>

Examples:
    python manual_control.py COM8 1      # Control Crawler 1 (motors 1-3)
    python manual_control.py COM8 2      # Control Crawler 2 (motors 4-6)

Requirements:
-------------
pip install pyserial keyboard

Note: Run as Administrator on Windows for keyboard capture.

Author: Claude Code
Date: November 2025
"""

import serial
import keyboard
import time
import sys

class CrawlerController:
    def __init__(self, port, crawler_num, baud=115200):
        """Initialize crawler controller"""
        self.port = port
        self.baud = baud
        self.crawler_num = crawler_num
        self.ser = None
        self.running = False

        # Map motor indices based on crawler selection
        if crawler_num == 1:
            self.motor_ids = [1, 2, 3]
        else:
            self.motor_ids = [4, 5, 6]

        # Motor states (0=stopped, 1=forward, -1=backward)
        self.motor_states = [0, 0, 0]

        # Motor speed (percentage)
        self.default_speed = 30

        # Key mapping: key -> (motor_index, direction)
        # motor_index is 0, 1, 2 (mapped to actual motor IDs above)
        self.forward_keys = {'q': 0, 'w': 1, 'e': 2}
        self.backward_keys = {'a': 0, 's': 1, 'd': 2}

        # Track which keys are currently pressed
        self.keys_pressed = set()

    def connect(self):
        """Connect to Teensy via serial"""
        try:
            print(f"Connecting to {self.port} at {self.baud} baud...")
            self.ser = serial.Serial(self.port, self.baud, timeout=0.1)
            time.sleep(2)  # Wait for connection

            # Clear startup messages
            while self.ser.in_waiting:
                self.ser.readline()

            print("Connected!\n")
            return True
        except serial.SerialException as e:
            print(f"Failed to connect: {e}")
            return False

    def send_command(self, cmd):
        """Send command to Teensy"""
        if self.ser and self.ser.is_open:
            self.ser.write(f"{cmd}\n".encode())
            time.sleep(0.02)

    def control_motor(self, motor_index, direction):
        """Control a single motor"""
        motor_id = self.motor_ids[motor_index]
        speed = self.default_speed

        if direction == 1:
            cmd = f"M{motor_id} FWD {speed}"
        elif direction == -1:
            cmd = f"M{motor_id} BACK {speed}"
        else:
            cmd = f"M{motor_id} FWD 0"

        self.motor_states[motor_index] = direction
        self.send_command(cmd)

    def stop_all(self):
        """Stop all motors"""
        for i in range(3):
            self.control_motor(i, 0)
        print("\r[STOPPED]" + " " * 40, end='', flush=True)

    def emergency_stop(self):
        """Emergency stop and exit"""
        self.send_command("ESTOP")
        self.motor_states = [0, 0, 0]
        print("\n\nEMERGENCY STOP!")

    def print_status(self):
        """Print current motor states"""
        symbols = {0: '-', 1: '^', -1: 'v'}
        status = f"Crawler {self.crawler_num}: "
        for i, state in enumerate(self.motor_states):
            motor_id = self.motor_ids[i]
            status += f"M{motor_id}:{symbols[state]} "
        print(f"\r{status}", end='', flush=True)

    def process_keys(self):
        """Process currently pressed keys"""
        new_states = [0, 0, 0]

        # Check forward keys
        for key, motor_idx in self.forward_keys.items():
            if keyboard.is_pressed(key):
                new_states[motor_idx] = 1

        # Check backward keys (backward overrides forward)
        for key, motor_idx in self.backward_keys.items():
            if keyboard.is_pressed(key):
                new_states[motor_idx] = -1

        # Update motors that changed state
        for i in range(3):
            if new_states[i] != self.motor_states[i]:
                self.control_motor(i, new_states[i])

        self.print_status()

    def print_instructions(self):
        """Print control instructions"""
        print("=" * 50)
        print(f" CRAWLER {self.crawler_num} MANUAL CONTROLLER ".center(50))
        print("=" * 50)
        print(f"\nControlling motors: {self.motor_ids}")
        print(f"Speed: {self.default_speed}%")
        print("\nKEY MAPPING:")
        print(f"  Q/A = Motor {self.motor_ids[0]} Forward/Backward")
        print(f"  W/S = Motor {self.motor_ids[1]} Forward/Backward")
        print(f"  E/D = Motor {self.motor_ids[2]} Forward/Backward")
        print("\n  SPACE = Stop all motors")
        print("  ESC   = Exit")
        print("\nHold key to move, release to stop.")
        print("=" * 50 + "\n")

    def run(self):
        """Main control loop"""
        if not self.connect():
            return

        self.print_instructions()
        self.running = True

        try:
            while self.running:
                # Check for exit
                if keyboard.is_pressed('esc'):
                    self.emergency_stop()
                    break

                # Check for stop all
                if keyboard.is_pressed('space'):
                    self.stop_all()
                    time.sleep(0.2)  # Debounce
                    continue

                # Process motor control keys
                self.process_keys()

                # Small delay to prevent CPU hogging
                time.sleep(0.05)

                # Read Teensy responses
                while self.ser.in_waiting:
                    response = self.ser.readline().decode('utf-8', errors='ignore').strip()
                    if response and '[CMD]' not in response:
                        print(f"\n{response}")

        except KeyboardInterrupt:
            print("\n\nInterrupted")

        finally:
            print("\nShutting down...")
            if self.ser and self.ser.is_open:
                self.send_command("ESTOP")
                self.ser.close()
            print("Disconnected")


def main():
    if len(sys.argv) < 3:
        print("Usage: python manual_control.py <COM_PORT> <CRAWLER>")
        print("\nExamples:")
        print("  python manual_control.py COM8 1   # Crawler 1 (motors 1-3)")
        print("  python manual_control.py COM8 2   # Crawler 2 (motors 4-6)")
        sys.exit(1)

    port = sys.argv[1]
    crawler = int(sys.argv[2])

    if crawler not in [1, 2]:
        print("ERROR: Crawler must be 1 or 2")
        sys.exit(1)

    controller = CrawlerController(port, crawler)
    controller.run()


if __name__ == "__main__":
    main()
