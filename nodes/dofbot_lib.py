#!/usr/bin/env python3
"""
Dofbot Pi Arm Controller - ROS Node Helper

This module provides the ArmController class that can be used both:
1. As a standalone module for testing arm control
2. As part of the ROS dofbot_tracker node

Usage (standalone):
    python3 dofbot_lib.py --servo 1 --angle 90 --time 500
"""

import argparse
import time
import sys

try:
    import smbus
    I2C_AVAILABLE = True
except ImportError:
    I2C_AVAILABLE = False
    print("Warning: smbus not available - running in simulation mode")


class ArmController:
    """
    Dofbot Pi Arm Controller using I2C communication.

    Control individual servos on the Dofbot Pi 6-DOF arm.

    I2C Details:
        - Address: 0x15
        - Bus: 1 (Raspberry Pi I2C bus 1)
        - Commands: 0x10 + servo_id (1-6)

    Servo Information:
        - Servos 1,2,3,4,6: 0-180 degrees
        - Servo 5: 0-270 degrees (extended range)
        - Servos 2,3,4 are mechanically reversed
    """

    # Position ranges (12-bit values)
    SERVO_MIN = 900   # 0 degrees
    SERVO_MAX = 3100  # 180 degrees
    SERVO5_MIN = 380  # 0 degrees (extended range)
    SERVO5_MAX = 3700 # 270 degrees (extended range)

    def __init__(self, servo_id, address=0x15, bus_number=1):
        """
        Initialize the arm controller.

        Args:
            servo_id: Servo ID (1-6)
            address: I2C address (default: 0x15)
            bus_number: I2C bus number (default: 1)
        """
        self.servo_id = servo_id
        self.address = address
        self.bus_number = bus_number
        self.current_angle = 90
        self.initialized = False

        if not I2C_AVAILABLE:
            print(f"[ArmController] smbus not available - running in simulation mode")
            return

        try:
            self.bus = smbus.SMBus(bus_number)
            self.initialized = True
            print(f"[ArmController] Initialized servo {servo_id} on I2C 0x{address:02X}, Bus {bus_number}")
        except Exception as e:
            print(f"[ArmController] Failed to initialize I2C: {e} - running in simulation mode")

    def angle_to_position(self, angle):
        """
        Convert angle in degrees to I2C position value.

        Args:
            angle: Target angle in degrees (0-180 or 0-270 for servo 5)

        Returns:
            12-bit position value for I2C communication
        """
        if self.servo_id == 5:
            # Servo 5 has extended range (0-270°)
            position = int((self.SERVO5_MAX - self.SERVO5_MIN) * (angle - 0) / (270 - 0) + self.SERVO5_MIN)
        else:
            # Standard servos (1,2,3,4,6) have 0-180° range
            # Servos 2,3,4 are mechanically reversed - invert the angle
            if self.servo_id in [2, 3, 4]:
                angle = 180 - angle
            position = int((self.SERVO_MAX - self.SERVO_MIN) * (angle - 0) / (180 - 0) + self.SERVO_MIN)

        return position

    def write_angle(self, angle, time_ms=100):
        """
        Write angle to servo.

        Args:
            angle: Target angle in degrees
            time_ms: Movement duration in milliseconds
        """
        if not self.initialized:
            print(f"[ArmController] Simulated: Servo {self.servo_id} -> {angle}° in {time_ms}ms")
            self.current_angle = angle
            return

        # Clamp angle to valid range
        if self.servo_id == 5:
            angle = max(0, min(270, angle))
        else:
            angle = max(0, min(180, angle))

        # Convert to position
        position = self.angle_to_position(angle)

        # Calculate high and low bytes for position and time
        value_H = (position >> 8) & 0xFF
        value_L = position & 0xFF
        time_H = (time_ms >> 8) & 0xFF
        time_L = time_ms & 0xFF

        try:
            # Write to I2C: command register 0x10 + servo_id
            self.bus.write_i2c_block_data(
                self.address, 0x10 + self.servo_id,
                [value_H, value_L, time_H, time_L]
            )
            self.current_angle = angle
            print(f"[ArmController] Servo {self.servo_id}: {angle}° in {time_ms}ms (pos={position})")
        except Exception as e:
            print(f"[ArmController] I2C write error: {e}")

    def get_angle(self):
        """Return the last known angle."""
        return self.current_angle

    def move_to_center(self, time_ms=200):
        """Move servo to center position (90°)."""
        self.write_angle(90, time_ms)

    @staticmethod
    def move_all_to_center(address=0x15, bus_number=1, time_ms=200):
        """
        Move all 6 servos to center position (90°) simultaneously.

        This uses the special I2C command that controls all servos at once.

        Args:
            address: I2C address (default: 0x15)
            bus_number: I2C bus number (default: 1)
            time_ms: Movement duration in milliseconds
        """
        if not I2C_AVAILABLE:
            print(f"[ArmController] Simulated: All servos -> 90° in {time_ms}ms")
            return

        try:
            bus = smbus.SMBus(bus_number)

            # Time bytes
            time_H = (time_ms >> 8) & 0xFF
            time_L = time_ms & 0xFF

            # Position for 90° for each servo
            # Servos 1,2,3,4,6: 90° = ~2000
            # Servo 5: 90° = ~1430 (within 0-270° range)
            positions = []

            # Servo 1 (0-180°)
            positions.extend([(2000 >> 8) & 0xFF, 2000 & 0xFF])
            # Servo 2 (0-180°, reversed)
            positions.extend([(2000 >> 8) & 0xFF, 2000 & 0xFF])
            # Servo 3 (0-180°, reversed)
            positions.extend([(2000 >> 8) & 0xFF, 2000 & 0xFF])
            # Servo 4 (0-180°, reversed)
            positions.extend([(2000 >> 8) & 0xFF, 2000 & 0xFF])
            # Servo 5 (0-270°)
            pos5 = int((3700 - 380) * (90 - 0) / (270 - 0) + 380)
            positions.extend([(pos5 >> 8) & 0xFF, pos5 & 0xFF])
            # Servo 6 (0-180°)
            positions.extend([(2000 >> 8) & 0xFF, 2000 & 0xFF])

            # Send time first, then positions
            bus.write_i2c_block_data(address, 0x1e, [time_H, time_L])
            bus.write_i2c_block_data(address, 0x1d, positions)

            print(f"[ArmController] All servos -> 90° in {time_ms}ms")
        except Exception as e:
            print(f"[ArmController] I2C write error: {e}")

    @staticmethod
    def move_all_servos(s1, s2, s3, s4, s5, s6, address=0x15, bus_number=1, time_ms=200):
        """
        Control all 6 servos simultaneously.

        Args:
            s1, s2, s3, s4, s6: Angles 0-180 degrees
            s5: Angle 0-270 degrees
            address: I2C address (default: 0x15)
            bus_number: I2C bus number (default: 1)
            time_ms: Movement duration in milliseconds
        """
        if not I2C_AVAILABLE:
            print(f"[ArmController] Simulated: All servos -> [{s1},{s2},{s3},{s4},{s5},{s6}]° in {time_ms}ms")
            return

        # Validate angles
        if any(v < 0 or v > 180 for v in [s1, s2, s3, s4, s6]):
            print("[ArmController] Error: Servos 1,2,3,4,6 must be 0-180°")
            return
        if s5 < 0 or s5 > 270:
            print("[ArmController] Error: Servo 5 must be 0-270°")
            return

        try:
            bus = smbus.SMBus(bus_number)

            # Time bytes
            time_H = (time_ms >> 8) & 0xFF
            time_L = time_ms & 0xFF

            # Calculate positions for each servo
            def angle_to_pos(angle, servo_id):
                if servo_id == 5:
                    return int((3700 - 380) * (angle - 0) / (270 - 0) + 380)
                else:
                    # Servos 2,3,4 are reversed
                    if servo_id in [2, 3, 4]:
                        angle = 180 - angle
                    return int((3100 - 900) * (angle - 0) / (180 - 0) + 900)

            positions = []
            for angle, servo_id in [(s1, 1), (s2, 2), (s3, 3), (s4, 4), (s5, 5), (s6, 6)]:
                pos = angle_to_pos(angle, servo_id)
                positions.extend([(pos >> 8) & 0xFF, pos & 0xFF])

            # Send time first, then positions
            bus.write_i2c_block_data(address, 0x1e, [time_H, time_L])
            bus.write_i2c_block_data(address, 0x1d, positions)

            print(f"[ArmController] All servos -> [{s1},{s2},{s3},{s4},{s5},{s6}]° in {time_ms}ms")
        except Exception as e:
            print(f"[ArmController] I2C write error: {e}")


def parse_args():
    """Parse command line arguments."""
    parser = argparse.ArgumentParser(
        description='Dofbot Pi Arm Controller - Test individual servos'
    )
    parser.add_argument('--servo', '-s', type=int, required=True,
                        help='Servo ID (1-6), or 0 for all servos')
    parser.add_argument('--angle', '-a', type=float, required=True,
                        help='Target angle in degrees')
    parser.add_argument('--time', '-t', type=int, default=100,
                        help='Movement time in ms (default: 100)')
    parser.add_argument('--center', '-c', action='store_true',
                        help='Move to center position (90°)')

    return parser.parse_args()


def main():
    """Main function for standalone testing."""
    args = parse_args()

    if args.servo < 0 or args.servo > 6:
        print("Error: Servo ID must be 0-6 (0 = all servos)")
        sys.exit(1)

    if args.servo == 0:
        # Control all servos
        ArmController.move_all_servos(
            args.angle, args.angle, args.angle, args.angle, args.angle, args.angle,
            time_ms=args.time
        )
    elif args.center:
        # Single servo to center
        controller = ArmController(args.servo)
        controller.move_to_center(time_ms=args.time)
    else:
        # Single servo to specific angle
        controller = ArmController(args.servo)
        controller.write_angle(args.angle, time_ms=args.time)

    print("Done!")


if __name__ == '__main__':
    main()
