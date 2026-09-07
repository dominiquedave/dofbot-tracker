#!/usr/bin/env python3
"""
Phase 1: Tilt Servo Validation Script

This standalone script tests Servo 3 (tilt down) and Servo 4 (tilt up)
without depending on the full ROS color tracking system.

Usage:
    python3 test_tilt.py
"""

import time
import sys
import os

# Add the nodes directory to the path for importing dofbot_lib
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from dofbot_lib import ArmController


def initialize_servos(time_ms=1000):
    """
    Initialize both tilt servos to their correct starting positions:
    - Servo 3: 90°
    - Servo 4: 5°
    """
    print(f"\n--- INITIALIZING SERVOS (90°, 5°) ---")
    servo3 = ArmController(3)
    servo4 = ArmController(4)

    # Check Servo 3 current angle before writing
    current3 = servo3.read_angle(3)
    if current3 is None or abs(current3 - 90) > 10:
        # Need to take a sleep before each write so
        # that the servo can be properly called
        time.sleep((time_ms / 1000) * 1.2)
        servo3.write_angle(90, time_ms=time_ms)
    else:
        print(f"  Servo 3: skipped (already at {current3}°)")

    # Check Servo 4 current angle before writing
    current4 = servo4.read_angle(4)
    if current4 is None or abs(current4 - 5) > 10:
        # Need to take a sleep before each write so
        # that the servo can be properly called
        time.sleep((time_ms / 1000) * 1.2)
        servo4.write_angle(5, time_ms=time_ms)
    else:
        print(f"  Servo 4: skipped (already at {current4}°)")
   
    print("Servos initialized to starting positions.")


def tilt_down(time_ms=1000):
    """
    Smoothly move Servo 3 from 90° to 40° (downward tilt) and back.

    Servo 3 is mechanically reversed, so we need to account for that
    when calculating the position.
    """
    print("\n--- TILT DOWN: Servo 3 moving from 90° to 40° ---")
    servo3 = ArmController(3)  # Servo 3 for tilt down

    # Tilt down to 40 degrees
    current3 = servo3.read_angle(3)
    if current3 is None or abs(current3 - 40) > 10:
        time.sleep((time_ms / 1000) * 1.2)
        servo3.write_angle(40, time_ms=time_ms)
    else:
        print(f"  Servo 3: already at 40° (within tolerance)")

    print("Tilt down sequence complete.")


def tilt_up(time_ms=1000):
    """
    Smoothly move Servo 4 from 5° to 60° (upward tilt).

    Servo 4 is mechanically reversed, so we need to account for that
    when calculating the position.
    """
    print("\n--- TILT UP: Servo 4 moving from 5° to 60° ---")
    servo4 = ArmController(4)  # Servo 4 for tilt up
    
    # Check Servo 4 current angle before writing
    current4 = servo4.read_angle(4)
    if current4 is None or abs(current4 - 5) > 10:
        # Need to take a sleep before each write so
        # that the servo can be properly called
        time.sleep((time_ms / 1000) * 1.2)
        servo4.write_angle(60, time_ms=time_ms)
    else:
        print(f"  Servo 4: skipped (already at {current4}°)")

    print("Tilt up sequence complete.")


def full_sequence():
    """
    Run the complete tilt test sequence:
    1. Tilt down (Servo 3: 90° -> 40°)
    2. Pause
    3. Return to start
    4. Tilt up (Servo 4: 5° -> 60°)
    5. Pause
    6. Return to start
    """
    print("=" * 60)
    print("DOFBOT TILT SERVO VALIDATION TEST")
    print("=" * 60)
    print("Testing Servo 3 (tilt down, 90° -> 40°)")
    print("Testing Servo 4 (tilt up, 5° -> 60°)")
    print("=" * 60)

    try:
        #breakpoint()
        # Initialize servos to correct starting positions
        initialize_servos(time_ms=1000)

        # Sequence 1: Tilt down
        tilt_down(time_ms=1000)

        # Return to start
        initialize_servos(time_ms=1000)

        # Sequence 2: Tilt up
        tilt_up(time_ms=1000)
       
        # Return to start
        initialize_servos(time_ms=1000)

    except KeyboardInterrupt:
        print("\n\nTest interrupted by user.")
        print("Returning servos to center position...")
        ArmController.move_all_to_center(time_ms=1000)


if __name__ == '__main__':
    full_sequence()
