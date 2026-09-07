#!/usr/bin/env python3
"""
Tracking Controller Module

Handles servo position control with smoothing and rate limiting.
Extracted from StandaloneTracker.control_arm() to follow Single Responsibility Principle.

This module combines three related concerns into one cohesive class:
1. Position smoothing (exponential moving average)
2. Proportional control (pixel error -> servo angles)
3. Rate limiting (prevent excessive servo updates)

Usage:
    from tracker_controller import TrackingController, ServoCommand
    from tracker_config import TrackerConfig

    config = TrackerConfig()
    controller = TrackingController(config)

    # In tracking loop
    command = controller.update(object_x=350, object_y=200)
    if command.should_update:
        pan_servo.write_angle(command.pan_angle, time_ms=command.move_time_ms)
        tilt_servo.write_angle(command.tilt_angle, time_ms=command.move_time_ms)
"""

import time as time_module
from dataclasses import dataclass
from tracker_config import TrackerConfig


@dataclass
class ServoCommand:
    """
    Servo movement command with validation.

    This dataclass encapsulates all information needed to execute a servo
    movement, making it easy to pass between components.

    Attributes:
        pan_angle: Horizontal servo angle (0-180 degrees)
        tilt_down_angle: Tilt-down servo (Servo 3) angle, 40-90 degrees
        tilt_up_angle: Tilt-up servo (Servo 4) angle, 5-60 degrees
        move_time_ms: Movement duration in milliseconds
        should_update: False if in deadzone or rate-limited
    """
    pan_angle: int
    tilt_down_angle: int
    tilt_up_angle: int
    move_time_ms: int
    should_update: bool = True

    def __post_init__(self):
        """Validate servo angles are within valid range."""
        if not (0 <= self.pan_angle <= 180):
            raise ValueError(f"pan_angle must be 0-180: {self.pan_angle}")
        # Tilt-down servo (Servo 3) range: 40-90 degrees
        if not (40 <= self.tilt_down_angle <= 90):
            raise ValueError(f"tilt_down_angle must be 40-90: {self.tilt_down_angle}")
        # Tilt-up servo (Servo 4) range: 5-60 degrees
        if not (5 <= self.tilt_up_angle <= 60):
            raise ValueError(f"tilt_up_angle must be 5-60: {self.tilt_up_angle}")


class TrackingController:
    """
    Controls servo tracking with smoothing and rate limiting.

    This class combines position smoothing, proportional control, and rate
    limiting into a single cohesive component. It maintains internal state
    for smoothed positions and timing.

    Design rationale:
    - These three concerns (smoothing, control, rate limiting) are tightly
      related and always used together for tracking control
    - Separating them would create unnecessary coupling and complexity
    - Medium-granularity class strikes balance between SRP and cohesion

    Extracted from: StandaloneTracker.control_arm() lines 243-282

    Example:
        config = TrackerConfig(smoothing_alpha=0.5, deadzone=20)
        controller = TrackingController(config)

        # In tracking loop
        command = controller.update(object_x=350, object_y=240)
        if command.should_update:
            # Execute servo movement
            pan_servo.write_angle(command.pan_angle, time_ms=command.move_time_ms)
    """

    def __init__(self, config: TrackerConfig):
        """
        Initialize tracking controller with configuration.

        Args:
            config: TrackerConfig instance with all tuning parameters
        """
        self.config = config

        # State for position smoothing (exponential moving average)
        self.filtered_x = config.center_x
        self.filtered_y = config.center_y

        # State for rate limiting
        self.last_update_time = 0.0

        # State for deadzone hysteresis - holds last command when in deadzone
        self.last_command = ServoCommand(
            pan_angle=90,
            tilt_down_angle=90,
            tilt_up_angle=5,
            move_time_ms=config.servo_move_time_ms,
            should_update=False
        )

        print(f"[TrackingController] Initialized with:")
        print(f"  Smoothing alpha: {config.smoothing_alpha}")
        print(f"  Deadzone: {config.deadzone}px")
        print(f"  Gains: pan={config.pan_gain}, tilt={config.tilt_gain}")
        print(f"  Update interval: {config.servo_update_interval}s")

    def update(self, object_x: int, object_y: int) -> ServoCommand:
        """
        Calculate servo command from object position.

        This method performs the complete tracking control pipeline:
        1. Check rate limiting
        2. Apply position smoothing (EMA filter)
        3. Calculate error from center
        4. Check deadzone
        5. Apply proportional control
        6. Clamp angles to valid range

        Args:
            object_x: Object X position in pixels
            object_y: Object Y position in pixels

        Returns:
            ServoCommand with angles and should_update flag
        """
        # Step 1: Rate limiting check - don't update servos more than every 150ms
        current_time = time_module.time()
        if current_time - self.last_update_time < self.config.servo_update_interval:
            return self.last_command

        # Update time
        self.last_update_time = current_time

        # Step 2: Apply position smoothing using exponential moving average (EMA)
        # - This filters out noisy pixel-to-pixel jumps in detection
        self.filtered_x = self.config.smoothing_alpha * object_x + (1 - self.config.smoothing_alpha) * self.filtered_x
        self.filtered_y = self.config.smoothing_alpha * object_y + (1 - self.config.smoothing_alpha) * self.filtered_y

        # Step 3: Calculate error from frame center
        x_error = self.filtered_x - self.config.center_x
        y_error = self.filtered_y - self.config.center_y

        # Step 4: Deadzone check with hysteresis
        # - When object is within deadzone, return last command without updating
        # - This prevents oscillation when detection noise pushes position in/out of deadzone
        if abs(x_error) < self.config.deadzone and abs(y_error) < self.config.deadzone:
            return self.last_command

        # Step 5: Apply proportional control - calculate new angles
        pan_offset = x_error * self.config.pan_gain
        tilt_offset = y_error * self.config.tilt_gain
        pan_angle = 90 - pan_offset  # left/right from center

        # Tilt control logic using two servos:
        # - Servo 3 (tilt_down): 90° start, moves to 40° for downward tilt (object below center)
        # - Servo 4 (tilt_up): 5° start, moves to 60° for upward tilt (object above center)
        # Servos 2,3,4 are mechanically reversed in dofbot_lib.py

        # Start with both servos at rest positions
        tilt_down_angle = 90  # Servo 3 rest position
        tilt_up_angle = 5     # Servo 4 rest position

        if y_error > self.config.deadzone:
            # Object is below center - move arm UP (tilt down with Servo 3)
            # To move arm UP, reduce Servo 3 angle (mechanically reversed)
            tilt_down_angle = 90 - tilt_offset
            tilt_down_angle = max(40, min(90, tilt_down_angle))
        elif y_error < -self.config.deadzone:
            # Object is above center - move arm DOWN (tilt up with Servo 4)
            # To move arm DOWN, increase Servo 4 angle (mechanically reversed)
            # y_error is negative, so -tilt_offset becomes positive
            tilt_up_angle = 5 - tilt_offset
            tilt_up_angle = max(5, min(60, tilt_up_angle))

        # Debug logging for tilt tracking
        print(f"[Controller] y_error={y_error:6.1f}, tilt_down={tilt_down_angle:6.1f}°, tilt_up={tilt_up_angle:6.1f}°")

        # Log what will be sent to servo
        print(f"[Controller] COMMAND: pan_angle={int(pan_angle)}, tilt_down={int(tilt_down_angle)}, tilt_up={int(tilt_up_angle)}")

        # Step 6: Clamp pan angle to valid servo range [0, 180]
        pan_angle = max(0, min(180, pan_angle))

        # Create new command and store as last_command before returning
        new_command = ServoCommand(
            pan_angle=int(pan_angle),
            tilt_down_angle=int(tilt_down_angle),
            tilt_up_angle=int(tilt_up_angle),
            move_time_ms=self.config.servo_move_time_ms,
            should_update=True
        )
        self.last_command = new_command
        return new_command

    def reset(self):
        """
        Reset controller state to initial values.

        Call this when tracking stops or restarts to clear smoothed positions.
        """
        self.filtered_x = self.config.center_x
        self.filtered_y = self.config.center_y
        self.last_command = ServoCommand(
            pan_angle=90,
            tilt_down_angle=90,
            tilt_up_angle=5,
            move_time_ms=self.config.servo_move_time_ms,
            should_update=False
        )
        print("[TrackingController] State reset")


if __name__ == '__main__':
    """Test tracking controller logic."""
    print("Testing TrackingController...")

    # Create test configuration
    config = TrackerConfig()
    controller = TrackingController(config)

    print("\n--- Test 1: Center position (should be in deadzone) ---")
    command = controller.update(object_x=320, object_y=240)
    print(f"Result: should_update={command.should_update}, tilt_down={command.tilt_down_angle}, tilt_up={command.tilt_up_angle}")

    print("\n--- Test 2: Object to the right (should command left turn) ---")
    command = controller.update(object_x=450, object_y=240)
    print(f"Result: pan_angle={command.pan_angle}, should_update={command.should_update}")
    print(f"Expected: pan_angle < 90 (turn left to center object)")

    print("\n--- Test 3: Object below center (should move arm up) ---")
    command = controller.update(object_x=320, object_y=350)
    print(f"Result: tilt_down={command.tilt_down_angle}, tilt_up={command.tilt_up_angle}")
    print(f"Expected: tilt_down < 90 (arm up), tilt_up = 5 (rest)")

    print("\n--- Test 4: Object above center (should move arm down) ---")
    command = controller.update(object_x=320, object_y=100)
    print(f"Result: tilt_down={command.tilt_down_angle}, tilt_up={command.tilt_up_angle}")
    print(f"Expected: tilt_down = 90 (rest), tilt_up > 5 (arm down)")

    print("\n--- Test 5: Rate limiting (immediate second call) ---")
    command = controller.update(object_x=450, object_y=240)
    print(f"Result: should_update={command.should_update}")
    print(f"Expected: should_update=False (rate limited)")

    print("\n--- Test 6: Reset state ---")
    controller.reset()
    print("State reset complete")

    print("\n✓ Manual testing complete - verify expected behavior above")
