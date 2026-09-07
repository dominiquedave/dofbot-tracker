#!/usr/bin/env python3
"""
Tracker Configuration Module

Centralized configuration for the Dofbot color tracker system.
Consolidates all hardcoded parameters into a single, validated dataclass.

Usage:
    from tracker_config import TrackerConfig

    # Use defaults
    config = TrackerConfig()

    # Override specific values
    config = TrackerConfig(frame_width=1280, pan_gain=0.2)

    # Future: Load from YAML
    # config = TrackerConfig.from_yaml("tracker.yaml")
"""

from dataclasses import dataclass


@dataclass
class TrackerConfig:
    """
    Configuration for the Dofbot color tracker.

    This dataclass consolidates all tuning parameters that were previously
    scattered throughout StandaloneTracker. It follows the pattern of
    centralizing configuration for easier tuning and future YAML loading.

    Attributes are organized by category:
    - Camera: Resolution settings
    - Tracking: Control gains and deadzone
    - Performance: Rate limiting and frame skipping
    - Servo: Hardware IDs and movement timing
    - HSV: Color learning tolerances
    """

    # CAMERA CONFIGURATION
    frame_width: int = 320   # Reduced from 640 for higher FPS (75% fewer pixels)
    frame_height: int = 240  # Reduced from 480 for higher FPS

    # TRACKING CONFIGURATION
    deadzone: int = 20                # pixels
    pan_gain: float = 0.18            # proportional control (reduced from 0.28 for stability)
    tilt_gain: float = 0.18           # proportional control (reduced from 0.28 for stability)
    smoothing_alpha: float = 0.3      # 0=smooth/slow, 1=responsive/jittery

    # PERFORMANCE CONFIGURATION
    process_interval: int = 1           # process every frame (aligned with servo_update_interval)
    servo_update_interval: float = 0.2  # seconds between servo updates
    servo_move_time_ms: int = 400     # milliseconds for servo movement

    # SERVO CONFIGURATION
    pan_servo_id: int = 1             # horizontal servo
    # Tilt uses two servos: Servo 3 (down tilt, 90° start) and Servo 4 (up tilt, 5° start)
    # Both are mechanically reversed (handled automatically by dofbot_lib.angle_to_position)
    # Note: tilt_servo_id is kept for backward compatibility but not used by dual-servo tilt
    tilt_servo_id: int = 3            # kept for backward compatibility (now uses servos 3 & 4)

    # HSV COLOR LEARNING CONFIGURATION
    hue_tolerance: int = 5            # expand H range by ±5
    saturation_tolerance: int = 20    # expand S range by -20
    value_tolerance: int = 20         # expand V range by -20

    # BRIGHTNESS THRESHOLDS FOR DYNAMIC TOLERANCE ADJUSTMENT
    brightness_threshold_low: int = 100    # avg brightness below this: use loose tolerances
    brightness_threshold_high: int = 200   # avg brightness above this: use tight tolerances

    def __post_init__(self):
        """
        Validate configuration after initialization.

        This method runs automatically after __init__ to ensure
        all values are within acceptable ranges.
        """
        # Validate frame dimensions
        if self.frame_width <= 0 or self.frame_height <= 0:
            raise ValueError(f"Frame dimensions must be positive: {self.frame_width}x{self.frame_height}")

        # Validate gains
        if not (0.0 <= self.pan_gain <= 1.0):
            raise ValueError(f"pan_gain must be 0.0-1.0: {self.pan_gain}")
        if not (0.0 <= self.tilt_gain <= 1.0):
            raise ValueError(f"tilt_gain must be 0.0-1.0: {self.tilt_gain}")
        if not (0.0 <= self.smoothing_alpha <= 1.0):
            raise ValueError(f"smoothing_alpha must be 0.0-1.0: {self.smoothing_alpha}")

        # Validate servo IDs
        if not (1 <= self.pan_servo_id <= 6):
            raise ValueError(f"pan_servo_id must be 1-6: {self.pan_servo_id}")
        if not (1 <= self.tilt_servo_id <= 6):
            raise ValueError(f"tilt_servo_id must be 1-6: {self.tilt_servo_id}")

        # Validate tolerances
        if self.hue_tolerance < 0:
            raise ValueError(f"hue_tolerance must be positive: {self.hue_tolerance}")
        if self.saturation_tolerance < 0:
            raise ValueError(f"saturation_tolerance must be positive: {self.saturation_tolerance}")
        if self.value_tolerance < 0:
            raise ValueError(f"value_tolerance must be positive: {self.value_tolerance}")

        # Validate brightness thresholds
        if self.brightness_threshold_low < 0 or self.brightness_threshold_low > 255:
            raise ValueError(f"brightness_threshold_low must be 0-255: {self.brightness_threshold_low}")
        if self.brightness_threshold_high < 0 or self.brightness_threshold_high > 255:
            raise ValueError(f"brightness_threshold_high must be 0-255: {self.brightness_threshold_high}")
        if self.brightness_threshold_low >= self.brightness_threshold_high:
            raise ValueError(f"brightness_threshold_low ({self.brightness_threshold_low}) must be < high ({self.brightness_threshold_high})")

    @property
    def center_x(self) -> float:
        """Calculate center X coordinate from frame width."""
        return self.frame_width / 2

    @property
    def center_y(self) -> float:
        """Calculate center Y coordinate from frame height."""
        return self.frame_height / 2


if __name__ == '__main__':
    """Test configuration creation and validation."""
    print("Testing TrackerConfig...")

    # Test default config
    config = TrackerConfig()
    print(f"\nDefault config:")
    print(f"  Frame: {config.frame_width}x{config.frame_height}")
    print(f"  Center: ({config.center_x}, {config.center_y})")
    print(f"  Gains: pan={config.pan_gain}, tilt={config.tilt_gain}")
    print(f"  Deadzone: {config.deadzone}px")
    print(f"  Servos: pan={config.pan_servo_id}, tilt={config.tilt_servo_id}")

    # Test custom config
    custom = TrackerConfig(frame_width=1280, frame_height=720, pan_gain=0.2)
    print(f"\nCustom config:")
    print(f"  Frame: {custom.frame_width}x{custom.frame_height}")
    print(f"  Center: ({custom.center_x}, {custom.center_y})")
    print(f"  Servos: pan={custom.pan_servo_id}, tilt={custom.tilt_servo_id}")

    print("\n✓ TrackerConfig tests passed!")
