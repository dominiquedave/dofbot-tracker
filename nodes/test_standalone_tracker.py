#!/usr/bin/env python3
"""
Standalone test script for Dofbot Pi Arm Tracking

This script tests the arm tracking functionality without ROS.
It uses the same algorithm as the ROS node but runs independently.

Usage:
    python3 test_standalone_tracker.py

Controls:
    SPACE: Start/stop tracking
    'r': Reset (deselect region)
    'q': Quit
"""

import cv2 as cv
import numpy as np
import time
import sys
import os

# Import the components
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from dofbot_lib import ArmController
from tracker_config import TrackerConfig
from vision_lib import CameraManager, RegionSelector, VisionProcessor, DisplayManager, DetectionResult, ROI
from tracker_controller import TrackingController, ServoCommand


class StandaloneTracker:
    """
    Standalone color tracker with arm control (no ROS required).

    This class serves as the main orchestrator, wiring together the
    component classes following the Dependency Inversion Principle.

    State machine: IDLE → REGION_SELECTED → LEARNING → TRACKING

    Usage:
        config = TrackerConfig()
        tracker = StandaloneTracker(config=config)
        tracker.run()
    """

    def __init__(self, config=None, camera=None, detector=None, controller=None):
        """
        Initialize the tracker with dependency injection.

        Args:
            config: TrackerConfig instance (creates default if None)
            camera: CameraManager instance (creates default if None)
            detector: VisionProcessor instance (creates default if None)
            controller: TrackingController instance (creates default if None)
        """
        self.config = config or TrackerConfig()

        # Create components with dependency injection
        self.camera = camera or CameraManager(
            width=self.config.frame_width,
            height=self.config.frame_height
        )

        self.region_selector = RegionSelector()
        self.vision_processor = detector or VisionProcessor(self.config)
        self.display_manager = DisplayManager(
            frame_width=self.config.frame_width,
            frame_height=self.config.frame_height
        )
        self.controller = controller or TrackingController(self.config)

        # Arm controllers
        self.pan_arm = ArmController(servo_id=self.config.pan_servo_id)
        # Tilt uses two servos: Servo 3 for downward tilt (start: 90°),
        # Servo 4 for upward tilt (start: 5°)
        self.tilt_down_arm = ArmController(servo_id=3)   # Mechanically reversed
        self.tilt_up_arm = ArmController(servo_id=4)     # Mechanically reversed

        # Tracking state
        self.tracking = False
        self.frame_count = 0

        # Setup window and mouse
        cv.namedWindow("Color Tracker - Standalone")
        cv.setMouseCallback("Color Tracker - Standalone", self._mouse_callback)

    def _mouse_callback(self, event, x, y, flags, param):
        """Internal mouse callback wrapper."""
        self.region_selector.handle_event(event, x, y, flags, param)

    def _learn_color(self, frame):
        """Learn color from selected region."""
        roi = self.region_selector.get_roi()
        if roi is None:
            print("[StandaloneTracker] No valid ROI selected")
            return False

        print("[StandaloneTracker] Learning color from selected region...")
        self.vision_processor.learn_color_from_roi(frame, roi)
        self.tracking = True
        print("[StandaloneTracker] Tracking enabled!")
        return True

    def _stop_tracking(self):
        """Stop tracking and reset arm."""
        print("[StandaloneTracker] Disabling tracking...")
        self.tracking = False
        self.pan_arm.move_to_center()
        # Return tilt servos to start positions with angle checks
        current_down = self.tilt_down_arm.read_angle(3)
        if current_down is None or abs(current_down - 90) > 10:
            time.sleep((300 / 1000) * 1.2)
            self.tilt_down_arm.write_angle(90, time_ms=300)
        else:
            print(f"  Tilt down: skipped (already at {current_down}°)")
        current_up = self.tilt_up_arm.read_angle(4)
        if current_up is None or abs(current_up - 5) > 10:
            time.sleep((300 / 1000) * 1.2)
            self.tilt_up_arm.write_angle(5, time_ms=300)
        else:
            print(f"  Tilt up: skipped (already at {current_up}°)")
        time.sleep(0.3)

    def _reset(self):
        """Reset tracker state."""
        print("[StandaloneTracker] Resetting...")
        self.region_selector.reset()
        self.vision_processor.reset()
        self.controller.reset()
        self.tracking = False
        self.pan_arm.move_to_center()
        # Return tilt servos to start positions with angle checks
        current_down = self.tilt_down_arm.read_angle(3)
        if current_down is None or abs(current_down - 90) > 10:
            time.sleep((300 / 1000) * 1.2)
            self.tilt_down_arm.write_angle(90, time_ms=300)
        else:
            print(f"  Tilt down: skipped (already at {current_down}°)")
        current_up = self.tilt_up_arm.read_angle(4)
        if current_up is None or abs(current_up - 5) > 10:
            time.sleep((300 / 1000) * 1.2)
            self.tilt_up_arm.write_angle(5, time_ms=300)
        else:
            print(f"  Tilt up: skipped (already at {current_up}°)")
        time.sleep(0.3)

    def _draw_pipeline(self, frames: dict) -> np.ndarray:
        """
        Combine pipeline frames into single visualization.

        Layout:
        +------------------+------------------+
        |   ORIGINAL       |     MASK         |
        +------------------+------------------+
        |   FILTERED       |    DETECTED      |
        +------------------+------------------+
        """
        h, w = self.config.frame_height, self.config.frame_width

        # Create empty frames if missing
        original = frames.get('original')
        mask = frames.get('mask', np.zeros((h, w, 3), dtype=np.uint8))
        filtered = frames.get('filtered', np.zeros((h, w, 3), dtype=np.uint8))
        detected = frames.get('detected', np.zeros((h, w, 3), dtype=np.uint8))

        if original is None:
            original = np.zeros((h, w, 3), dtype=np.uint8)

        # Draw selection rectangle on original
        selection = self.region_selector.get_current_selection()
        if selection:
            x1, y1, x2, y2 = selection
            cv.rectangle(original, (x1, y1), (x2, y2), (0, 255, 0), 2)

        # Draw detection info
        if hasattr(self, 'last_detection') and self.last_detection:
            info_text = f"Found: {self.last_detection.found}"
            cv.putText(original, info_text, (10, h - 80),
                      cv.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)

            if self.last_detection.found:
                cv.putText(original, f"Pos: ({self.last_detection.x}, {self.last_detection.y})",
                          (10, h - 100), cv.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)
                cv.putText(original, f"Radius: {self.last_detection.radius}px",
                          (10, h - 120), cv.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)

        # Build 2x2 grid
        top_row = np.hstack([original, mask])
        bottom_row = np.hstack([filtered, detected])
        combined = np.vstack([top_row, bottom_row])

        return combined

    def _process_frame(self, frame):
        """
        Process a single frame during tracking.

        Args:
            frame: Input BGR frame from camera

        Returns:
            Dictionary with pipeline frames: 'original', 'mask', 'filtered', 'detected'
        """
        # Detect object
        result = self.vision_processor.detect(frame)
        self.last_detection = result

        # Build pipeline visualization
        frames = {'original': frame.copy()}

        if result.found:
            # Update controller with detected position
            command = self.controller.update(result.x, result.y)

            if command.should_update:
                # Execute servo movement with dual-servo tilt logic
                print(f"[TestTracker] Writing pan={command.pan_angle}°, tilt_down={command.tilt_down_angle}°, tilt_up={command.tilt_up_angle}°")
                # Pan servo with angle check
                current_pan = self.pan_arm.read_angle(self.config.pan_servo_id)
                if current_pan is None or abs(current_pan - command.pan_angle) > 10:
                    time.sleep((command.move_time_ms / 1000) * 1.2)
                    self.pan_arm.write_angle(command.pan_angle, time_ms=command.move_time_ms)
                else:
                    print(f"  Pan: skipped (already at {current_pan}°)")
                # Tilt down servo with angle check
                current_down = self.tilt_down_arm.read_angle(3)
                if current_down is None or abs(current_down - command.tilt_down_angle) > 10:
                    time.sleep((command.move_time_ms / 1000) * 1.2)
                    self.tilt_down_arm.write_angle(command.tilt_down_angle, time_ms=command.move_time_ms)
                else:
                    print(f"  Tilt down: skipped (already at {current_down}°)")
                # Tilt up servo with angle check
                current_up = self.tilt_up_arm.read_angle(4)
                if current_up is None or abs(current_up - command.tilt_up_angle) > 10:
                    time.sleep((command.move_time_ms / 1000) * 1.2)
                    self.tilt_up_arm.write_angle(command.tilt_up_angle, time_ms=command.move_time_ms)
                else:
                    print(f"  Tilt up: skipped (already at {current_up}°)")

            # Get HSV range for mask
            if self.vision_processor.hsv_range:
                lower = np.array(self.vision_processor.hsv_range[0], dtype="uint8")
                upper = np.array(self.vision_processor.hsv_range[1], dtype="uint8")

                # Create mask
                hsv_image = cv.cvtColor(frame, cv.COLOR_BGR2HSV)
                mask = cv.inRange(hsv_image, lower, upper)

                # Apply morphological closing
                kernel = cv.getStructuringElement(cv.MORPH_RECT, (5, 5))
                filtered = cv.morphologyEx(mask, cv.MORPH_CLOSE, kernel)

                # Convert to 3-channel for display
                mask_rgb = cv.cvtColor(mask, cv.COLOR_GRAY2BGR)
                filtered_rgb = cv.cvtColor(filtered, cv.COLOR_GRAY2BGR)

                frames['mask'] = mask_rgb
                frames['filtered'] = filtered_rgb

                # Draw detection on detected frame
                detected = frame.copy()
                cv.circle(detected, (result.x, result.y), result.radius, (255, 0, 255), 2)
                cv.circle(detected, (result.x, result.y), 3, (0, 0, 255), -1)

                # Draw center crosshair
                center_x = self.config.frame_width // 2
                center_y = self.config.frame_height // 2
                cv.line(detected, (center_x, 0), (center_x, self.config.frame_height), (255, 255, 0), 1)
                cv.line(detected, (0, center_y), (self.config.frame_width, center_y), (255, 255, 0), 1)

                frames['detected'] = detected

        return frames

    def run(self):
        """Main loop."""
        print("=" * 60)
        print("Standalone Dofbot Arm Tracker")
        print("=" * 60)
        print("\nControls:")
        print("  Mouse: Click and drag to select colored region")
        print("  SPACE: Learn color and start/stop tracking")
        print("  'r':   Reset selection")
        print("  'q':   Quit\n")

        # Reset arm to start positions with angle checks
        print("[StandaloneTracker] Moving arm to start position...")
        current_pan = self.pan_arm.read_angle(self.config.pan_servo_id)
        if current_pan is None or abs(current_pan - 90) > 10:
            time.sleep((500 / 1000) * 1.2)
            self.pan_arm.write_angle(90, time_ms=500)
        else:
            print(f"  Pan: skipped (already at {current_pan}°)")
        current_down = self.tilt_down_arm.read_angle(3)
        if current_down is None or abs(current_down - 90) > 10:
            time.sleep((500 / 1000) * 1.2)
            self.tilt_down_arm.write_angle(90, time_ms=500)
        else:
            print(f"  Tilt down: skipped (already at {current_down}°)")
        current_up = self.tilt_up_arm.read_angle(4)
        if current_up is None or abs(current_up - 5) > 10:
            time.sleep((500 / 1000) * 1.2)
            self.tilt_up_arm.write_angle(5, time_ms=500)
        else:
            print(f"  Tilt up: skipped (already at {current_up}°)")
        time.sleep(0.5)

        print("\n[StandaloneTracker] Starting main loop...\n")

        while True:
            ret, frame = self.camera.read()
            if not ret:
                print("[StandaloneTracker] Error reading frame!")
                break

            # Update FPS counter
            self.display_manager.update_fps()
            self.frame_count += 1

            # Determine if we should process this frame
            should_process = (self.frame_count % self.config.process_interval == 0)

            # Display mode and process frame
            if self.tracking and should_process:
                frames = self._process_frame(frame)
                combined = self._draw_pipeline(frames)

                # Add FPS overlay on pipeline
                cv.putText(combined, f"FPS: {self.display_manager.fps:.1f}", (10, 30),
                          cv.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)

                # Add status overlay
                status_y = combined.shape[0] - 60
                cv.putText(combined, "DETECTION ACTIVE", (10, status_y),
                          cv.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
                status_y += 30
                if self.vision_processor.hsv_range:
                    cv.putText(combined, f"HSV: ({self.vision_processor.hsv_range[0]}) - ({self.vision_processor.hsv_range[1]})",
                              (10, status_y), cv.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 0), 1)

                display_frame = combined
            else:
                # Show original frame with status
                selection = self.region_selector.get_current_selection()
                self.display_manager.render_selection(frame, selection)
                self.display_manager.render_status(frame, self.tracking, self.region_selector.get_roi() is not None)
                display_frame = frame

            cv.imshow("Color Tracker - Standalone", display_frame)

            # Handle keys
            key = cv.waitKey(1) & 0xFF

            if key == ord('q') or key == 27:
                print("\n[StandaloneTracker] Quitting...")
                break

            elif key == ord(' '):  # SPACE
                if not self.tracking:
                    if self.region_selector.get_roi() is not None:
                        if not self._learn_color(frame):
                            continue
                    else:
                        print("[StandaloneTracker] Please select a region first!")
                else:
                    self._stop_tracking()

            elif key == ord('r') or key == ord('R'):
                self._reset()

        # Cleanup
        self.camera.release()
        cv.destroyAllWindows()
        self.pan_arm.move_to_center()
        # Return tilt servos to start positions with angle checks
        current_down = self.tilt_down_arm.read_angle(3)
        if current_down is None or abs(current_down - 90) > 10:
            time.sleep((300 / 1000) * 1.2)
            self.tilt_down_arm.write_angle(90, time_ms=300)
        else:
            print(f"  Tilt down: skipped (already at {current_down}°)")
        current_up = self.tilt_up_arm.read_angle(4)
        if current_up is None or abs(current_up - 5) > 10:
            time.sleep((300 / 1000) * 1.2)
            self.tilt_up_arm.write_angle(5, time_ms=300)
        else:
            print(f"  Tilt up: skipped (already at {current_up}°)")
        print("[StandaloneTracker] Done!")


if __name__ == '__main__':
    tracker = StandaloneTracker()
    tracker.run()
