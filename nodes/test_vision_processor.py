#!/usr/bin/env python3
"""
Vision Processor Test Harness

This script provides an interactive test harness for the VisionProcessor
component, allowing isolated testing of color detection logic without
arm movement or mechanical components.

Controls:
    Mouse: Click and drag to select ROI
    SPACE: Learn color from ROI / Toggle detection
    'r': Reset selection
    'h': Toggle HSV trackbar panel
    's': Save current HSV settings to file
    'q': Quit
"""

import cv2 as cv
import numpy as np
import time
import sys
import json
import os
from typing import Tuple, Optional

# Import the components
sys.path.insert(0, '/home/pi/yahboomcar_ws/src/dofbot_tracker/nodes')
from tracker_config import TrackerConfig
from vision_lib import CameraManager, RegionSelector, VisionProcessor, DisplayManager, DetectionResult, ROI


class HSVTuner:
    """
    Manages HSV trackbars for interactive color range tuning.

    Provides real-time adjustment of HSV thresholds and callbacks
    when trackbar values change.
    """

    def __init__(self, window_name: str, initial_range: Optional[Tuple[Tuple[int, int, int], Tuple[int, int, int]]] = None):
        """
        Initialize HSV trackbars.

        Args:
            window_name: Name of the window to attach trackbars to
            initial_range: Optional initial (lower, upper) HSV range
        """
        # Initialize state (lazy initialization - window/trackbars created on first show)
        self.window_name = window_name
        self.h_min = 0
        self.h_max = 179
        self.s_min = 0
        self.s_max = 255
        self.v_min = 0
        self.v_max = 255
        self.callback = None
        self.visible = False
        self._initialized = False

        # Update from initial range if provided
        if initial_range:
            self.h_min = initial_range[0][0]
            self.h_max = initial_range[1][0]
            self.s_min = initial_range[0][1]
            self.s_max = initial_range[1][1]
            self.v_min = initial_range[0][2]
            self.v_max = initial_range[1][2]

    def _ensure_initialized(self):
        """
        Create window and trackbars if not already initialized.
        Called lazily before showing/hiding trackbars.
        """
        if self._initialized:
            return

        cv.namedWindow(self.window_name)
        cv.createTrackbar("H-min", self.window_name, self.h_min, 179, self._on_h_min)
        cv.createTrackbar("H-max", self.window_name, self.h_max, 179, self._on_h_max)
        cv.createTrackbar("S-min", self.window_name, self.s_min, 255, self._on_s_min)
        cv.createTrackbar("S-max", self.window_name, self.s_max, 255, self._on_s_max)
        cv.createTrackbar("V-min", self.window_name, self.v_min, 255, self._on_v_min)
        cv.createTrackbar("V-max", self.window_name, self.v_max, 255, self._on_v_max)

        self._initialized = True

    def _on_h_min(self, value):
        """Callback for H-min trackbar."""
        self.h_min = value
        self._update_range()

    def _on_h_max(self, value):
        """Callback for H-max trackbar."""
        self.h_max = value
        self._update_range()

    def _on_s_min(self, value):
        """Callback for S-min trackbar."""
        self.s_min = value
        self._update_range()

    def _on_s_max(self, value):
        """Callback for S-max trackbar."""
        self.s_max = value
        self._update_range()

    def _on_v_min(self, value):
        """Callback for V-min trackbar."""
        self.v_min = value
        self._update_range()

    def _on_v_max(self, value):
        """Callback for V-max trackbar."""
        self.v_max = value
        self._update_range()

    def _update_range(self):
        """Notify callback with updated HSV range."""
        if self.callback:
            lower = (self.h_min, self.s_min, self.v_min)
            upper = (self.h_max, self.s_max, self.v_max)
            self.callback(lower, upper)

    def set_callback(self, callback):
        """
        Set callback function for when trackbars change.

        Args:
            callback: Function taking (lower, upper) HSV tuples
        """
        self.callback = callback

    def get_hsv_range(self) -> Tuple[Tuple[int, int, int], Tuple[int, int, int]]:
        """Get current HSV range from trackbars."""
        return ((self.h_min, self.s_min, self.v_min), (self.h_max, self.s_max, self.v_max))

    def show(self):
        """Show trackbars by restoring their positions."""
        self._ensure_initialized()
        self.visible = True
        cv.setTrackbarPos("H-min", self.window_name, self.h_min)
        cv.setTrackbarPos("H-max", self.window_name, self.h_max)
        cv.setTrackbarPos("S-min", self.window_name, self.s_min)
        cv.setTrackbarPos("S-max", self.window_name, self.s_max)
        cv.setTrackbarPos("V-min", self.window_name, self.v_min)
        cv.setTrackbarPos("V-max", self.window_name, self.v_max)

    def hide(self):
        """Hide trackbars by moving them off-screen."""
        self._ensure_initialized()
        self.visible = False
        # Trackbars can't be truly hidden, so we move them below window
        h = cv.getWindowProperty(self.window_name, cv.WND_PROP_ASPECT_RATIO)
        window_height = 500  # Default window height
        offset = window_height + 50
        cv.setTrackbarPos("H-min", self.window_name, 0)
        cv.setTrackbarPos("H-max", self.window_name, 0)
        cv.setTrackbarPos("S-min", self.window_name, 0)
        cv.setTrackbarPos("S-max", self.window_name, 0)
        cv.setTrackbarPos("V-min", self.window_name, 0)
        cv.setTrackbarPos("V-max", self.window_name, 0)

    def is_visible(self):
        """Check if trackbars are currently visible."""
        return self.visible

    def toggle(self):
        """Toggle trackbar visibility."""
        if self.visible:
            self.hide()
        else:
            self.show()


class VisionProcessorTestHarness:
    """
    Interactive test harness for VisionProcessor.

    Provides:
    - Interactive ROI selection
    - Color learning from ROI
    - Real-time HSV tuning via trackbars
    - Pipeline visualization (original, mask, filtered, detected)
    - Detection statistics and status
    """

    def __init__(self, config: TrackerConfig = None):
        """
        Initialize the test harness.

        Args:
            config: TrackerConfig instance (creates default if None)
        """
        self.config = config or TrackerConfig()

        # Create components
        self.camera = CameraManager(
            width=self.config.frame_width,
            height=self.config.frame_height
        )

        self.region_selector = RegionSelector()
        self.vision_processor = VisionProcessor(self.config)
        self.display_manager = DisplayManager(
            frame_width=self.config.frame_width,
            frame_height=self.config.frame_height
        )

        # HSV Tuner
        self.hsv_tuner = HSVTuner("Vision Processor", None)
        self.hsv_tuner.set_callback(self._on_hsv_change)

        # State
        self.tracking = False
        self.show_trackbars = False
        self.last_detection = None
        self.start_time = time.time()
        self.frame_count = 0
        self.fps = 0.0

        # Setup window
        cv.namedWindow("Vision Processor Test")
        cv.setMouseCallback("Vision Processor Test", self._mouse_callback)

        # HSV settings save path
        self.hsv_save_path = "/home/pi/yahboomcar_ws/src/dofbot_tracker/nodes/hsv_settings.json"

    def _mouse_callback(self, event, x, y, flags, param):
        """Internal mouse callback wrapper."""
        self.region_selector.handle_event(event, x, y, flags, param)

    def _on_hsv_change(self, lower: Tuple[int, int, int], upper: Tuple[int, int, int]):
        """Handle HSV range changes from trackbars."""
        self.vision_processor.set_hsv_range(lower, upper)

    def _learn_color(self, frame):
        """Learn color from selected ROI."""
        roi = self.region_selector.get_roi()
        if roi is None:
            print("[TestHarness] No valid ROI selected")
            return False

        print("[TestHarness] Learning color from selected region...")
        self.vision_processor.learn_color_from_roi(frame, roi)
        self.tracking = True

        # Initialize trackbars with learned range
        if self.vision_processor.hsv_range:
            lower, upper = self.vision_processor.hsv_range
            self.hsv_tuner = HSVTuner("Vision Processor", (lower, upper))
            self.hsv_tuner.set_callback(self._on_hsv_change)
            self.hsv_tuner.show()
            self.show_trackbars = True

        print("[TestHarness] Tracking enabled!")
        return True

    def _save_hsv_settings(self):
        """Save current HSV settings to file."""
        if self.vision_processor.hsv_range is None:
            print("[TestHarness] No HSV range to save!")
            return

        lower, upper = self.vision_processor.hsv_range
        settings = {
            "h_min": lower[0],
            "s_min": lower[1],
            "v_min": lower[2],
            "h_max": upper[0],
            "s_max": upper[1],
            "v_max": upper[2],
            "timestamp": time.strftime("%Y-%m-%d %H:%M:%S")
        }

        try:
            with open(self.hsv_save_path, 'w') as f:
                json.dump(settings, f, indent=2)
            print(f"[TestHarness] HSV settings saved to {self.hsv_save_path}")
        except Exception as e:
            print(f"[TestHarness] Error saving HSV settings: {e}")

    def _reset(self):
        """Reset tracker state."""
        print("[TestHarness] Resetting...")
        self.region_selector.reset()
        self.vision_processor.reset()
        self.tracking = False
        self.hsv_tuner.hide()
        self.show_trackbars = False

    def _process_frame(self, frame):
        """
        Process a single frame and return processed frames for visualization.

        Args:
            frame: Input BGR frame from camera

        Returns:
            Dictionary with 'original', 'mask', 'filtered', 'detected' frames
        """
        # Detect object
        result = self.vision_processor.detect(frame)
        self.last_detection = result

        # Build pipeline visualization
        frames = {'original': frame.copy()}

        if result.found:
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

        # Draw overlay on original
        if not self.region_selector.is_selecting() and self.region_selector.get_roi():
            cv.putText(original, "SPACE: Learn Color", (10, h - 60),
                      cv.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 1)

        # Draw detection info
        if self.last_detection:
            info_text = f"Found: {self.last_detection.found}"
            cv.putText(original, info_text, (10, h - 80),
                      cv.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)

            if self.last_detection.found:
                cv.putText(original, f"Pos: ({self.last_detection.x}, {self.last_detection.y})",
                          (10, h - 100), cv.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)
                cv.putText(original, f"Radius: {self.last_detection.radius}px",
                          (10, h - 120), cv.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)

        # Draw selection rectangle on original
        selection = self.region_selector.get_current_selection()
        if selection:
            x1, y1, x2, y2 = selection
            cv.rectangle(original, (x1, y1), (x2, y2), (0, 255, 0), 2)

        # Build 2x2 grid
        top_row = np.hstack([original, mask])
        bottom_row = np.hstack([filtered, detected])
        combined = np.vstack([top_row, bottom_row])

        return combined

    def _update_fps(self):
        """Calculate and update FPS counter."""
        self.frame_count += 1
        elapsed = time.time() - self.start_time
        if elapsed >= 1.0:
            self.fps = self.frame_count / elapsed
            self.frame_count = 0
            self.start_time = time.time()

    def run(self):
        """Main loop."""
        print("=" * 60)
        print("Vision Processor Test Harness")
        print("=" * 60)
        print("\nControls:")
        print("  Mouse: Click and drag to select colored region")
        print("  SPACE: Learn color from ROI / Enable detection")
        print("  'r':   Reset selection")
        print("  'h':   Toggle HSV trackbar panel")
        print("  's':   Save current HSV settings")
        print("  'q':   Quit\n")

        # Show instructions on first frame
        ret, frame = self.camera.read()
        if not ret:
            print("[TestHarness] Error reading frame!")
            self.camera.release()
            return

        # Show instructions on frame
        instructions = [
            "SELECT REGION: Click and drag to select object",
            "SPACE: Learn color and enable tracking",
            "'h': Toggle HSV trackbars",
            "'s': Save HSV settings",
            "'q': Quit"
        ]
        for i, instr in enumerate(instructions):
            y = 50 + i * 30
            cv.putText(frame, instr, (20, y), cv.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 1)
        cv.imshow("Vision Processor Test", frame)
        cv.waitKey(0)

        print("[TestHarness] Starting main loop...\n")

        while True:
            ret, frame = self.camera.read()
            if not ret:
                print("[TestHarness] Error reading frame!")
                break

            # Update FPS
            self._update_fps()

            # Get current selection for overlay
            selection = self.region_selector.get_current_selection()
            roi_present = self.region_selector.get_roi() is not None

            # Render selection rectangle
            if selection:
                x1, y1, x2, y2 = selection
                cv.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 0), 2)

            # Handle detection
            if self.tracking and self.vision_processor.hsv_range is not None:
                frames = self._process_frame(frame)
                combined = self._draw_pipeline(frames)

                # Add FPS overlay
                cv.putText(combined, f"FPS: {self.fps:.1f}", (10, 30),
                          cv.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)

                # Add status overlay
                status_y = combined.shape[0] - 60
                cv.putText(combined, "DETECTION ACTIVE", (10, status_y),
                          cv.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
                status_y += 30
                cv.putText(combined, f"HSV: ({self.vision_processor.hsv_range[0]}) - ({self.vision_processor.hsv_range[1]})",
                          (10, status_y), cv.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 0), 1)
            else:
                # Show original frame with status
                cv.putText(frame, "SELECT REGION - Press SPACE to track", (20, 50),
                          cv.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)
                cv.putText(frame, f"FPS: {self.fps:.1f}", (10, self.config.frame_height - 30),
                          cv.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)
                combined = frame

            cv.imshow("Vision Processor Test", combined)

            # Handle keys
            key = cv.waitKey(1) & 0xFF

            if key == ord('q') or key == 27:
                print("\n[TestHarness] Quitting...")
                break

            elif key == ord(' '):  # SPACE
                if not self.tracking:
                    if self.region_selector.get_roi() is not None:
                        if not self._learn_color(frame):
                            continue
                    else:
                        print("[TestHarness] Please select a region first!")

            elif key == ord('r') or key == ord('R'):
                self._reset()

            elif key == ord('h') or key == ord('H'):
                self.show_trackbars = not self.show_trackbars
                if self.show_trackbars:
                    self.hsv_tuner.show()
                else:
                    self.hsv_tuner.hide()

            elif key == ord('s') or key == ord('S'):
                self._save_hsv_settings()

        # Cleanup
        self.camera.release()
        cv.destroyAllWindows()
        print("[TestHarness] Done!")


def main():
    """Main entry point."""
    config = TrackerConfig()
    harness = VisionProcessorTestHarness(config)
    harness.run()


if __name__ == '__main__':
    main()
