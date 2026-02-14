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

# Import the arm controller
sys.path.insert(0, '/home/pi/yahboomcar_ws/src/dofbot_tracker/nodes')
from dofbot_lib import ArmController


class StandaloneTracker:
    """
    Standalone color tracker with arm control (no ROS required).

    This class demonstrates the same tracking algorithm as the ROS node
    but runs independently for testing.
    """

    def __init__(self):
        # Camera setup
        self.cap = None
        self.frame_width = 640
        self.frame_height = 480

        # Tracking state
        self.tracking = False
        self.object_x = 0
        self.object_y = 0
        self.object_radius = 0

        # Selection state
        self.selecting = False
        self.selection_start = (0, 0)
        self.selection_end = (0, 0)
        self.roi = None

        # HSV range for color detection
        self.hsv_range = None

        # Arm controllers
        self.pan_arm = ArmController(servo_id=1)
        self.tilt_arm = ArmController(servo_id=2)

        # Tracking parameters
        self.deadzone = 20
        self.pan_gain = 0.15  # Reduced from 0.5 for smoother control
        self.tilt_gain = 0.15  # Reduced from 0.5 for smoother control

        # Center of frame
        self.center_x = self.frame_width / 2
        self.center_y = self.frame_height / 2

        # Rate limiting - don't update servos every frame
        self.last_servo_update = 0
        self.servo_update_interval = 0.2  # Update servos at most every 200ms

        # Position smoothing
        self.filtered_x = self.center_x
        self.filtered_y = self.center_y

        # Frame skipping for performance
        self.frame_count = 0
        self.process_interval = 2  # Process every Nth frame

        # FPS tracking
        self.fps = 0
        self.fps_update_time = time.time()
        self.fps_frame_count = 0

        # Initialize camera
        self.init_camera()

    def init_camera(self):
        """Initialize camera with multiple fallback methods."""
        methods = [
            ("Camera index 0", 0, None),
            ("Device path /dev/video0", "/dev/video0", None),
        ]

        for name, device, backend in methods:
            print(f"Trying: {name}...", end=" ")
            try:
                if backend is not None:
                    cap = cv.VideoCapture(device, backend)
                else:
                    cap = cv.VideoCapture(device)

                if cap.isOpened():
                    cap.set(cv.CAP_PROP_FRAME_WIDTH, self.frame_width)
                    cap.set(cv.CAP_PROP_FRAME_HEIGHT, self.frame_height)
                    self.cap = cap
                    print("Success!")
                    break
                else:
                    print("Failed")
                    cap.release()
            except Exception as e:
                print(f"Error: {e}")

        if self.cap is None or not self.cap.isOpened():
            print("\nERROR: Cannot open camera!")
            print("Check that camera is not being used by another process.")
            print("Try: sudo pkill -f YahboomArm")
            sys.exit(1)

    def mouse_callback(self, event, x, y, flags, param):
        """Handle mouse events for region selection."""
        if event == cv.EVENT_LBUTTONDOWN:
            self.selecting = True
            self.selection_start = (x, y)
            self.selection_end = (x, y)

        elif event == cv.EVENT_MOUSEMOVE:
            if self.selecting:
                self.selection_end = (x, y)

        elif event == cv.EVENT_LBUTTONUP:
            self.selecting = False
            self.selection_end = (x, y)

            # Calculate ROI
            x1 = min(self.selection_start[0], self.selection_end[0])
            y1 = min(self.selection_start[1], self.selection_end[1])
            x2 = max(self.selection_start[0], self.selection_end[0])
            y2 = max(self.selection_start[1], self.selection_end[1])

            if x2 - x1 > 10 and y2 - y1 > 10:
                self.roi = (x1, y1, x2, y2)
                print(f"Region selected: {self.roi}")
                print("Press SPACE to learn color and start tracking")

    def learn_color_from_roi(self, image):
        """Learn HSV color range from selected region."""
        if self.roi is None:
            return None

        x1, y1, x2, y2 = self.roi

        # Convert to HSV
        hsv_image = cv.cvtColor(image, cv.COLOR_BGR2HSV)

        # Extract ROI using NumPy slicing (much faster than loops)
        roi_hsv = hsv_image[y1:y2, x1:x2]

        # Extract each channel and calculate min/max using vectorized operations
        H_values = roi_hsv[:, :, 0]
        S_values = roi_hsv[:, :, 1]
        V_values = roi_hsv[:, :, 2]

        # Calculate range with tolerance
        H_min = max(0, int(np.min(H_values)) - 5)
        H_max = min(255, int(np.max(H_values)) + 5)
        S_min = max(0, int(np.min(S_values)) - 20)
        S_max = 253
        V_min = max(0, int(np.min(V_values)) - 20)
        V_max = 255

        self.hsv_range = (
            (int(H_min), int(S_min), int(V_min)),
            (int(H_max), int(S_max), int(V_max))
        )

        print(f"Learned HSV range:")
        print(f"  Lower: H={H_min:.0f}, S={S_min:.0f}, V={V_min:.0f}")
        print(f"  Upper: H={H_max:.0f}, S={S_max:.0f}, V={V_max:.0f}")
        print("Tracking ready! Press SPACE to enable tracking.")
        return self.hsv_range

    def detect_object(self, image, hsv_range):
        """Detect colored object in image."""
        if hsv_range is None:
            return image, False

        # Convert to HSV
        hsv_image = cv.cvtColor(image, cv.COLOR_BGR2HSV)

        # Create mask
        lower = np.array(hsv_range[0], dtype="uint8")
        upper = np.array(hsv_range[1], dtype="uint8")
        mask = cv.inRange(hsv_image, lower, upper)

        # Morphological operations
        kernel = cv.getStructuringElement(cv.MORPH_RECT, (5, 5))
        mask = cv.morphologyEx(mask, cv.MORPH_CLOSE, kernel)

        # Threshold
        _, binary = cv.threshold(mask, 10, 255, cv.THRESH_BINARY)

        # Find contours
        contours, _ = cv.findContours(binary, cv.RETR_EXTERNAL, cv.CHAIN_APPROX_SIMPLE)

        if len(contours) > 0:
            # Find largest contour
            areas = [cv.contourArea(c) for c in contours]
            max_index = areas.index(max(areas))
            largest = contours[max_index]

            # Get bounding circle
            (cx, cy), radius = cv.minEnclosingCircle(largest)
            self.object_x = int(cx)
            self.object_y = int(cy)
            self.object_radius = int(radius)

            # Draw on image
            cv.circle(image, (self.object_x, self.object_y), self.object_radius, (255, 0, 255), 2)
            cv.circle(image, (self.object_x, self.object_y), 3, (0, 0, 255), -1)

            return image, True

        self.object_x = 0
        self.object_y = 0
        self.object_radius = 0
        return image, False

    def update_fps(self):
        """Calculate and update FPS counter."""
        self.fps_frame_count += 1
        current_time = time.time()
        elapsed = current_time - self.fps_update_time

        # Update FPS every second
        if elapsed >= 1.0:
            self.fps = self.fps_frame_count / elapsed
            self.fps_frame_count = 0
            self.fps_update_time = current_time

    def control_arm(self):
        """Control arm servos based on object position."""
        if not self.tracking or self.object_x == 0:
            return

        # Rate limiting - don't update too frequently
        current_time = time.time()
        if current_time - self.last_servo_update < self.servo_update_interval:
            return
        self.last_servo_update = current_time

        # Position smoothing using exponential moving average (EMA)
        # Filters out noisy pixel-to-pixel jumps in color detection
        alpha = 0.5  # Balance: 0.3=smooth/slow, 0.7=responsive/jittery
        self.filtered_x = alpha * self.object_x + (1 - alpha) * self.filtered_x
        self.filtered_y = alpha * self.object_y + (1 - alpha) * self.filtered_y

        # Calculate error from center
        x_error = self.filtered_x - self.center_x
        y_error = self.filtered_y - self.center_y

        # Deadzone check
        if abs(x_error) < self.deadzone and abs(y_error) < self.deadzone:
            return

        # Calculate target angles
        pan_offset = x_error * self.pan_gain
        tilt_offset = y_error * self.tilt_gain

        pan_angle = 90 - pan_offset
        tilt_angle = 90 + tilt_offset  # Invert Y

        # Clamp to valid ranges
        pan_angle = max(0, min(180, pan_angle))
        tilt_angle = max(0, min(180, tilt_angle))

        # Send to servos with slower movement time
        self.pan_arm.write_angle(int(pan_angle), time_ms=400)
        self.tilt_arm.write_angle(int(tilt_angle), time_ms=400)

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

        # Setup window and mouse
        cv.namedWindow("Color Tracker - Standalone")
        cv.setMouseCallback("Color Tracker - Standalone", self.mouse_callback)

        # Reset arm to center
        print("Moving arm to center position...")
        self.pan_arm.move_to_center()
        self.tilt_arm.move_to_center()
        time.sleep(0.5)

        print("\nStarting main loop...\n")

        while True:
            ret, frame = self.cap.read()
            if not ret:
                print("Error reading frame!")
                break

            # Update FPS counter
            self.update_fps()
            self.frame_count += 1

            # Determine if we should process this frame
            should_process = (self.frame_count % self.process_interval == 0)

            # Only copy frame if we need to draw on it (avoid unnecessary copy)
            display = frame

            # Draw selection rectangle
            if self.selecting or self.roi is not None:
                if self.selecting:
                    x1, y1 = self.selection_start
                    x2, y2 = self.selection_end
                else:
                    x1, y1, x2, y2 = self.roi

                cv.rectangle(display, (x1, y1), (x2, y2), (0, 255, 0), 2)

            # Display mode
            if self.tracking and should_process:
                cv.putText(display, "TRACKING ACTIVE", (10, 30),
                          cv.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)

                # Detect and track
                display, found = self.detect_object(frame, self.hsv_range)
                if found:
                    self.control_arm()

                    # Draw center crosshair
                    center_x = self.frame_width // 2
                    center_y = self.frame_height // 2
                    cv.line(display, (center_x, 0), (center_x, self.frame_height), (255, 255, 0), 1)
                    cv.line(display, (0, center_y), (self.frame_width, center_y), (255, 255, 0), 1)

                    # Draw error indicators
                    cv.arrowedLine(display, (center_x, center_y), (self.object_x, self.object_y),
                                  (0, 255, 255), 2)

            elif self.tracking and not should_process:
                # Skipped frame - still show tracking status
                cv.putText(display, "TRACKING ACTIVE", (10, 30),
                          cv.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
            else:
                cv.putText(display, "SELECT REGION - Press SPACE to track", (10, 30),
                          cv.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)

            # Show FPS and instructions
            cv.putText(display, f"FPS: {self.fps:.1f}", (10, self.frame_height - 30),
                      cv.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)
            cv.putText(display, "Press SPACE to track, 'r' to reset, 'q' to quit",
                      (10, self.frame_height - 10), cv.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)

            cv.imshow("Color Tracker - Standalone", display)

            # Handle keys
            key = cv.waitKey(1) & 0xFF

            if key == ord('q') or key == 27:
                print("\nQuitting...")
                break

            elif key == ord(' '):  # SPACE
                if self.roi is not None:
                    if not self.tracking:
                        print("\nLearning color from selected region...")
                        self.hsv_range = self.learn_color_from_roi(frame)
                        if self.hsv_range is not None:
                            self.tracking = True
                            print("Tracking enabled!")
                    else:
                        print("\nDisabling tracking...")
                        self.tracking = False
                        self.pan_arm.move_to_center()
                        self.tilt_arm.move_to_center()
                        time.sleep(0.3)
                else:
                    print("Please select a region first!")

            elif key == ord('r') or key == ord('R'):
                print("\nResetting...")
                self.roi = None
                self.hsv_range = None
                self.tracking = False
                self.object_x = 0
                self.object_y = 0
                self.pan_arm.move_to_center()
                self.tilt_arm.move_to_center()
                time.sleep(0.3)

        # Cleanup
        self.cap.release()
        cv.destroyAllWindows()
        self.pan_arm.move_to_center()
        self.tilt_arm.move_to_center()
        print("Done!")


if __name__ == '__main__':
    tracker = StandaloneTracker()
    tracker.run()
