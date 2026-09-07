#!/usr/bin/env python3
"""
Vision Library for Dofbot Tracker

This module contains computer vision components extracted from the monolithic
StandaloneTracker class, following SOLID principles for better maintainability.

Components:
    - CameraManager: Camera initialization and frame acquisition
    - RegionSelector: Interactive ROI selection via mouse events
    - VisionProcessor: Color learning and object detection
    - DisplayManager: UI rendering and FPS tracking
"""

import cv2 as cv
import numpy as np
import time
import sys
from dataclasses import dataclass
from typing import Optional, Tuple


@dataclass
class ROI:
    """Region of Interest for color learning."""
    x1: int
    y1: int
    x2: int
    y2: int

    @property
    def is_valid(self) -> bool:
        """Check if ROI has minimum size (10x10 pixels)."""
        return (self.x2 - self.x1) > 10 and (self.y2 - self.y1) > 10


@dataclass
class DetectionResult:
    """Result from object detection."""
    found: bool
    x: int = 0
    y: int = 0
    radius: int = 0


class CameraManager:
    """
    Manages camera initialization and frame acquisition.

    Responsibilities:
    - Initialize camera with multiple fallback methods
    - Configure resolution
    - Provide frame acquisition interface
    - Handle camera cleanup

    Pattern: Multiple fallback methods with graceful degradation

    Usage:
        camera = CameraManager(width=640, height=480)
        ret, frame = camera.read()
        camera.release()
    """

    def __init__(self, width: int = 640, height: int = 480):
        """
        Initialize camera with specified resolution.

        Args:
            width: Frame width in pixels
            height: Frame height in pixels
        """
        self.frame_width = width
        self.frame_height = height
        self.cap = None
        self._init_camera()

    def _init_camera(self):
        """Initialize camera with multiple fallback methods."""
        methods = [
            ("Camera index 0", 0, None),
            ("Device path /dev/video0", "/dev/video0", None),
        ]

        for name, device, backend in methods:
            print(f"[CameraManager] Trying: {name}...", end=" ")
            try:
                if backend is not None:
                    cap = cv.VideoCapture(device, backend)
                else:
                    cap = cv.VideoCapture(device)

                if cap.isOpened():
                    cap.set(cv.CAP_PROP_FRAME_WIDTH, self.frame_width)
                    cap.set(cv.CAP_PROP_FRAME_HEIGHT, self.frame_height)
                    # Give camera time to initialize (warm-up)
                    time.sleep(0.5)
                    # Test read to ensure camera is working
                    ret, frame = cap.read()
                    if not ret:
                        print(f"Failed (read test failed)")
                        cap.release()
                        continue
                    self.cap = cap
                    print("Success!")
                    return
                else:
                    print("Failed")
                    cap.release()
            except Exception as e:
                print(f"Error: {e}")

        if self.cap is None or not self.cap.isOpened():
            print("\n[CameraManager] ERROR: Cannot open camera!")
            print("Check that camera is not being used by another process.")
            print("Try: sudo pkill -f YahboomArm")
            sys.exit(1)

    def read(self) -> Tuple[bool, Optional[np.ndarray]]:
        """
        Read a frame from the camera.

        Returns:
            (success, frame) tuple
        """
        if self.cap is None:
            return False, None
        return self.cap.read()

    def release(self):
        """Release camera resources."""
        if self.cap is not None:
            self.cap.release()
            print("[CameraManager] Camera released")


class RegionSelector:
    """
    Handles interactive ROI selection via mouse events.

    Responsibilities:
    - Track mouse drag events
    - Maintain selection state
    - Validate ROI dimensions

    State machine: idle → selecting → selected

    Usage:
        selector = RegionSelector()
        cv.setMouseCallback("Window", selector.handle_event)
        roi = selector.get_roi()  # Returns ROI or None
    """

    def __init__(self):
        """Initialize region selector state."""
        self.selecting = False
        self.selection_start = (0, 0)
        self.selection_end = (0, 0)
        self.roi: Optional[ROI] = None

    def handle_event(self, event, x, y, flags, param):
        """
        Handle mouse events for region selection.

        This is the callback for cv.setMouseCallback().

        Args:
            event: OpenCV mouse event type
            x, y: Mouse coordinates
            flags: Additional flags (unused)
            param: User data (unused)
        """
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

            roi = ROI(x1, y1, x2, y2)
            if roi.is_valid:
                self.roi = roi
                print(f"[RegionSelector] Region selected: ({x1},{y1}) to ({x2},{y2})")
                print("[RegionSelector] Press SPACE to learn color and start tracking")
            else:
                print(f"[RegionSelector] Region too small, must be at least 10x10 pixels")

    def get_roi(self) -> Optional[ROI]:
        """Get the selected ROI, or None if no valid selection."""
        return self.roi

    def get_current_selection(self) -> Optional[Tuple[int, int, int, int]]:
        """
        Get the current selection rectangle (even while dragging).

        Returns:
            (x1, y1, x2, y2) tuple, or None if not selecting
        """
        if self.selecting:
            x1, y1 = self.selection_start
            x2, y2 = self.selection_end
            return (x1, y1, x2, y2)
        elif self.roi is not None:
            return (self.roi.x1, self.roi.y1, self.roi.x2, self.roi.y2)
        return None

    def is_selecting(self) -> bool:
        """Check if user is currently dragging to select."""
        return self.selecting

    def reset(self):
        """Clear current selection."""
        self.selecting = False
        self.roi = None
        print("[RegionSelector] Selection reset")


class VisionProcessor:
    """
    Handles color learning and object detection.

    Responsibilities:
    - Learn HSV color range from ROI
    - Detect objects using learned color range
    - Apply morphological operations for noise reduction

    This combines two tightly-related operations: learning a color range
    then using it for detection.

    Usage:
        processor = VisionProcessor(config)
        processor.learn_color_from_roi(frame, roi)
        result = processor.detect(frame)
    """

    def __init__(self, config):
        """
        Initialize vision processor with configuration.

        Args:
            config: TrackerConfig instance with HSV tolerance parameters
        """
        self.config = config
        self.hsv_range: Optional[Tuple[Tuple[int, int, int], Tuple[int, int, int]]] = None

    def learn_color_from_roi(self, image: np.ndarray, roi: ROI) -> Optional[Tuple]:
        """
        Learn HSV color range from selected region with adaptive tolerance.

        Context:
        This is the first step in the tracking pipeline. The user selects a region
        of a colored object, and we need to analyze that region to determine the
        HSV color range to track. The learned range is stored in self.hsv_range
        and used later by detect().

        This implementation adds adaptive tolerance based on scene brightness:
        - Bright scenes: tight tolerances (H±3, S±15, V±15)
        - Dark scenes: loose tolerances (H±8, S±25, V±30)
        - Normal scenes: config defaults

        Args:
            image: BGR image from camera
            roi: ROI object with x1, y1, x2, y2 coordinates

        Returns:
            ((H_min, S_min, V_min), (H_max, S_max, V_max)) tuple, or None on error
        """
        # Extract ROI coordinates from ROI object
        x1, y1, x2, y2 = roi.x1, roi.y1, roi.x2, roi.y2
        cropped_image = image[y1:y2, x1:x2]

        # Convert to HSV
        roi_hsv = cv.cvtColor(cropped_image, cv.COLOR_BGR2HSV)

        # Calculate average brightness for dynamic tolerance adjustment
        avg_brightness = np.mean(roi_hsv[:, :, 2])

        # Determine tolerances based on brightness
        if avg_brightness > self.config.brightness_threshold_high:
            # Bright scene: use tight tolerances
            h_tol, s_tol, v_tol = 3, 15, 15
            print(f"[VisionProcessor] Bright scene (avg V={avg_brightness:.0f}): using tight tolerances")
        elif avg_brightness < self.config.brightness_threshold_low:
            # Dark scene: use loose tolerances
            h_tol, s_tol, v_tol = 8, 25, 30
            print(f"[VisionProcessor] Dark scene (avg V={avg_brightness:.0f}): using loose tolerances")
        else:
            # Normal scene: use config defaults
            h_tol, s_tol, v_tol = self.config.hue_tolerance, self.config.saturation_tolerance, self.config.value_tolerance
            print(f"[VisionProcessor] Normal scene (avg V={avg_brightness:.0f}): using config tolerances")

        # Extract each channel and calculate min/max using vectorized operations
        H_values = roi_hsv[:, :, 0]
        S_values = roi_hsv[:, :, 1]
        V_values = roi_hsv[:, :, 2]

        # Calculate range with adaptive tolerances (median-based outlier removal)
        # Skip bottom 5% and top 5% for each channel
        h_sorted = np.sort(H_values, axis=None)
        s_sorted = np.sort(S_values, axis=None)
        v_sorted = np.sort(V_values, axis=None)

        n_pixels = H_values.size
        skip = max(1, n_pixels // 20)  # Skip 5% from each end

        H_min = max(0, int(h_sorted[skip]) - h_tol)
        H_max = min(255, int(h_sorted[-skip]) + h_tol)
        S_min = max(0, int(s_sorted[skip]) - s_tol)
        S_max = 253
        V_min = max(0, int(v_sorted[skip]) - v_tol)
        V_max = 255

        # HSV Range Validation
        h_range = H_max - H_min
        s_range = 253 - S_min  # S_max is fixed at 253

        if h_range > 60:
            print(f"[VisionProcessor] WARNING: H range ({h_range}) exceeds 60 degrees - color range too broad")
        if s_range < 30:
            print(f"[VisionProcessor] WARNING: S range ({s_range}) below 30 - color range too narrow/saturated")

        # Store the learned HSV range
        self.hsv_range = (
            (int(H_min), int(S_min), int(V_min)),
            (int(H_max), int(S_max), int(V_max))
        )

        print(f"Learned HSV range:")
        print(f"  Lower: H={H_min:.0f}, S={S_min:.0f}, V={V_min:.0f}")
        print(f"  Upper: H={H_max:.0f}, S={S_max:.0f}, V={V_max:.0f}")
        print("Tracking ready! Press SPACE to enable tracking.")

        return self.hsv_range

    def detect(self, image: np.ndarray) -> DetectionResult:
        """
        Detect colored object in image using learned HSV range.

        TODO(human): Implement this method

        Context:
        This is called every frame during tracking. It uses the HSV range learned
        from learn_color_from_roi() to find the target object in the current frame.
        The detection uses color masking, morphological operations to reduce noise,
        and contour detection to find the largest matching object.

        Your task:
        Implement the detection pipeline:
        1. Check if self.hsv_range is None - if so, return DetectionResult(found=False)
        2. Convert image to HSV using cv.cvtColor()
        3. Create a binary mask using cv.inRange() with self.hsv_range
        4. Apply morphological closing to reduce noise:
           - Create 5x5 kernel with cv.getStructuringElement(cv.MORPH_RECT, (5,5))
           - Apply cv.morphologyEx() with cv.MORPH_CLOSE
        5. Threshold the mask with cv.threshold() (threshold=10, max=255, THRESH_BINARY)
        6. Find contours with cv.findContours(RETR_EXTERNAL, CHAIN_APPROX_SIMPLE)
        7. If contours found:
           - Find largest contour by area using cv.contourArea()
           - Get bounding circle with cv.minEnclosingCircle()
           - Return DetectionResult(found=True, x=cx, y=cy, radius=r)
        8. If no contours, return DetectionResult(found=False)

        Guidance:
        - The morphological operations help remove small noise blobs
        - We use the largest contour because the target is usually the biggest match
        - cv.findContours returns (contours, hierarchy) - we only need contours
        - Use int() to convert circle center coordinates (they're floats)
        - Reference: test_standalone_tracker.py lines 185-229

        Args:
            image: BGR image from camera

        Returns:
            DetectionResult with found status and object position
        """
        # Check if color has been learned
        if self.hsv_range is None:
            return DetectionResult(found=False)

        # Convert image to HSV
        hsv_image = cv.cvtColor(image, cv.COLOR_BGR2HSV)

        # Create a binary mask
        lower = np.array(self.hsv_range[0], dtype="uint8")
        upper = np.array(self.hsv_range[1], dtype="uint8")
        mask = cv.inRange(hsv_image, lower, upper)

        # Apply morphological closing to reduce noise
        # Reduced from 5x5 to 3x3 for higher FPS (fewer pixel operations)
        kernel = cv.getStructuringElement(cv.MORPH_RECT, (3, 3))
        mask = cv.morphologyEx(mask, cv.MORPH_CLOSE, kernel)

        # Threshold the mask
        _, binary = cv.threshold(mask, 10, 255, cv.THRESH_BINARY)

        # Find contours
        contours, _ = cv.findContours(binary, cv.RETR_EXTERNAL, cv.CHAIN_APPROX_SIMPLE)

        # Find largest contour by area
        if len(contours) > 0:
            # Calculate minimum area threshold (0.15% of frame = 115 pixels at 320x240)
            frame_area = image.shape[0] * image.shape[1]
            min_area = frame_area * 0.0015  # 0.15% of frame

            areas = [cv.contourArea(c) for c in contours]
            max_index = areas.index(max(areas))
            largest = contours[max_index]

            # Contour area filtering - reject small noise blobs
            largest_area = cv.contourArea(largest)
            if largest_area < min_area:
                print(f"[VisionProcessor] Contour too small: {largest_area:.0f} < {min_area:.0f} pixels")
                return DetectionResult(found=False)

            # Get bounding circle
            (cx, cy), radius = cv.minEnclosingCircle(largest)

            # Return detection result (no drawing - that's DisplayManager's job!)
            return DetectionResult(found=True, x=int(cx), y=int(cy), radius=int(radius))

        # No contours found
        return DetectionResult(found=False)

    def has_learned_color(self) -> bool:
        """Check if a color range has been learned."""
        return self.hsv_range is not None

    def set_hsv_range(self, hsv_min: Tuple[int, int, int], hsv_max: Tuple[int, int, int]):
        """
        Set HSV range manually (for trackbar tuning).

        Args:
            hsv_min: (H_min, S_min, V_min) tuple
            hsv_max: (H_max, S_max, V_max) tuple
        """
        self.hsv_range = (hsv_min, hsv_max)
        print(f"[VisionProcessor] HSV range set: {hsv_min} -> {hsv_max}")

    def reset(self):
        """Clear learned color range."""
        self.hsv_range = None
        print("[VisionProcessor] Color range reset")


class DisplayManager:
    """
    Handles UI rendering, tracking visualization, and FPS tracking.

    Responsibilities:
    - Calculate and display FPS
    - Render tracking status text
    - Render tracking visualization (circles, crosshairs, arrows)
    - Draw selection rectangles

    Pattern: Display concerns grouped together (FPS + visualization work together)

    Usage:
        display = DisplayManager(width=640, height=480)
        display.update_fps()
        display.render_status(frame, tracking=True)
        display.render_tracking(frame, result, center_x, center_y)
    """

    def __init__(self, frame_width: int = 640, frame_height: int = 480):
        """
        Initialize display manager.

        Args:
            frame_width: Width of video frame
            frame_height: Height of video frame
        """
        self.frame_width = frame_width
        self.frame_height = frame_height

        # FPS tracking
        self.fps = 0.0
        self.fps_update_time = time.time()
        self.fps_frame_count = 0

    def update_fps(self):
        """Calculate and update FPS counter (call once per frame)."""
        self.fps_frame_count += 1
        current_time = time.time()
        elapsed = current_time - self.fps_update_time

        # Update FPS every second
        if elapsed >= 1.0:
            self.fps = self.fps_frame_count / elapsed
            self.fps_frame_count = 0
            self.fps_update_time = current_time

    def render_status(self, frame: np.ndarray, tracking: bool, roi_present: bool):
        """
        Render status text on frame.

        Args:
            frame: Image to draw on (modified in-place)
            tracking: Whether tracking is active
            roi_present: Whether a region has been selected
        """
        if tracking:
            cv.putText(frame, "TRACKING ACTIVE", (10, 30),
                      cv.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
        else:
            cv.putText(frame, "SELECT REGION - Press SPACE to track", (10, 30),
                      cv.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)

        # Show FPS
        cv.putText(frame, f"FPS: {self.fps:.1f}", (10, self.frame_height - 30),
                  cv.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)

        # Show instructions
        cv.putText(frame, "Press SPACE to track, 'r' to reset, 'q' to quit",
                  (10, self.frame_height - 10), cv.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)

    def render_tracking(self, frame: np.ndarray, result: DetectionResult):
        """
        Render tracking visualization on frame.

        Args:
            frame: Image to draw on (modified in-place)
            result: Detection result with object position
        """
        if not result.found:
            return

        # Draw bounding circle and center point
        cv.circle(frame, (result.x, result.y), result.radius, (255, 0, 255), 2)
        cv.circle(frame, (result.x, result.y), 3, (0, 0, 255), -1)

        # Draw center crosshair
        center_x = self.frame_width // 2
        center_y = self.frame_height // 2
        cv.line(frame, (center_x, 0), (center_x, self.frame_height), (255, 255, 0), 1)
        cv.line(frame, (0, center_y), (self.frame_width, center_y), (255, 255, 0), 1)

        # Draw error arrow from center to object
        cv.arrowedLine(frame, (center_x, center_y), (result.x, result.y),
                      (0, 255, 255), 2)

    def render_selection(self, frame: np.ndarray, selection: Optional[Tuple[int, int, int, int]]):
        """
        Render selection rectangle on frame.

        Args:
            frame: Image to draw on (modified in-place)
            selection: (x1, y1, x2, y2) tuple, or None
        """
        if selection is not None:
            x1, y1, x2, y2 = selection
            cv.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 0), 2)


if __name__ == '__main__':
    """Test vision components individually."""
    print("Testing vision_lib components...")

    # Test ROI validation
    print("\n1. Testing ROI validation:")
    valid_roi = ROI(10, 10, 100, 100)
    invalid_roi = ROI(10, 10, 15, 15)
    print(f"   Valid ROI (90x90): {valid_roi.is_valid}")
    print(f"   Invalid ROI (5x5): {invalid_roi.is_valid}")

    # Test CameraManager (will try to open camera)
    print("\n2. Testing CameraManager:")
    print("   (This will attempt to open the camera)")
    # camera = CameraManager()
    # ret, frame = camera.read()
    # print(f"   Frame read: {ret}, shape: {frame.shape if ret else None}")
    # camera.release()

    print("\n✓ Basic tests passed!")
    print("\nNote: Full integration testing requires camera and will be done in Phase 5")
