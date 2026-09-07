#!/usr/bin/env python3
"""
Color Tracker ROS Node

Publishes the pixel position of a colour-selected object. This is the
perception half of the tracker; dofbot_arm_tracker.py consumes it and drives
the servos.

It wraps the components in vision_lib rather than reimplementing detection,
so the same code path is exercised by test_vision_processor.py without ROS.

Published topics:
    /Current_point      (yahboomcar_msgs/Position)  angleX = x px,
                                                    angleY = y px,
                                                    distance = radius px
    /tracker/detected   (std_msgs/Bool)             detection state changes

Parameters:
    ~frame_width   (int)   camera width, default 320
    ~frame_height  (int)   camera height, default 240
    ~show_window   (bool)  open the OpenCV UI, default True
    ~publish_rate  (float) Hz, default 30
    ~hsv_min       (list)  optional preset [H,S,V] - skips interactive pick
    ~hsv_max       (list)  optional preset [H,S,V]

Window controls (when ~show_window):
    drag  select a region      SPACE  learn colour / start tracking
    r     reset selection      q      quit
"""

import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

import cv2 as cv
import rospy
from std_msgs.msg import Bool
from yahboomcar_msgs.msg import Position

from tracker_config import TrackerConfig
from vision_lib import CameraManager, RegionSelector, VisionProcessor, DisplayManager

WINDOW = "Dofbot Color Tracker"


class ColorTrackerNode:
    """Detects a colour-selected object and publishes its pixel position."""

    def __init__(self):
        rospy.init_node("color_tracker", anonymous=False)

        self.config = TrackerConfig(
            frame_width=rospy.get_param("~frame_width", 320),
            frame_height=rospy.get_param("~frame_height", 240),
        )
        self.show_window = rospy.get_param("~show_window", True)
        rate_hz = rospy.get_param("~publish_rate", 30.0)

        self.camera = CameraManager(self.config.frame_width, self.config.frame_height)
        self.processor = VisionProcessor(self.config)
        self.selector = RegionSelector()
        self.display = DisplayManager(self.config.frame_width, self.config.frame_height)

        # An HSV preset lets the node run headless (no display, no mouse).
        hsv_min = rospy.get_param("~hsv_min", None)
        hsv_max = rospy.get_param("~hsv_max", None)
        if hsv_min and hsv_max:
            self.processor.set_hsv_range(tuple(hsv_min), tuple(hsv_max))
            self.tracking = True
            rospy.loginfo("Using preset HSV range %s - %s", hsv_min, hsv_max)
        else:
            self.tracking = False
            if not self.show_window:
                rospy.logwarn(
                    "No ~hsv_min/~hsv_max preset and ~show_window is false - "
                    "there is no way to pick a colour, so nothing will be tracked."
                )

        self.pub_position = rospy.Publisher("/Current_point", Position, queue_size=1)
        self.pub_detected = rospy.Publisher("/tracker/detected", Bool, queue_size=1, latch=True)

        self.was_detected = None  # None so the first result always publishes
        self.rate = rospy.Rate(rate_hz)

        if self.show_window:
            cv.namedWindow(WINDOW)
            cv.setMouseCallback(WINDOW, self.selector.handle_event)

        rospy.on_shutdown(self.shutdown)
        rospy.loginfo(
            "color_tracker up: %dx%d, publishing /Current_point at %.0f Hz",
            self.config.frame_width, self.config.frame_height, rate_hz,
        )

    def publish(self, result):
        """Publish a detection, and the detected/lost edge when it changes."""
        if result.found:
            msg = Position()
            msg.angleX = float(result.x)
            msg.angleY = float(result.y)
            msg.distance = float(result.radius)
            self.pub_position.publish(msg)

        # Only publish the boolean on a transition - subscribers care about the
        # edge, and a latched topic means late joiners still get current state.
        if result.found != self.was_detected:
            self.pub_detected.publish(Bool(data=result.found))
            rospy.loginfo("object %s", "acquired" if result.found else "lost")
            self.was_detected = result.found

    def handle_keys(self, frame):
        """Process window keystrokes. Returns False to quit."""
        key = cv.waitKey(1) & 0xFF
        if key == ord("q"):
            return False
        if key == ord("r"):
            self.selector.reset()
            self.processor.reset()
            self.tracking = False
            rospy.loginfo("selection reset")
        elif key == ord(" "):
            roi = self.selector.get_roi()
            if roi is None or not roi.is_valid:
                rospy.logwarn("drag a larger region before pressing SPACE")
            elif self.processor.learn_color_from_roi(frame, roi) is None:
                rospy.logwarn("could not learn a colour from that region")
            else:
                self.tracking = True
                rospy.loginfo("colour learned - tracking")
        return True

    def spin(self):
        while not rospy.is_shutdown():
            ok, frame = self.camera.read()
            if not ok or frame is None:
                rospy.logwarn_throttle(5, "camera read failed")
                self.rate.sleep()
                continue

            if self.tracking and self.processor.has_learned_color():
                result = self.processor.detect(frame)
                self.publish(result)
            else:
                result = None

            if self.show_window:
                self.display.update_fps()
                if result is not None:
                    self.display.render_tracking(frame, result)
                self.display.render_selection(frame, self.selector.get_current_selection())
                self.display.render_status(
                    frame, tracking=self.tracking, roi_present=self.selector.get_roi() is not None
                )
                cv.imshow(WINDOW, frame)
                if not self.handle_keys(frame):
                    break

            self.rate.sleep()

    def shutdown(self):
        rospy.loginfo("color_tracker shutting down")
        self.camera.release()
        if self.show_window:
            cv.destroyAllWindows()


def main():
    try:
        ColorTrackerNode().spin()
    except rospy.ROSInterruptException:
        pass


if __name__ == "__main__":
    main()
