#!/usr/bin/env python3
"""
Dofbot Arm Tracker ROS Node

Subscribes to object positions and drives the arm to keep the object centred.
This is the control half of the tracker; color_tracker_node.py produces the
positions it consumes.

This node deliberately owns no control maths and no I2C code. Smoothing,
proportional control, deadzone and rate limiting live in TrackingController;
servo I/O lives in dofbot_lib.ArmController. The node's whole job is to
translate between ROS and those components, so the same behaviour can be
exercised without a ROS master by test_standalone_tracker.py.

Subscribed topics:
    /Current_point     (yahboomcar_msgs/Position)  object position in pixels
    /JoyState          (std_msgs/Bool)             true pauses auto tracking

Parameters (defaults come from TrackerConfig):
    ~frame_width  ~frame_height     must match the vision node, since the
                                    control error is measured from frame centre
    ~pan_gain  ~tilt_gain  ~smoothing_alpha  ~tracking_deadzone
    ~servo_update_interval  ~servo_move_time_ms
    ~pan_servo_id
    ~debug                          per-update console logging

Tilt uses two servos: servo 3 (rest 90, range 40-90) tilts down and servo 4
(rest 5, range 5-60) tilts up. Both are mechanically reversed; dofbot_lib
handles the inversion, so angles here are ordinary 0-180 values.
"""

import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

import rospy
from std_msgs.msg import Bool
from yahboomcar_msgs.msg import Position

from dofbot_lib import ArmController
from tracker_config import TrackerConfig
from tracker_controller import TrackingController

TILT_DOWN_SERVO = 3   # rest 90 deg
TILT_UP_SERVO = 4     # rest 5 deg
TILT_DOWN_REST = 90
TILT_UP_REST = 5


class DofbotArmTracker:
    """Drives pan and dual-servo tilt to centre the tracked object."""

    def __init__(self):
        rospy.init_node("dofbot_arm_tracker", anonymous=False)

        defaults = TrackerConfig()
        self.config = TrackerConfig(
            frame_width=rospy.get_param("~frame_width", defaults.frame_width),
            frame_height=rospy.get_param("~frame_height", defaults.frame_height),
            deadzone=rospy.get_param("~tracking_deadzone", defaults.deadzone),
            pan_gain=rospy.get_param("~pan_gain", defaults.pan_gain),
            tilt_gain=rospy.get_param("~tilt_gain", defaults.tilt_gain),
            smoothing_alpha=rospy.get_param("~smoothing_alpha", defaults.smoothing_alpha),
            servo_update_interval=rospy.get_param(
                "~servo_update_interval", defaults.servo_update_interval),
            servo_move_time_ms=rospy.get_param(
                "~servo_move_time_ms", defaults.servo_move_time_ms),
            pan_servo_id=rospy.get_param("~pan_servo_id", defaults.pan_servo_id),
            debug=rospy.get_param("~debug", False),
        )

        self.controller = TrackingController(self.config)
        self.pan = ArmController(self.config.pan_servo_id)
        self.tilt_down = ArmController(TILT_DOWN_SERVO)
        self.tilt_up = ArmController(TILT_UP_SERVO)

        self.joy_active = False

        rospy.loginfo("dofbot_arm_tracker configuration:")
        rospy.loginfo("  frame        %dx%d (centre %.0f,%.0f)",
                      self.config.frame_width, self.config.frame_height,
                      self.config.center_x, self.config.center_y)
        rospy.loginfo("  pan servo    %d", self.config.pan_servo_id)
        rospy.loginfo("  tilt servos  %d (down, rest %d) / %d (up, rest %d)",
                      TILT_DOWN_SERVO, TILT_DOWN_REST, TILT_UP_SERVO, TILT_UP_REST)
        rospy.loginfo("  gains        pan=%.2f tilt=%.2f alpha=%.2f",
                      self.config.pan_gain, self.config.tilt_gain,
                      self.config.smoothing_alpha)
        rospy.loginfo("  deadzone     %d px, update every %.2fs",
                      self.config.deadzone, self.config.servo_update_interval)

        self.go_to_rest(move_time_ms=500)

        rospy.Subscriber("/Current_point", Position, self.on_position, queue_size=1)
        rospy.Subscriber("/JoyState", Bool, self.on_joy, queue_size=1)
        rospy.on_shutdown(self.shutdown)

        rospy.loginfo("waiting for /Current_point ...")

    def go_to_rest(self, move_time_ms=300):
        """Park pan centred and both tilt servos at their rest angles."""
        self.pan.write_angle(90, move_time_ms)
        self.tilt_down.write_angle(TILT_DOWN_REST, move_time_ms)
        self.tilt_up.write_angle(TILT_UP_REST, move_time_ms)

    def on_joy(self, msg):
        """Manual control takes priority; drop smoothing state on release."""
        if msg.data == self.joy_active:
            return
        self.joy_active = msg.data
        if self.joy_active:
            rospy.loginfo("manual control active - auto tracking paused")
        else:
            # Stale smoothed positions would cause a jump on resume.
            self.controller.reset()
            rospy.loginfo("manual control released - auto tracking resumed")

    def on_position(self, msg):
        """Turn an object position into a servo command, if one is due."""
        if self.joy_active:
            return

        command = self.controller.update(int(msg.angleX), int(msg.angleY))
        if not command.should_update:
            return  # in deadzone, or rate limited

        self.pan.write_angle(command.pan_angle, command.move_time_ms)
        self.tilt_down.write_angle(command.tilt_down_angle, command.move_time_ms)
        self.tilt_up.write_angle(command.tilt_up_angle, command.move_time_ms)

        rospy.logdebug("pan=%d tilt_down=%d tilt_up=%d",
                       command.pan_angle, command.tilt_down_angle, command.tilt_up_angle)

    def shutdown(self):
        rospy.loginfo("dofbot_arm_tracker shutting down - parking arm")
        self.go_to_rest(move_time_ms=500)
        rospy.sleep(0.6)


def main():
    try:
        DofbotArmTracker()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass


if __name__ == "__main__":
    main()
