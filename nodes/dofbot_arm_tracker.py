#!/usr/bin/env python3
"""
Dofbot Pi Arm Color Tracker - ROS Node

This node subscribes to /Current_point from the color tracker and controls
the Dofbot Pi 6-DOF arm to track colored objects continuously.

ROS Subscription:
    - /Current_point (yahboomcar_msgs/Position): Object position data
    - /JoyState (std_msgs/Bool): Manual control override

ROS Parameters:
    - ~pan_servo_id (int): Servo ID for pan movement (default: 1)
    - ~tilt_servo_id (int): Servo ID for tilt movement (default: 2)
    - ~tracking_deadzone (int): Deadzone for object position (default: 20)
    - ~max_pan_angle (float): Maximum pan angle in degrees (default: 180)
    - ~max_tilt_angle (float): Maximum tilt angle in degrees (default: 180)
    - ~pan_gain (float): Proportional gain for pan control (default: 0.5)
    - ~tilt_gain (float): Proportional gain for tilt control (default: 0.5)

Usage:
    rosrun dofbot_tracker dofbot_arm_tracker.py
"""

import rospy
import numpy as np
from yahboomcar_msgs.msg import Position
from std_msgs.msg import Bool
from std_msgs.msg import Int32


class ArmController:
    """
    Dofbot Pi Arm Controller using I2C communication.

    This class provides a simplified interface to control the arm servos.
    It handles the I2C communication and angle-to-position conversion.

    I2C Details:
        - Address: 0x15
        - Bus: 1 (Raspberry Pi)
        - Servos: 1-6 (Servo 5 has 0-270° range)
    """

    # Servo position range (pulse width in microseconds * 4 for 12-bit resolution)
    # Standard servos: 900-3100 (0-180°)
    # Servo 5 (extended): 380-3700 (0-270°)
    SERVO_MIN = 900
    SERVO_MAX = 3100
    SERVO5_MIN = 380
    SERVO5_MAX = 3700

    def __init__(self, servo_id=1):
        """
        Initialize the arm controller.

        Args:
            servo_id: The servo ID to control (1-6)
        """
        self.servo_id = servo_id
        self.current_angle = 90  # Default start position

        # Import smbus for I2C communication
        try:
            import smbus
            self.bus = smbus.SMBus(1)
            self.addr = 0x15
            self.initialized = True
            rospy.loginfo(f"ArmController: Initialized servo {servo_id} on I2C address 0x15")
        except ImportError:
            rospy.logwarn("ArmController: smbus not available - running in simulation mode")
            self.bus = None
            self.initialized = False
        except Exception as e:
            rospy.logwarn(f"ArmController: Failed to initialize I2C: {e} - running in simulation mode")
            self.bus = None
            self.initialized = False

    def angle_to_position(self, angle):
        """
        Convert angle to servo position value.

        Args:
            angle: Target angle in degrees

        Returns:
            Position value for I2C communication
        """
        if self.servo_id == 5:
            # Servo 5 has extended range (0-270°)
            position = int((self.SERVO5_MAX - self.SERVO5_MIN) * (angle - 0) / (270 - 0) + self.SERVO5_MIN)
        else:
            # Standard servos (1,2,3,4,6) have 0-180° range
            # Note: servos 2,3,4 are mechanically reversed
            if self.servo_id in [2, 3, 4]:
                angle = 180 - angle
            position = int((self.SERVO_MAX - self.SERVO_MIN) * (angle - 0) / (180 - 0) + self.SERVO_MIN)

        return max(self.SERVO_MIN, min(self.SERVO_MAX, position))

    def write_angle(self, angle, time=100):
        """
        Write angle to servo.

        Args:
            angle: Target angle in degrees
            time: Movement duration in milliseconds (optional)
        """
        if not self.initialized:
            return

        # Clamp angle to valid range
        if self.servo_id == 5:
            angle = max(0, min(270, angle))
        else:
            angle = max(0, min(180, angle))

        # Convert to position
        position = self.angle_to_position(angle)

        # Calculate high and low bytes
        value_H = (position >> 8) & 0xFF
        value_L = position & 0xFF
        time_H = (time >> 8) & 0xFF
        time_L = time & 0xFF

        try:
            # Write to I2C: command register 0x10 + servo_id
            self.bus.write_i2c_block_data(self.addr, 0x10 + self.servo_id,
                                         [value_H, value_L, time_H, time_L])
            self.current_angle = angle
        except Exception as e:
            rospy.logdebug(f"ArmController: I2C write error: {e}")

    def move_to_center(self, time=200):
        """Move servo to center position (90°)."""
        self.write_angle(90, time)


class DofbotArmTracker:
    """
    Dofbot Pi Arm Tracker - Main tracking logic.

    This node subscribes to object position data and controls the arm servos
    to track the object. It uses a simple proportional controller for smooth
    tracking movement.

    The tracking works as follows:
    1. Subscribe to /Current_point topic for object position (X, Y, radius)
    2. Calculate offset from image center (320, 240 for 640x480)
    3. Apply proportional control to determine servo angles
    4. Send angle commands to servos via I2C
    """

    def __init__(self):
        """Initialize the arm tracker node."""
        rospy.init_node('dofbot_arm_tracker', anonymous=True)

        # Get parameters
        self.pan_servo_id = rospy.get_param('~pan_servo_id', 1)
        self.tilt_servo_id = rospy.get_param('~tilt_servo_id', 2)
        self.deadzone = rospy.get_param('~tracking_deadzone', 20)
        self.max_pan_angle = rospy.get_param('~max_pan_angle', 180)
        self.max_tilt_angle = rospy.get_param('~max_tilt_angle', 180)
        self.pan_gain = rospy.get_param('~pan_gain', 0.5)
        self.tilt_gain = rospy.get_param('~tilt_gain', 0.5)

        # Image dimensions (matching color tracker)
        self.image_width = 640
        self.image_height = 480
        self.center_x = self.image_width / 2
        self.center_y = self.image_height / 2

        # Initialize servos
        self.pan_controller = ArmController(self.pan_servo_id)
        self.tilt_controller = ArmController(self.tilt_servo_id)

        # Current object position
        self.object_x = 0
        self.object_y = 0
        self.object_radius = 0

        # Tracking state
        self.tracking_enabled = True
        self.joy_active = False

        # Subscriber callbacks use rospy Timer for periodic updates
        self.last_update_time = rospy.get_time()
        self.update_interval = 0.1  # 100ms update rate

        # Logging configuration
        rospy.loginfo("Dofbot Arm Tracker Configuration:")
        rospy.loginfo(f"  Pan Servo ID: {self.pan_servo_id}")
        rospy.loginfo(f"  Tilt Servo ID: {self.tilt_servo_id}")
        rospy.loginfo(f"  Deadzone: {self.deadzone} pixels")
        rospy.loginfo(f"  Pan Gain: {self.pan_gain}")
        rospy.loginfo(f"  Tilt Gain: {self.tilt_gain}")
        rospy.loginfo("  Waiting for object position data...")

        # Setup ROS subscriptions
        self.sub_position = rospy.Subscriber(
            '/Current_point', Position, self.position_callback, queue_size=1
        )
        self.sub_joy = rospy.Subscriber(
            '/JoyState', Bool, self.joy_callback, queue_size=1
        )

        # Setup timer for periodic tracking updates
        self.tracking_timer = rospy.Timer(
            rospy.Duration(self.update_interval),
            self.tracking_callback
        )

        # Cleanup handler
        rospy.on_shutdown(self.shutdown_handler)

        # Initialize servos to center
        rospy.sleep(0.5)
        self.pan_controller.move_to_center()
        self.tilt_controller.move_to_center()
        rospy.sleep(0.5)

        rospy.loginfo("Dofbot Arm Tracker initialized successfully!")

    def position_callback(self, msg):
        """
        Callback for object position messages.

        Args:
            msg: Position message containing:
                - angleX: Object X position (pixels from left)
                - angleY: Object Y position (pixels from top)
                - distance: Object distance/size
        """
        if not isinstance(msg, Position):
            return

        self.object_x = msg.angleX
        self.object_y = msg.angleY
        self.object_radius = msg.distance

        # Log occasionally (every 10th update to avoid spam)
        if rospy.get_time() - self.last_update_time > 1.0:
            rospy.logdebug(f"Object detected: X={self.object_x:.1f}, Y={self.object_y:.1f}, R={self.object_radius:.1f}")
            self.last_update_time = rospy.get_time()

    def joy_callback(self, msg):
        """
        Callback for joystick/manual control state.

        Args:
            msg: Bool message - True if manual control is active
        """
        if not isinstance(msg, Bool):
            return

        self.joy_active = msg.data

        if self.joy_active:
            rospy.loginfo("Manual control activated - pausing auto tracking")
        else:
            rospy.loginfo("Manual control deactivated - resuming auto tracking")

    def tracking_callback(self, event):
        """
        Periodic tracking update callback.

        This is called at a fixed interval (e.g., 100ms) to update
        the arm position based on the latest object position.

        Args:
            event: Timer event (automatically passed by ROS)
        """
        # Don't track if manual control is active
        if self.joy_active:
            return

        # Don't track if no object detected
        if self.object_x == 0 and self.object_y == 0:
            return

        # Calculate position error (offset from image center)
        x_error = self.object_x - self.center_x
        y_error = self.object_y - self.center_y

        # Check if we need to move (deadzone)
        if abs(x_error) < self.deadzone and abs(y_error) < self.deadzone:
            return

        # Calculate target angles using proportional control
        # The gain determines how aggressively the arm tracks
        pan_offset = x_error * self.pan_gain
        tilt_offset = y_error * self.tilt_gain

        # Calculate new angles (current angle + offset)
        # Note: Y is inverted in image coordinates (top=0, bottom=height)
        # so we invert the tilt calculation
        pan_angle = 90 + pan_offset
        tilt_angle = 90 - tilt_offset  # Invert Y for natural movement

        # Clamp to valid ranges
        pan_angle = max(0, min(self.max_pan_angle, pan_angle))
        tilt_angle = max(0, min(self.max_tilt_angle, tilt_angle))

        # Send commands to servos
        self.pan_controller.write_angle(int(pan_angle), time=100)
        self.tilt_controller.write_angle(int(tilt_angle), time=100)

        # Log occasionally
        if rospy.get_time() - self.last_update_time > 1.0:
            rospy.logdebug(f"Tracking: X_err={x_error:+.1f}, Y_err={y_error:+.1f} -> "
                          f"Pan={pan_angle:.1f}°, Tilt={tilt_angle:.1f}°")
            self.last_update_time = rospy.get_time()

    def shutdown_handler(self):
        """ROS shutdown handler - clean up servos."""
        rospy.loginfo("Shutting down Dofbot Arm Tracker...")
        self.pan_controller.move_to_center()
        self.tilt_controller.move_to_center()
        rospy.sleep(0.2)
        rospy.loginfo("ArmTracker shutdown complete.")


def main():
    """Main entry point for the arm tracker node."""
    try:
        tracker = DofbotArmTracker()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
    except Exception as e:
        rospy.logerr(f"Fatal error in DofbotArmTracker: {e}")
        import traceback
        traceback.print_exc()


if __name__ == '__main__':
    main()
