# Dofbot Pi Arm Tracker

A ROS package for tracking colored objects with the Dofbot Pi 6-DOF robotic arm.

## Overview

This package provides a ROS node that:
1. Subscribes to `/Current_point` from the color tracker
2. Controls the arm servos via I2C to track objects continuously
3. Supports manual control override via `/JoyState`

## Hardware

- **Arm**: Dofbot Pi (6-DOF)
- **Controller**: Raspberry Pi with I2C interface
- **Camera**: Astra or USB camera for color detection

### Servo Configuration

| Servo ID | Function | Range | Notes |
|----------|----------|-------|-------|
| 1 | Base (pan) | 0-180° | Rotates entire arm left/right |
| 2 | Shoulder | 0-180° | Up/down movement |
| 3 | Elbow | 0-180° | Bend/extend elbow |
| 4 | Wrist pitch | 0-180° | Wrist up/down |
| 5 | Wrist roll | 0-270° | Wrist rotation |
| 6 | Gripper | 0-180° | Open/close claw |

## Installation

This package runs **standalone on the Raspberry Pi host** (Python 3.11, OpenCV
4.8.1, `smbus`). ROS is *not* required for the tracker or any test harness —
the `package.xml` / `CMakeLists.txt` / `launch/` scaffolding exists only for the
optional ROS1 node (`dofbot_arm_tracker.py`), which must run inside the Yahboom
`ros-melodic:dofbot` container.

```bash
git clone git@github.com:dominiquedave/dofbot-tracker.git ~/robot/dofbot-tracker
cd ~/robot/dofbot-tracker/nodes
python3 test_standalone_tracker.py
```

Scripts resolve their own imports relative to `__file__`, so the repo can live
anywhere.

### Hardware preflight

The servo board is at I2C `0x15` on bus 1 (the OLED is at `0x3c`):

```bash
sudo i2cdetect -y 1          # expect 15 and 3c
python3 nodes/test_read_angle.py   # read all six servo angles, no movement
```

**Important:** the `ros-melodic:dofbot` container runs `YahboomArm.pyc`, which
also drives `0x15`. I2C will not report a conflict — the two writers simply
interleave and the arm fights itself. Stop the container before running host
code that commands servos:

```bash
docker stop ecstatic_goodall
```

### Optional: ROS1 mode

```bash
# inside the ros-melodic:dofbot container
cd /root/catkin_ws && catkin_make && source devel/setup.bash
roslaunch dofbot_tracker arm_tracker.launch
```

## Usage

### Quick Start

1. **Start the color tracker** (provides object position):
```bash
roscore
rosrun yahboomcar_astra colorHSV.py
```
- Click and drag to select a colored region
- Press SPACE to start tracking
- Note: The colorHSV node publishes to `/Current_point`

2. **Start the arm tracker**:
```bash
# Using launch file
roslaunch dofbot_tracker arm_tracker.launch

# Or directly
rosrun dofbot_tracker dofbot_arm_tracker.py
```

### Testing the Arm (Standalone)

Test the arm controller without ROS:
```bash
# Move servo 1 (pan) to 90 degrees
python3 ~/robot/dofbot-tracker/nodes/dofbot_lib.py --servo 1 --angle 90

# Run a sequence of angles
python3 dofbot_lib.py --servo 1 --sequence 0 45 90 135 180
```

### ROS Parameters

| Parameter | Default | Description |
|-----------|---------|-------------|
| `pan_servo_id` | 1 | Servo ID for pan (base rotation) |
| `tilt_servo_id` | 2 | Servo ID for tilt (shoulder) |
| `tracking_deadzone` | 20 | Pixels of tolerance before moving |
| `max_pan_angle` | 180 | Maximum pan angle (0-180) |
| `max_tilt_angle` | 180 | Maximum tilt angle (0-180) |
| `pan_gain` | 0.5 | Proportional gain for pan (higher = faster) |
| `tilt_gain` | 0.5 | Proportional gain for tilt (higher = faster) |

### Tuning the Tracker

**If the arm is too slow/slaggy:**
```bash
# Increase the gains
rosrun dofbot_tracker dofbot_arm_tracker.py _pan_gain:=1.0 _tilt_gain:=1.0
```

**If the arm osculates/over-shoots:**
```bash
# Decrease the gains
rosrun dofbot_tracker dofbot_arm_tracker.py _pan_gain:=0.2 _tilt_gain:=0.2
```

**If the arm is always moving even when object is stationary:**
```bash
# Increase the deadzone
rosrun dofbot_tracker dofbot_arm_tracker.py _tracking_deadzone:=40
```

## ROS Topics

### Subscribed Topics

| Topic | Type | Description |
|-------|------|-------------|
| `/Current_point` | `yahboomcar_msgs/Position` | Object position (angleX, angleY, distance) |
| `/JoyState` | `std_msgs/Bool` | Manual control override (true = pause tracking) |

### Published Topics

None - the node only controls servos via I2C.

## Working with the Full 6-DOF Arm

### Current Implementation (Pan/Tilt Only)

Currently, the node only controls servos 1 (pan) and 2 (tilt) for basic tracking.

### Extending to Full 6-DOF

To control all 6 servos, modify `dofbot_arm_tracker.py`:

```python
# Add more controllers
self.elbow_controller = ArmController(3)
self.wrist_controller = ArmController(4)
# etc.

# In tracking_callback(), calculate angles for each servo
# based on the object's 3D position (including distance)
```

### Inverse Kinematics

For true tracking with all 6 servos, you'll need inverse kinematics:
1. Calculate object position in 3D space (using camera + distance)
2. Use IK to determine all 6 joint angles
3. Send all angles to servos simultaneously via `Arm_serial_servo_write6()`

## Troubleshooting

### Arm doesn't move
1. Check I2C is enabled: `sudo raspi-config` → Interface Options → I2C
2. Verify I2C devices: `sudo i2cdetect -y 1` (should show 0x15)
3. Check permissions: `groups $USER` (should include 'i2c' and 'video')

### Arm moves erratically
1. Reduce gain values: `_pan_gain:=0.2 _tilt_gain:=0.2`
2. Increase deadzone: `_tracking_deadzone:=40`

### "smbus not available" error
```bash
sudo apt-get install python3-smbus
```

### Camera not detected
Check the color tracker's camera setup:
```bash
ls -l /dev/video*
```

## Learning ROS Concepts

### 1. Packages and catkin
- A ROS package is a directory with source code and metadata (package.xml, CMakeLists.txt)
- catkin is the build system for ROS

### 2. Nodes
- An executable program that uses ROS
- Created with `rospy.init_node()`

### 3. Topics and Messages
- Topics are named buses for message exchange
- Messages are data structures (like Position with angleX, angleY, distance)

### 4. Subscribers and Callbacks
- Subscribers listen to topics
- Callbacks are functions called when messages arrive

### 5. Parameters
- Configuration values stored in the parameter server
- Accessed via `rospy.get_param()`

## Next Steps

1. **Test pan/tilt tracking** - Get the basic setup working
2. **Add gripper control** - Trigger gripper when object is centered
3. **Implement full 6-DOF** - Add inverse kinematics for complete tracking
4. **Integration with movement** - Combine with robot base movement for mobility
