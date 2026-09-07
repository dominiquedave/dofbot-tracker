# Dofbot Pi Arm Tracker

A ROS package for tracking colored objects with the Dofbot Pi 6-DOF robotic arm.

## Overview

Two ROS nodes, split along the perception / control seam:

```
  camera ──> color_tracker ──/Current_point──> dofbot_arm_tracker ──I2C──> servos
                                                      ^
                                              /JoyState (pause)
```

- **`color_tracker_node.py`** detects a colour-selected object and publishes
  its pixel position.
- **`dofbot_arm_tracker.py`** turns that position into servo angles.

Neither node contains control maths or I2C code. Those live in reusable
modules, so the same behaviour runs with or without a ROS master:

| Module | Responsibility |
|--------|----------------|
| `vision_lib.py` | camera, ROI selection, colour learning, detection, display |
| `tracker_controller.py` | smoothing, proportional control, deadzone, rate limiting |
| `tracker_config.py` | every tuning parameter, validated in one dataclass |
| `dofbot_lib.py` | servo I2C driver (write angles, read positions) |

That split is the point: `test_vision_processor.py` exercises detection with
no arm attached, and `test_standalone_tracker.py` runs the whole loop with no
ROS master.

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

Run inside the `dofbot` container (`bash ~/Docker_Ros.sh`).

**Stop the vendor arm service first.** `YahboomArm.pyc` also writes to I2C
`0x15`, and I2C reports no error for two writers - the arm simply fights
itself:

```bash
pkill -f YahboomArm
```

### Both nodes at once

```bash
roslaunch dofbot_tracker arm_tracker.launch
```

Drag a box around a coloured object in the window, press SPACE, and the arm
tracks it. Press `r` to pick a different colour, `q` to quit.

Override tuning without editing files:

```bash
roslaunch dofbot_tracker arm_tracker.launch pan_gain:=0.3 tracking_deadzone:=40
roslaunch dofbot_tracker arm_tracker.launch frame_width:=640 frame_height:=480
```

### Nodes individually

```bash
rosrun dofbot_tracker color_tracker_node.py      # perception only
rosrun dofbot_tracker dofbot_arm_tracker.py      # control only
```

Useful for driving the arm from synthetic positions with no camera:

```bash
rostopic pub -r 5 /Current_point yahboomcar_msgs/Position \
    "{angleX: 260.0, angleY: 120.0, distance: 20.0}"
```

### Without ROS

```bash
python3 nodes/test_standalone_tracker.py    # whole loop, no master
python3 nodes/test_vision_processor.py      # detection only, no arm
python3 nodes/test_tilt.py                  # tilt servo pair
python3 nodes/test_read_angle.py            # read all six angles
python3 nodes/dofbot_lib.py --servo 1 --angle 90 --time 400
```

## Parameters

Set on the launch file (`name:=value`) or per node (`_name:=value`).
Defaults come from `TrackerConfig`.

| Parameter | Default | Effect |
|-----------|---------|--------|
| `frame_width` / `frame_height` | 320 / 240 | camera resolution. **Both nodes must agree** - see below |
| `pan_gain` | 0.18 | horizontal response. Higher = faster, oscillates above ~0.3 |
| `tilt_gain` | 0.18 | vertical response |
| `smoothing_alpha` | 0.3 | EMA filter. 0 = smooth and slow, 1 = responsive and jittery |
| `tracking_deadzone` | 20 | pixels of error tolerated before moving |
| `servo_update_interval` | 0.2 | seconds between servo commands |
| `servo_move_time_ms` | 400 | commanded duration of each movement |
| `show_window` | true | false runs headless; then `hsv_min`/`hsv_max` are required |
| `debug` | false | per-update console logging |

**Frame size must match across the two nodes.** The arm node measures its
error from the frame centre, so a vision node publishing 320-wide positions
into an arm node assuming 640 puts the centre off by 160 px - the arm slews
to one side and stays there. The launch file declares it once and passes it
to both for exactly this reason.

### Tuning

| Symptom | Change |
|---------|--------|
| sluggish | raise `pan_gain` / `tilt_gain` |
| overshoots, oscillates | lower the gains, or lower `smoothing_alpha` |
| fidgets while object is still | raise `tracking_deadzone` |
| jerky in steps | lower `servo_update_interval` |

## ROS Topics

### Published by `color_tracker`

| Topic | Type | Description |
|-------|------|-------------|
| `/Current_point` | `yahboomcar_msgs/Position` | `angleX` = x px, `angleY` = y px, `distance` = radius px |
| `/tracker/detected` | `std_msgs/Bool` | latched; published on acquire/lose transitions only |

### Subscribed by `dofbot_arm_tracker`

| Topic | Type | Description |
|-------|------|-------------|
| `/Current_point` | `yahboomcar_msgs/Position` | object position |
| `/JoyState` | `std_msgs/Bool` | true pauses auto tracking; smoothing state resets on release |

Inspect it live:

```bash
rostopic echo /Current_point
rostopic hz /Current_point
rosnode info /dofbot_arm_tracker
rqt_graph
```

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
