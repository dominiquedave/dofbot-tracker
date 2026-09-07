#!/usr/bin/env python3
"""Test script to read current angle from all servos."""
import sys
sys.path.insert(0, '/home/pi/yahboomcar_ws/src/dofbot_tracker/nodes')
from dofbot_lib import ArmController

print('Testing read_angle() for each servo:')
for i in range(1, 7):
    angle = ArmController.read_angle(i)
    print(f'  Servo {i}: {angle}°')
