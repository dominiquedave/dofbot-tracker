#!/usr/bin/env python3
"""Test script to read current angle from all servos."""
import sys
import os
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from dofbot_lib import ArmController

print('Testing read_angle() for each servo:')
for i in range(1, 7):
    angle = ArmController.read_angle(i)
    print(f'  Servo {i}: {angle}°')
