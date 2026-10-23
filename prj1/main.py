#!/usr/bin/env pybricks-micropython

"""
Example LEGO® MINDSTORMS® EV3 Robot Educator Driving Base Program
-----------------------------------------------------------------

This program requires LEGO® EV3 MicroPython v2.0.
Download: https://education.lego.com/en-us/support/mindstorms-ev3/python-for-ev3

Building instructions can be found at:
https://education.lego.com/en-us/support/mindstorms-ev3/building-instructions#robot
"""

from pybricks.hubs import EV3Brick
from pybricks.ev3devices import Motor
from pybricks.parameters import Port
from low_level_control.differential_drive import DifferentialDrive
import math

# Initialize the EV3 Brick.
ev3 = EV3Brick()

drivebase = DifferentialDrive(x=0, y=0, theta=0)
tile = 0.3048 # 1 foot

# Bottom right square
drivebase.drive(tile, math.radians(-90))
drivebase.drive(tile, math.radians(-180))
drivebase.drive(tile, math.radians(-270))
drivebase.drive(tile, math.radians(-360))

# Bottom left square
drivebase.drive(tile, math.radians(90))
drivebase.drive(tile, math.radians(180))
drivebase.drive(tile, math.radians(270))
drivebase.drive(tile, math.radians(360))

# Top left square
drivebase.drive(tile, math.radians(0))
drivebase.drive(tile, math.radians(90))
drivebase.drive(tile, math.radians(180))
drivebase.drive(tile, math.radians(270))

# Top right square
drivebase.drive(tile, math.radians(-0))
drivebase.drive(tile, math.radians(-90))
drivebase.drive(tile, math.radians(-180))
drivebase.drive(tile, math.radians(-270))