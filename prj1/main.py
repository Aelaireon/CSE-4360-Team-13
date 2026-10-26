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
from path_finder import astar
import math

# Initialize the EV3 Brick.
ev3 = EV3Brick()

# Initialize test values for start, goal, and obstacles
start = (0.305, 1.219)
goal = (3.658, 1.829)
obstacle = [    
    (0.610, 2.743),
    (0.915, 2.743),
    (1.219, 2.743),
    (1.829, 1.219),
    (1.829, 1.524),
    (1.829, 1.829),
    (1.829, 2.134),
    (2.743, 0.305),
    (2.743, 0.610),
    (2.743, 0.915),
    (2.743, 2.743),
    (3.048, 2.743),
    (3.353, 2.743)
]

# notice: start[0], start[1] 
drivebase = DifferentialDrive(x=start[0], y=start[1], theta=0)
tile = 0.3048 # = 1 foot

"""
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
"""

maze = 0
"""
# returns something like

[
    [0,0,0,0,1,0],
    [1,1,0,1,1,0],
    [0,0,0,0,0,0],
    [0,1,1,1,1,0],
    [0,0,0,0,0,0]
]
"""

start_point = (0,0)
end_point = (4,5)

shortest_path = astar(maze, start_point, end_point)
print("Path found:", shortest_path)

"""
will be given something like:
MAX_OBSTACLES = 25  # maximum number of obstacles
num_obstacles = 13  # number of obstacles

# obstacle locations
obstacle = [
    [0.61, 2.743],
    [0.915, 2.743],
    [1.219, 2.743],
    [1.829, 1.219],
    [1.829, 1.524],
    [1.829, 1.829],
    [1.829, 2.134],
    [2.743, 0.305],
    [2.743, 0.61],
    [2.743, 0.915],
    [2.743, 2.743],
    [3.048, 2.743],
    [3.353, 2.743],
    [-1, -1],
    [-1, -1],
    [-1, -1],
    [-1, -1],
    [-1, -1],
    [-1, -1],
    [-1, -1],
    [-1, -1],
    [-1, -1],
    [-1, -1],
    [-1, -1],
    [-1, -1],
]
start = [0.305, 1.219]  # start location
goal = [3.658, 1.829]  # goal location

NEED TO HAVE PATHFINDER PROCESS IT INTO THE MAZE LIKE GRID IT CURRENTLY USES. (or switch up how the pathfinder works)
"""