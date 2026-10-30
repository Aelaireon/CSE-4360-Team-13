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
from path_finder import PathFinder
import math

from path_follower import PathFollower

# Initialize the EV3 Brick.
ev3 = EV3Brick()

# Initialize test values for start, goal, and obstacles
start_world = (0.305, 1.219)
goal_world = (3.658, 1.829)

# notice: start[0], start[1]
drivebase = DifferentialDrive(x=start_world[0], y=start_world[1], theta=0)
tile = 0.305  # = 1 foot

# The pathfinder uses (row, column), while world coordinates are (x, y).
start_cell = (round(start_world[1] / tile), round(start_world[0] / tile))
goal_cell = (round(goal_world[1] / tile), round(goal_world[0] / tile))

map_x = 16
map_y = 10

# will be given something like:
MAX_OBSTACLES = 25  # maximum number of obstacles
num_obstacles = 13  # number of obstacles

# # obstacle locations
obstacles = [
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

def create_map(obs, max, num, map_x=16, map_y=10):
    default_maze = [[0 for _ in range(map_x)] for _ in range(map_y)]

    if not obs or num <= 0:
        return default_maze

    maze = [[0 for _ in range(map_x)] for _ in range(map_y)]
    has_valid_obstacle = False

    for i in range(min(num, max, len(obs))):
        x, y = obs[i]
        if x == -1 and y == -1:
            continue

        has_valid_obstacle = True
        
        # Otherwise, interpret it as a world-space position and convert to the
        # tile map using the robot's origin and tile size.
        col = int(round(x / tile))
        row = int(round(y / tile))

        if 0 <= row < map_y and 0 <= col < map_x:
            maze[row][col] = 1

    return maze if has_valid_obstacle else default_maze

maze = create_map(obstacles, MAX_OBSTACLES, num_obstacles, map_x, map_y)

for row in maze:
    print(row)

print("Start:", start_cell, "Goal:", goal_cell)
shortest_path = PathFinder.astar(maze, start_cell, goal_cell)
print("Path found:", shortest_path)

if shortest_path:
    path_waypoints = [(col * tile, row * tile) for row, col in shortest_path]
    # The planner includes the starting cell, but the robot is already there.
    # Skip the initial waypoint so it does not try to drive to its own position.
    path_waypoints = path_waypoints[1:]

    follower = PathFollower(start_world[0], start_world[1], drivebase)
    print("Following path:", path_waypoints)
    follower.follow_path(path_waypoints)
    print("Goal Reached!", drivebase.x, drivebase.y, math.degrees(drivebase.theta))
    ev3.speaker.beep()
