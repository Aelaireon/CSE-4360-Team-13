#!/usr/bin/env pybricks-micropython

"""
CSE 4360 - Project 1: navigate a known obstacle course with the LEGO EV3 robot.

Requires LEGO(R) EV3 MicroPython v2.0.
Download: https://education.lego.com/en-us/support/mindstorms-ev3/python-for-ev3

Data flow (numbered steps are in main() below):
    course data (m) -> path_finder -> waypoints (m) -> PathFollower
        -> DifferentialDrive.drive() -> motors
    DifferentialDrive.getPose() (odometry) is read back by PathFollower.

Coordinate contract (same in every file): METERS, global frame. Origin is the
bottom-left corner of the 4.88 x 3.05 m workspace (16 x 10 tiles), +x right,
+y up, heading 0 rad = +x. The robot is placed at `start` facing +x (theta = 0).
"""

from pybricks.hubs import EV3Brick
from low_level_control.differential_drive import DifferentialDrive
from path_follower import PathFollower
from path_finder import find_waypoints, to_cell, format_map
import math

start = (0.305, 1.219)
# goal = (3.658, 1.829)
goal = (3.658, 0.305)

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
    (3.353, 2.743),

    # outlet on ground (AVOID!)
    (1.525, 1.219),
    (1.525, 1.524)
]


def main():
    ev3 = EV3Brick()
    drivebase = DifferentialDrive(x=start[0], y=start[1], theta=0)
    follower = PathFollower(start[0], start[1], drivebase)

    waypoints, grid, cells = find_waypoints(start, goal, obstacle)

    # show the solved path in the console before the robot moves
    # (# obstacle, + padding, * path, S start, G goal)
    print(format_map(grid, cells, to_cell(start[0], start[1]), to_cell(goal[0], goal[1])))
    if waypoints is None:
        print("No path found")
        return
    print("Waypoints (m):", waypoints)

    follower.follow_path(waypoints)

    x, y, _ = drivebase.getPose()
    distance_x = goal[0] - x
    distance_y = goal[1] - y
    distance = math.sqrt(distance_x**2 + distance_y**2)
    if distance <= 0.305:  # inside the goal circle (radius 0.305 m)
        print("Goal reached")
    else:
        print("Goal not reached, pose: (%.3f, %.3f)" % (x, y))
        print("trying to go directly to goal as last ditch effort.")
        follower.navigate_to(goal[0], goal[1])

if __name__ == "__main__":
    main()
