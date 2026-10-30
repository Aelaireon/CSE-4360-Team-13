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
from path_finder import find_waypoints
import math

# =============================================================================
# COURSE DATA  (in meters)
# =============================================================================
# The instructor's header also has MAX_OBSTACLES = 25 / num_obstacles = 13 and pads
# the obstacle list with (-1, -1); the padding is omitted here, so len(obstacle)
# is the real count. The 2026 workspace is 16 x 10 tiles; this data fits in it.
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
    (3.353, 2.743),
]


def main():
    # -------------------------------------------------------------------------
    # STEP 1 - PLAN FIRST, BEFORE ANY HARDWARE IS STARTED          [PROPOSED]
    # -------------------------------------------------------------------------
    # DifferentialDrive.__init__ starts the odometry thread and configures the
    # motors, so a planning failure should happen before that.
    #
    #   waypoints = find_waypoints(start, goal, obstacle)   # [(x_m, y_m), ...] or None
    #   if waypoints is None:
    #       print("No path found")
    #       ev3.speaker.beep()          # optional
    #       return
    #   print("Waypoints:", waypoints)

    # -------------------------------------------------------------------------
    # STEP 2 - INITIALIZE HARDWARE AND CONTROLLERS                 [EXISTING]
    # -------------------------------------------------------------------------
    ev3 = EV3Brick()
    drivebase = DifferentialDrive(x=start[0], y=start[1], theta=0)
    follower = PathFollower(start[0], start[1], drivebase)

    try:
        waypoints = find_waypoints(start, goal, obstacle)
        follower.follow_path(waypoints)

        x, y, _ = drivebase.getPose()
        distance_x = goal[0] - x
        distance_y = goal[1] - y
        distance = math.sqrt(distance_x**2 + distance_y**2)
        if distance <= 1:
            print("Goal reached")
        else:
            print("Goal not reached, pose: (%.3f, %.3f)" % (x, y))
            print("trying to go directly to goal as last ditch effort.")
            follower.navigate_to(goal[0], goal[1])
        pass
    finally:
        drivebase.shutdown()

if __name__ == "__main__":
    main()
