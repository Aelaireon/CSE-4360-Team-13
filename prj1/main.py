#!/usr/bin/env pybricks-micropython

"""
CSE 4360 / 5364 - Project 1: navigate a known obstacle course with the LEGO EV3 robot.

Requires LEGO(R) EV3 MicroPython v2.0.
Download: https://education.lego.com/en-us/support/mindstorms-ev3/python-for-ev3

Data flow (numbered steps are in main() below):
    course data (m) -> path_finder -> waypoints (m) -> PathFollower
        -> DifferentialDrive.drive() -> motors
    DifferentialDrive.getPose() (odometry) is read back by PathFollower.

Comment tags:  [EXISTING] = implemented and usable now
               [PROPOSED] = pseudocode only, does not exist yet

Coordinate contract (same in every file): METERS, global frame. Origin is the
bottom-left corner of the 4.88 x 3.05 m workspace (16 x 10 tiles), +x right,
+y up, heading 0 rad = +x. The robot is placed at `start` facing +x (theta = 0).
"""

from pybricks.hubs import EV3Brick
from low_level_control.differential_drive import DifferentialDrive
from path_follower import PathFollower
# from path_finder import find_waypoints   # [PROPOSED] enable once path_finder.py is fixed (see its notes)


# =============================================================================
# COURSE DATA  (meters; the 2008 example course from the project PDF, for testing)
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
        # ---------------------------------------------------------------------
        # STEP 3 - FOLLOW THE PATH (this IS the main loop, for now)  [PROPOSED wiring]
        # ---------------------------------------------------------------------
        # follower.follow_path() is blocking and already implemented. For each
        # waypoint, navigate_to() reads drivebase.getPose(), computes distance and
        # global heading, calls drivebase.drive(distance, heading) (turns, drives,
        # stops), and raises ValueError before moving if any waypoint is outside
        # the workspace.
        #
        #   follower.follow_path(waypoints)

        # ---------------------------------------------------------------------
        # STEP 4 - GOAL CHECK AND TERMINATION                        [PROPOSED]
        # ---------------------------------------------------------------------
        #   x, y, _ = drivebase.getPose()
        #   if distance((x, y), goal) <= GOAL_TOLERANCE_M:      # tolerance still to decide
        #       print("Goal reached")
        #   else:
        #       optional bounded retry (1-2 times): follower.navigate_to(goal[0], goal[1])
        #       report the final pose
        #
        # The run ends when: all waypoints are done, the planner returned None,
        # ValueError was raised by the workspace check, or any other exception
        # occurred. An optional global timeout can use pybricks.tools.StopWatch.
        pass
    finally:
        # Always leave the motors off and let the odometry thread end so the
        # program can exit.                                        [EXISTING]
        drivebase.shutdown()


# If DifferentialDrive changes (teammate plans drive(x, y), a threshold parameter
# and a larger turn-on-spot threshold): only the single drive(...) call inside
# PathFollower.navigate_to() needs to change. If drive() becomes non-blocking or
# per-step (e.g. for a local planner), STEP 3 turns into a loop of
# "read getPose() -> pick next target -> command drivebase -> sleep briefly" until
# the goal or a timeout. That is a different design and should be agreed with the
# low-level owner first.

if __name__ == "__main__":
    main()
