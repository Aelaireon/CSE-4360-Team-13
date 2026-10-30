#!/usr/bin/env pybricks-micropython
import math
from low_level_control.differential_drive import DifferentialDrive

# ---------------------------------------------------------------------------
# Units / coordinate frame used by this module
# ---------------------------------------------------------------------------
# Every coordinate handled here is in METERS in the GLOBAL (world) frame, which
# is the same frame DifferentialDrive uses for getPose() and drive():
#   - origin (0, 0) = bottom-left corner of the workspace, +x right, +y up
#   - heading 0 rad = +x axis, positive angles = counter-clockwise
# Any tile/feet conversion (1 tile = 1 ft = 0.3048 m) belongs in path_finder.py,
# which must hand this class waypoints already converted to meters.
# ---------------------------------------------------------------------------
TILE_M = 0.3048            # 1 floor tile = 1 ft (only used for the demo below)
WORKSPACE_WIDTH_M = 4.88   # 16 tiles (rounded 0.3048*16)
WORKSPACE_HEIGHT_M = 3.05  # 10 tiles (rounded 0.3048*10)
MIN_MOVE_M = 0.001         # waypoints closer than this to the robot are skipped
BOUNDS_EPSILON_M = 1e-6    # float slack for the workspace check


class PathFollower:
    def __init__(self, start_x, start_y, drivebase: DifferentialDrive):
        """
        Initializes the follower with the starting coordinates (METERS, global frame) and a configured differential drive controller.

        start_x / start_y should match the pose the drivebase was constructed with. 
        navigation itself always starts from drivebase.getPose().
        """
        self.drivebase = drivebase

    def _check_in_workspace(self, x, y):
        """Raises ValueError if (x, y) in meters lies outside the workspace."""
        if (x < -BOUNDS_EPSILON_M or x > WORKSPACE_WIDTH_M + BOUNDS_EPSILON_M or
                y < -BOUNDS_EPSILON_M or y > WORKSPACE_HEIGHT_M + BOUNDS_EPSILON_M):
            raise ValueError(
                "Waypoint (%.3f, %.3f) m is outside the %.2f x %.2f m workspace"
                % (x, y, WORKSPACE_WIDTH_M, WORKSPACE_HEIGHT_M))

    def navigate_to(self, target_x, target_y):
        """
        Calculates the distance and the absolute (global-frame) heading from the
        robot's CURRENT odometry pose to the target (meters), then sends the
        movement to the differential drive controller.
        """
        self._check_in_workspace(target_x, target_y)

        # Start from the drivebase's actual odometry pose rather than the last
        # commanded waypoint, so odometry error is corrected on the next segment
        # instead of being carried forward (drive() targets pose + distance*dir).
        pose_x, pose_y, pose_theta = self.drivebase.getPose()
        dx = target_x - pose_x
        dy = target_y - pose_y
        distance = math.sqrt(dx**2 + dy**2)  # meters

        # likely wont ever run realistically in our specific environment, but included for safety
        if distance < MIN_MOVE_M:
            return

        # drive(distance, heading) expects an ABSOLUTE global heading (measured
        # from +x), not a turn relative to the robot, so this is what we pass.
        target_heading_rad = math.atan2(dy, dx)

        # Informational only: the turn the robot will need (drive() performs it).
        # pose_theta is unbounded in the odometry, so normalize both angles.
        turn_angle_rad = (target_heading_rad - pose_theta + math.pi) % (2 * math.pi) - math.pi
        current_heading_deg = math.degrees((pose_theta + math.pi) % (2 * math.pi) - math.pi)
        print("Current: %.1f deg | Target: %.1f deg | Turn: %.1f deg | Dist: %.3f m" % (
            current_heading_deg, math.degrees(target_heading_rad),
            math.degrees(turn_angle_rad), distance))

        self.drivebase.drive(distance, target_heading_rad)

        # Record where the robot actually ended up (odometry), not the ideal waypoint.
        end_x, end_y, _ = self.drivebase.getPose()
        print("Reached (%.3f, %.3f) m | error to waypoint: %.3f m" % (
            end_x, end_y, math.sqrt((target_x - end_x)**2 + (target_y - end_y)**2)))

    def follow_path(self, waypoints):
        """
        Follows the ordered list of (x, y) waypoint tuples (METERS, global frame)
        given by the path planner. The whole list is validated against the
        workspace bounds before the robot moves, so a bad waypoint raises
        ValueError up front instead of halfway through the run.
        """
        for pt in waypoints:
            self._check_in_workspace(pt[0], pt[1])

        for pt in waypoints:
            target_x, target_y = pt
            print("Navigating to waypoint: (%.3f, %.3f) m" % (target_x, target_y))
            self.navigate_to(target_x, target_y)


if __name__ == "__main__":
    start_x, start_y = 0.305, 1.219  # meters
    x = PathFollower(start_x, start_y, DifferentialDrive(x=start_x, y=start_y, theta=0))
    # up one tile (+y), then right one tile (+x)
    waypoints = [(0.305, 1.524), (0.610, 1.524)]
    x.follow_path(waypoints)
    print("done")