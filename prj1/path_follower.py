#!/usr/bin/env pybricks-micropython
import math
from low_level_control.differential_drive import DifferentialDrive

# 1 ft = 0.3048 m, 1 tile = 1 ft, so 1 tile = 0.3048 m

class PathFollower:
    def __init__(self, start_x, start_y, drivebase: DifferentialDrive):
        """
        Initializes the follower with starting coordinates (in feet/tiles coordinate system)
        and a configured differential drive controller.
        """
        self.x = start_x
        self.y = start_y
        self.drivebase = drivebase

    def navigate_to(self, target_x, target_y):
        """
        Calculates distance and absolute target heading to target, 
        then sends the movement to the differential drive controller.
        """
        dx = target_x - self.x
        dy = target_y - self.y
        
        # calculate distance in tile/feet units (flatter values to view/debug)
        distance = math.sqrt(dx**2 + dy**2)
        
        if distance < 0.001:
            return
            
        current_heading = math.degrees(self.drivebase.getPose()[2])
        target_heading_rad = math.atan2(dy, dx)
        target_heading_deg = math.degrees(target_heading_rad)
        
        # calculate the relative turn angle needed 
        turn_angle = target_heading_deg - current_heading
        # normalized between -180 and 180, realistically shouldnt be more than 90 degrees in either direction if path is reasonable
        turn_angle = (turn_angle + 180) % 360 - 180  
        
        print(f"Current: {current_heading}° | Target: {target_heading_deg}° | Turn: {turn_angle}°")
        
        self.drivebase.drive(distance * 0.3048, target_heading_rad)
        
        # update internal position state after moving
        self.x = target_x
        self.y = target_y

    def follow_path(self, waypoints):
        """
        Follows the ordered list of (x, y) waypoint tuples given by the path planner.
        """
        for pt in waypoints:
            target_x, target_y = pt
            print(f"Navigating to waypoint: ({target_x}, {target_y})")
            self.navigate_to(target_x, target_y)


if __name__ == "__main__":
    x = PathFollower(0.305, 1.219, DifferentialDrive(x=0.305, y=1.219, theta=0))
    # right one tile, then turn and go up one tile
    waypoints = [(0.305, 1.524), (0.610, 1.524)]
    x.follow_path(waypoints)
    print("hello world")