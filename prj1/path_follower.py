#!/usr/bin/env pybricks-micropython
import math
from low_control import drive, getHeading

# 1 ft = 0.3048 m, 1 tile = 1 ft, so 1 tile = 0.3048 m

class PathFollower:
    def __init__(self, start_x, start_y):
        """
        Initializes the follower with starting coordinates (in feet/tiles coordinate system).
        """
        self.x = start_x
        self.y = start_y

    def navigate_to(self, target_x, target_y):
        """
        Calculates distance and absolute target heading to target, 
        then calls drive(speed, heading).
        """
        dx = target_x - self.x
        dy = target_y - self.y
        
        # calculate distance in tile/feet units (flatter values to view/debug)
        distance = math.sqrt(dx**2 + dy**2)
        
        if distance < 0.001:
            return
            
        # fetch the live current heading from low-control
        current_heading = getHeading()
        
        # calculate the target heading into degrees (to match getHeading() format)
        target_heading_rad = math.atan2(dy, dx)
        target_heading_deg = math.degrees(target_heading_rad)
        
        # calculate the relative turn angle needed 
        turn_angle = target_heading_deg - current_heading
        # normalized between -180 and 180, realistically shouldnt be more than 90 degrees in either direction if path is reasonable
        turn_angle = (turn_angle + 180) % 360 - 180  
        
        print(f"Current: {current_heading}° | Target: {target_heading_deg}° | Turn: {turn_angle}°")
        
        drive(distance, target_heading_deg)
        
        # update internal position state after moving
        self.x = target_x
        self.y = target_y

    def follow_path(self, waypoints):
        """
        Follows the ordered list of (x, y) waypoint tuples given by the path planner.
        """
        for pt in waypoints:
            target_x, target_y = pt
            print("Navigating to waypoint: ({}, {})".format(target_x, target_y))
            self.navigate_to(target_x, target_y)