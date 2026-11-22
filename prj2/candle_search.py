#!/usr/bin/env pybricks-micropython

"""
CSE 4360 - Project 2: Wander, wall following, and goal finding to extinguish a fire.

Requires LEGO(R) EV3 MicroPython v2.0.
Download: https://education.lego.com/en-us/support/mindstorms-ev3/python-for-ev3

Coordinate contract (same in every file): METERS, global frame. Origin is the
bottom-left corner of the 4.88 x 3.05 m workspace (16 x 10 tiles), +x right,
+y up, heading 0 rad = +x. The robot is placed at `start` facing +x (theta = 0).
"""

from pybricks.hubs import EV3Brick
from low_level_control.differential_drive import DifferentialDrive
from pybricks.parameters import Port
from pybricks.ev3devices import UltrasonicSensor
import math

class CandleSearch:
    """
    Standalone code, uses ultraasonic sensor to find the candle and extinguish it. 
    """
    
    def __init__(self, ev3: EV3Brick, drivebase: DifferentialDrive, ultrasonic: UltrasonicSensor):
        self.ev3 = ev3
        self.drivebase = drivebase # Used to turn the robot and search for candle candidates to investigate
        self.ultrasonic = ultrasonic
        
    def _getMaxSweepAngle(self, distance):
        x1, y1 = 170, 52
        x2, y2 = 490, 76

        return abs(math.radians(y1 + (distance - x1) * (y2 - y1) / (x2 - x1)))

    def _calcThetaOffset(self, distance):
        # At 300 gives 20
        # At 600 gives 15
        return math.radians(-distance / 25 + 25)
        
    def run(self):
        # Track when candle is found for logic
        found_candle = False # True only after candle is found when distance is less than 300, over 300 -> False
        stop_searching = False # True only after candle is found and then distanec is greater than 300 mm (0.5 m)
        
        # Max and min distance of the sweep while candle is found
        max_distance = None
        min_distance = None
        max_difference = 60 # difference between max and min distance
        
        # Theta tracking when candle found and not from odometry for now, maybe gyro later on
        initial_theta = None
        final_theta = None
        max_sweep_angle = None
        min_sweep_angle = None
        theta_margin = math.radians(5) # degrees margin of error for theta
        
        # Constant speed turning CCW
        lspeed = -15
        rspeed = 15
        
        # Max detection distance
        max_distance_threshold = 600 # mm
        
        while True:
            if not(found_candle and stop_searching):
                self.drivebase.runMotors(lspeed, rspeed)
            else:
                self.ev3.speaker.beep()
                self.drivebase.stop()
                print("Candle found! Stopping search.")
                print("Initial theta: ", math.degrees(initial_theta), " Final theta: ", math.degrees(final_theta), " Max distance: ", max_distance, " Min distance: ", min_distance)
                print("Difference between max and min distance: ", max_distance - min_distance)
                print("Theta difference between initial and final theta: ", math.degrees(final_theta) - math.degrees(initial_theta))
                break # Exit the loop if candle is found and then lost (distance > 300 mm)
            distance = self.ultrasonic.distance() # milimeters
                
            if not found_candle:
                found_candle = True
                initial_theta = self.drivebase.getPose()[2] # Get the current heading of the robot
                max_sweep_angle = self._getMaxSweepAngle(distance) + theta_margin
                min_sweep_angle = max_sweep_angle - theta_margin - theta_margin
                print("Max sweep angle: ", math.degrees(max_sweep_angle), " Min sweep angle: ", math.degrees(min_sweep_angle))
                max_distance = distance
                min_distance = distance
                print("Candle found! Distance: ", distance, " mm")
            elif abs(self.drivebase.getPose()[2] - initial_theta) < max_sweep_angle:
                # Update max and min distance
                if distance > max_distance:
                    max_distance = distance
                elif distance < min_distance:
                    min_distance = distance
                final_theta = self.drivebase.getPose()[2] # Get the current heading of the robot
                if abs(self.drivebase.getPose()[2] - initial_theta) > min_sweep_angle and (max_distance - min_distance) < max_difference:
                    stop_searching = True
            else:
                found_candle = False
                final_theta = self.drivebase.getPose()[2] # Get the current heading of the robot
                # print("Candle lost! Stopping search. Final theta: ", final_theta, " Difference between max and min distance: ", max_distance - min_distance)
                print("Candle lost! Stopping search. Difference theta: ", math.degrees(final_theta) - math.degrees(initial_theta), " Difference between max and min distance: ", max_distance - min_distance)
                print()
        
        # Turn CW until face halfway between initial and final theta
        target_theta = self.drivebase.getPose()[2] - (final_theta - initial_theta) / 2 - self._calcThetaOffset(max_distance)
        print("Current theta: ", math.degrees(self.drivebase.getPose()[2]))
        print("Turning to target theta: ", math.degrees(target_theta))
        while self.drivebase.getPose()[2] > target_theta:
            self.drivebase.runMotors(-lspeed, -rspeed)
        self.drivebase.stop()

def main():
    ev3 = EV3Brick()
    drivebase = DifferentialDrive(x=0, y=0, theta=0)
    ultrasonic = UltrasonicSensor(Port.S4)
    candle_search = CandleSearch(ev3, drivebase, ultrasonic)
    candle_search.run()

if __name__ == "__main__":

    def calcThetaOffset(self, distance):
        # At 300 gives 20
        # At 600 gives 15
        return math.radians(-distance / 25 + 25)
    main()
