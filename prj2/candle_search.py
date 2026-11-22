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
from collections import deque
import math

class CircularBuffer:
    def __init__(self, capacity: int):
        self.capacity = capacity
        self.buffer = [None] * capacity
        self.head = 0  # Points to the oldest element
        self.size = 0

    def append(self, item):
        # Calculate the next write position
        write_idx = (self.head + self.size) % self.capacity
        self.buffer[write_idx] = item
        
        if self.size < self.capacity:
            self.size += 1
        else:
            # Buffer is full; advance head to overwrite oldest element
            self.head = (self.head + 1) % self.capacity

    def get_all(self) -> list:
        # Return items in FIFO order (oldest to newest)
        return [self.buffer[(self.head + i) % self.capacity] for i in range(self.size)]
    
    def clear(self):
        self.buffer = [None] * self.capacity
        self.head = 0
        self.size = 0

    def is_full(self) -> bool:
        return self.size == self.capacity

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
        found_candle = False
        stop_searching = False
        
        # Max and min distance of the sweep while candle is found
        max_distance = None
        min_distance = None
        max_difference = 60 # difference between max and min distance
        max_scans = 3000
        distance_buffer = CircularBuffer(capacity=max_scans)
        theta_buffer = CircularBuffer(capacity=max_scans)
        ultrasonic_distance_offset = 80 # mm
        
        # Theta tracking when candle found and not from odometry for now, maybe gyro later on
        initial_theta = None
        final_theta = None
        max_sweep_angle = None
        min_sweep_angle = None
        initial_scan_theta = None
        theta_margin = math.radians(5) # degrees margin of error for theta
        
        # Constant speed turning CCW
        lspeed = -15
        rspeed = 15
        
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
                break
            distance_buffer.append(self.ultrasonic.distance() + ultrasonic_distance_offset) # milimeters
            theta_buffer.append(self.drivebase.getPose()[2]) # radians

            if initial_scan_theta is None:
                initial_scan_theta = self.drivebase.getPose()[2]
            print("Initial scan theta: ", math.degrees(initial_scan_theta), " Current theta: ", math.degrees(self.drivebase.getPose()[2]))
            print("Theta difference: ", math.degrees(abs(self.drivebase.getPose()[2] - initial_scan_theta)))
            if abs(self.drivebase.getPose()[2] - initial_scan_theta) < math.radians(360) and not distance_buffer.is_full():
                continue # Wait until buffer is full to start processing
            initial_scan_theta = None # Reset initial scan theta for next buffer
            self.drivebase.stop() # Stop the robot to process the buffer
            print("Processing scans ...")
            """
            We look through all the scans and check for candle
            Candle needs to satisfy the following conditions:
            1. Difference between max and min distance is less than max_difference
            2. The sweep angle is less than max_sweep_angle
            3. The sweep angle is greater than min_sweep_angle
            4. Surrounding 5 scans are greater than max_distance from minimum or less than min_distance from maximum
            """
            for distance, theta in zip(distance_buffer.get_all(), theta_buffer.get_all()):
                if distance is None or theta is None:
                    continue
                if max_distance is None or distance > max_distance:
                    max_distance = distance
                    final_theta = theta
                if min_distance is None or distance < min_distance:
                    min_distance = distance
                    initial_theta = theta
            if max_distance is not None and min_distance is not None:
                if max_distance - min_distance < max_difference:
                    # Check if the sweep angle is less than max_sweep_angle
                    sweep_angle = abs(final_theta - initial_theta)
                    max_sweep_angle = self._getMaxSweepAngle(max_distance)
                    min_sweep_angle = math.radians(5) # 5 degrees
                    print("Sweep angle: ", math.degrees(sweep_angle), " Max sweep angle: ", math.degrees(max_sweep_angle), " Min sweep angle: ", math.degrees(min_sweep_angle))
                    if sweep_angle < max_sweep_angle and sweep_angle > min_sweep_angle:
                        found_candle = True
                        stop_searching = True
                        print("Candle found! Stopping search.")
                        print("Initial theta: ", math.degrees(initial_theta), " Final theta: ", math.degrees(final_theta), " Max distance: ", max_distance, " Min distance: ", min_distance)
                        print("Difference between max and min distance: ", max_distance - min_distance)
                        print("Theta difference between initial and final theta: ", math.degrees(final_theta) - math.degrees(initial_theta))
                        break

            print("Distance buffer: ", distance_buffer.get_all())
            print("Theta buffer: ", theta_buffer.get_all())
            exit(0)
        
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
        return math.radians(-distance / 25 + 25)
    main()
