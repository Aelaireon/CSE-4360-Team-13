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

    def cleanse(self):
        # Extract elements in FIFO order and remove duplicates
        unique_items = list(dict.fromkeys(self.get_all()))
        
        # Reset buffer and write unique items starting at index 0
        self.clear()
        for i, item in enumerate(unique_items):
            self.append(item)

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
    
    def _cleanScans(self, distance_buffer, theta_buffer):
        if not theta_buffer:
            return [], []
        
        # Keep the first element, then only include elements where the current theta != previous theta
        keep = [True] + [curr != prev for curr, prev in zip(theta_buffer[1:], theta_buffer[:-1])]
        
        clean_dist = [d for d, k in zip(distance_buffer, keep) if k]
        clean_theta = [t for t, k in zip(theta_buffer, keep) if k]
        
        return clean_dist, clean_theta

    def _calcThetaOffset(self, distance):
        # At 300 gives 20
        # At 600 gives 15
        return math.radians(-distance / 25 + 25)


    def _findCandleByDip(self, dists, thetas, bin_deg=1.0, side_deg=30,
                     drop_mm=100, min_w_deg=4, max_w_deg=50, no_return=2500):
        n = int(360 / bin_deg)
        r = [no_return] * n
        for d, t in zip(dists, thetas):
            if d is None or d <= 0 or d >= no_return:
                continue
            b = int((t % (2 * math.pi)) / (2 * math.pi) * n) % n
            r[b] = min(r[b], d)

        def med(idx):
            v = sorted(r[i % n] for i in idx)
            return v[len(v) // 2]

        S = int(side_deg / bin_deg)
        dip = []
        for i in range(n):
            left = med(range(i - S - 5, i - S))
            right = med(range(i + S, i + S + 5))
            bg = min(left, right)                 # must be in front of BOTH sides
            dip.append(r[i] < no_return and r[i] < bg - drop_mm)

        # group circular runs of dip bins
        start = next((i for i in range(n) if not dip[i]), None)
        if start is None:
            return None, None
        runs, cur = [], []
        for k in range(1, n + 1):
            i = (start + k) % n
            if dip[i]:
                cur.append(i)
            elif cur:
                runs.append(cur); cur = []
        best = None
        for run in runs:
            w = len(run) * bin_deg
            if not (min_w_deg <= w <= max_w_deg):
                continue
            rng = sorted(r[i] for i in run)[len(run) // 2]
            if best is None or rng < best[1]:
                best = (run, rng)
        if best is None:
            return None, None
        run, rng = best
        mid = run[len(run) // 2]
        return (mid + 0.5) * bin_deg * math.pi / 180.0, rng

    def turnToSmallestSegment(self, target_theta, turn_speed=15, tol_deg=2.0):
        if target_theta is None:
            return False

        tol_rad = math.radians(tol_deg)
        while True:
            pose_theta = self.drivebase.getPose()[2]
            error = math.atan2(math.sin(target_theta - pose_theta), math.cos(target_theta - pose_theta))
            if abs(error) <= tol_rad:
                break
            if error > 0:
                self.drivebase.runMotors(-turn_speed, turn_speed)
            else:
                self.drivebase.runMotors(turn_speed, -turn_speed)

        self.drivebase.stop()
        print("Aligned! Target:", round(math.degrees(target_theta), 1),
              "Pose:", round(math.degrees(self.drivebase.getPose()[2]), 1))
        return True
        
    def run(self):
        # Track when candle is found for logic
        found_candle = False
        stop_searching = False
        
        # Max and min distance of the sweep while candle is found
        max_distance = None
        min_distance = None
        max_difference = 60 # difference between max and min distance
        max_scans = 3000
        max_detection = 700 # mm, max distance to detect candle
        distance_buffer = CircularBuffer(capacity=max_scans)
        theta_buffer = CircularBuffer(capacity=max_scans)
        ultrasonic_distance_offset = 80 # mm
        
        # Theta tracking when candle found and not from odometry for now, maybe gyro later on
        initial_theta = None
        final_theta = None
        last_theta = None
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
            
            dist = self.ultrasonic.distance()
            theta = self.drivebase.getPose()[2]
            if dist < max_detection and (last_theta is None or theta != last_theta):
                distance_buffer.append(dist + ultrasonic_distance_offset)
                theta_buffer.append(theta)
                last_theta = theta

            if initial_scan_theta is None:
                initial_scan_theta = theta
            if abs(theta - initial_scan_theta) < math.radians(360) and not distance_buffer.is_full():
                continue # Wait until a full revolution (or full buffer) to start processing
            initial_scan_theta = None # Reset initial scan theta for next buffer
            self.drivebase.halt() # Stop the robot to process the buffer
            self.drivebase.halt()
            self.drivebase.halt()
            self.drivebase.halt()
            print("Processing scans ...")
            
            raw_dist = distance_buffer.get_all()
            raw_theta = theta_buffer.get_all()
            
            # print("Finding smallest segment target ...")
            # target_theta, target_dist = self._findSmallestSegmentTarget(
            #     raw_dist,
            #     raw_theta,
            #     beam_deg=40.0,
            #     tol_mm=27.0,
            #     gap_mm=70.0,
            #     min_samples=12,
            #     no_return=2500 + ultrasonic_distance_offset,
            #     # Optional: reject wall fragments / huge walls by physical width, e.g.
            #     # min_width_mm=30.0, max_width_mm=250.0   (tune on real scans)
            # )
            
            print("Finding candle by dip ...")
            target_theta, target_dist = self._findCandleByDip(
                raw_dist,
                raw_theta,
                bin_deg=1.0,
                side_deg=30,
                drop_mm=100,
                min_w_deg=4,
                max_w_deg=50,
                no_return=2500 + ultrasonic_distance_offset
            )
            
            print("Target theta:", target_theta, "Target distance:", target_dist)
            if target_theta is not None:
                self.ev3.speaker.beep()
                print("Candle candidate at dist:", round(target_dist), "mm, theta:", round(math.degrees(target_theta), 1), "deg")
                self.turnToSmallestSegment(target_theta, turn_speed=abs(lspeed))
                break  # Exit the search loop once aligned!
            else:
                print("No valid candle segment found; clearing buffers and rescanning.")
                distance_buffer = CircularBuffer(capacity=max_scans)
                theta_buffer = CircularBuffer(capacity=max_scans)
                initial_scan_theta = None

        
        print("Aligned! Current theta:", round(math.degrees(self.drivebase.getPose()[2]), 1))
        print("Cleaning scans ...")
        print("===============================================")
        print("Distance buffer:", distance_buffer.get_all())
        print("Theta buffer:", theta_buffer.get_all())
        print("===============================================")
        self.drivebase.halt()

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