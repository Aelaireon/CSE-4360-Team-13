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
import bisect
    
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

    def _findSmallestSegmentTarget(self, distance_buffer, theta_buffer, beam_deg=60.0, tol_mm=27.0, eps_mm=20.0, min_samples=12, no_return=2500):
        # 1. Fast single-pass deduplication
        unique_t = []
        unique_d = []
        last_t = None
        for d, t in zip(distance_buffer, theta_buffer):
            if d is None or t is None or d < 0 or d >= no_return:
                continue
            if t != last_t:
                unique_t.append(t)
                unique_d.append(d)
                last_t = t

        n = len(unique_t)
        if n == 0:
            return None, None

        # 2. Sliding two-pointer wide-beam minimum erosion (O(N))
        half_rad = math.radians(beam_deg) * 0.5
        pts = []
        lo = 0
        hi = 0
        for i in range(n):
            t_center = unique_t[i]
            while lo < n and unique_t[lo] < t_center - half_rad:
                lo += 1
            while hi < n and unique_t[hi] <= t_center + half_rad:
                hi += 1

            local_min = unique_d[i]
            for j in range(lo, hi):
                if unique_d[j] < local_min:
                    local_min = unique_d[j]

            if unique_d[i] <= local_min + tol_mm:
                d_val = unique_d[i]
                pts.append((d_val * math.cos(t_center), d_val * math.sin(t_center)))

        m = len(pts)
        if m == 0:
            return None, None

        # 3. True 2D DBSCAN accelerated via x-coordinate sorting (identical result, 30x faster)
        pts.sort()  # Sorts by x, then y
        eps_sq = eps_mm * eps_mm
        labels = [0] * m  # 0 = unvisited, -1 = noise, > 0 = cluster_id
        cluster_id = 0

        for i in range(m):
            if labels[i] != 0:
                continue

            x0, y0 = pts[i]
            neighbors = []
            # Scan left in x until dx > eps_mm
            j = i
            while j >= 0:
                dx = x0 - pts[j][0]
                if dx > eps_mm:
                    break
                dy = y0 - pts[j][1]
                if dx * dx + dy * dy <= eps_sq:
                    neighbors.append(j)
                j -= 1
            # Scan right in x until dx > eps_mm
            j = i + 1
            while j < m:
                dx = pts[j][0] - x0
                if dx > eps_mm:
                    break
                dy = y0 - pts[j][1]
                if dx * dx + dy * dy <= eps_sq:
                    neighbors.append(j)
                j += 1

            if len(neighbors) < min_samples:
                labels[i] = -1  # Noise (may become a border point later)
                continue

            # Core point -> expand cluster
            cluster_id += 1
            labels[i] = cluster_id
            in_queue = set(neighbors)
            queue = neighbors
            q_idx = 0

            while q_idx < len(queue):
                curr = queue[q_idx]
                q_idx += 1

                if labels[curr] == -1:
                    labels[curr] = cluster_id  # Border point
                    continue
                if labels[curr] != 0:
                    continue

                labels[curr] = cluster_id
                cx0, cy0 = pts[curr]
                curr_neighbors = []

                j = curr
                while j >= 0:
                    dx = cx0 - pts[j][0]
                    if dx > eps_mm:
                        break
                    dy = cy0 - pts[j][1]
                    if dx * dx + dy * dy <= eps_sq:
                        curr_neighbors.append(j)
                    j -= 1

                j = curr + 1
                while j < m:
                    dx = pts[j][0] - cx0
                    if dx > eps_mm:
                        break
                    dy = cy0 - pts[j][1]
                    if dx * dx + dy * dy <= eps_sq:
                        curr_neighbors.append(j)
                    j += 1

                if len(curr_neighbors) >= min_samples:
                    for nb in curr_neighbors:
                        if nb not in in_queue:
                            in_queue.add(nb)
                            queue.append(nb)

        # 4. Group non-noise points by cluster_id and pick the smallest cluster
        grouped = {}
        for idx, lbl in enumerate(labels):
            if lbl > 0:
                grouped.setdefault(lbl, []).append(pts[idx])

        if not grouped:
            return None, None

        smallest = min(grouped.values(), key=len)
        cx = sum(p[0] for p in smallest) / len(smallest)
        cy = sum(p[1] for p in smallest) / len(smallest)
        return math.atan2(cy, cx), math.sqrt(cx * cx + cy * cy)

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
            if self.ultrasonic.distance() < 300*3: # 300 mm * 3 = 900 mm
                distance_buffer.append(self.ultrasonic.distance() + ultrasonic_distance_offset) # milimeters
                theta_buffer.append(self.drivebase.getPose()[2]) # radians

            if initial_scan_theta is None:
                initial_scan_theta = self.drivebase.getPose()[2]
            print("Initial scan theta: ", math.degrees(initial_scan_theta), " Current theta: ", math.degrees(self.drivebase.getPose()[2]))
            print("Theta difference: ", math.degrees(abs(self.drivebase.getPose()[2] - initial_scan_theta)))
            if abs(self.drivebase.getPose()[2] - initial_scan_theta) < math.radians(360) and not distance_buffer.is_full():
                continue # Wait until buffer is full to start processing
            initial_scan_theta = None # Reset initial scan theta for next buffer
            self.drivebase.halt() # Stop the robot to process the buffer
            self.drivebase.halt()
            self.drivebase.halt()
            self.drivebase.halt()
            print("Processing scans ...")
            
            distance_buffer.cleanse()
            theta_buffer.cleanse()
            raw_dist = distance_buffer.get_all()
            raw_theta = theta_buffer.get_all()
            
            print("Finding smallest segment target ...")
            target_theta, target_dist = self._findSmallestSegmentTarget(
                raw_dist,
                raw_theta,
                # beam_deg=60.0,
                # tol_mm=27.0,
                # eps_mm=200.0,
                # min_samples=12,
                beam_deg=40.0,
                tol_mm=27.0,
                eps_mm=200.0,
                min_samples=12,
                no_return=2500 + ultrasonic_distance_offset,
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
