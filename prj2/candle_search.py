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

    def _findSmallestSegmentTarget(
        self,
        distance_buffer,
        theta_buffer,
        beam_deg=40.0,
        tol_mm=27.0,
        gap_mm=70.0,
        min_samples=12,
        no_return=2500,
        k_spread=3.0,
        ref_range_mm=500.0,
        min_samples_floor=3,
        min_width_mm=0.0,      # ignore segments narrower than this (0 = off)
        max_width_mm=1.0e9,    # ignore segments wider than this (huge = off)
    ):
        # 1. Single-pass consecutive-theta dedupe
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
 
        if n > 1:
            diffs = [
                abs(unique_t[i] - unique_t[i - 1])
                for i in range(1, n)
                if unique_t[i] != unique_t[i - 1]
            ]
            d_theta = sorted(diffs)[len(diffs) // 2] if diffs else math.radians(1.0)
        else:
            d_theta = math.radians(1.0)
 
        # 2. Wide-beam minimum erosion, O(N) monotonic queue. Output stays in scan order.
        half_rad = math.radians(beam_deg) * 0.5
        px = []  # x, y, range, theta of surviving points, in angular order
        py = []
        pr = []
        pt = []
        q = []
        head = 0
        hi = 0
        for i in range(n):
            t_center = unique_t[i]
            upper = t_center + half_rad
            lower = t_center - half_rad
 
            while hi < n and unique_t[hi] <= upper:
                d_hi = unique_d[hi]
                while len(q) > head and unique_d[q[-1]] >= d_hi:
                    q.pop()
                q.append(hi)
                hi += 1
 
            while head < len(q) and unique_t[q[head]] < lower:
                head += 1
 
            d_i = unique_d[i]
            local_min = unique_d[q[head]] if head < len(q) else d_i
            if d_i <= local_min + tol_mm:
                px.append(d_i * math.cos(t_center))
                py.append(d_i * math.sin(t_center))
                pr.append(d_i)
                pt.append(t_center)
 
        m = len(px)
        if m == 0:
            return None, None
 
        # 3. Sequential breakpoint segmentation, O(M).
        #    A new segment starts whenever the jump between two consecutive points
        #    exceeds a range-adaptive threshold (same form as the old eps).
        seg_n = []    # point count
        seg_sx = []   # sum of x
        seg_sy = []   # sum of y
        seg_f = []    # index of first point
        seg_l = []    # index of last point
        seg_n_cur = 1
        sx = px[0]
        sy = py[0]
        first = 0
        for i in range(1, m):
            r = pr[i] if pr[i] > pr[i - 1] else pr[i - 1]
            thr = k_spread * r * d_theta
            if thr < gap_mm:
                thr = gap_mm
            dx = px[i] - px[i - 1]
            dy = py[i] - py[i - 1]
            if dx * dx + dy * dy <= thr * thr:
                seg_n_cur += 1
                sx += px[i]
                sy += py[i]
            else:
                seg_n.append(seg_n_cur)
                seg_sx.append(sx)
                seg_sy.append(sy)
                seg_f.append(first)
                seg_l.append(i - 1)
                seg_n_cur = 1
                sx = px[i]
                sy = py[i]
                first = i
        seg_n.append(seg_n_cur)
        seg_sx.append(sx)
        seg_sy.append(sy)
        seg_f.append(first)
        seg_l.append(m - 1)
 
        # Wrap-around: an object straddling theta = 0 / 2*pi is split between the
        # last and first segments. Join them if the end points really are neighbours.
        if len(seg_n) > 1:
            wrap_gap = pt[0] + 2.0 * math.pi - pt[m - 1]
            r = pr[0] if pr[0] > pr[m - 1] else pr[m - 1]
            thr = k_spread * r * d_theta
            if thr < gap_mm:
                thr = gap_mm
            dx = px[0] - px[m - 1]
            dy = py[0] - py[m - 1]
            if wrap_gap <= 10.0 * d_theta and dx * dx + dy * dy <= thr * thr:
                last_n = seg_n.pop()
                last_sx = seg_sx.pop()
                last_sy = seg_sy.pop()
                last_f = seg_f.pop()
                seg_l.pop()
                seg_n[0] = seg_n[0] + last_n
                seg_sx[0] = seg_sx[0] + last_sx
                seg_sy[0] = seg_sy[0] + last_sy
                seg_f[0] = last_f   # merged segment runs last_f ... (wrap) ... seg_l[0]
 
        # 4. Smallest valid segment -> centroid. A segment is valid if it has enough
        #    points (min_samples at ref range, fewer when farther away, never below floor)
        #    and, if enabled, a plausible physical width (chord between its end points).
        best_n = 0
        best_cx = 0.0
        best_cy = 0.0
        found = False
        for s in range(len(seg_n)):
            cnt = seg_n[s]
            cx = seg_sx[s] / cnt
            cy = seg_sy[s] / cnt
            rc = math.sqrt(cx * cx + cy * cy)
            need = int(round(min_samples * (ref_range_mm / max(ref_range_mm, rc))))
            if need < min_samples_floor:
                need = min_samples_floor
            if cnt < need:
                continue
            wx = px[seg_l[s]] - px[seg_f[s]]
            wy = py[seg_l[s]] - py[seg_f[s]]
            width = math.sqrt(wx * wx + wy * wy)
            if width < min_width_mm or width > max_width_mm:
                continue
            if (not found) or cnt < best_n:
                found = True
                best_n = cnt
                best_cx = cx
                best_cy = cy
 
        if not found:
            return None, None
        return math.atan2(best_cy, best_cx), math.sqrt(best_cx * best_cx + best_cy * best_cy)

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
        max_detection = 900 # mm, max distance to detect candle
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

            # dist = self.ultrasonic.distance()
            # theta = self.drivebase.getPose()[2]  # radians
            # if dist < 300*3: # 300 mm * 3 = 900 mm
            #     distance_buffer.append(dist + ultrasonic_distance_offset) # milimeters
            #     theta_buffer.append(theta)
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
            
            print("Finding smallest segment target ...")
            target_theta, target_dist = self._findSmallestSegmentTarget(
                raw_dist,
                raw_theta,
                beam_deg=40.0,
                tol_mm=27.0,
                gap_mm=70.0,
                min_samples=12,
                no_return=2500 + ultrasonic_distance_offset,
                # Optional: reject wall fragments / huge walls by physical width, e.g.
                # min_width_mm=30.0, max_width_mm=250.0   (tune on real scans)
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