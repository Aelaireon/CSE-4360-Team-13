#!/usr/bin/env pybricks-micropython
"""
goal_finding.py - fire detection (background thread) and the extinguish sequence.

THREADS
1. color monitor thread     FireDetector.run()   
    - reads the floor color, sets fireFound
    - ALWAYS RUNNING
    - Never touches the motors

2. extinguish thread        GoalFinding.loop()   
waits for fireFound
    - takes drivebase ownership
    - runs the "extinguish" sequence
    - hands ownership back if it fails.

DRIVEBASE OWNERSHIP (NEW in `low_level_control/differential_drive.py`)
drivebase.driveOwner is EXPLORE or EXTINGUISH. Only the current owner's commands can interact with the motors. 
If you're not the owner -> drive() / runMotors() / stop() is ignored and returns False.
  - setOwner() stops the motors on every handoff, so the wheels are still when the new owner starts.
  - This file passes owner=EXTINGUISH on every call
  - If drivebase.drive() returns False the exploring code lost ownership. Must wait.
        if not drivebase.drive(d, h):
            while drivebase.driveOwner != EXPLORE and not goal.finished:
                wait(20)
  - After the fire is handled, ownership stays EXTINGUISH and goal.finished is True. Exploring
    loops should end on goal.finished so the program can exit safely.
  - If the sequence gives up (the red pad was not confirmed after stopping)
        - fireFound goes back to False
        - ownership returns to EXPLORE
        - exploring resumes

SEQUENCE (the extinguish thread)
0. TAKE OVER: ownership goes to EXTINGUISH and the robot STOPS IMMEDIATELY
1. CONFIRM: after settling, red must show again within ON_PAD_CONFIRM_MS. If not, it was likely a stray reading 
    - give up and hand control back.
2. CENTER: oscillate in place through +/- SCAN_HALF_ANGLE_DEG and logging (heading, ultrasonic distance).
    - Find the nearest object (the candle holder) within MAX_CANDLE_RANGE_MM and get as aligned to the middle of it as possible 
    - If nothing is in range, keep the OG heading.
3. POSITION: drive forward/backward until the median ultrasonic reading is within DISTANCE_TOLERANCE_MM of TARGET_DISTANCE_MM. 
    - A bump = touched the holder: back off and re-measure. After MAX_BUMPS it just stays where it is.
4. RE-CENTER: a second, narrower scan from the final distance for a better fan aim.
5. ALARM beeps, then the optional fan.
"""

import math
import threading

from pybricks.ev3devices import ColorSensor, Motor, TouchSensor, UltrasonicSensor
from pybricks.hubs import EV3Brick
from pybricks.parameters import Port
from pybricks.tools import StopWatch, wait

from low_level_control.differential_drive import EXPLORE, EXTINGUISH

# detection (rgb based)
RED_INTENSITY_MIN = 20      # red channel (0-100) must be at least this to count as the pad. TODO calibrate
RED_RATIO_MIN = 0.55        # red / (r+g+b) at or above this counts as "on pad". TODO calibrate
CONFIRM_COUNT = 3           # consecutive hits before fireFound latches (~60 ms)
POLL_MS = 20

# step 1: confirm
SETTLE_MS = 100             # pause after each stop so readings are stable
ON_PAD_CONFIRM_MS = 300     # red must be seen within this window after stopping

# step 2 & 4: center (ultrasonic scan)
SCAN_HALF_ANGLE_DEG = 40    # first sweep, each side of the current heading
RECENTER_HALF_ANGLE_DEG = 20   # second, narrower sweep after positioning
RECENTER = True             # can make it false w/o removing the need to remove the code in case the second scan is too slow or unnecessary
SCAN_SPIN_SPEED = 12        # duty cycle %, slow so enough samples are taken. TODO: may stall if too low, tune
SCAN_TIMEOUT_MS = 8000
MIN_SCAN_SAMPLES = 8        # fewer samples than this = scan failed, keep heading
MAX_CANDLE_RANGE_MM = 500   # nearest reading must be at least this close to count as a candle
CLUSTER_TOL_MM = 25         # readings within this of the minimum belong to the candle cluster

# step 3: position/move
TARGET_DISTANCE_MM = 80     # ultrasonic-to-candle distance to stop at. TODO: tune
                            # sensor can't read accurately below about 30 mm
DISTANCE_TOLERANCE_MM = 15
POSITION_SPEED = 15         # duty cycle %, slow (also used for reversing)
MAX_POSITION_TRAVEL = 0.15  # cant drive farther than this while positioning
FILTER_N = 3                # median of the last N ultrasonic readings
BACKOFF_DISTANCE = 0.02     # m to reverse after touching the candle holder
MAX_BUMPS = 3

# shared spin helper 
SPIN_SPEED = 20
SPIN_TOLERANCE = math.radians(3)
SPIN_TIMEOUT_MS = 4000

# alarm / fan 
ALARM_FREQ = 880
FAN_SECONDS = 6
FAN_DUTY = 100


def wrapAngle(angle):
    """Wrap to [-pi, pi]."""
    return (angle + math.pi) % (2 * math.pi) - math.pi


# FireDetector: color monitor thread, sensing only
class FireDetector:
    def __init__(self, colorSensor):
        """
        :param colorSensor: ColorSensor pointed at the floor
        """
        self.sensor = colorSensor
        self.onPad = False          # live: red bright enough AND red enough compared to the other channels
        self.fireFound = False      # latched: confirmed pad hit (this is the "fire_detected" flag)
        self.running = True
        self.thread = threading.Thread(target=self.run)
        self.thread.start()

    def readOnPad(self):
        r, g, b = self.sensor.rgb()   # each 0-100
        if r < RED_INTENSITY_MIN:
            return False
        return (float(r) / (r + g + b)) >= RED_RATIO_MIN

    def run(self):
        hits = 0
        while self.running:
            self.onPad = self.readOnPad()
            hits = hits + 1 if self.onPad else 0

            if hits >= CONFIRM_COUNT and not self.fireFound:
                self.fireFound = True
            wait(POLL_MS)

    def clear(self):
        # forget that the fire was found (used when the sequence gives up)
        self.fireFound = False

    def stop(self):
        self.running = False


# GoalFinding: extinguish thread (takes drivebase ownership, then the sequence)
class GoalFinding:
    def __init__(self, ev3, drivebase, detector, ultrasonic, bumper=None, fan=None):
        """
        :param EV3Brick ev3: for the speaker
        :param DifferentialDrive drivebase: shared drivebase (ownership enforced inside it)
        :param FireDetector detector: the running color monitor
        :param UltrasonicSensor ultrasonic: front-facing, used to find and position on the candle
        :param TouchSensor bumper: optional, shared with the exploring code
        :param Motor fan: optional third motor, skipped if None
        """
        self.ev3 = ev3
        self.drivebase = drivebase
        self.detector = detector
        self.ultrasonic = ultrasonic
        self.bumper = bumper
        self.fan = fan

        self.finished = False # True = fire handled, every loop should end
        self.running = False
        self.thread = None

    # drivebase helpers (remember: always as EXTINGUISH)
    def motors(self, left, right):
        self.drivebase.runMotors(left, right, EXTINGUISH)

    def halt(self):
        self.drivebase.stop(EXTINGUISH)

    def bumped(self):
        return self.bumper is not None and self.bumper.pressed()

    # thread control 
    def start(self):
        self.running = True
        self.thread = threading.Thread(target=self.loop)
        self.thread.start()

    def stop(self):
        self.running = False

    def loop(self):
        # wait for a fire, take ownership, run the sequence, hand ownership back if it fails
        while self.running:
            if self.detector.fireFound and not self.finished:
                self.drivebase.setOwner(EXTINGUISH) # halts the wheels; exploring commands are ignored from here on
                done = self.run() # blocks until finished or gave up
                if done:
                    self.finished = True # ownership deliberately stays EXTINGUISH
                    return
                self.halt() # make sure the wheels are stopped
                self.detector.clear() # fireFound = False
                self.drivebase.setOwner(EXPLORE) # exploring resumes
            wait(POLL_MS)

    # the full sequence! 
    def run(self):
        # runs steps 0 to 5. Returns True when done, False if it gave up (red not confirmed)
        # 0.
        self.halt()
        wait(SETTLE_MS)
        self.halt() # again: an exploring loop that passed its ownership check just before
                    # setOwner() may have written one last motor command in between

        # 1. confirm found red is real
        if not self.confirmOnPad():
            return False
        # temp cue: beep once to indicate the fire is confirmed
        self.ev3.speaker.beep(1000, 100)

        # 2. center on the candle (best effort)
        self.centerOnCandle(SCAN_HALF_ANGLE_DEG)

        # 3. forward / backward to the target distance
        self.positionAtDistance()

        # 4. narrower re-center from the final distance. possibly unnecessary.
        if RECENTER:
            self.centerOnCandle(RECENTER_HALF_ANGLE_DEG)

        # 5. alarm, then fan
        self.arrive()
        self.extinguish()
        return True

    def confirmOnPad(self):
        # red must show up again shortly after stopping, otherwise the first reading was a stray
        timer = StopWatch()
        while timer.time() < ON_PAD_CONFIRM_MS:
            if self.detector.onPad:
                return True
            wait(10)
        return False

    # step 2/4 calls:
    def centerOnCandle(self, halfAngleDeg):
        # best effort. Sweep the ultrasonic +/- halfAngleDeg, find the candle, spin to its middle.
        _, _, theta0 = self.drivebase.getPose()
        half = math.radians(halfAngleDeg)

        self.spinTo(theta0 - half) # go to one end of the sweep
        samples = self.sweep(theta0 + half)  # sweep to the other end, logging ultrasonic values
        target = self.findCandleHeading(samples)
        if target is None:
            target = theta0 # scan failed: face the OG heading
        self.spinTo(target)
        wait(SETTLE_MS)

    def sweep(self, endTheta):
        # spin CCW (heading increases) to endTheta, returning [(theta, distance_mm), ...]
        samples = []
        timer = StopWatch()
        while timer.time() < SCAN_TIMEOUT_MS:
            _, _, theta = self.drivebase.getPose() # theta accumulates, it is not wrapped
            if theta >= endTheta or self.bumped():
                break
            self.motors(-SCAN_SPIN_SPEED, SCAN_SPIN_SPEED)
            samples.append((theta, self.ultrasonic.distance()))
            wait(10)
        self.halt()
        return samples

    def findCandleHeading(self, samples):
        """
        Returns the heading at the middle of the nearest object within MAX_CANDLE_RANGE_MM,
        or None if the scan failed or nothing is in range.
        """
        n = len(samples)
        if n < MIN_SCAN_SAMPLES:
            return None

        # 3-point moving average to mitigate any single-reading noise
        raw = [s[1] for s in samples]
        smooth = []
        for i in range(n):
            lo = max(0, i - 1)
            hi = min(n - 1, i + 1)
            window = raw[lo:hi + 1]
            smooth.append(sum(window) / len(window))

        nearest = min(smooth)
        if nearest > MAX_CANDLE_RANGE_MM:
            return None

        # grow a contiguous cluster around the nearest reading
        i = smooth.index(nearest)
        lo = i
        hi = i
        while lo > 0 and smooth[lo - 1] <= nearest + CLUSTER_TOL_MM:
            lo -= 1
        while hi < n - 1 and smooth[hi + 1] <= nearest + CLUSTER_TOL_MM:
            hi += 1
        return (samples[lo][0] + samples[hi][0]) / 2.0

    # step 3: position at the target distance 
    def positionAtDistance(self):
        """
        Drive forward or backward until the median ultrasonic reading is within DISTANCE_TOLERANCE_MM of TARGET_DISTANCE_MM. 
        Best effort: also stops where it is if the travel cap is hit or MAX_BUMPS bumps happen.
        """
        x0, y0, _ = self.drivebase.getPose()
        recent = []
        bumps = 0

        while True:
            if self.bumped():
                self.halt()
                bumps += 1
                if bumps > MAX_BUMPS:
                    return
                self.backOff() # touched the holder (or a wall): back off, re-measure
                recent = []
                continue

            recent.append(self.ultrasonic.distance())
            if len(recent) > FILTER_N:
                recent.pop(0)
            if len(recent) < FILTER_N:
                wait(10)
                continue

            distance = sorted(recent)[FILTER_N // 2]
            error = distance - TARGET_DISTANCE_MM
            if abs(error) <= DISTANCE_TOLERANCE_MM:
                self.halt()
                return

            x, y, _ = self.drivebase.getPose()
            dx = x - x0
            dy = y - y0
            if math.sqrt(dx * dx + dy * dy) >= MAX_POSITION_TRAVEL:
                self.halt() # best effort: we are as close as the cap allows
                return

            speed = POSITION_SPEED if error > 0 else -POSITION_SPEED # too far: forward, too close: back
            self.motors(speed, speed)
            wait(10)

    def backOff(self):
        x0, y0, _ = self.drivebase.getPose()
        timer = StopWatch()
        while timer.time() < 1500:
            x, y, _ = self.drivebase.getPose()
            dx = x - x0
            dy = y - y0
            if math.sqrt(dx * dx + dy * dy) >= BACKOFF_DISTANCE:
                break
            self.motors(-POSITION_SPEED, -POSITION_SPEED)
            wait(10)
        self.halt()

    def spinTo(self, targetTheta):
        # spin in place toward targetTheta. Returns when within SPIN_TOLERANCE or on timeout
        timer = StopWatch()
        while timer.time() < SPIN_TIMEOUT_MS:
            _, _, theta = self.drivebase.getPose()
            error = wrapAngle(targetTheta - theta)
            if abs(error) < SPIN_TOLERANCE:
                break
            speed = SPIN_SPEED if error > 0 else -SPIN_SPEED
            self.motors(-speed, speed) # right faster than left turns CCW (positive theta)
            wait(10)
        self.halt()

    # step 5: alarm and fan
    def arrive(self):
        # halt, then the one alarm cue
        self.halt()
        for _ in range(3):
            self.ev3.speaker.beep(ALARM_FREQ, 200)
            wait(100)

    def extinguish(self):
        # OPTIONAL: run the fan motor for FAN_SECONDS
        if self.fan is None:
            return
        self.fan.dc(FAN_DUTY)
        wait(FAN_SECONDS * 1000)
        self.fan.dc(0)

"""
Calibration: 
> python goal_finding.py
COLOR: aim at plain floor, pad edge, fully on the pad. Note the r value and the red ratio for each
    RED_INTENSITY_MIN  -> above the floor's r value, below the pad's r value
    RED_RATIO_MIN      -> just above the floor reading
ULTRASONIC: put the candle holder at 3, 5, 10, 20, 30, etc. cm and note the distance
    Then try it against the wall behind it. Use these to pick TARGET_DISTANCE_MM
"""
if __name__ == "__main__":
    # TODO confirm port(s)
    colorSensor = ColorSensor(Port.S1)
    ultrasonic = UltrasonicSensor(Port.S4)    

    # inf loop: print the red ratio and the ultrasonic distance
    while True:
        r, g, b = colorSensor.rgb()
        total = r + g + b
        ratio = (float(r) / total) if total > 0 else 0.0
        print("rgb:", r, g, b, " red ratio:", ratio, " distance mm:", ultrasonic.distance())
        wait(200)
