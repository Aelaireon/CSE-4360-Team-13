#!/usr/bin/env pybricks-micropython
"""
goal_finding.py - fire detection (background thread) and the extinguish sequence.

THREADS
1. color monitor thread     FireDetector.run()   
    - reads the floor reflection, sets fireFound
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
  - If the sequence gives up (the black pad was not confirmed after stopping)
        - fireFound goes back to False
        - ownership returns to EXPLORE
        - exploring resumes

SEQUENCE (the extinguish thread)
0. TAKE OVER: ownership goes to EXTINGUISH and the robot STOPS IMMEDIATELY
1. CONFIRM: after settling, the black pad must show again within ON_PAD_CONFIRM_MS. If not, it was likely a stray reading
    - give up and hand control back.
2. CENTER: spin CW until the color sensor runs off the pad, then CCW across the pad until it runs off the other side.
    - The ultrasonic is logged (heading, distance) during the CCW pass.
    - Turn toward the nearest object (the candle holder) within MAX_CANDLE_RANGE_MM if the ultrasonic saw one
    - If it didn't, turn to the middle of the two pad edges.
3. POSITION: drive forward/backward until the median ultrasonic reading is within DISTANCE_TOLERANCE_MM of TARGET_DISTANCE_MM. 
    - A bump = touched the holder: back off and re-measure. After MAX_BUMPS it just stays where it is.
4. RE-CENTER: a second, narrower scan from the final distance for a better fan aim.
5. ALARM beeps, then the optional fan.
"""

import math
import threading

from pybricks.ev3devices import ColorSensor, Motor, TouchSensor, UltrasonicSensor
from pybricks.hubs import EV3Brick
from pybricks.parameters import Button, Port
from pybricks.tools import StopWatch, wait
from low_level_control.differential_drive import DifferentialDrive, EXPLORE, EXTINGUISH

# detection (black pad on a white floor, using reflected light)
BLACK_REFLECTION_MAX = 15   # reflection (0-100) at or below this = on the black pad
                            # measured: pad reads up to ~6, grid lines/divots dip to the 40s, floor is ~55-65
CONFIRM_COUNT = 3           # consecutive dark readings before fireFound latches (~60 ms)
                            # the pad/line gap is wide, so this can stay short. Raise it if lines ever trigger it
POLL_MS = 20

# step 1: confirm
SETTLE_MS = 100             # pause after each stop so readings are stable
ON_PAD_CONFIRM_MS = 300     # the black pad must be seen within this window after stopping

# step 2 & 4: center (ultrasonic scan)
MAX_PAD_SWEEP_DEG = 120     # step 2: safety limit for the CW spin if the color sensor never runs off the pad
                            # (the CCW spin back across the pad gets twice this)
OFF_PAD_MS = 60             # step 2: color sensor must read "not the pad" this long to count as run off the pad
RECENTER_HALF_ANGLE_DEG = 20   # second, narrower sweep after positioning
RECENTER = True             # can make it false w/o removing the need to remove the code in case the second scan is too slow or unnecessary
SCAN_SPIN_SPEED = 12        # duty cycle %, slow so enough samples are taken. TODO: may stall if too low, tune
SCAN_TIMEOUT_MS = 8000
MIN_SCAN_SAMPLES = 8        # fewer samples than this = scan failed, keep heading
MAX_CANDLE_RANGE_MM = 200   # nearest reading must be at least this close to count as a candle (untested starting value)
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
        self.onPad = False          # live: floor reflection is at or below BLACK_REFLECTION_MAX
        self.fireFound = False      # latched: confirmed pad hit (this is the "fire_detected" flag)
        self.running = True
        self.thread = threading.Thread(target=self.run)
        self.thread.start()

    def readOnPad(self):
        # the ONE place that decides what "the pad" looks like. Change only this (and the constants
        # above) if the pad color changes
        return self.sensor.reflection() <= BLACK_REFLECTION_MAX

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
        # runs steps 0 to 5. Returns True when done, False if it gave up (pad not confirmed)
        # 0.
        self.halt()
        wait(SETTLE_MS)
        self.halt() # again: an exploring loop that passed its ownership check just before
                    # setOwner() may have written one last motor command in between

        # 1. confirm the black pad is real
        if not self.confirmOnPad():
            return False
        # temp cue: beep once to indicate the fire is confirmed
        self.ev3.speaker.beep(1000, 100)

        # 2. spin to both pad edges, then face the candle (or the middle of the pad)
        self.scanPadForCandle()

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
        # the pad must show up again shortly after stopping, otherwise the first reading was a stray
        timer = StopWatch()
        while timer.time() < ON_PAD_CONFIRM_MS:
            if self.detector.onPad:
                return True
            wait(10)
        return False

    # step 2: find the pad's edges, then face the candle
    def scanPadForCandle(self, verbose=False):
        # spin CW until the color sensor runs off the pad, then CCW across the pad until off the pad
        # If candle seen, turn towards it, otherwise go to the middle of the two pad edges.
        # Returns True if the candle was seen, False if it aimed at the middle of the pad.
        _, _, theta0 = self.drivebase.getPose()
        maxAngle = math.radians(MAX_PAD_SWEEP_DEG)

        thetaCW = self.spinUntilOffPad(-1, maxAngle)
        samples = []
        thetaCCW = self.spinUntilOffPad(1, 2 * maxAngle, samples)

        target = self.findCandleHeading(samples)
        found = target is not None
        if not found:
            target = (thetaCW + thetaCCW) / 2.0 # no candle seen -> aim at the middle of the pad
        if verbose:
            print("pad edges at deg:", round(math.degrees(thetaCW - theta0), 1), "(CW) and",
                  round(math.degrees(thetaCCW - theta0), 1), "(CCW)")
            self.printScan(samples, theta0, target if found else None)
            print("aiming at deg:", round(math.degrees(target - theta0), 1))
        self.spinTo(target)
        wait(SETTLE_MS)
        return found

    def spinUntilOffPad(self, direction, maxAngle, samples=None):
        # spin in place (direction +1 = CCW, -1 = CW) until the color sensor has run off the pad
        # for OFF_PAD_MS or maxAngle / SCAN_TIMEOUT_MS is hit.
        # logs heading, ultrasonic mm. Returns the heading it stopped at.
        _, _, start = self.drivebase.getPose()
        seenPad = self.detector.onPad
        offTimer = StopWatch()
        timer = StopWatch()
        while timer.time() < SCAN_TIMEOUT_MS:
            _, _, theta = self.drivebase.getPose()
            if abs(theta - start) >= maxAngle or self.bumped():
                break
            if self.detector.onPad:
                seenPad = True
                offTimer.reset()
            elif seenPad and offTimer.time() >= OFF_PAD_MS:
                break
            self.motors(-direction * SCAN_SPIN_SPEED, direction * SCAN_SPIN_SPEED)
            if samples is not None:
                samples.append((theta, self.ultrasonic.distance()))
            wait(10)
        self.halt()
        _, _, theta = self.drivebase.getPose()
        return theta

    # step 4 call (fixed-angle scan around the current heading):
    def centerOnCandle(self, halfAngleDeg, verbose=False):
        # best effort. Sweep the ultrasonic +/- halfAngleDeg, find the candle, spin to its middle.
        # Returns True if a candle was found, False if it fell back to the OG heading.
        # verbose=True prints every scan sample (used by the test script)
        _, _, theta0 = self.drivebase.getPose()
        half = math.radians(halfAngleDeg)

        self.spinTo(theta0 - half) # go to one end of the sweep
        samples = self.sweep(theta0 + half)  # sweep to the other end, logging ultrasonic values
        target = self.findCandleHeading(samples)
        found = target is not None
        if verbose:
            self.printScan(samples, theta0, target)
        if not found:
            target = theta0 # scan failed: face the OG heading
        self.spinTo(target)
        wait(SETTLE_MS)
        return found

    def printScan(self, samples, theta0, target):
        # debug print: every sample as (degrees from the heading the scan started at, mm)
        print("scan samples:", len(samples))
        for theta, mm in samples:
            print("  deg:", round(math.degrees(theta - theta0), 1), " mm:", mm)
        if target is None:
            print("no candle found in the scan")
        else:
            print("candle middle at deg:", round(math.degrees(target - theta0), 1))

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
live readings (distance traveled, reflection, rgb, whether it counts as the pad, ultrasonic mm).
DRIVE_ENABLED below: True = the robot drives straight while printing, False = it sits still and you aim it by hand.
"""
if __name__ == "__main__":
    # calibration settings 
    DRIVE_ENABLED = False        # True: drive forward while printing. False: sit still and print
    # to stop early: press the brick's CENTER button (Ctrl+C is unreliable with the background threads)
    DRIVE_SPEED = 20            # duty cycle %
    MAX_DRIVE_DISTANCE = 0.60   # m, stops on its own after this so it can't run off the table
    PRINT_MS = 50               # time between printed lines. Short so thin grid lines don't slip between prints
    READ_ULTRASONIC = True      # False skips the ultrasonic read (it adds a bit of delay to every line)

    colorSensor = ColorSensor(Port.S1)
    ultrasonic = UltrasonicSensor(Port.S4)
    drivebase = DifferentialDrive(x=0, y=0, theta=0)
    ev3 = EV3Brick()

    darkest = 100
    brightest = 0
    try:
        # runs until the CENTER button is pressed, the distance cap is hit, or the odometry thread has stopped
        while drivebase.running and Button.CENTER not in ev3.buttons.pressed():
            x, y, _ = drivebase.getPose()
            traveled = math.sqrt(x * x + y * y)
            if DRIVE_ENABLED:
                if traveled >= MAX_DRIVE_DISTANCE:
                    break
                drivebase.runMotors(DRIVE_SPEED, DRIVE_SPEED)

            reflection = colorSensor.reflection()
            r, g, b = colorSensor.rgb()
            darkest = min(darkest, reflection)
            brightest = max(brightest, reflection)

            distance = ultrasonic.distance() if READ_ULTRASONIC else "-"
            print("cm:", round(traveled * 100, 1),
                  " reflection:", reflection,
                  " rgb:", r, g, b,
                  " onPad:", reflection <= BLACK_REFLECTION_MAX,
                  " distance mm:", distance)
            wait(PRINT_MS)
    except KeyboardInterrupt:
        pass # Ctrl+C: fall through to the cleanup below
    finally:
        drivebase.shutdown() # stops the motors and ends the odometry thread so the program can exit
        print("darkest reflection:", darkest, " brightest reflection:", brightest)
