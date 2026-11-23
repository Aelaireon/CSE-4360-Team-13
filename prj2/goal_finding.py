#!/usr/bin/env pybricks-micropython
"""
goal_finding.py - fire detection (background thread) and the extinguish sequence.

THREADS
1. color monitor thread     FireDetector.run()   
    - reads the floor reflection, sets onPad / fireFound
    - ALWAYS RUNNING
    - Never touches the motors

2. extinguish thread        GoalFinding.loop()   
    - waits for fireFound
    - takes drivebase ownership
    - runs the sequence below, once

DRIVEBASE OWNERSHIP (NEW in `low_level_control/differential_drive.py`)
drivebase.driveOwner is WANDER, WALL or EXTINGUISH. Only the current owner's commands can interact with the motors. 
If you're not the owner -> drive() / runMotors() / stop() is ignored and returns False.
  - setOwner() stops the motors on every handoff, so the wheels are still when the new owner starts.
  - This file passes owner=EXTINGUISH on every call
  - If drivebase.drive() returns False the exploring code lost ownership. Must wait.
        if not drivebase.drive(d, h):
            while drivebase.driveOwner != WALL and not goal.finished:
                wait(20)
  - Once the fire sequence starts, ownership never goes back: it stays EXTINGUISH and goal.finished
    becomes True at the end. Exploring loops should end on goal.finished so the program can exit safely.

SEQUENCE
0. The color monitor reads the floor reflection every POLL_MS and decides `onPad`.
1. onPad is True CONFIRM_COUNT times in a row -> fireFound. The extinguish thread:
    - takes drivebase ownership (the wheels stop)
    - plays the initial beep
2. CENTER: spin CW until the color sensor runs off the pad, then CCW across the pad until it runs off the other side.
    - The ultrasonic is logged (heading, distance) during the CCW pass.
    - Turn toward the nearest object (the candle holder) within MAX_CANDLE_RANGE_MM if the ultrasonic saw one
    - If it didn't, turn to the middle of the two pad edges.
3. FINISH: final beeps, then the optional fan.
"""
import math
import threading

from pybricks.ev3devices import ColorSensor, Motor, TouchSensor, UltrasonicSensor
from pybricks.hubs import EV3Brick
from pybricks.parameters import Button, Port
from pybricks.tools import StopWatch, wait
from low_level_control.differential_drive import DifferentialDrive, EXTINGUISH
from mode import announceMode

# detection (black pad on a white floor, using reflected light)
BLACK_REFLECTION_MAX = 15   # reflection (0-100) at or below this = on the black pad
                            # measured: pad reads up to ~6, grid lines/divots dip to the 40s, floor is ~55-65
CONFIRM_COUNT = 3           # consecutive dark readings before fireFound latches (~60 ms)
                            # the pad/line gap is wide, so this can stay short. Raise it if lines ever trigger it
POLL_MS = 20

# step 1: take over
SETTLE_MS = 100             # pause after stopping so the wheels and readings are stable

# step 2: center (find the pad's edges, then the candle between them)
MAX_PAD_SWEEP_DEG = 120     # safety limit for the CW spin if the color sensor never runs off the pad
                            # (the CCW spin back across the pad gets twice this)
OFF_PAD_MS = 60             # color sensor must read "not the pad" this long to count as run off the pad
SCAN_SPIN_SPEED = 12        # duty cycle %, slow so enough samples are taken. TODO: may stall if too low, tune
SCAN_TIMEOUT_MS = 8000      # per spin
MIN_SCAN_SAMPLES = 8        # fewer samples than this = scan failed, aim at the middle of the pad instead
MAX_CANDLE_RANGE_MM = 200   # nearest reading must be at least this close to count as a candle (untested starting value)
CLUSTER_TOL_MM = 25         # readings within this of the minimum belong to the candle cluster
SPIN_SPEED = 20             # duty cycle %, the turn to the final heading
SPIN_TOLERANCE = math.radians(3)
SPIN_TIMEOUT_MS = 4000
DEBUG_PRINT = False         # True: print the pad edges and every scan sample (the test script turns this on)

# step 3: alarm / fan
ALARM_FREQ = 880
FAN_SECONDS = 6
FAN_DUTY = 100              # max%


def wrapAngle(angle):
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
        # the ONE place that decides what "the pad" looks like
        return self.sensor.reflection() <= BLACK_REFLECTION_MAX

    def run(self):
        hits = 0
        while self.running:
            self.onPad = self.readOnPad()
            hits = hits + 1 if self.onPad else 0

            if hits >= CONFIRM_COUNT and not self.fireFound:
                self.fireFound = True
            wait(POLL_MS) # 20ms

    def stop(self):
        self.running = False


# GoalFinding: extinguish thread (takes drivebase ownership, then the sequence)
class GoalFinding:
    def __init__(self, ev3, drivebase, detector, ultrasonic, bumper=None, fan=None):
        """
        :param EV3Brick ev3: for the speaker
        :param DifferentialDrive drivebase: shared drivebase (ownership enforced inside it)
        :param FireDetector detector: the running color monitor
        :param UltrasonicSensor ultrasonic: front-facing, used to find the candle
        :param TouchSensor bumper: optional, shared with the exploring code. A bump ends a spin early
        :param Motor fan: optional third motor, skipped if None
        """
        self.ev3 = ev3
        self.drivebase = drivebase
        self.detector = detector
        self.ultrasonic = ultrasonic
        self.bumper = bumper
        self.fan = fan

        self.finished = False   # True = sequence done, every loop should end
        self.candleSeen = False # after the scan: True = ultrasonic saw the candle, False = aimed at the middle of the pad
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
        # wait for a fire, take ownership, run the sequence once
        while self.running:
            if self.detector.fireFound and not self.finished:
                if not self.drivebase.setOwner(EXTINGUISH): # halts the wheels; exploring commands are ignored from here on
                    return # False = the drivebase was already shut down: don't revive it
                if self.run():
                    self.finished = True # ownership deliberately stays EXTINGUISH. Stays False if stop() aborted the sequence
                return
            wait(POLL_MS)

    # the full sequence! 
    def run(self):
        # 1. we own the drivebase (the wheels were halted by setOwner). initial cue
        self.halt()
        announceMode(self.ev3, EXTINGUISH) # red light + beep
        wait(SETTLE_MS)
        self.halt() # again, harmless: setOwner() is locked inside the drivebase, so no stale exploring command can slip in
                    # (kept as a cheap safety net)

        # 2. spin to both pad edges, then face the candle (or the middle of the pad)
        self.candleSeen = self.scanPadForCandle(verbose=DEBUG_PRINT)
        if not self.running:
            return False # stop() was called mid-scan: don't beep / fan / report success

        # 3. final beeps, then fan
        self.arrive()
        self.extinguish()
        return self.running # False if stop() cut the fan run short

    # step 2: find the pad's edges, then face the candle
    def scanPadForCandle(self, verbose=False):
        # spin CW until the color sensor runs off the pad, then CCW across the pad (logging the
        # ultrasonic) until it runs off the other side. Then turn to the candle if the ultrasonic
        # saw it in between, otherwise to the middle of the two pad edges.
        # Returns True if the candle was seen, False if it aimed at the middle of the pad.
        _, _, theta0 = self.drivebase.getPose()
        maxAngle = math.radians(MAX_PAD_SWEEP_DEG)

        thetaCW = self.spinUntilOffPad(-1, maxAngle)
        samples = []
        thetaCCW = self.spinUntilOffPad(1, 2 * maxAngle, samples)

        target = self.findCandleHeading(samples)
        found = target is not None
        if not found:
            target = (thetaCW + thetaCCW) / 2.0 # no candle seen: aim at the middle of the pad
        if verbose:
            print("pad edges at deg:", round(math.degrees(thetaCW - theta0), 1), "(CW) and",
                  round(math.degrees(thetaCCW - theta0), 1), "(CCW)")
            self.printScan(samples, theta0, target if found else None)
            print("aiming at deg:", round(math.degrees(target - theta0), 1))
        self.spinTo(target)
        wait(SETTLE_MS)
        return found

    def spinUntilOffPad(self, direction, maxAngle, samples=None):
        # spin in place (direction +1 = CCW, -1 = CW) until the color sensor has run off the pad for
        # OFF_PAD_MS, or maxAngle / SCAN_TIMEOUT_MS is hit.
        # If it starts OFF the pad (it just ran off the other side), it first waits to get back ON.
        # If samples is a list it logs (heading, ultrasonic mm). Returns the heading it stopped at.
        _, _, start = self.drivebase.getPose()
        seenPad = self.detector.onPad
        offTimer = StopWatch()
        timer = StopWatch()
        while self.running and timer.time() < SCAN_TIMEOUT_MS:
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

    def printScan(self, samples, theta0, target):
        # debug print: every sample as (degrees from the heading the scan started at, mm)
        print("scan samples:", len(samples))
        for theta, mm in samples:
            print("  deg:", round(math.degrees(theta - theta0), 1), " mm:", mm)
        if target is None:
            print("no candle found in the scan")
        else:
            print("candle middle at deg:", round(math.degrees(target - theta0), 1))

    def spinTo(self, targetTheta):
        # spin in place toward targetTheta. Returns when within SPIN_TOLERANCE or on timeout
        timer = StopWatch()
        while self.running and timer.time() < SPIN_TIMEOUT_MS:
            _, _, theta = self.drivebase.getPose()
            error = wrapAngle(targetTheta - theta)
            if abs(error) < SPIN_TOLERANCE:
                break
            speed = SPIN_SPEED if error > 0 else -SPIN_SPEED
            self.motors(-speed, speed) # right faster than left turns CCW (positive theta)
            wait(10)
        self.halt()

    # step 3: final beeps and fan
    def arrive(self):
        # halt, then the final cue
        self.halt()
        for _ in range(3):
            self.ev3.speaker.beep(ALARM_FREQ, 200)
            wait(100)

    def extinguish(self):
        # OPTIONAL: run the fan motor for FAN_SECONDS
        if self.fan is None:
            return
        self.fan.dc(FAN_DUTY)
        fanTimer = StopWatch()
        while self.running and fanTimer.time() < FAN_SECONDS * 1000:
            wait(20) # polled (not one long wait) so stop() can cut the fan run short
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
