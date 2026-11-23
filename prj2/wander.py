#!/usr/bin/env pybricks-micropython
"""
wander.py - explore open floor, handle first wall hit with an ultrasonic-guided
            left turn, and hand off to wall following on the second wall hit.

exist (always drivebase.handoff(WANDER, WALL) after the bumper (touch sensor) is hit TWICE
"""
import math
import random
import threading

from low_level_control.differential_drive import DifferentialDrive, WANDER, WALL
from prj2.mode import announceMode
from pybricks.hubs import EV3Brick
from pybricks.parameters import Port, Button
from pybricks.ev3devices import TouchSensor, UltrasonicSensor
from pybricks.tools import StopWatch, wait

SPEED = 60                      # duty speed % while driving a leg
TURN_SPEED = 25                 # duty % when turning in place
TURN_TOLERANCE = math.radians(4)
TURN_TIMEOUT = 6000             # safety timeout (ms)
FAR_WALL = 400                  # >= this on first wall hit -> likely hit on sidewall -> turn ~90 deg left
JITTER = 15                     # +/- random degrees around left turns
BUMP_BACK_MS = 1000             # how long to back up after a bumper hit
WEAVE_AMPLITUDE_DEG = 20        # heading swing either side of leg heading (0 to disable)
WEAVE_WAVELENGTH_M = 0.7        # meters per full side-to-side weave
POLL_MS = 20
DEBUG_PRINT = False

def wrapAngle(angle):
    return (angle + math.pi) % (2 * math.pi) - math.pi

class Wander:
    def __init__(self, ev3, drivebase, ultrasonic, bumper):
        self.ev3 = ev3
        self.drive = drivebase
        self.ultrasonic = ultrasonic
        self.bumper = bumper

        self.running = False
        self.thread = None
        self.wall_hits = 0
        self.picked_initial_dir = False

    def start(self):
        random.seed(42)
        self.wall_hits = 0
        self.picked_initial_dir = False
        self.running = True
        self.thread = threading.Thread(target=self.loop)
        self.thread.start()

    def stop(self):
        self.running = False
        self.halt()

    def halt(self):
        return self.drive.stop(WANDER)

    def runMotors(self, left, right):
        return self.drive.runMotors(left, right, WANDER)

    def owns(self):
        return self.running and self.drive.driveOwner == WANDER

    def bumped(self):
        return self.bumper.pressed()

    def readMm(self):
        if self.ultrasonic is None:
            return 0
        return self.ultrasonic.distance()

    def giveToWall(self):
        self.drive.handoff(WANDER, WALL)

    def backUp(self):
        timer = StopWatch()
        while timer.time() < BUMP_BACK_MS:
            if not self.owns():
                self.halt()
                return False
            self.runMotors(-SPEED, -SPEED)
            wait(10)
        self.halt()
        return True

    def turnTo(self, target, track_farthest=False):
        """
        Turns in place toward `target`.
        If `track_farthest` is True, records the heading with the largest ultrasonic
        reading seen while turning and returns (ok, farthest_heading, farthest_mm)
        Otherwise returns ok (bool).
        """
        timer = StopWatch()
        _, _, best_theta = self.drive.getPose()
        best_mm = self.readMm() if track_farthest else -1

        while timer.time() < TURN_TIMEOUT:
            if not self.owns() or self.bumped():
                self.halt()
                return (False, best_theta, best_mm) if track_farthest else False

            _, _, theta = self.drive.getPose()
            if track_farthest:
                mm = self.readMm()
                if mm > best_mm:
                    best_mm = mm
                    best_theta = theta

            error = wrapAngle(target - theta)
            if abs(error) < TURN_TOLERANCE:
                break

            speed = TURN_SPEED if error > 0 else -TURN_SPEED
            self.runMotors(-speed, speed)
            wait(10)

        self.halt()
        if track_farthest:
            return True, best_theta, best_mm
        return True

    def handleWallHit(self):
        """
        Handles bumper contact:
        - 1st hit: back up, read ultrasonic. Far -> ~90 deg left; close -> ~180 deg left.
        - 2nd hit: back up and hand off to WALL.
        Returns True if wandering should continue, False if handed off or aborted.
        """
        if not self.backUp():
            return False

        self.wall_hits += 1
        if self.wall_hits >= 2:
            self.giveToWall()
            return False

        mm = self.readMm()
        if mm >= FAR_WALL:
            deg = 90.0 + random.uniform(-JITTER, JITTER)
        else:
            deg = 180.0 + random.uniform(-JITTER, JITTER)

        if DEBUG_PRINT:
            print("wander: wall hit 1, mm:", mm, "turning left deg:", round(deg, 1))

        _, _, theta = self.drive.getPose()
        target = wrapAngle(theta + math.radians(deg))
        return self.turnTo(target, track_farthest=False)

    def checkActive(self):
        if not self.owns():
            self.halt()
            return False
        if self.bumped():
            return self.handleWallHit()
        return True

    def pickHeading(self, initial=False):
        """
        Picks a random heading and turns to it
        When `initial=True`, checks ultrasonic readings while turning and redirects to the farthest distance seen
        """
        _, _, theta = self.drive.getPose()
        if initial:
            chosen = wrapAngle(theta + random.uniform(-math.pi, math.pi))
            ok, best_theta, best_mm = self.turnTo(chosen, track_farthest=True)
            if not ok:
                return None
            if best_mm >= FAR_WALL:
                if DEBUG_PRINT:
                    print("wander: redirecting to farthest mm:", best_mm)
                if not self.turnTo(best_theta, track_farthest=False):
                    return None
                return best_theta
            return chosen

        delta = random.uniform(-math.radians(120), math.radians(120))
        target = wrapAngle(theta + delta)
        if not self.turnTo(target, track_farthest=False):
            return None
        return target

    def driveLeg(self):
        """Drives forward until a wall is hit or ownership changes."""
        x0, y0, _ = self.drive.getPose()
        weave_amp = WEAVE_AMPLITUDE_DEG / 90.0 * (SPEED * 0.25)

        while self.checkActive():
            if weave_amp > 0:
                x, y, _ = self.drive.getPose()
                traveled = math.sqrt((x - x0) ** 2 + (y - y0) ** 2)
                bias = weave_amp * math.sin(2 * math.pi * traveled / WEAVE_WAVELENGTH_M)
                self.runMotors(SPEED - bias, SPEED + bias)
            else:
                self.runMotors(SPEED, SPEED)
            wait(10)

        self.halt()
        return True

    def step(self):
        if not self.checkActive():
            return False

        if not self.picked_initial_dir:
            target = self.pickHeading(initial=True)
            self.picked_initial_dir = True
        else:
            target = self.pickHeading(initial=False)

        if target is None:
            return False
        return self.driveLeg()

    def loop(self):
        had_ownership = False
        while self.running:
            if self.drive.driveOwner != WANDER:
                had_ownership = False
                wait(POLL_MS)
                continue

            if not had_ownership:
                had_ownership = True
                self.wall_hits = 0
                self.picked_initial_dir = False
                announceMode(self.ev3, WANDER)

            self.step()
            wait(POLL_MS)
        self.halt()


if __name__ == "__main__":
    bumper = TouchSensor(Port.S2)
    ultrasonic = UltrasonicSensor(Port.S4)
    drivebase = DifferentialDrive(x=0, y=0, theta=0)
    ev3 = EV3Brick()

    wander = Wander(ev3, drivebase, ultrasonic, bumper)
    drivebase.setOwner(WANDER)
    wander.start()

    try:
        while drivebase.running and Button.CENTER not in ev3.buttons.pressed():
            if drivebase.driveOwner == WALL:
                wait(1500)
                drivebase.handoff(WALL, WANDER)
            wait(100)
    except KeyboardInterrupt:
        pass
    finally:
        drivebase.shutdown()
        wander.stop()