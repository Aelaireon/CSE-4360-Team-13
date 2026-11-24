#!/usr/bin/env pybricks-micropython
"""
wander.py - explore open floor with random headings, use ultrasonic on wall hits,
            and hand off to wall following after two wall hits or total timeout.
"""
import math
import random
import threading

from low_level_control.differential_drive import DifferentialDrive, WANDER, WALL
from mode import announceMode
from pybricks.hubs import EV3Brick
from pybricks.parameters import Port, Button
from pybricks.ev3devices import TouchSensor, UltrasonicSensor
from pybricks.tools import StopWatch, wait

SPEED = 60                      # duty speed % while driving a leg
TURN_SPEED = 25                 # duty % when turning in place
TURN_TOLERANCE = math.radians(4)
TURN_TIMEOUT = 6000             # safety timeout (ms) for a single turn
WANDER_TIMEOUT = 20000          # max time (ms) in WANDER before handing off to WALL
JITTER = 15                     # +/- random degrees around turns
BUMP_BACK_MS = 1000             # how long to back up after a bumper hit
POLL_MS = 20
DEBUG_PRINT = True

def wrapAngle(angle):
    return (angle + math.pi) % (2 * math.pi) - math.pi

class Wander:
    def __init__(self, ev3, drivebase, ultrasonic, bumper, wander_timeout=WANDER_TIMEOUT):
        self.ev3 = ev3
        self.drive = drivebase
        self.ultrasonic = ultrasonic
        self.bumper = bumper
        self.wander_timeout = wander_timeout

        self.running = False
        self.thread = None
        self.wall_hits = 0
        self.picked_initial_dir = False
        self.wander_timer = StopWatch()

    def start(self):
        random.seed(42)
        self.wall_hits = 0
        self.picked_initial_dir = False
        self.wander_timer.reset()
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
        self.wall_hits = 0
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

    def turnTo(self, target):
        timer = StopWatch()
        reached = False

        while timer.time() < TURN_TIMEOUT:
            if not self.owns() or self.bumped():
                self.halt()
                return False
            if self.wander_timer.time() >= self.wander_timeout:
                self.halt()
                self.giveToWall()
                return False

            _, _, theta = self.drive.getPose()
            error = wrapAngle(target - theta)
            if abs(error) < TURN_TOLERANCE:
                reached = True
                break

            speed = TURN_SPEED if error > 0 else -TURN_SPEED
            self.runMotors(-speed, speed)
            wait(10)

        self.halt()
        if reached:
            return True

        self.wall_hits += 1
        if self.wall_hits >= 2:
            self.giveToWall()
        return False

    def handleWallHit(self):
        if not self.backUp():
            return False

        mm = self.readMm()
        if DEBUG_PRINT:
            print("wander: wall hit, mm:", mm)
        self.wall_hits += 1
        if self.wall_hits >= 2:
            self.giveToWall()
            return False

        deg = random.uniform(-180.0, 180.0) + random.uniform(-JITTER, JITTER)
        _, _, theta = self.drive.getPose()
        target = wrapAngle(theta + math.radians(deg))
        return self.turnTo(target)

    def checkActive(self):
        if not self.owns():
            self.halt()
            return False
        if self.wander_timer.time() >= self.wander_timeout:
            self.halt()
            self.giveToWall()
            return False
        if self.bumped():
            return self.handleWallHit()
        return True

    def pickHeading(self, initial=False):
        _, _, theta = self.drive.getPose()
        if initial:
            target = wrapAngle(theta + random.uniform(-math.pi, math.pi))
        else:
            delta = random.uniform(-math.radians(120), math.radians(120))
            target = wrapAngle(theta + delta)

        if not self.turnTo(target):
            return None
        return target

    def driveLeg(self):
        while self.checkActive():
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
                self.wander_timer.reset()
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