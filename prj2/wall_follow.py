#!/usr/bin/env pybricks-micropython

import threading
import time
import math

from low_level_control.differential_drive import DifferentialDrive, WANDER, WALL
from pybricks.hubs import EV3Brick
from pybricks.parameters import Port, Button
from pybricks.ev3devices import TouchSensor, UltrasonicSensor
from pybricks.tools import StopWatch, wait

SPEED = 30                      # duty speed %
FRONT_OBSTACLE = 100            # front distance (mm) that counts as approaching a wall/obstacle
BUMP_BACK = 0.05                # meters to back up slightly on a bumper hit
TURN_AWAY = 50              # degrees to turn left (CCW = +theta) after tapping the wall
FRONT_TURN = 80             # degrees to turn left (CCW = +theta) if blocked straight ahead
TIMEOUT_WALL = 15000           # time budget before signaling time to return to wander
POLL_MS = 20

contact = False
#(left_motor, right_motor) 

class WallFollower:
    def __init__(self, drivebase, ultrasonic, bumper, timeout_ms=TIMEOUT_WALL):
        self.drive = drivebase
        self.ultrasonic = ultrasonic
        self.bumper = bumper
        self.timeout_ms = timeout_ms

        self.running = False # default starts w/ wander
        self.thread = None
        self.wall_timer = StopWatch()

    def start(self):
        self.wall_timer.reset() # reset stopwatch match to 0 so that it gets the full alloted time
        self.running = True
        self.thread = threading.Thread(target=self.loop)
        self.thread.start()

    def stop(self):
        self.running = False
        self.halt()

    def halt(self):
        return self.drive.stop(WALL)
    
    def runMotors(self, left, right):
        self.drive.runMotors(left, right*0.98, WALL)

    def bumped(self):
        return self.bumper is not None and self.bumper.pressed()
    
    def step(self):
        if not self.running or self.drive.driveOwner != WALL:
            return False

        # 1. Bounded time budget so we dont loop endlessly
        if self.followTimer.time() >= self.maxFollowMs:
            self.halt()
            self.drive.setOwner(WANDER)
            return False

        # 2. ultrasonic sensor senses upcoming wall: turn left to clear
        if self.ultrasonic.distance() <= FRONT_OBSTACLE:
            self.halt()
            _, _, theta = self.drivebase.getPose()
            if not self.drivebase.drive(0.0, theta + math.radians(FRONT_TURN), WALL):   # 80 DEGREES, TUNABLE
                return False
            return True

        # 3. touch sensor hits wall: back up slightly and angle left
        if self.bumped():
            self.halt()
            _, _, theta = self.drivebase.getPose()
            if not self.drivebase.drive(-BUMP_BACK, theta, WALL):
                return False
            if not self.drivebase.drive(0.0, theta + math.radians(TURN_AWAY), WALL):    # 50 DEGREES, TUNABLE
                return False
            return True

        # 4. move forward normally
        self.motors(SPEED, SPEED)
        return True

    def loop(self):
        while self.running:
            if self.drive.driveOwner != WALL:
                wait(POLL_MS)
                continue

            self.wall_timer.reset()
            self.step()
            wait(POLL_MS)
        self.halt()

if __name__ == "__main__":
    bumper = TouchSensor(Port.S2)
    ultrasonic = UltrasonicSensor(Port.S4)
    drivebase = DifferentialDrive(x=0, y=0, theta=0)
    ev3 = EV3Brick()

    wall_follower = WallFollower(drivebase=drivebase, ultrasonic=ultrasonic, bumper=bumper)
    drivebase.setOwner(WALL)
    wall_follower.start()

    try:
        while drivebase.running and Button.CENTER not in ev3.buttons.pressed():
            # For standalone testing: if timeout handed ownership to WANDER, reset and give WALL back
            if drivebase.driveOwner == WANDER:
                wait(500)
                wall_follower.wall_timer.reset()
                drivebase.setOwner(WALL)
            wait(100)
    except KeyboardInterrupt:
        pass
    finally:
        wall_follower.stop()
        drivebase.shutdown()