#!/usr/bin/env pybricks-micropython

import threading
import math

from low_level_control.differential_drive import DifferentialDrive, WANDER, WALL
from pybricks.hubs import EV3Brick
from pybricks.parameters import Port, Button, Color
from pybricks.ev3devices import TouchSensor, UltrasonicSensor
from pybricks.tools import StopWatch, wait

SPEED = 60                      # duty speed %
FRONT_OBSTACLE = 50             # front distance (mm) that counts as approaching a wall (only used by the disabled ultrasonic block in step())
FRONT_TURN = 80                 # degrees to turn left (CCW = +theta) if blocked straight ahead (only used by the disabled ultrasonic block)
BACK_UP_MS = 1000               # how long to back straight up after a bumper hit
BUMP_TURN_MS = 800              # how long to turn left (CCW) after backing up
TIMEOUT_WALL = 15000            # time budget (ms) before signaling time to return to wander
POLL_MS = 20


class WallFollower:
    def __init__(self, ev3, drivebase, ultrasonic, bumper, timeout_ms=TIMEOUT_WALL):
        self.ev3 = ev3
        self.drive = drivebase
        self.ultrasonic = ultrasonic
        self.bumper = bumper
        self.timeout_ms = timeout_ms

        self.running = False # default starts w/ wander
        self.thread = None
        self.wall_timer = StopWatch()

    def start(self):
        self.ev3.light.on(Color.BLUE)
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
        self.drive.runMotors(left, right*0.8, WALL)

    def bumped(self):
        return self.bumper is not None and self.bumper.pressed()
    
    def step(self):
        if not self.running or self.drive.driveOwner != WALL:
            return False

        # 1. Bounded time budget so we dont loop endlessly
        if self.wall_timer.time() >= self.timeout_ms:
            # handoff() (not setOwner) only gives WANDER the drivebase if we STILL own it. If goal
            # finding just took over, this does nothing instead of stealing it back from EXTINGUISH.
            self.drive.handoff(WALL, WANDER)
            self.wall_timer.reset() # fresh budget whenever WALL gets the drivebase back
            return False

        # 2. ultrasonic sensor senses upcoming wall: turn left to clear
        
        if self.ultrasonic.distance() <= FRONT_OBSTACLE:
            pass
        """
            self.halt()
            _, _, startTheta = self.drive.getPose()
            turn_timer = StopWatch()
            while turn_timer.time() < 2000:
                if not self
        """

        # 3. touch sensor hits wall: back up slightly and angle left
        if self.bumped():
            self.halt()
            # Back straight up for BACK_UP_MS
            back_timer = StopWatch()
            while back_timer.time() < BACK_UP_MS:
                if not self.running or self.drive.driveOwner != WALL:
                    self.halt()
                    return False
                self.runMotors(-30, -30)
                wait(10)
            self.halt()
            wait(100)

            turn_timer = StopWatch()
            while turn_timer.time() < BUMP_TURN_MS:
                if not self.running or self.drive.driveOwner != WALL:
                    self.halt()
                    return False
                # Left wheel backward, right wheel forward -> turns CCW (left)
                self.runMotors(-30, 30)
                wait(10)
            self.halt()
            wait(100)

            return True

        # 4. move forward normally
        self.runMotors(SPEED, SPEED)
        return True

    def loop(self):
        hadOwnership = False
        while self.running:
            if self.drive.driveOwner != WALL:
                hadOwnership = False
                wait(POLL_MS)
                continue

            # Start the time budget when we GAIN the drivebase. It must NOT be reset every pass,
            # or step()'s timeout check could never fire.
            if not hadOwnership:
                self.wall_timer.reset()
                hadOwnership = True

            self.step()
            wait(POLL_MS)
        self.halt()

if __name__ == "__main__":
    bumper = TouchSensor(Port.S2)
    ultrasonic = UltrasonicSensor(Port.S4)
    drivebase = DifferentialDrive(x=0, y=0, theta=0)
    ev3 = EV3Brick()

    wall_follower = WallFollower(ev3=ev3, drivebase=drivebase, ultrasonic=ultrasonic, bumper=bumper)
    drivebase.setOwner(WALL)
    wall_follower.start()

    try:
        while drivebase.running and Button.CENTER not in ev3.buttons.pressed():
            # For standalone testing: if the timeout handed ownership to WANDER, give WALL back
            # (the loop restarts the time budget when it regains the drivebase)
            if drivebase.driveOwner == WANDER:
                wait(500)
                drivebase.handoff(WANDER, WALL)
            wait(100)
    except KeyboardInterrupt:
        pass
    finally:
        drivebase.shutdown() # blocks all motor commands first, then the threads are told to end
        wall_follower.stop()
