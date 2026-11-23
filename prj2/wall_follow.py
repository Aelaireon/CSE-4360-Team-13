#!/usr/bin/env pybricks-micropython

import threading

from low_level_control.differential_drive import DifferentialDrive, WANDER, WALL
from prj2.mode import announceMode
from pybricks.hubs import EV3Brick
from pybricks.parameters import Port, Button
from pybricks.ev3devices import TouchSensor, UltrasonicSensor
from pybricks.tools import StopWatch, wait

SPEED = 60                      # duty speed %
FRONT_OBSTACLE = 100            # front distance (mm) that counts as approaching a wall (only used by the disabled ultrasonic block in step())
BACK_UP_MS = 1000               # how long to back straight up after a bumper hit
TURN = 800                      # how long (ms) to turn left (CCW) after backing up
TIMEOUT_WALL = 15000            # time budget (ms) for one stretch of wall following
NO_CONTACT_MS = 8000            # no bump for this long (ms) -> hand back to wander (possibly circling)
POLL_MS = 20


class WallFollower:
    def __init__(self, ev3, drivebase, ultrasonic, bumper, timeout_ms=TIMEOUT_WALL, no_contact_ms=NO_CONTACT_MS):
        self.ev3 = ev3
        self.drive = drivebase
        self.ultrasonic = ultrasonic
        self.bumper = bumper
        self.timeout_ms = timeout_ms
        self.no_contact_ms = no_contact_ms

        self.running = False # default starts w/ wander
        self.thread = None
        self.wall_timer = StopWatch()       # time since wall following gained the drivebase
        self.contact_timer = StopWatch()    # time since the last bump

    def start(self):
        self.wall_timer.reset() # reset stopwatch match to 0 so that it gets the full alloted time
        self.contact_timer.reset()
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

        # 1. touch sensor hits wall: back up slightly and angle left
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
            while turn_timer.time() < TURN:
                if not self.running or self.drive.driveOwner != WALL:
                    self.halt()
                    return False
                # Left wheel backward, right wheel forward -> turns CCW (left)
                self.runMotors(-30, 30)
                wait(10)
            self.halt()
            wait(100)

            self.contact_timer.reset() # "time since the last wall contact" counts from the end of the recovery
            return True

        # 2. time to hand over to wander? Only checked when NOT in the middle of a bump, so wander never starts with the bumper pressed
        if self.wall_timer.time() >= self.timeout_ms or self.contact_timer.time() >= self.no_contact_ms:
            # handoff() (not setOwner) only gives WANDER the drivebase if we STILL own it. If goal
            # finding just took over, this does nothing instead of stealing it back from EXTINGUISH.
            self.drive.handoff(WALL, WANDER)
            self.wall_timer.reset() # fresh budgets whenever WALL gets the drivebase back
            self.contact_timer.reset()
            return False

        # 3. ultrasonic sensor senses upcoming wall: turn left to clear
        if self.ultrasonic is not None and self.ultrasonic.distance() <= FRONT_OBSTACLE:
            self.halt()
            turn_timer = StopWatch()
            while turn_timer.time() < TURN:
                if not self.running or self.drive.driveOwner != WALL:
                    self.halt()
                    return False
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

            # on GAINING the drivebase: start the time budgets and play the mode cue (the wheels are stopped).
            # The budgets must NOT be reset every pass, or step()'s checks could never fire.
            if not hadOwnership:
                self.wall_timer.reset()
                self.contact_timer.reset()
                hadOwnership = True
                announceMode(self.ev3, WALL)

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
            # For standalone testing: if wall following handed ownership to WANDER, give WALL back
            # (the loop restarts the time budgets when it regains the drivebase)
            if drivebase.driveOwner == WANDER:
                wait(500)
                drivebase.handoff(WANDER, WALL)
            wait(100)
    except KeyboardInterrupt:
        pass
    finally:
        drivebase.shutdown() # blocks all motor commands first, then the threads are told to end
        wall_follower.stop()