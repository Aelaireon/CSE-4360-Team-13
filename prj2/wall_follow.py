#!/usr/bin/env pybricks-micropython

import threading
import time

from low_level_control.differential_drive import DifferentialDrive, WANDER, WALL
from pybricks.parameters import Port
from pybricks.ev3devices import TouchSensor, UltrasonicSensor

SPEED = 30                      # duty speed %
ULTRASONIC_WALL_MM = 100        # front distance (mm) that counts as approaching a wall/obstacle
BUMP_BACK = 0.05              # meters to back up slightly on a bumper hit
POLL_MS = 20

touchSensor = TouchSensor(Port.S1)

while not touchSensor.is_pressed:
    pass

contact = False
#(left_motor, right_motor) 

class WallFollower:
    def __init__(self, drivebase, ultrasonic, bumper):
        self.thread = threading.Thread(target=self.loop)
        self.running = False
        self.drive = drivebase
        self.bumper = bumper

    def runMotors(self, left, right):
        self.drive.runMotors(left, right, WALL)

    def halt(self):
        return self.drivebase.stop(WALL)

    def bumped(self):
        return self.bumper is not None and self.bumper.pressed()

if __name__ == "__main__":
    while True and time.wait(100):
        if(touchSensor.pressed()):
            print("Pressed!\n")
        else:
            print("Not Pressed!\n")