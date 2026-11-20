#!/usr/bin/env pybricks-micropython

import threading
import random 

from low_level_control.differential_drive import DifferentialDrive, WANDER, WALL
from pybricks.hubs import EV3Brick
from pybricks.parameters import Port, Button
from pybricks.ev3devices import TouchSensor, UltrasonicSensor
from pybricks.tools import wait

SPEED = 30                      # duty speed %
ULTRASONIC_WALL_MM = 100        # front distance (mm) that counts as approaching a wall/obstacle
OPEN_SPACE_MM = 350             # front distance (mm) considered open room space
TURN_STEP_MS = 250              # duration of a small turn step
TURN_INTERVAL_MIN_MS = 1400     # minimum ms between gentle heading shifts
TURN_INTERVAL_MAX_MS = 2800     # maximum ms between gentle heading shifts
BUMP_BACK = 0.05                # meters to back up slightly on a bumper hit
POLL_MS = 20

touchSensor = TouchSensor(Port.S2)

#while not touchSensor.is_pressed:
#    pass

contact = False
#(left_motor, right_motor) 

class Wander:
    def __init__(self, drive_base, bumper):
        self.thread = threading.Thread(target=self.loop)
        self.running = False
        self.drive = drive_base
        self.bumper = bumper

    def runMotors(self, left, right):
        self.drive.runMotors(left, right, WANDER)
        
    def loop(self):
        while self.running:
            if contact:
                self.drive.setOwner(WALL)
            elif(not contact):
                self.runMotors(SPEED,0) #left
                self.runMotors(SPEED,SPEED) #forward
                self.runMotors(0,SPEED) #right

if __name__ == "__main__":
    while True and Button.CENTER not in EV3Brick().buttons.pressed():
        if(touchSensor.pressed()):
            print("Pressed!")
        else:
            print("Not Pressed!")
        wait(150)