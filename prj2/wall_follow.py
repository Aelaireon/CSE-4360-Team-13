import threading
import time

from low_level_control.differential_drive import DifferentialDrive, WANDER, WALL
from pybricks.parameters import Port
from pybricks.ev3devices import TouchSensor, UltrasonicSensor

SPEED = 30  # duty speed %

touchSensor = TouchSensor(Port.S1)

while not touchSensor.is_pressed:
    pass

contact = False
#(left_motor, right_motor) 

class WallFollower:
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
    while True and time.wait(100):
        if(touchSensor.pressed()):
            print("Pressed!\n")
        else:
            print("Not Pressed!\n")