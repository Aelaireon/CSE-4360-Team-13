import threading
import goal_finding as goal
from low_level_control.differential_drive import DifferentialDrive, EXPLORE, EXTINGUISH, WALL
from ev3dev2.sensor import INPUT_1 as Input
from ev3dev2.sensor.lego import TouchSensor as Sensor

sp = 30

ts = Sensor(Input)

while not ts.is_pressed:
    pass

ts.wait_for_pressed()
print("Pressed!\n")

ts.wait_for_bump()
print("Pressed and released!\n")

contact = False
#(left_motor, right_motor) 

class Wandering:
    def __init__(self, drive_base, bumper):
        self.thread = threading.Thread(target=self.loop)
        self.running = False
        self.drive = drive_base
        self.bumper = bumper

    def runMotors(self, left, right):
        self.drive.runMotors(left, right, WALL)
        
    def loop(self):
        while self.running:
            if contact:
                self.drive.setOwner(WALL)
            elif(not contact):
                if(goal.left is True):
                    self.drive.runMotors(sp,0,EXPLORE) #left
                elif(goal.forward is True):
                    self.drive.runMotors(sp,sp,EXPLORE) #forward
                elif(goal.right is True):
                    self.drive.runMotors(0,sp,EXPLORE) #right
                else:
                    self.drive.runMotors(sp,0,EXPLORE) #left
                    self.drive.runMotors(sp,sp,EXPLORE) #forward
                    self.drive.runMotors(0,sp,EXPLORE) #right