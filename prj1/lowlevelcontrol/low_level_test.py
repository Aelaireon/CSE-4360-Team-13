#!/usr/bin/env pybricks-micropython
import math
import threading

from pybricks.ev3devices import GyroSensor, Motor
from pybricks.parameters import Direction, Port, Stop
from pybricks.tools import StopWatch, wait
from pid import PID

class PIDMotor(Motor):
    def __init__(self, port, pid: PID):
        super().__init__(port)
        self.pid = pid

    def set_speed(self, targetSpeed):
        controlValue = self.pid.calculate(target=targetSpeed, current=self.speed())
        self.dc(controlValue)  # Set motor duty cycle based on PID output

class DifferentialDrive:
    def __init__(self, x, y, theta, wheelDiameter=30.37, axleTrack=123.31, leftMotorPort=Port.A, rightMotorPort=Port.D):
        self.x = x  # Current x position in meters
        self.y = y  # Current y position in meters
        self.theta = theta  # Current heading in radians
            
        self.wheelDiameter = wheelDiameter / 1000.0 # mm to meters
        self.axleTrack = axleTrack / 1000.0 # mm to meters
        self.leftMotorPort = leftMotorPort
        self.rightMotorPort = rightMotorPort
                
        self.wheelCircumference = math.pi * self.wheelDiameter
        self.prevLeftMotorRotation = 0.0
        self.prevRightMotorRotation = 0.0

        self.configureSensors()
        self.configureMotors()
        self.timer = StopWatch()
        self.odometryThread = threading.Thread(target=self.odometryLoop)
        # self.odometryThread.start()
        self.odometryLoop() # Testing only odometry
    
    def odometryLoop(self):
        prevTime = self.timer.time() - 0.000001
        while True:
            currentTime = self.timer.time()
            # Get the current motor rotations
            leftMotorRotation = self.leftMotor.angle()  # in degrees
            rightMotorRotation = self.rightMotor.angle()  # in degrees
            
            # Calculate the change in motor rotations since the last update
            deltaLeftMotorRotation = leftMotorRotation - self.prevLeftMotorRotation
            deltaRightMotorRotation = rightMotorRotation - self.prevRightMotorRotation
            
            # Update previous motor rotations for the next iteration
            self.prevLeftMotorRotation = leftMotorRotation
            self.prevRightMotorRotation = rightMotorRotation
            
            # Update odometry based on motor rotations
            self.odometryUpdate(deltaLeftMotorRotation, deltaRightMotorRotation)
            
            # Update display
            print("Position: (", self.x * 1000, ", ", self.y * 1000, "), Heading: ", math.degrees(self.theta), " degrees")
        
    def odometryUpdate(self, leftMotorRotation, rightMotorRotation):
        # Calculate the distance each wheel has traveled
        leftDistance = (leftMotorRotation / 360.0) * self.wheelCircumference
        rightDistance = (rightMotorRotation / 360.0) * self.wheelCircumference
        
        # Calculate the change in position and heading
        deltaDistance = (leftDistance + rightDistance) / 2.0
        deltaTheta = (rightDistance - leftDistance) / self.axleTrack
        
        # Update the robot's position and heading
        self.x += deltaDistance * math.cos(self.theta + deltaTheta / 2.0)
        self.y += deltaDistance * math.sin(self.theta + deltaTheta / 2.0)
        self.theta += deltaTheta
    
    def configureSensors(self):
        # self.gyro = GyroSensor(Port.S1)
        # wait(100)
        # self.gyro.reset_angle(0)
        pass
        
    def configureMotors(self):
        # self.leftMotor = PIDMotor(self.leftMotorPort, pid=PID(kP=1.0, kI=0.0, kD=0.0, outputRange=[-100, 100]))
        # self.rightMotor = PIDMotor(self.rightMotorPort, pid=PID(kP=1.0, kI=0.0, kD=0.0, outputRange=[-100, 100]))
        self.leftMotor = Motor(self.leftMotorPort, positive_direction=Direction.CLOCKWISE)
        self.rightMotor = Motor(self.rightMotorPort, positive_direction=Direction.CLOCKWISE)

    def turnTo(self, targetHeading):
        """
        Turn the robot to a specific heading. no gyro
        
        :param targetHeading: Target heading in radians.
        """
        # 1. get total rotations for each motor to travel the arc distance calculated from theta and axle track
        error = targetHeading - self.getHeading()
        percentError = error / (2.0 * math.pi)  # Normalize error to a fraction of a full rotation
        axleCircumference = 2.0 * math.pi * (self.axleTrack / 2.0)
        arcDistanceMeters = percentError * axleCircumference
        
        # 2. convert to motor targets
        leftMotorRadians = arcDistanceMeters / (self.wheelDiameter / 2.0)
        rightMotorRadians = arcDistanceMeters / (self.wheelDiameter / 2.0)
        
        # 3. set motors to run to target positions
        self.leftMotor.run_target(-100, math.degrees(leftMotorRadians), Stop.HOLD, wait=False)
        self.rightMotor.run_target(100, math.degrees(rightMotorRadians), Stop.HOLD, wait=True)
    
    def driveStraight(self, distance):
        """
        Drive the robot straight for a specific distance. no gyro
        
        :param distance: Distance to drive in meters.
        """
        # 1. convert distance to motor rotations
        wheelRotationsRadians = 2.0 * math.pi * (distance / self.wheelCircumference)
        
        # 2. set motors to run to target positions
        self.leftMotor.run_target(100, -math.degrees(wheelRotationsRadians), Stop.HOLD, wait=False)
        self.rightMotor.run_target(100, math.degrees(wheelRotationsRadians), Stop.HOLD, wait=True)

    def drive(self, distance, heading):
        """
        Drive the robot at a given distance and heading.
        
        :param distance: Distance to drive in meters.
        :param heading: Heading to maintain in radians.
        """
        
        targetDistance = distance  # in meters
        targetHeading = heading  # in radians
        # while 
    
    def getHeading(self):
        """
        Returns the current heading of the robot in radians.
        """
        # return self.gyro.angle() * math.pi / 180  # Convert degrees to radians
        return self.theta

    def stop(self):
        self.leftMotor.dc(0)  # Stop left motor
        self.rightMotor.dc(0)  # Stop right motor

if __name__ == "__main__":
    drivebase = DifferentialDrive(x=0, y=0, theta=0)
    # drivebase.driveStraight(0.03)  # Drive forward 1000 degrees with a heading of 360 degrees
    # drivebase.turnTo(math.radians(90))
    # drivebase.driveStraight(0.03)  # Drive forward 1000 degrees with a heading of 360 degrees
    # drivebase.turnTo(math.radians(90))