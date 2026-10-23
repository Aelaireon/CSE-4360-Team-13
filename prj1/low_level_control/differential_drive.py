#!/usr/bin/env pybricks-micropython
import math
import threading

from pybricks.ev3devices import GyroSensor, Motor
from pybricks.parameters import Direction, Port, Stop
from pybricks.tools import StopWatch, wait
from low_level_control.pid import PID

class DifferentialDrive:
    def __init__(self, x, y, theta, maxMotorSpeed=40, wheelDiameter=30.37, axleTrack=123.31, leftMotorPort=Port.A, rightMotorPort=Port.D):
        """
        Creates a two-wheeled differential robot controller.
        
        :param m x: The starting x coordinate
        :param m y: The starting y coordinate
        :param rad theta: The starting orientation
        :param % maxMotorSpeed: The max motor duty cycle in percent
        :param mm wheelDiameter: Wheel diameter
        :param mm axleTrack: Distance between the two wheels
        :param Port leftMotorPort: Left motor port
        :param Port rightMotorPort: Right motor port
        """
        
        self.x = x # Current x position in meters
        self.y = y # Current y position in meters
        self.theta = theta # Current heading in radians
        
        self.maxMotorSpeed = maxMotorSpeed
        
        self.wheelDiameter = wheelDiameter / 1000.0 # mm to meters
        self.axleTrack = axleTrack / 1000.0 # mm to meters
        self.leftMotorPort = leftMotorPort
        self.rightMotorPort = rightMotorPort
        
        # Turn on spot feature
        self.enableTurnOnSpot = True
        self.isTurningOnSpot = False
        self.turnOnSpotThreshold = math.radians(10)
        self.stopTurnOnSpotThreshold = math.radians(8)
        
        # Robot physical parameters
        self.mass = 0.592 # kg
        self.wheelCircumference = math.pi * self.wheelDiameter
        
        # PID controller parameters
        kp_position = 50.0
        kd_position = 2 * math.sqrt(self.mass * kp_position)
        self.positionPID = PID(
            kP=kp_position, 
            kI=0.0, 
            kD=kd_position, 
            outputRange=[-maxMotorSpeed, maxMotorSpeed]
        )
        
        kp_heading = 100.0
        # ki_heading = 10.0
        ki_heading = 0.0
        kd_heading = 2 * math.sqrt(self.mass * kp_heading)
        self.headingPID = PID(
            kP=kp_heading, 
            kI=ki_heading, 
            kD=kd_heading, 
            accumLimit=10, 
            accumUnder=math.radians(10), 
            resetIntegralOnSignChange=True,
            outputRange=[-maxMotorSpeed, maxMotorSpeed]
        )

        # Configuration
        self.timer = StopWatch()
        self.leftSpeed = 0.0
        self.rightSpeed = 0.0
        self.rampLeftSpeed = 0.0
        self.rampRightSpeed = 0.0
        self.configureSensors()
        self.configureMotors()
        
        # Start threads
        self.odometryThread = threading.Thread(target=self.odometryLoop)
        self.odometryThread.start()
        self.logger = threading.Thread(target=self.loggerLoop)
        self.logger.start()
    
    def loggerLoop(self):
        while True:
            # Update pose
            print("Position: (", self.x * 1000, ", ", self.y * 1000, "), Heading: ", math.degrees(self.theta), " degrees")
            # Update commands
            print("OnSpotTurn: ", self.isTurningOnSpot, "left: ", self.rampLeftSpeed, "right: ", self.rampRightSpeed)
            print()
    
    def odometryLoop(self):
        prevLeftMotorRotation = 0.0
        prevRightMotorRotation = 0.0
        while True:
            # Get the current motor rotations
            leftMotorRotation = self.leftMotor.angle() # in degrees
            rightMotorRotation = self.rightMotor.angle() # in degrees
            
            # Calculate the change in motor rotations since the last update
            deltaLeftMotorRotation = leftMotorRotation - prevLeftMotorRotation
            deltaRightMotorRotation = rightMotorRotation - prevRightMotorRotation
            
            # Update previous motor rotations for the next iteration
            prevLeftMotorRotation = leftMotorRotation
            prevRightMotorRotation = rightMotorRotation
            
            # Update odometry based on motor rotations
            self.odometryUpdate(deltaLeftMotorRotation, deltaRightMotorRotation)
        
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
        self.leftMotor = Motor(self.leftMotorPort, positive_direction=Direction.CLOCKWISE) # only way to get forward to be positive
        self.rightMotor = Motor(self.rightMotorPort, positive_direction=Direction.CLOCKWISE)

    def drive(self, distance, heading):
        """
        Drive the robot at a given distance and heading.
        
        :param distance: Distance to drive in meters.
        :param heading: Heading to maintain in radians.
        """
        targetX = self.x + distance * math.cos(heading)
        targetY = self.y + distance * math.sin(heading)
        
        prevTime = -0.0001
        while math.sqrt((targetX - self.x) ** 2 + (targetY - self.y) ** 2) > 0.01: # Continue until within 1 cm of target
            currentTime = self.timer.time()
            dt = currentTime - prevTime
            prevTime = currentTime
            
            # Calculate the desired heading to the target position
            desiredHeading = math.atan2(targetY - self.y, targetX - self.x)
            headingError = desiredHeading - self.getHeading()
            headingError = (headingError + math.pi) % (2 * math.pi) - math.pi # Normalize to [-pi, pi]
            
            # Use PID controllers for speed and heading
            distanceError = math.sqrt((targetX - self.x) ** 2 + (targetY - self.y) ** 2) # in meters
            distanceError *= 100 # Convert to cm
            speed = self.positionPID.calculate(0, -distanceError, dt) # Negative because we want to reduce the error
            turnSpeed = self.headingPID.calculate(0, -headingError, dt) # Negative because we want to reduce the error
            
            # Turn on spot feature
            if self.enableTurnOnSpot and abs(headingError) > self.turnOnSpotThreshold:
                self.isTurningOnSpot = True
            if self.enableTurnOnSpot and abs(headingError) < self.stopTurnOnSpotThreshold:
                self.isTurningOnSpot = False
            if self.enableTurnOnSpot and self.isTurningOnSpot:
                speed = 0
            
            # Set motor speeds based on PID outputs
            self.leftSpeed = speed - turnSpeed
            self.rightSpeed = speed + turnSpeed
            
            # Limit speed to maxMotorSpeed
            self.leftSpeed = self.limit(self.leftSpeed, self.maxMotorSpeed)
            self.rightSpeed = self.limit(self.rightSpeed, self.maxMotorSpeed)
            
            # Ramp rate control
            self.rampLeftSpeed, self.rampRightSpeed = self.rampRateControl(self.leftSpeed, self.rightSpeed, self.rampLeftSpeed, self.rampRightSpeed)
            
            # self.leftMotor.dc(self.leftSpeed)
            # self.rightMotor.dc(self.rightSpeed)
            self.leftMotor.dc(self.rampLeftSpeed)
            self.rightMotor.dc(self.rampRightSpeed)
            
            wait(10) # Wait for a short duration before the next update
        self.stop()
    
    def rampRateControl(self, leftSpeed, rightSpeed, rampLeftSpeed, rampRightSpeed):
        leftIncrement = self.limit(leftSpeed - rampLeftSpeed, self.maxMotorSpeed / 10)
        rampLeftSpeed += leftIncrement
        
        rightIncrement = self.limit(rightSpeed - rampRightSpeed, self.maxMotorSpeed / 10)
        rampRightSpeed += rightIncrement
        return rampLeftSpeed, rampRightSpeed
    
    def limit(self, value, limit):
        if value > limit:
            return limit
        if value < -limit:
            return -limit
        return value
    
    def getPose(self):
        """
        Returns the current pose of the robot in meters and radians as a tuple.
        
        Example:
        global_x, global_y, global_theta = drivebase.getPose()
        """
        # return self.gyro.angle() * math.pi / 180 # Convert degrees to radians
        return self.x, self.y, self.theta

    def stop(self):
        self.leftMotor.dc(0) # Stop left motor
        self.rightMotor.dc(0) # Stop right motor

if __name__ == "__main__":
    drivebase = DifferentialDrive(x=0, y=0, theta=0)
    tile = 0.3048 # 1 foot in meters
    
    # Bottom right square
    drivebase.drive(tile, math.radians(-90))
    drivebase.drive(tile, math.radians(-180))
    drivebase.drive(tile, math.radians(-270))
    drivebase.drive(tile, math.radians(-360))
    
    # Bottom left square
    drivebase.drive(tile, math.radians(90))
    drivebase.drive(tile, math.radians(180))
    drivebase.drive(tile, math.radians(270))
    drivebase.drive(tile, math.radians(360))
    
    # Top left square
    drivebase.drive(tile, math.radians(0))
    drivebase.drive(tile, math.radians(90))
    drivebase.drive(tile, math.radians(180))
    drivebase.drive(tile, math.radians(270))
    
    # Top right square
    drivebase.drive(tile, math.radians(-0))
    drivebase.drive(tile, math.radians(-90))
    drivebase.drive(tile, math.radians(-180))
    drivebase.drive(tile, math.radians(-270))
