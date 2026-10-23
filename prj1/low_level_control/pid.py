#!/usr/bin/env pybricks-micropython

class PID:
    """
    ### PID class - create PID controller

    This class is used to simplify creation of PID controllers.

    #### Arguments:
        kP: The proportional constant (how reactive the PID controller is)
        kI: The integral constant (how impactful the PID controllelr is for small movements)
        kD: The derivative constant (how much to dampen the PID controller)
        outputRange: Upper and lower limit for PID output
        accumLimit: Limit for integral accumlation from errors
        accumUnder: Threshold for error under which integral accumulation starts (i.e. integral is 0 until abs(error) < accumUnder)

    #### Returns:
        A new PID controller object

    #### Examples:
        pid1 = PID(1)\\
        pid2 = PID(1, 0.5)\\
        pid3 = PID(1, 0.5, 0.25)
        pid4 = PID(1, 0.5, 0.25, [-1.0,1.0])
        pid5 = PID(1, outputRange=[0, 1.0])

        pid.calculate(target=90.0, current=0.0) # 90 degrees
    """

    def __init__(self,
                 kP=0.0,
                 kI=0.0,
                 kD=0.0,
                 outputRange=[-1.0, 1.0],
                 accumLimit=1000.0,
                 accumUnder=1000.0,
                 integralAccumRatio=1.0,
                 resetIntegralOnSignChange=False):
        self.kP = kP
        self.kI = kI
        self.kD = kD
        self.outputRange = outputRange
        self.accumLimit = accumLimit
        self.accumUnder = accumUnder
        self.integralAccumRatio = integralAccumRatio
        self.resetIntegralOnSignChange = resetIntegralOnSignChange

        self.prevError = 0.0
        self.integral = 0.0
        self.prevDir = 0.0 # Tracks current direction (i.e. which sign error is +/-)

    def getIntegralAccum(self):
        return self.integral
    
    def resetIntegral(self):
        self.integral = 0

    def _getSign(self, val):
        return val/abs(val) if val != 0 else 0

    def calculate(self, target, current, dt) -> float:
        """
        ### Calculate method - calculate PID output

        This method is used to calculate a new output based on the previous output

        #### Arguments:
            target: Desired value
            current: Current value to compare to

        #### Returns:
            The control value as a float
        """
        # Calculate error
        error = target - current
        self.prevDir = self._getSign(error)

        # Accumlate error
        # ratio > 1.0 means that vehicle went too fast and needs to decrease integral faster
        self.integral += error * (self.integralAccumRatio if (error < 0 and target > 0) or (error > 0 and target < 0) else 1.0)

        # Update derivative value
        derivative = (error - self.prevError) / dt

        # Limit integral accumlation
        if self.kI != 0.0 and self.resetIntegralOnSignChange and self._getSign(error) != self._getSign(self.prevError): self.integral = 0.0
        if self.integral > self.accumLimit: self.integral = self.accumLimit
        if self.integral < -self.accumLimit: self.integral = -self.accumLimit

        # Keep track of previous error
        self.prevError = error

        # Integral Accum stay 0 until near target
        if abs(error) > self.accumUnder:
            self.integral = 0.0

        # Calculate PID output
        controlValue = (self.kP * error) + (self.kI * self.integral) + (self.kD * derivative)

        # Limit PID output
        if controlValue > self.outputRange[1]: controlValue = self.outputRange[1]
        if controlValue < self.outputRange[0]: controlValue = self.outputRange[0]

        return controlValue