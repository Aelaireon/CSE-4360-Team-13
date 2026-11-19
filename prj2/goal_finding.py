from pybricks.hubs import EV3Brick
from pybricks.ev3devices import ColorSensor
from pybricks.parameters import Port, Color
from pybricks.tools import wait
import threading

ev3 = EV3Brick()
sensor = ColorSensor(Port.S1)

""" 
color sensor stuff in here
could either use color()
    - returns detected color (RED, YELLOW, GREEN, BLUE, WHITE, NONE)

if this proves to be inconsistent we could switch to rgb()
    - tuple[int, int, int]
    - percentage of RGB value from 0% to 100% reflection. 
"""

class FireDetector:
    def __init__(self, colorSensor, confirmCount=3):
        self.sensor = colorSensor
        self.confirmCount = confirmCount
        self.onPad = False          # live: sensor sees the pad right now
        self.fireFound = False      # latched: once True, stays True
        self.lastSeenPose = None    # drivebase.getPose() when first confirmed
        threading.Thread(target=self.run).start()

    def run(self):
        hits = 0
        while True:
            self.onPad = (self.sensor.color() == Color.RED)
            hits = hits + 1 if self.onPad else 0
            if hits >= self.confirmCount:
                self.fireFound = True
            wait(20)

"""
detected = sensor.color()

if detected == Color.RED:
    # start extinguishing fire sequence
elif detected == Color.NONE:
    # give up and wait a couple more seconds
"""