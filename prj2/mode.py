#!/usr/bin/env pybricks-micropython

# the visual/audio cue played whenever the robot switches behavior.
from pybricks.parameters import Color
from pybricks.tools import wait

from low_level_control.differential_drive import WANDER, WALL, EXTINGUISH

def announceMode(ev3, mode):
    if mode == WANDER:
        ev3.light.on(Color.ORANGE)
        ev3.speaker.beep(1200, 100)
        wait(160)   # longer than the beep, so second one doesnt cut it off
        ev3.speaker.beep(1200, 100)
    elif mode == WALL:
        ev3.light.on(Color.GREEN)
        ev3.speaker.beep(600, 300)
    elif mode == EXTINGUISH:
        ev3.light.on(Color.RED)
        ev3.speaker.beep(1000, 100)
    print("mode:", mode)