#!/usr/bin/env pybricks-micropython

"""
CSE 4360 - Project 2: Wander, wall following, and goal finding to extinguish a fire.

Requires LEGO(R) EV3 MicroPython v2.0.
Download: https://education.lego.com/en-us/support/mindstorms-ev3/python-for-ev3

THREADS
1. odometry            (DifferentialDrive)   always running
2. color monitor       (FireDetector)        always running, sensing only
3. wander              (Wander.loop)         drives while it owns the drivebase (WANDER)
4. wall following      (WallFollower.loop)   drives while it owns the drivebase (WALL)
5. extinguish          (GoalFinding.loop)    waits for fireFound, then takes ownership (EXTINGUISH)
6. main thread         (this file)           only watches for the end condition / CENTER button

WHO DRIVES (the drivebase has exactly one owner at a time, every other behavior's commands are ignored)
  start -> WANDER (the robot starts in open floor, so it first looks for a wall)
  WANDER -> WALL        a wall is touched (bumper), or WANDER_MAX_MS passes without touching one
  WALL   -> WANDER      the wall time budget is used up, or no wall was touched for NO_CONTACT_MS
  any    -> EXTINGUISH  the color monitor latches fireFound. Goal finding always wins and never gives it back.
Every switch plays a light + beep cue (mode_cue.py). The behaviors hand the drivebase over themselves,
so this file has nothing to coordinate: it only starts the threads and waits.

Stop at any time: press the brick's CENTER button.
"""
from pybricks.ev3devices import ColorSensor, Motor, TouchSensor, UltrasonicSensor
from pybricks.hubs import EV3Brick
from pybricks.parameters import Button, Port
from pybricks.tools import wait

from low_level_control.differential_drive import DifferentialDrive, WALL, WANDER
from wander import Wander
from wall_follow import WallFollower
from goal_finding import FireDetector, GoalFinding

# ---------------- ports ----------------
COLOR_PORT = Port.S1        # floor-facing color sensor (fire pad detection)
BUMPER_PORT = Port.S2       # touch sensor bumper (shared by wander, wall following and goal finding)
ULTRASONIC_PORT = Port.S4   # front-facing ultrasonic (wander's open-space check, candle finding)
FAN_PORT = None             # e.g. Port.B once the fan motor is mounted. None = skip the fan

START_MODE = WANDER         # WALL to start by wall following instead (robot placed against a wall)
MONITOR_MS = 50


def stopPressed(ev3):
    return Button.CENTER in ev3.buttons.pressed()


def main():
    ev3 = EV3Brick()
    drivebase = DifferentialDrive(x=0, y=0, theta=0)

    bumper = TouchSensor(BUMPER_PORT)
    ultrasonic = UltrasonicSensor(ULTRASONIC_PORT)
    fan = Motor(FAN_PORT) if FAN_PORT is not None else None

    detector = FireDetector(ColorSensor(COLOR_PORT))    # its thread starts immediately
    wander = Wander(ev3, drivebase, ultrasonic, bumper)
    wall = WallFollower(ev3, drivebase, ultrasonic, bumper)
    goal = GoalFinding(ev3, drivebase, detector, ultrasonic, bumper=bumper, fan=fan)

    try:
        drivebase.setOwner(START_MODE)  # before the threads start. The behavior that owns it announces itself when its thread starts
        wander.start()
        wall.start()
        goal.start()
        print("running: wander + wall following + goal finding. CENTER button to stop")

        announced = False
        while drivebase.running and not goal.finished and not stopPressed(ev3):
            if detector.fireFound and not announced:
                announced = True
                print("fire pad detected, goal finding taking over")
            wait(MONITOR_MS)

        if goal.finished:
            if goal.candleSeen:
                print("done: candle seen by the ultrasonic, robot faced it")
            else:
                print("done: candle not seen, robot aimed at the middle of the pad")
        else:
            print("stopped before the sequence finished")
    except KeyboardInterrupt:
        print("interrupted")
    finally:
        # 1. shutdown() first: it clears the drivebase owner, so from this instant every motor
        #    command from any thread (including a goal thread mid-spin) is ignored
        drivebase.shutdown()
        # 2. tell the threads to end (their loops check running). The program exits once they have
        wander.stop()
        wall.stop()
        goal.stop()
        detector.stop()
        if fan is not None:
            fan.dc(0)


if __name__ == "__main__":
    main()
