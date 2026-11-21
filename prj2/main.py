#!/usr/bin/env pybricks-micropython

"""
CSE 4360 - Project 2: Wander, wall following, and goal finding to extinguish a fire.

Requires LEGO(R) EV3 MicroPython v2.0.
Download: https://education.lego.com/en-us/support/mindstorms-ev3/python-for-ev3

CURRENT TEST CONFIG: wall following + goal finding running together.
(wander is NOT wired in yet, so place the robot near a wall to start.)

THREADS
1. odometry            (DifferentialDrive)   always running
2. color monitor       (FireDetector)        always running, sensing only
3. wall following      (WallFollower.loop)   drives while it owns the drivebase (WALL)
4. extinguish          (GoalFinding.loop)    waits for fireFound, then takes ownership (EXTINGUISH)
5. main thread         (this file)           watches for the end condition / CENTER button

PRIORITY: goal finding always wins. When the color monitor latches fireFound, GoalFinding calls
setOwner(EXTINGUISH): the wheels stop and every WALL command is ignored from then on.

Stop at any time: press the brick's CENTER button.
"""
from pybricks.ev3devices import ColorSensor, Motor, TouchSensor, UltrasonicSensor
from pybricks.hubs import EV3Brick
from pybricks.parameters import Button, Port
from pybricks.tools import wait

from low_level_control.differential_drive import DifferentialDrive, WALL, WANDER
from wall_follow import WallFollower
from goal_finding import FireDetector, GoalFinding

# ---------------- ports ----------------
COLOR_PORT = Port.S1        # floor-facing color sensor (fire pad detection)
BUMPER_PORT = Port.S2       # touch sensor bumper (shared by wall following and goal finding)
ULTRASONIC_PORT = Port.S4   # front-facing ultrasonic (candle finding)
FAN_PORT = None             # e.g. Port.B once the fan motor is mounted. None = skip the fan

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
    wall = WallFollower(ev3, drivebase, ultrasonic, bumper)
    goal = GoalFinding(ev3, drivebase, detector, ultrasonic, bumper=bumper, fan=fan)

    try:
        drivebase.setOwner(WALL)
        wall.start()
        goal.start()
        print("running: wall following + goal finding. CENTER button to stop")

        announced = False
        while drivebase.running and not goal.finished and not stopPressed(ev3):
            if detector.fireFound and not announced:
                announced = True
                print("fire pad detected, goal finding taking over")

            # wall following hands ownership to WANDER when its time budget runs out.
            # Wander isn't integrated yet, so give the drivebase straight back to WALL instead of
            # leaving the robot idle. TODO: replace this with the real wander behavior.
            # handoff() is atomic: it does nothing if goal finding already took the drivebase.
            if not detector.fireFound:
                drivebase.handoff(WANDER, WALL)

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
        wall.stop()
        goal.stop()
        detector.stop()
        if fan is not None:
            fan.dc(0)


if __name__ == "__main__":
    main()
