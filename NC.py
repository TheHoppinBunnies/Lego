#!/usr/bin/env pybricks-micropython

from pybricks.hubs import EV3Brick
from pybricks.ev3devices import Motor, UltrasonicSensor, TouchSensor, ColorSensor
from pybricks.parameters import Port, Color
from pybricks.robotics import DriveBase
import time

ev3 = EV3Brick()

lm = Motor(Port.A)
rm = Motor(Port.D)

us = UltrasonicSensor(Port.S1)

cs_front = ColorSensor(Port.S3)
cs_back = ColorSensor(Port.S4)

ts = TouchSensor(Port.S2)

robot = DriveBase(lm, rm, wheel_diameter=56, axle_track=120)

# front = False
# back = False

def ring_detected():
    if cs_front.reflection() >= 5:
        print("Detected front")
        robot.stop()
        robot.drive(-700, 0)
        time.sleep(0.7)
        robot.stop()
        robot.turn(120)

    elif cs_back.reflection() > 6:
        print("Detected back")
        if ts.pressed():
            robot.stop()
            robot.drive(3000, 0)
            time.sleep(0.7)
            robot.stop()
            robot.turn(120)

        else:
            robot.stop()
            robot.drive(700, 0)
            time.sleep(0.7)
            robot.stop()
            robot.turn(120)
        

def touch():
    if ts.pressed():
        robot.stop()
        robot.turn(180)
        ring_detected()
        charge()

def charge():
    # if charge:
    while us.distance() < 350:
        ring_detected()
        robot.drive(3000, 0)

def other_charge():
    while us.distance() <= 5:
        ring_detected()
        robot.turn(180)
        robot.drive(3000, 0)

    


def main():
    time.sleep(0.04)
    ring_detected()
    robot.drive(3000, 0)
    time.sleep(4)
    robot.stop()
    while True:
        robot.drive(200, 0)
        ring_detected()
        touch()
        charge()
        other_charge()


if __name__ == '__main__':
    main()
