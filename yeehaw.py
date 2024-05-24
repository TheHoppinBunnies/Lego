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

cs_front = ColorSensor(Port.S4)
cs_back = ColorSensor(Port.S3)

ts = TouchSensor(Port.S2)

robot = DriveBase(lm, rm, wheel_diameter=56, axle_track=120)

isd = False


while True:
    if cs_back.color() == Color.WHITE:
        print("Yeehaw front")

    elif cs_front.color() == Color.WHITE:
        print("Yeehaw back")

    else:
        ev3.speaker.beep(100, 10)
        # print("Front Sensor:" + str(cs_back.reflection()))
        # print("Back Sensor:" + str(cs_front.reflection()))