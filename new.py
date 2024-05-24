#!/usr/bin/env pybricks-micropython

from pybricks.hubs import EV3Brick
from pybricks.ev3devices import Motor, UltrasonicSensor, TouchSensor, ColorSensor
from pybricks.parameters import Port
from pybricks.robotics import DriveBase
import time

u = UltrasonicSensor(Port.S1)

while True:
    print(u.distance())