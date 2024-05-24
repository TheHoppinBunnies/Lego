#!/usr/bin/env pybricks-micropython

from pybricks.hubs import EV3Brick
from pybricks.ev3devices import Motor, UltrasonicSensor, TouchSensor, ColorSensor
from pybricks.parameters import Port
from pybricks.robotics import DriveBase
import time

ev3 = EV3Brick()

left_motor = Motor(Port.A)
right_motor = Motor(Port.D)

ultrasonic_sensor = UltrasonicSensor(Port.S1)

cs_front = ColorSensor(Port.S3)
# cs_back = ColorSensor(Port.S4)

touch_sensor = TouchSensor(Port.S2)

robot = DriveBase(left_motor, right_motor, wheel_diameter=56, axle_track=120)

# front = False
# back = False

def ring_detected():
    # global front
    # global back

    if cs_front.reflection() >= 5:
        print("Detected front")
        robot.stop()
        robot.drive(-700, 0)
        time.sleep(0.7)
        robot.stop()
        robot.turn(120)
        

def touch():
    if touch_sensor.pressed():
        robot.stop()
        robot.turn(180)

def charge():
    # if charge:
    while ultrasonic_sensor.distance() < 350:
        ring_detected()
        robot.drive(3000, 0)


def main():
    # time.sleep(0.5)
    while True:
        robot.drive(200, 0)
        ring_detected()
        touch()
        if ultrasonic_sensor.distance() < 350:
            robot.drive(3000, 0)


if __name__ == '__main__':
    main()
