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

    elif cs_back.reflection() > 8:
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
            

    # global front
    # global back

    # while cs_front.color() == Color.WHITE and back == False:
    #     print("Detected back")
    #     robot.stop()
    #     robot.drive(570, 0)
    #     time.sleep(0.2)
    #     robot.stop()

    #     if cs_back.reflection() >= 69:
    #         print("Lets go")
    #         front = True
    #         robot.drive(570, -100)

    #     else:
    #         robot.drive(570, 100)
    #         time.sleep(1)
    #         robot.stop()
    #         front = False

    # while cs_back.color() == Color.WHITE and front == False:
    #     print("Detected front")
    #     robot.stop()
    #     robot.drive(-1000, 0)
    #     time.sleep(0.2)
    #     robot.stop()

    #     if cs_front.reflection() >= 8:
    #         back = True
    #         robot.drive(-1000, 100)

    #     else:
    #         robot.drive(-1000, 100)
    #         time.sleep(1)
    #         robot.stop()
    #         back = False
        

def touch():
    if ts.pressed():
        robot.stop()
        robot.turn(180)
        ring_detected()
        # charge()

# def charge():
#     # if charge:
#     # while 40 > us.distance() < 350:
#     #     ring_detected()
#     #     robot.drive(3000, 0)
#     while 40 > us.distance() < 350:
        # ring_detected()


# def other_carge():

#     while us.distance() <= 40 or us.distance() == 2550:
#         ring_detected()
#         robot.turn(180)
#         robot.drive(3000, 0)


def main():
    time.sleep(0.04)
    robot.turn(9)
    while True:
        # robot.drive(200, 0)
        robot.drive(3000, 0)
        
        ring_detected()
        touch()
        # charge()
        # other_carge()

if __name__ == '__main__':
    main()
