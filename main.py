#!/usr/bin/env pybricks-micropython
from pybricks.hubs import EV3Brick
from pybricks.ev3devices import (Motor, TouchSensor, ColorSensor,
                                 InfraredSensor, UltrasonicSensor, GyroSensor)
from pybricks.parameters import Port, Stop, Direction, Button, Color
from pybricks.tools import wait, StopWatch, DataLog
from pybricks.robotics import DriveBase
from pybricks.media.ev3dev import SoundFile, ImageFile

import math


# This program requires LEGO EV3 MicroPython v2.0 or higher.
# Click "Open user guide" on the EV3 extension tab for more information.
# Initialize the EV3 Brick.


class RobotControl():

    def __init__(self):
        self.ev3 = EV3Brick()
        
        # Initialize motors
        self.motorR = Motor(Port.A, positive_direction=Direction.CLOCKWISE, gears=None)
        self.motorL = Motor(Port.B, positive_direction=Direction.CLOCKWISE, gears=None)
        self.motorPull = Motor(Port.C, positive_direction=Direction.CLOCKWISE, gears=None)
        self.motorGrip = Motor(Port.D, positive_direction=Direction.CLOCKWISE, gears=None)
        
        # Initialize drive base
        self.robot = DriveBase(self.motorR, self.motorL, wheel_diameter=50, axle_track=120)
        
        # Initialize sensors
        self.line_sensor = ColorSensor(Port.S1)
        
        # Constants
        self.BLACK = 9
        self.WHITE = 85
        self.speed = 360  # degs/s
        self.threshold = (self.BLACK + self.WHITE) / 2
        
        # Set the gain of the proportional line controller. This means that for every
        # percentage point of light deviating from the threshold, we set the turn
        # rate of the drivebase to 1.2 degrees per second.
        self.PROPORTIONAL_GAIN = 1.2
        
        # State variables
        self.MOVE = True
        self.PICKED_UP = False
        self.ColorToDrop:type(Color)
    
    def turnLeft90Robot(self):
        self.robot.turn(90)
        self.robot.stop()
    
    def turnRight90Robot(self):
        self.robot.turn(90)
        self.robot.stop()
    
    def turnRight90Tank(self):
        # I think I calculated this wrong , you should have a look
        estimated_wheel_raduis = 0.02 #m
        speed = 360
        tangential_speed = speed * estimated_wheel_raduis
        estimated_robot_radius = 0.20  # m
        
        time = (90 * math.pi * estimated_robot_radius)/(180 * tangential_speed)

        self.MotorL.run_time(speed=speed, time=time, then=Stop.HOLD, wait=True)
        self.MotorR.run_time(speed= -speed, time=time, then=Stop.HOLD, wait=True)

    def turnLeft90Tank(self):
        # I think I calculated this wrong , you should have a look
        estimated_wheel_raduis = 0.02 #m
        speed = 360
        tangential_speed = speed * estimated_wheel_raduis
        estimated_robot_radius = 0.20  # m
        
        time = (90 * math.pi * estimated_robot_radius)/(180 * tangential_speed)

        self.MotorL.run_time(speed= -speed, time=time, then=Stop.HOLD, wait=True)
        self.MotorR.run_time(speed= speed, time=time, then=Stop.HOLD, wait=True)
    
    def putDown(self):
        #turn then turn back on the track we shall see which one works best for now just puttin both 
        self.turnRight90Tank()
        self.turnLeft90Tank()
        # ---Shadi Will Work on it ---
        # speed = 100
        # carry_off_angle = -360
        # drop_angle = -360

        # self.motorGrip.run_until_stalled(speed=speed, then=Stop.COAST, duty_limit=None)
        # self.motorPull.run_target(speed, target_angle=carry_off_angle, then=Stop.HOLD, wait=True)
        # self.motorGrip.run_target(speed,target_angle=drop_angle,then=Stop.HOLD,wait=True)
        self.turnRight90Robot()
        self.turnLeft90Robot()

        self.PICKED_UP = False
        self.ColorToDrop = None

        self.ev3.speaker.say("I am about to Drop it")

        # once put down sequence is complete we can start moving again
        self.Moving()
        

    def pickUp(self):
       
        # ---Shadi Will Work on it---
        # speed = 100
        # # time = 1000
        # # rot_angle = 360
        # target_angle = 360
        # # self.motorPull.run_time(speed, time, then=Stop.HOLD, wait=True)
        # # self.motorPull.run_angle(speed, rotation_angle = rot_angle, then=Stop.HOLD, wait=True)

        # self.motorPull.run_target(speed, target_angle=target_angle, then=Stop.HOLD, wait=True)

        # self.motorGrip.run_until_stalled(speed=speed, then=Stop.COAST, duty_limit=None)
        # self.motorPull.run_target(speed, target_angle=target_angle, then=Stop.HOLD, wait=True)
        self.PICKED_UP = True
        self.MOVE = True
        self.Moving()
        
    def getColor(self):
        return self.line_sensor.color()

    def detection(self):
        detectedColor = self.getColor()
        if detectedColor != Color.BLACK or detectedColor != Color.WHITE:
            self.robot.stop()
            self.MOVE = False
            if self.PICKED_UP:
                if self.ColorToDrop == detectedColor:
                    self.putDown()
                else:
                    self.turnLeft90Robot()
                    self.Moving()
            else:
                self.pickUp()
        return detectedColor
       
    def Moving(self):
        if self.MOVE:
            self.detection()
            # Calculate the deviation from the threshold.
            deviation = self.line_sensor.reflection() - self.threshold

            # Calculate the turn rate.
            turn_rate = self.PROPORTIONAL_GAIN * deviation

            # Set the drive base speed and turn rate.
            self.robot.drive(speed=self.speed, turn_rate=turn_rate)
            # You can wait for a short time or do other things in this loop.
            # wait(10)
            self.Moving()
        self.ev3.speaker.say("I have stopped")
           

    def main(self):
        self.ev3.speaker.beep(frequency=500, duration=500)
        self.ev3.speaker.say("B")
        wait(1)
        self.ev3.speaker.say("M")
        wait(1)
        self.ev3.speaker.say("V")
        self.ev3.speaker.say("Start")
        self.Moving()


if __name__ == "__main__":
    robot_control = RobotControl()
    robot_control.ev3.speaker.beep(frequency=1000, duration=500)
    robot_control.main()

