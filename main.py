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
        self.motorPulley = Motor(Port.C, positive_direction=Direction.CLOCKWISE, gears=None)
        self.motorGear = Motor(Port.D, positive_direction=Direction.CLOCKWISE, gears=None)
        

        
        
        # Initialize sensors
        self.line_sensor = ColorSensor(Port.S1)
        self.box_color_sensor = ColorSensor(Port.S2)
        self.obstacle_sensor = UltrasonicSensor(Port.S4)
        
        # Constants
        self.BLACK = 9
        self.WHITE = 85
        self.pick_up_area_color = Color.YELLOW

        self.DRIVE_SPEED = 360  # deg/s
        self.TURN_SPEED = 200 # deg/s
        self.PULLEY_SPEED = 180 # deg/s
        self.GEAR_SPEED = 180 # deg/s

        self.GEAR_PITCH_DIAMETER = 180
        self.PULLEY_DIAMTER = 


        self.WHEEL_DIAMETER = 50
        self.AXLE_TRACK = 120

        
        # Set the gain of the proportional line controller. This means that for every
        # percentage point of light deviating from the threshold, we set the turn
        # rate of the drivebase to 1.2 degrees per second.
        self.PROPORTIONAL_GAIN = 1.2
        
        # State variables
        self.picked_up = False
        self.color_to_drop:type(Color)
        self.threshold = (self.BLACK + self.WHITE) / 2


        # Initialize drive base
        self.robot = DriveBase(self.motorR, self.motorL, wheel_diameter=self.WHEEL_DIAMETER, axle_track=self.AXLE_TRACK)

    





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
        self.ev3.speaker.say("Placing down box")
        self.picked_up = False
        self.ColorToDrop = None

        self.turnRight90Robot()
        self.motorPulley.run_until_stalled(speed = self.PULLEY_SPEED,then = Stop.Hold, duty_limit = 40)

        self.motorGear.run_angle(speed= -self.GEAR_SPEED,then = Stop.Hold)
        self.motorPulley.run_angle(speed= -self.PULLEY_SPEED,then = Stop.Hold)



        self.turnRight90Robot()

        self.picked_up = False
        self.ColorToDrop = None


      
        

    def pickUp(self):
        self.color_to_drop = self.box_color_sensor.color()
       
        self.motorGear.hold()

        self.motorPulley.run_until_stalled(speed = self.PULLEY_SPEED,then = Stop.Hold, duty_limit = 40)

        self.motorGear.run_angle(speed= self.GEAR_SPEED,then = Stop.Hold)
        self.motorPulley.run_angle(speed= self.PULLEY_SPEED,then = Stop.Hold)


        
        
        self.turnRight90Robot()
        self.turnRight90Robot()

        self.picked_up = True



    def follow_line_until_change(self):
        current_color = self.line_sensor.color()
        while self.line_sensor.color() == current_color:

            if self.obstacle_sensor().distance() <= 300
                self.ev3.speaker.say("Please help me")
                wait(3000)
                break

            deviation = self.line_sensor.reflection() - threshold
            turn_rate = self.PROPORTIONAL_GAIN * deviation
            robot.drive(self.DRIVE_SPEED, turn_rate)
            wait(10)
            current_color = self.line_sensor.color()
        robot.stop()
        return current_color
            


           

    def main(self):
        self.ev3.speaker.beep(frequency=500, duration=500)
        self.ev3.speaker.say("B")
        wait(100)
        self.ev3.speaker.say("M")
        wait(100)
        self.ev3.speaker.say("V")
        wait(100)
        self.ev3.speaker.beep(frequency=700, duration=200)
        self.ev3.speaker.say("Start")


        while True
            new_color = self.follow_line_until_change()
            self.threshold = self.line_sensor.reflection()

            if self.picked_up():
                if new_color == self.color_to_drop():
                    self.putDown()

            else:
                if new_color == self.pick_up_area_color
                    while self.obstacle_sensor().distance() > 300:
                        deviation = self.line_sensor.reflection() - threshold
                        turn_rate = self.PROPORTIONAL_GAIN * deviation
                        robot.drive(self.DRIVE_SPEED, turn_rate)
                        wait(10)
                    self.pickUp()
                



if __name__ == "__main__":
    robot_control = RobotControl()
    robot_control.ev3.speaker.beep(frequency=1000, duration=500)
    robot_control.main()

