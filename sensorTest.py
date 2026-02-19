from pybricks.hubs import EV3Brick
from pybricks.ev3devices import (Motor, TouchSensor, ColorSensor,InfraredSensor, GyroSensor)
from pybricks.parameters import Port, Stop, Direction, Button, Color
from pybricks.nxtdevices import LightSensor, UltrasonicSensor
from pybricks.tools import wait, StopWatch, DataLog
from pybricks.robotics import DriveBase
from pybricks.media.ev3dev import SoundFile, ImageFile

import math


# This program requires LEGO EV3 MicroPython v2.0 or higher.
# Click "Open user guide" on the EV3 extension tab for more information.
# Initialize the EV3 Brick.


class RobotControlSense():

    def __init__(self):
        self.ev3 = EV3Brick()
        
        # Initialize motors
        self.motorR = Motor(Port.A, positive_direction=Direction.CLOCKWISE, gears=None)
        self.motorL = Motor(Port.B, positive_direction=Direction.CLOCKWISE, gears=None)
        self.motorPulley = Motor(Port.C, positive_direction=Direction.CLOCKWISE, gears=None)
        self.motorGear = Motor(Port.D, positive_direction=Direction.COUNTERCLOCKWISE, gears=None)
        

        
        
        # Initialize sensors
        self.line_sensor = LightSensor(Port.S1)
        self.box_color_sensor = LightSensor(Port.S2)
        self.obstacle_sensor = UltrasonicSensor(Port.S3)

        # Constants
        self.BLACK = 9
        self.WHITE = 85
        self.pick_up_area_color = Color.YELLOW

        self.DRIVE_SPEED = 360  # deg/s
        self.TURN_SPEED = 200 # deg/s
        self.PULLEY_SPEED = 360 # deg/s
        self.GEAR_SPEED = 180 # deg/s

        self.GEAR_PITCH_DIAMETER = 180
        self.PULLEY_DIAMTER = 10
        


        self.WHEEL_DIAMETER = 50
        self.AXLE_TRACK = 320

        
        # Set the gain of the proportional line controller. This means that for every
        # percentage point of light deviating from the threshold, we set the turn
        # rate of the drivebase to 1.2 degrees per second.
        self.PROPORTIONAL_GAIN = 1.2
        
        # State variables
        self.picked_up = False
        self.color_to_drop = None
        self.threshold = (self.BLACK + self.WHITE) / 2


        # Initialize drive base
        self.robot = DriveBase(self.motorR, self.motorL, wheel_diameter=self.WHEEL_DIAMETER, axle_track=self.AXLE_TRACK)

    
    def testUltraSonicSenor(self):
        print("Sonic value: ",self.obstacle_sensor.distance())
    
    def lightSenor(self):
        print("Light sense value: ",self.box_color_sensor.reflection())


    def testDrive(self):
        self.robot.drive(100,0)
    
    def turn90Left(self):
        self.robot.turn(90)
    
    def turn90Right(self):
        self.robot.turn(-90)

    def correctionPath(self, by):
        self.robot.turn(by)
    
    def main(self):
        self.testUltraSonicSenor()
        self.lightSenor()
        while True:
            if self.obstacle_sensor.distance() > 50:
                self.testDrive()
                self.testUltraSonicSenor()
                self.lightSenor()
            else:
                self.robot.stop()

           
            
            


if __name__ == "__main__":
    robot = RobotControlSense()
    robot.main()
    
    

    