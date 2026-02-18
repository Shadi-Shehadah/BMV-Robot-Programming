#!/usr/bin/env pybricks-micropython
from pybricks.hubs import EV3Brick
from pybricks.ev3devices import (Motor, TouchSensor, ColorSensor,
                                 InfraredSensor, UltrasonicSensor, GyroSenso, LightSensor)
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
        # self.motorPulley = Motor(Port.C, positive_direction=Direction.CLOCKWISE, gears=None)
        # self.motorGear = Motor(Port.D, positive_direction=Direction.CLOCKWISE, gears=None)
        

        
        
        # Initialize sensors
        self.line_sensor = LightSensor(Port.S1)
        self.box_color_sensor = LightSensor(Port.S2)
        self.obstacle_sensor = UltrasonicSensor(Port.S3)
    
    def followLineUntilStopped(self):
        

    def main(self):
        while True:
            followLineUntilStopped()


if __name__ == "__main__":
    main()
    
        