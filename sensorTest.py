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

        # Box colors: 
        self.COLOR1 = 0     # green   (these are placeholders currently)
        self.COLOR2 = 100    # silver
        self.COLOR3 = 50   # red

        self.DRIVE_SPEED = 360  # deg/s
        self.TURN_SPEED = 200 # deg/s
        self.PULLEY_SPEED = 360 # deg/s
        self.GEAR_SPEED = 180 # deg/s


        


        self.WHEEL_DIAMETER = 50
        self.AXLE_TRACK = 320

        
        # Set the gain of the proportional line controller. This means that for every
        # percentage point of light deviating from the threshold, we set the turn
        # rate of the drivebase to 1.2 degrees per second.
        self.PROPORTIONAL_GAIN = 1.2
        
        # State variables
        self.picked_up = False
        self.color_to_drop = ""
        self.threshold = (self.BLACK + self.WHITE) / 2
        self.running_process = False


        # Initialize drive base
        self.robot = DriveBase(self.motorR, self.motorL, wheel_diameter=self.WHEEL_DIAMETER, axle_track=self.AXLE_TRACK)
        # Preventing it from stalling too early
        self.robot.distance_control.stall_tolerances(10, 50000)
        self.robot.distance_control.stall_tolerances(10, 50000)


    
    def testUltraSonicSenor(self):
        print("Sonic value: ",self.obstacle_sensor.distance())
    
    def lightSenor(self):
        print("Light sense value: ", self.box_color_sensor.reflection())
    
    def testDrive(self):
        if not self.running_process:
            self.robot.drive(100,0)

    def linefollowDrive(self):
        if not self.running_process:
            deviation = self.line_sensor.reflection() - self.threshold
            turn_rate = self.PROPORTIONAL_GAIN * deviation
            self.robot.drive(100, turn_rate)
        
    
    def turn90Left(self):
        self.robot.turn(90)
    
    def turn90Right(self):
        self.robot.turn(-90)

    def turnBy(self, by):
        self.robot.turn(by)
    
    def pulleyDown(self):
        
        # self.motorPulley.run_until_stalled(speed = self.PULLEY_SPEED,then = Stop.HOLD, duty_limit = 40)

        self.motorPulley.run_angle(speed=(self.PULLEY_SPEED * -5), rotation_angle=400, then=Stop.HOLD)
    
    def pulleyUp(self):
        
        #self.motorPulley.run_until_stalled(speed = self.PULLEY_SPEED,then = Stop.HOLD, duty_limit = 40)
        self.motorGear.hold()
        self.motorPulley.run_angle(speed=self.PULLEY_SPEED, rotation_angle=420, then=Stop.HOLD)

    def boxIn(self):
        for i in range(11):
            self.motorGear.run_angle(speed=self.GEAR_SPEED, rotation_angle=20, then=Stop.HOLD)
            self.motorPulley.run_angle(speed=self.GEAR_SPEED, rotation_angle=24, then=Stop.HOLD)
            i+1
        return
    
    def boxOut(self):
        for i in range(11):
            self.motorGear.run_angle(speed = -self.GEAR_SPEED,rotation_angle = 20,then=Stop.HOLD)
            self.motorPulley.run_angle(speed=-self.GEAR_SPEED, rotation_angle=18, then=Stop.HOLD)
            i+1
        return

    # def boxUpAndIn(self):
    #     self.pulleyUp()
    #     self.boxIn()
    
    # def boxOutAndDown(self):

        
    
    def pickUp(self):
        # move the pulleys and stuff 
        self.running_process = True
        self.pulleyUp()
        self.robot.straight(100)
        self.pulleyDown()
        wait(3000)
        self.pulleyUp()
        self.boxIn()

        self.picked_up = True
        self.color_to_drop = self.box_color_sensor.reflection()
        self.turnBy(170)
        self.running_process = False
    

    def pickUp_Experimental(self):
        self.running_process = True

        self.pulleyUp()
        self.motorPulley.hold()

        self.robot.straight(100)

        if self.box_color_sensor.reflection() <= self.COLOR1 + 10  and self.box_color_sensor.reflection() >= self.COLOR1 - 10:
            self.color_to_drop = 'green'
            self.ev3.speaker.say("picking up green box")
        elif self.box_color_sensor.reflection() <= self.COLOR2 + 10  and self.box_color_sensor.reflection() >= self.COLOR2 - 10:
            self.color_to_drop = 'grey'
            self.ev3.speaker.say("picking up grey box")
        elif self.box_color_sensor.reflection() <= self.COLOR3 + 10  and self.box_color_sensor.reflection() >= self.COLOR3 -10:
            self.color_to_drop = 'red'
            self.ev3.speaker.say("picking up red box")
        else:
            self.color_to_drop = 'unknown'
            self.ev3.speaker.say("picking up unknown box")

        self.pulleyDown()
        self.motorGear.hold()
        self.pulleyUp()
        self.boxIn()

        self.picked_up = True
        self.turnBy(170)
        self.running_process = False

    def putDown(self):
        self.running_process = True
       
        self.boxOut()
        if self.color_to_drop == 0:
            self.turn90Left()
            # pulley actions go here
            self.pulleyDown()
            self.pullyUp()
            self.pulleyDown()

            self.turn90Right()
        else:
            self.turn90Right()
            # pulley drop action
            self.pulleyDown()
            self.turn90Left()
        self.turnBy(180)
        self.running_process = False 
        self.picked_up = False


    def putDown_Experimental(self):
        self.running_process = True
        self.boxOut()
        if self.color_to_drop == 'green':
            self.ev3.speaker.say("placing down green box on the left")

            self.turn90Left()
            self.robot.straight(100)
            self.pulleyDown()
            self.robot.straight(-100)
            self.turn90Left()


        elif self.color_to_drop == 'grey':
            self.ev3.speaker.say("placing down grey box on the right")



        else:
            self.turn90Right()
            self.robot.straight(100)
            self.pulleyDown()
            self.robot.straight(-100)
            self.turn90Right()


        self.running_process = False 
        self.picked_up = False


    
    # pulley test

    # def main(self):
    #     while True:
    #         button = self.ev3.buttons.pressed()
    #         if button:
    #             if button[0] == Button.RIGHT:
    #                 self.boxIn()
    #             elif button[0] == Button.DOWN:
    #                 self.pulleyDown()
    #             elif button[0] == Button.UP:
    #                 self.pullyUp()
    #             elif button[0] == Button.LEFT:
    #                 self.boxOut()

                    
                

    
    # def main(self):
    #     self.testUltraSonicSenor()
    #     self.lightSenor()
    #     self.testDrive()
    #     emergencyStop = False
       
    #     while True and not emergencyStop:
    #         button = self.ev3.buttons.pressed()
    #         if button:
    #             if button[0] == Button.UP:
    #                 emergencyStop = True
    #             elif button[0] == Button.DOWN:
    #                 emergencyStop = False
            
    #         if self.obstacle_sensor.distance() < 150:
    #             self.robot.stop()
    #             if self.picked_up:
    #                 self.testUltraSonicSenor()
    #                 self.lightSenor()
    #                 self.putDown()
    #             else:
    #                 self.pickUp()
                
    #         else:
    #             self.testDrive()
    #             self.testUltraSonicSenor()
    #             self.lightSenor()




    def main(self):
        emergencyStop = False

        # setting the speaker.say options:
        self.ev3.speaker.set_speech_options(language='en-uk-north',voice = 'croak', pitch = 20)


        # Main Loop:
        while True and not emergencyStop:          # temporarily paused to test individual features first
            button = self.ev3.buttons.pressed()
            if button:
                if button[0] == Button.UP:
                    emergencyStop = True
                elif button[0] == Button.DOWN:
                    emergencyStop = False
            
            if self.obstacle_sensor.distance() < 150:
                self.robot.stop()
                if self.picked_up:
                    self.testUltraSonicSenor()
                    self.lightSenor()
                    self.putDown_Experimental()
                else:
                    self.pickUp_Experimental()
                
            else:
                self.linefollowDrive()
                self.testUltraSonicSenor()
                self.lightSenor()

           
            
            


if __name__ == "__main__":
    robot = RobotControlSense()
    robot.main()
    
    

    