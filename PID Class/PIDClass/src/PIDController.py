# ---------------------------------------------------------------------------- #
#                                                                              #
# 	Module:       main.py                                                      #
# 	Author:       Evan                                                         #
# 	Created:      1/25/2024, 8:59:40 PM                                        #
# 	Description:  V5 project                                                   #
#                                                                              #
# ---------------------------------------------------------------------------- #

# Library imports
from vex import * 
import time
# Brain should be defined by default
brain=Brain()

imu = Inertial(Ports.PORT2) # Interial on port two

leftDriveMotor = Motor(Ports.PORT10, True) # left drive motor (arm is on the back)
rightDriveMotor = Motor(Ports.PORT1, False) # right drive motor (arm is on the back)

imu.calibrate()
while (imu.is_calibrating()):
    sleep(50)
imu.reset_heading()    

brain.screen.print("if you see this, it works")

class PID():
    def __init__(self, kp, ki, kd) -> None:
        self.kp = kp # Proportional gain
        self.ki = ki # Integral gain
        self.kd = kd # derivative gain
        self.integral_error = 0
        self.error_last = 0
        self.derivative_error = 0
        self.output = 0
        self.TIME_STEP = 0.005

    def calculate(self, setpoint, pos):
        self.error = setpoint - pos # set point - output
        self.integral_error += self.error * self.TIME_STEP # change in time
        self.derivative_error = (self.error - self.error_last) / self.TIME_STEP #change in time
        self.error_last = self.error
        return self.kp*self.error + self.ki*self.integral_error + self.kd*self.derivative_error

    def reset(self):
        self.integral_error = 0
        self.error_last = 0
        self.derivative_error = 0
        self.output = 0

pid = PID(0.9, 0, 0) 

class Robot():
    def getIMUHeading(self): # Get IMU heading (between 0 to 359.9...)
        return imu.heading()
    
    def getRobotRotation(self): # Get IMU Rotation (continuous, counting)
        return imu.rotation()
    
    def getHeading(self): # function to determine the heading of the robot between 180 and -180
        self.sample = self.getIMUHeading()
        if self.sample > 180:
            return self.sample - 360.0 # Value returns negative from 0 if the imu reads 180-359.9...
        else:
            return self.sample # value returns positive imu reading 0-180

    def controlHeading(self, targetHeading, forwardSpeed):
        leftDriveMotor.spin(DirectionType.FORWARD, forwardSpeed + pid.calculate(setpoint=targetHeading, pos=self.getHeading()))
        rightDriveMotor.spin(DirectionType.FORWARD, forwardSpeed - pid.calculate(setpoint=targetHeading, pos=self.getHeading()))
        
robot = Robot() 

while True:
    robot.controlHeading(0, 100) # Degrees first, m/s after
    #print(robot.getHeading())
    #print('E:', 0 - robot.getHeading())
    print('Setpoint:', robot.getHeading(), 'Error:', 0-robot.getHeading(), 'PID:', pid.calculate(setpoint=0, pos=robot.getHeading()))
    print('P:', pid.kp*pid.error,'I:',pid.ki*pid.integral_error, 'D:', pid.kd*pid.derivative_error)
    time.sleep(0.005)