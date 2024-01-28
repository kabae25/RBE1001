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
from time import time
# Brain should be defined by default
brain=Brain()

imu = Inertial(Ports.PORT2) # Interial on port two

leftDriveMotor = Motor(Ports.PORT10, True) # left drive motor (arm is on the back)
rightDriveMotor = Motor(Ports.PORT1, False) # right drive motor (arm is on the back)

brain.screen.print("if you see this, it works")

imu.calibrate()
while (imu.is_calibrating()):
    sleep(50)
imu.reset_heading()    

class PID():
    def __init__(self) -> None:
        self.kp = 0.9 # Proportional gain
        self.ki = 0.5 # Integral gain
        self.kd = 0 # derivative gain
        self.integral_error = 0
        self.error_last = 0
        self.derivative_error = 0
        self.output = 0
        self.TIME_STEP = 0.005

    def calculate(self, setpoint, pos):
        error = setpoint - pos # set point - output
        self.integral_error += error * self.TIME_STEP # change in time
        self.derivative_error = (error - self.error_last) / self.TIME_STEP #change in time
        self.error_last = error
        return self.kp*error + self.ki*self.integral_error + self.kd*self.derivative_error

pid = PID()

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
    print('PID:', pid.calculate(setpoint=0, pos=robot.getHeading()))
    time.sleep(pid.TIME_STEP)



# State Machine Template 1/24/2024

#class State(): # Declare States Here, add a number to associate with each state
#    START = 0
#    FIRST = 1
#    END = 2
# 
#class Finite_State_Machine(): # Main State Machine
#    def __init__(self):
#        self.current_state = State.START
#        self.end_states = [State.END]
#        self.task()
#
#    def task(self): # 
#        while self.current_state not in self.end_states:
#            self.on_event()
#        return self.__END_handler()
#    
#    def on_event(self): # Events that determine which state method is executed
#        if self.current_state == State.START: # If the current state is the START state, run the start handler
#            return self.__START_handler()
#        elif self.current_state == State.FIRST:
#            return self.__FIRST_handler()
#        
#    def __START_handler(self):
#        #self.current_state = State.END # Set the current state to the END State
#        pass
#
#    def __FIRST_handler(self):
#        pass
#
#    def __END_handler(self):
#        pass
#
#finite_state_machine = Finite_State_Machine() # Starts the state machine
#        
#