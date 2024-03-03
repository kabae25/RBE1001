# ---------------------------------------------------------------------------- #
#                                                                              #
# 	Module:       main.py                                                      #
# 	Author:       EK, DS, JG, SH                                               #
# 	Created:      2/7/2024, 3:50:59 PM                                         #
# 	Description:  Mega file for high level robot control. Python sucks         #
#                                                                              #
# ---------------------------------------------------------------------------- #

# This is the mega file for a full high level control robot. Python sucks
# 2/7/24 EK

# ---- Imports:
from vex import *
import math
import time as time

brain=Brain()

# ---- Template Classes
class PIDController():
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

class Subsystem():
    def __init__(self) -> None:
        self.current_state = 1
    
    def setState(self, m_NEW_STATE):
        self.current_state = m_NEW_STATE
        return self.current_state
    
    def getState(self):
        return self.current_state
    
# ---- Constants
class constants ():
    # ---- Motors ----
    PORTS_MOTOR_1 = Ports.PORT1
    PORTS_MOTOR_2 = Ports.PORT10
    PORTS_MOTOR_3 = Ports.PORT11
    PORTS_MOTOR_4 = Ports.PORT20

    PORTS_MOTOR_ARM = Ports.PORT11

    PORTS_MOTOR_COLLECTOR = Ports.PORT9
    PORTS_MOTOR_COLLECTOR1 = Ports.PORT8

    PORTS_MOTOR_ELEVATOR = Ports.PORT14
    PORTS_MOTOR_DUMP = Ports.PORT12
    # ---- Sensors ----
    PORTS_IMU = Ports.PORT5
    PORTS_BUMPER = Ports.PORT19

    # ---- Subsystem ----
    kDRIVE_RPM = 200
    kWHEEL_DIAMETER = 2*math.pi


# ---- Subsystems
motor_1 = Motor (constants.PORTS_MOTOR_1)
motor_2 = Motor (constants.PORTS_MOTOR_2)
motor_3 = Motor (constants.PORTS_MOTOR_3)
motor_4 = Motor (constants.PORTS_MOTOR_4)

MOTOR_COLLECTOR = Motor (constants.PORTS_MOTOR_COLLECTOR, GearSetting.RATIO_18_1)
MOTOR_COLLECTOR2 = Motor (constants.PORTS_MOTOR_COLLECTOR1)

MOTOR_ELEVATOR = Motor (constants.PORTS_MOTOR_ELEVATOR)

MOTOR_DUMP = Motor (constants.PORTS_MOTOR_DUMP)
imu = Inertial (constants.PORTS_IMU)

driver_controller = Controller ()

# ---- Drivebase ----
class HolonomicDrive():
    def __init__(self, FL, FR, BL, BR):
        self.m_FL_MOTOR = FL
        self.m_FR_MOTOR = FR
        self.m_BL_MOTOR = BL
        self.m_BR_MOTOR = BR

        self.x_pos = 0
        self.y_pos = 0
        self.x_start_pos = 7
        self.y_start_pos = 7

    def __resetEncoders(self):
        self.m_FL_MOTOR.set_position(0)
        self.m_FR_MOTOR.set_position(0)
        self.m_BL_MOTOR.set_position(0)
        self.m_BR_MOTOR.set_position(0)


    def DriveFOC(self, i, j, r): 
        i = i * constants.kDRIVE_RPM
        j = j * constants.kDRIVE_RPM
        r = r * constants.kDRIVE_RPM
        # Find the vector of the robot from the imu heading

        theta = imu.heading()*math.pi/180.0 # get imu heaing in radians 

        ySpeed  = j*math.cos(theta) + i*math.sin(theta)
        xSpeed = -j*math.sin(theta) + i*math.cos(theta)

        # Find wheels speeds in x drive (r is a modifier - if no r modifier, the robot will slow itself)
        # 2/7/24 experiment - if r is full (i.e 1, or with a controller 127, will the resultant behavior of the robot be in-place spin and no translation?)
        fl_speed = ySpeed + xSpeed + r
        fr_speed = ySpeed - xSpeed - r
        bl_speed = ySpeed - xSpeed + r
        br_speed = ySpeed + xSpeed - r

        # normalize the wheel speeds based on the formula (MAX RPM * value / max value)
        # 2/7/24 experiment - what happens if this stage is bypassed?
        
        #highest_value = max(fl_speed, fr_speed, bl_speed, br_speed)
        maximum = constants.kDRIVE_RPM
        
        if abs(fl_speed) > maximum: maximum = abs(fl_speed)
        if abs(fr_speed) > maximum: maximum = abs(fr_speed)
        if abs(bl_speed) > maximum: maximum = abs(bl_speed)
        if abs(br_speed) > maximum: maximum = abs(br_speed)

        if maximum is 0:
            print('No drive input')
        elif maximum > constants.kDRIVE_RPM:
            fl_speed = constants.kDRIVE_RPM * fl_speed / maximum
            fr_speed = constants.kDRIVE_RPM * fr_speed / maximum
            bl_speed = constants.kDRIVE_RPM * bl_speed / maximum
            br_speed = constants.kDRIVE_RPM * br_speed / maximum

        # spin the motors according to the normalized speeds
        self.m_FL_MOTOR.spin(DirectionType.FORWARD, fl_speed)
        self.m_FR_MOTOR.spin(DirectionType.REVERSE, fr_speed)
        self.m_BL_MOTOR.spin(DirectionType.FORWARD, bl_speed)
        self.m_BR_MOTOR.spin(DirectionType.REVERSE, br_speed)

        return fl_speed, fr_speed, bl_speed, br_speed # return the values from the drive FC control function

    def Drive(self, i, j, r): # Expects inputs [-1, 1]
        self.m_FL_MOTOR.spin(DirectionType.FORWARD, j*constants.kDRIVE_RPM + i*constants.kDRIVE_RPM + r*constants.kDRIVE_RPM)
        self.m_FR_MOTOR.spin(DirectionType.REVERSE, j*constants.kDRIVE_RPM - i*constants.kDRIVE_RPM - r*constants.kDRIVE_RPM)
        self.m_BL_MOTOR.spin(DirectionType.FORWARD, j*constants.kDRIVE_RPM - i*constants.kDRIVE_RPM + r*constants.kDRIVE_RPM)
        self.m_BR_MOTOR.spin(DirectionType.REVERSE, j*constants.kDRIVE_RPM + i*constants.kDRIVE_RPM - r*constants.kDRIVE_RPM)

    def Lock(self):
        self.m_FL_MOTOR.stop()
        self.m_FR_MOTOR.stop()
        self.m_BL_MOTOR.stop()
        self.m_BR_MOTOR.stop()

    def updatepose(self):
        # this function derives the linear distance travelled by the wheels and ouputs (x, y) position in relation to the field.
        # the goal of this is to make a goto coordinate function that simplifies field navigation
        
        FL_rot = self.m_FL_MOTOR.position()
        FR_rot = self.m_FR_MOTOR.position()
        BL_rot = self.m_BL_MOTOR.position()
        BR_rot = self.m_BR_MOTOR.position()

        FL_rot_abs = abs(FL_rot)
        FR_rot_abs = abs(FR_rot)
        BL_rot_abs = abs(BL_rot)
        BR_rot_abs = abs(BR_rot)

        FL_unit = FL_rot/FL_rot_abs
        FR_unit = FR_rot/FR_rot_abs
        BL_unit = BL_rot/BL_rot_abs
        BR_unit = BR_rot/BR_rot_abs



        '''
        fl_speed = ySpeed + xSpeed
        fr_speed = ySpeed - xSpeed
        bl_speed = ySpeed - xSpeed
        br_speed = ySpeed + xSpeed
        '''
        # When moving forward: Positive, Negative, Positive, Negative
        # When moving left: Negative, Negative, Positive, Positive
        # When moving right: Positive, Positive, Negative, Negative
        # When moving backwards: Negative, Positive, Negative, Positive
        # Moving Positive: Negative, Negative, Negative, Negative

        # Steps:
        # 1: Get encoder rotations
        # 2: Convert to robot centric x and y positions
        # 3: Convert encoder rotations to radians, then to linear distance
        
        # A standard drivetrain wheel rotates 200 rpm, the x drivetrain wheel rotates the same, but covers 200rpm * sqrt(2) the speed



        return FL_rot, FR_rot, BL_rot, BR_rot
    
    def getpose(self):
        return self.x_pos, self.y_pos

holonomicdrive = HolonomicDrive(motor_1, motor_2, motor_3, motor_4) # initialize drivetrain

#holonomicdrive.__resetEncoders()

while True:
    print("E", holonomicdrive.updatepose())