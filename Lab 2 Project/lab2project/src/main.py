# ---------------------------------------------------------------------------- #
#                                                                              #
# 	Module:       main.py                                                      #
# 	Author:       Evan                                                         #
# 	Created:      1/28/2024, 6:54:04 PM                                        #
# 	Description:  V5 project                                                   #
#                                                                              #
# ---------------------------------------------------------------------------- #

# Library imports
from vex import *
from PIDController import PIDController
import time
# Brain should be defined by default

# --------------------------------------------------------------------------------------
brain=Brain()

imu_pid = PIDController(0, 0, 0)
line_pid = PIDController(0, 0, 0)

imu = Inertial(Ports.PORT2) # Interial on port two

leftDriveMotor = Motor(Ports.PORT10, True) # left drive motor (arm is on the back)
rightDriveMotor = Motor(Ports.PORT1, False) # right drive motor (arm is on the back)

leftLineFollower = Line(brain.three_wire_port.f)
rightLineFollower = Line(brain.three_wire_port.g)

button = Bumper(brain.three_wire_port.b)
# --------------------------------------------------------------------------------------

class State(): # Declare States Here, add a number to associate with each state
    START = 0
    IDLE = 1
    LINE_CONTROL = 2
    TURN = 3
    IMU_CONTROL = 4
    END = 5

class Finite_State_Machine(): # Main State Machine
    def __init__(self):
        self.current_state = State.START
        self.end_states = [State.END]
        self.active_states = [State.LINE_CONTROL, State.TURN, State.IMU_CONTROL]
        self.task()

    def task(self): # 
        while self.current_state not in self.end_states:
            if self.current_state in self.active_states and button is True: self.__END_handler() # If the button is pressed while in any active (moving) states, go to the end state
            else: self.on_event() # otherwise run the main loop
        return self.__END_handler()
    
    def on_event(self): # Events that determine which state method is executed
        if self.current_state == State.START: # If the current state is the START state, run the start handler
            return self.__START_handler()
        elif self.current_state == State.IDLE:
            return self.__IDLE_handler()
        elif self.current_state == State.IMU_CONTROL:
            return self.__IMU_CONTROL_handler()
        elif self.current_state == State.LINE_CONTROL:
            return self.__LINE_CONTROL_handler()
        elif self.current_state == State.TURN:
            return self.__TURN_handler()
        elif self.current_state == State.END:
            return self.__END_handler()
        
    def __START_handler(self):
        imu.calibrate() # Call IMU Calibration

        while (imu.is_calibrating()): # Prevent Anything from happening while the IMU is Calibrating
            sleep(50)
        imu.reset_heading() 

        brain.screen.print("STATE: ", self.current_state)

        self.current_state = State.IDLE

    def __IDLE_handler(self):
        pass

    def __END_handler(self):
        pass
    
    def __IMU_CONTROL_handler(self):
        pass

    def __LINE_CONTROL_handler(self):
        line_pid.calculate(leftLineFollower.reflectivity(), rightLineFollower.reflectivity())
        leftDriveMotor.spin(DirectionType.FORWARD, 100 + line_pid.calculate(leftLineFollower.reflectivity(), rightLineFollower.reflectivity()))
        rightDriveMotor.spin(DirectionType.FORWARD, 100 + line_pid.calculate(leftLineFollower.reflectivity(), rightLineFollower.reflectivity()))

    def __TURN_handler(self):
        pass
finite_state_machine = Finite_State_Machine() # Starts the state machine
        
