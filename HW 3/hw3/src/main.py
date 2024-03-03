# State Machine Template 1/24/2024
# ---------------------------------------------------------------------------- #
#                                                                              #
# 	Module:       main.py                                                      #
# 	Author:       Evan Kaba                                                    #
# 	Created:      9/11/2023, 8:28:37 AM                                        #
# 	Description:  V5 project                                                   #
#                                                                              #
# ---------------------------------------------------------------------------- #

from vex import *

brain = Brain ()

ultrasonic = Sonar (brain.three_wire_port.a)

bumper = Bumper (brain.three_wire_port.e)

PINK_FRUIT = Signature (1, 5243, 6221, 5732, 1435, 1865, 1650, 3, 0)
GREEN_FRUIT = Signature (2, -6647, -6143, -6394, -3495, -2835, -3164, 3, 0)
ORANGE_FRUIT = Signature (3, 5331, 6647, 5989, -2031, -1681, -1856, 3, 0)
YELLOW_FRUIT = Signature (4, 1889, 2285, 2087, -3339, -2997, -3168, 3, 0)

Vision3 = Vision (Ports.PORT3, 72, PINK_FRUIT, GREEN_FRUIT, YELLOW_FRUIT, ORANGE_FRUIT)

leftMotor = Motor (Ports.PORT10, GearSetting.RATIO_18_1, False)
rightMotor = Motor (Ports.PORT1, GearSetting.RATIO_18_1, True)

armMotor = Motor (Ports.PORT7, False)

objects = False

prev_button = 0


def handle_objects_c(objects):
    cx = -1*Vision3.largest_object().centerX
    cy = -1*Vision3.largest_object().centerY
    return cx, cy

def handle_objects_hw():
    h = Vision3.largest_object().height
    w = Vision3.largest_object().width
    return h, w

def handle_objects_a():
    a = handle_objects_hw()[0] * handle_objects_hw()[1]
    return a  

def calcDistanceFromPixels():
    pass

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

turn_pid = PIDController(2, 0, 2) # .1
approach_pid = PIDController(15, 0, .3)

class State(): # Declare States Here, add a number to associate with each state
    START = 0
    IDLE = 1
    SEARCHING = 2
    APPROACH = 3
    TOUCH = 4
    END = 5

class Finite_State_Machine(): # Main State Machine
    def __init__(self):
        self.current_state = State.START
        self.end_states = [State.END]
        self.task()

    def task(self): # 
        while self.current_state not in self.end_states:         
            self.on_event()
        return self.__END_handler()
    
    def on_event(self): # Events that determine which state method is executed
        if self.current_state == State.START: # If the current state is the START state, run the start handler
            return self.__START_handler()
        elif self.current_state == State.IDLE:
            return self.__IDLE_handler()
        elif self.current_state == State.SEARCHING:
            return self.__SEARCHING_handler()
        elif self.current_state == State.APPROACH:
            return self.__APPROACHING_handler()
        elif self.current_state == State.TOUCH:
            return self.__TOUCH_handler()
        
    def __START_handler(self):
        #print('State: Start')

        while ultrasonic.distance(DistanceUnits.CM) < 1:
            sleep(20)
        
        armMotor.spin_to_position(0)

        self.current_state = State.IDLE # Set the current state to the END State
        
    def __IDLE_handler(self):
        #print('State: Idle')
        global prev_button
        if bumper.value() == 0 and prev_button == 1:
            prev_button = 0
        if bumper.value() == 1 and prev_button == 0:
            print('switching states')
            self.current_state = State.SEARCHING
            prev_button = 1
        

    def __SEARCHING_handler(self):
        objects = Vision3.take_snapshot(YELLOW_FRUIT)
        #print('State: Searching')
        if objects:
            #print(Vision3.largest_object().width, Vision3.largest_object().height)
            #print(ultrasonic.distance(DistanceUnits.CM))

            if Vision3.largest_object().width >= 25:
                err_heading = turn_pid.calculate(153, abs(316-Vision3.largest_object().centerX))

                leftMotor.spin(DirectionType.FORWARD, err_heading)
                rightMotor.spin(DirectionType.FORWARD, -err_heading)
                
                if abs(err_heading) <= 10:
                    #print('h', err_heading)
                    self.current_state = State.APPROACH
            else:
                leftMotor.spin(DirectionType.FORWARD, 30)
                rightMotor.spin(DirectionType.FORWARD, -30)
        else:
            leftMotor.spin(DirectionType.FORWARD, 30)
            rightMotor.spin(DirectionType.FORWARD, -30)
        

    def __APPROACHING_handler(self):
        #print('State: Approach')
        objects = Vision3.take_snapshot(YELLOW_FRUIT)
        if objects:
            err_distance = -150
            err_heading = turn_pid.calculate(153, abs(316-Vision3.largest_object().centerX))

            if ultrasonic.distance(DistanceUnits.CM) <= 50:
                err_distance = approach_pid.calculate(5, ultrasonic.distance(DistanceUnits.CM))
                #print(ultrasonic.distance(DistanceUnits.CM))
                
            leftMotor.spin(DirectionType.FORWARD, -err_distance + err_heading)
            rightMotor.spin(DirectionType.FORWARD, -err_distance - err_heading)
            if ultrasonic.distance(DistanceUnits.CM) <= 5:
                leftMotor.stop()
                rightMotor.stop()
                brain.screen.clear_screen()
                brain.screen.print(ultrasonic.distance(DistanceUnits.CM))
                self.current_state = State.TOUCH
        else: self.current_state = State.SEARCHING

    def __TOUCH_handler(self):
        #print('State: Touching')
        armMotor.spin_to_position(-205.0*5)
        self.current_state = State.START


    def __END_handler(self):
        pass

finite_state_machine = Finite_State_Machine() # Starts the state machine
