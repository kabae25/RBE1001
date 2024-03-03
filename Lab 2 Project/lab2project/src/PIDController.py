# PIDController Class 1/28/24 - EK
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
