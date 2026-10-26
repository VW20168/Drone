
class AltPID:

    BUF_SIZE = 30

    def __init__(self, p, i, d, _max):
        self.pid_p_gain = p
        self.pid_i_gain = i
        self.pid_d_gain = d
        #if the drone is higher than the target altitude, 
        #the pid output will be a positive value,
        #then this maximum value will apply
        self.pid_max = _max * 2     
        self.pid_min = 0 - _max

        self.reset()

    def reset(self):
        #floats
        self.pid_i_sum = 0.0
        self.pid_last_d_error = 0.0

        self.parachute_throttle = 0
        self.parachute_rotating_mem_location = 0
        self.parachute_altitude_previous = -100
        self.parachute_buffer = [0] * (self.BUF_SIZE+5)


    def compute(self, pid_setpoint, pid_input):
        if self.parachute_altitude_previous < 0:
            self.parachute_altitude_previous = pid_input
        self.parachute_throttle -= self.parachute_buffer[self.parachute_rotating_mem_location]
        self.parachute_buffer[self.parachute_rotating_mem_location] = pid_input - self.parachute_altitude_previous
        self.parachute_throttle += self.parachute_buffer[self.parachute_rotating_mem_location]
        self.parachute_altitude_previous = pid_input
        self.parachute_rotating_mem_location += 1
        if self.parachute_rotating_mem_location >= self.BUF_SIZE:
            self.parachute_rotating_mem_location = 0

        pid_error = pid_input - pid_setpoint
        self.pid_i_sum += self.pid_i_gain * pid_error
        if self.pid_i_sum > self.pid_max:
            self.pid_i_sum = self.pid_max
        elif self.pid_i_sum < self.pid_min:
            self.pid_i_sum = self.pid_min

        #To get better results the P-gain is increased when the error between the setpoint and the actual altitude value increases.
        #The variable pid_error_gain_altitude will be used to adjust the P-gain of the PID-controller.
        pid_error_gain = 0
        if pid_error > 1 or pid_error < -1:
            pid_error_gain = (abs(pid_error) - 1) * self.pid_p_gain
        pid_output = (self.pid_p_gain + pid_error_gain) * pid_error + self.pid_i_sum + self.pid_d_gain * self.parachute_throttle
        if pid_output > self.pid_max:
            pid_output = self.pid_max
        elif pid_output < self.pid_min:
            pid_output = self.pid_min

        self.pid_last_d_error = pid_error

        return pid_output
