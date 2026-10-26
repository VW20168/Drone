
class PID:
    def __init__(self, p, i, d, _max):
        self.pid_p_gain = p
        self.pid_i_gain = i
        self.pid_d_gain = d
        self.pid_max = _max
        self.pid_min = 0 - self.pid_max
        self.reset()

    def reset(self):
        #floats
        self.pid_i_sum = 0.0
        self.pid_last_d_error = 0.0

    def compute(self, pid_setpoint, pid_input):
        pid_error = pid_input - pid_setpoint
        self.pid_i_sum += self.pid_i_gain * pid_error
        if self.pid_i_sum > self.pid_max:
            self.pid_i_sum = self.pid_max
        elif self.pid_i_sum < self.pid_min:
            self.pid_i_sum = self.pid_min

        pid_output = self.pid_p_gain * pid_error + self.pid_i_sum + self.pid_d_gain * (pid_error - self.pid_last_d_error)
        if pid_output > self.pid_max:
            pid_output = self.pid_max
        elif pid_output < self.pid_min:
            pid_output = self.pid_min

        self.pid_last_d_error = pid_error

        return pid_output
