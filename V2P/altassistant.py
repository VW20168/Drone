from assistant import PIDAssistant

class AltPIDAssistant(PIDAssistant):

    def __init__(self):
        super(AltPIDAssistant, self).__init__()

    def reset_more(self):
        self.INITIAL_P = 10
        self.FINAL_P   = 100
        self.STEP_P    = 5

        self.INITIAL_D = 100
        self.FINAL_D   = 300
        self.STEP_D    = 10

        self.stage  = self.TRAINING_STAGE_P
        self.p_gain = 67 #10, 40, 60 or 80
        self.i_gain = 0
        self.d_gain = 0
        '''
        self.stage  = self.TRAINING_STAGE_D
        self.p_gain = 50
        self.i_gain = 0
        self.d_gain = self.INITIAL_D
        '''
    def input(self, _target, _reading, _pid):
        reading = int((_reading - _target) * 100) #to cm
        self.data[self.repeats].append(reading)
        self.pids[self.repeats].append(_pid)
        self.samples += 1

