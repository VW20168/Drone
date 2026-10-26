import os
import logging

class PIDAssistant:

    TRAINING_STAGE_P = 0
    TRAINING_STAGE_D = 2

    INITIAL_P = 0
    INITIAL_D = 0
    FINAL_P   = 0
    FINAL_D   = 0

    STEP_P = 0
    STEP_D = 0

    PID_REPEATS = 5

    def __init__(self):
        self.reset()

    def reset(self):
        self.stage = 0
        self.data = [] #actual reading, for example, the altitude. its member is also array
        self.pids = [] #pid output
        self.p_gain = self.i_gain = self.d_gain = 0
        self.repeats = 0
        self.reset_round()
        self.reset_more()

    def reset_round(self):
        self.samples = 0
        self.data.append([])
        self.pids.append([])

    def will_start(self):
       logging.info("***Training*** PID:" + str(self.p_gain) + "," + str(self.i_gain)+","+str(self.d_gain))

    def shall_land(self):
        return self.samples >= 250 * 15 #15 seconds

    #drone landed, motors still running
    def finish_round(self):
        self.save(False)

        self.repeats += 1
        self.reset_round()
        logging.info("***Training@" + str(self.repeats) +"/"+str(self.PID_REPEATS) + "*** Tried PID:" + str(self.p_gain) + "," + str(self.i_gain)+","+str(self.d_gain))

    def shall_stop(self):
        stop_now = False
        if self.repeats >= self.PID_REPEATS:
            '''
            self.repeats = 0
            if self.stage == self.TRAINING_STAGE_P:
                self.p_gain += self.STEP_P
                stop_now = self.p_gain > self.FINAL_P
            elif self.stage == self.self.TRAINING_STAGE_D:
                self.d_gain += self.STEP_D
                c = self.d_gain > self.FINAL_D
            '''
            stop_now = True
        if stop_now:
            self.save(True)
        return stop_now

    def save(self, done):
        name = "training.csv"
        if self.stage == self.TRAINING_STAGE_P:
            name = "p"+str(self.p_gain)+".csv"
        file = open(name, "w")
        file.write("PIDv2,"+str(self.p_gain) + "," + str(self.i_gain) + "," + str(self.d_gain)+"(setpoint_diff,pid_output)\n")

        try:
            total = 0; rounds = len(self.data)
            for col in range(rounds):
                times = len(self.data[col])
                if times > total:
                    total = times
            for row in range(total):
                for col in range(rounds):
                    if row < len(self.data[col]):
                        file.write(str(self.data[col][row]))
                    file.write(",")
                file.write("      ")
                for col in range(rounds):
                    if row < len(self.pids[col]):
                        file.write(str(self.pids[col][row]))
                    file.write(",")
                file.write("\n")
        except Exception as e:
            pass
        file.close()
        if done:
            self.data.clear()
        logging.info("***Training*** Training data saved: " + name)

