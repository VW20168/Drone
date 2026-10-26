import datetime
import logging
from constants import Constants

class DroneConfiguration:

    DELIMITER = "\n"
    SEPARATOR = ","

    def __init__(self, m):
        self.mode = m
        self.reset()

    def reset(self):
        self.compensations =[0] * 4 #motor compensations
        self.silent_esc = False #True if the ESC needs a signal to keep it silent
        self.takeoff_value  = 1440 #1440
        self.idle_value     = 1200
        self.nudge_value    = 10
        self.movement_angle = 2
        self.turning_yaw    = 10
        self.ideal_battery_factor = 40 / (Constants.MAX_BATTERY_VOLTAGE - Constants.MIN_BATTERY_VOLTAGE) #when the battery voltage reaches the MIN_BATTERY_VOLTAGE, compensate 40 to the pwm

        #1.48&0.04&82
        self.p_gain = 1.48
        self.i_gain = 0.04
        self.d_gain = 100 #82
        #5&0.01&0
        self.yaw_p_gain = 5
        self.yaw_i_gain = 0.01
        self.yaw_d_gain = 0
        #60&0.02&226
        self.alt_p_gain = 66 #60
        self.alt_i_gain = 0.02
        self.alt_d_gain = 230

        #hardware configuration
        self.imu = "MPU9250" #9250 or 6050
        self.imu_filter_on = False
        self.altimeter = "VL53L1X" #"MS5611", "VL53L1X" or "BMP280"
        self.compass = ""
        self.load()

        if self.is_mode_training():
            self.altimeter = "VL53L1X"

        max_alt = 0 #in meters
        if self.is_mode_indoor() or self.is_mode_training():
            max_alt = 0.3 #meters
        elif self.is_mode_outdoor():
            if self.altimeter == "MS5611" or self.altimeter == "BMP280":
                max_alt = 1.5
            else: #VL53L1X in short range up to 1.3m
                max_alt = 0.6
        if max_alt > 0:
            Constants.MAX_ALTITUDE = max_alt

        print("!!!Max altitude", Constants.MAX_ALTITUDE)

    def is_mode_indoor(self):
        return self.mode == Constants.MODE_INDOOR

    def is_mode_outdoor(self):
        return self.mode == Constants.MODE_OUTDOOR

    def is_mode_training(self):
        return self.mode == Constants.MODE_TRAINING

    #parse the configuration submitted from the client
    def parse(self, content):
        try:
            lines = content.split(self.DELIMITER)
            for line in lines:
                components = line.split(":")
                if len(components) == 2:
                    name = components[0]
                    value= components[1]
                    self.parse_pid(name, value)
            logging.info("***PID configuration updated")
        except Exception as e:
            logging.error('An exception occurred when parse the pid configuration: {}'.format(e))

    def parse_pid(self, name, _value):
        value= _value.split(self.SEPARATOR)
        if len(value) == 3:
            if "pid" == name:
                self.p_gain = float(value[0].strip())
                self.i_gain = float(value[1].strip())
                self.d_gain = float(value[2].strip())
            elif "yaw_pid" == name:
                self.yaw_p_gain = float(value[0].strip())
                self.yaw_i_gain = float(value[1].strip())
                self.yaw_d_gain = float(value[2].strip())
            elif "alt_pid" == name:
                self.alt_p_gain = float(value[0].strip())
                self.alt_i_gain = float(value[1].strip())
                self.alt_d_gain = float(value[2].strip())

    def to_string(self):
        content = "takeoff:" + str(self.takeoff_value) + self.DELIMITER
        content += "pid:"+str(self.p_gain)+self.SEPARATOR+" "+str(self.i_gain)+self.SEPARATOR+" "+str(self.d_gain)+self.DELIMITER
        content += "yaw_pid:"+str(self.yaw_p_gain)+self.SEPARATOR+" "+str(self.yaw_i_gain)+self.SEPARATOR+" "+str(self.yaw_d_gain)+self.DELIMITER
        content += "alt_pid:"+str(self.alt_p_gain)+self.SEPARATOR+" "+str(self.alt_i_gain)+self.SEPARATOR+" "+str(self.alt_d_gain)
        return content

    #load the hardware configuration from file drone.ini
    def load(self):
        try:
            with open("drone.ini") as file:
                print("")
                for line in file:
                    if line.startswith("#"):
                        continue
                    pos = line.find("#")
                    if pos > 0:
                        line = line[0:pos]
                    line = line.strip()
                    namevalue = line.split("=")
                    if len(namevalue) == 2:
                        print("@", line)

                        name = namevalue[0]
                        value= namevalue[1]
                        if name == "takeoff":
                          speed = int(value)
                          if speed > 1400:
                              self.takeoff_value = speed
                        elif name == "silent_esc":
                            self.silent_esc = (value.lower() == "true")
                        elif name == "motors": #motor compensation
                            settings= value.split(self.SEPARATOR)
                            if len(settings) == 4:
                                self.compensations[0] = int(settings[0])
                                self.compensations[1] = int(settings[1])
                                self.compensations[2] = int(settings[2])
                                self.compensations[3] = int(settings[3])
                        elif name == "battery": #battery compensation
                            self.ideal_battery_factor = int(value) / (Constants.MAX_BATTERY_VOLTAGE - Constants.MIN_BATTERY_VOLTAGE)
                        elif name == "idle": #idle speed
                            self.idle_value = int(value)
                        elif name == "take_off": #take off speed
                            self.takeoff_value = int(value)
                        elif name == "imu": #imu sensor
                            self.imu = value
                        elif name == "imu_filter":
                            self.imu_filter_on = ("on" == value.strip())
                        elif name == "altimeter": #altimeter sensor
                            self.altimeter = value
                        elif name == "compass":
                            self.compass = value
                        else:
                            self.parse_pid(name, value)
                print("")
        except FileNotFoundError:
            pass


''' Drone2 Configuration
silent_esc=true
idle=1350
take_off=1600
imu=MPU6050
pid=1.48, 0.04, 200
alt_pid=180, 0.02, 300 #was 180, 0.02, 300
'''
