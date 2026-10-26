import time
import math
import os
import logging
from constants import Constants
from MPU6050 import MPU6050
from MPU9250 import MPU9250

class IMU:
    def __init__(self, sensor, compass_sensor, filter_on = False):
        if sensor == "MPU9250":
            self.mpu = MPU9250(0x68)
            self.compass = self.mpu
        elif sensor == "MPU6050":
            self.mpu = MPU6050(0x68)
            self.compass = None #no implementation for separate compass sensor, such as HMC5883L
        self.filter_on = filter_on
        self.mpu.set_config_raw(0x1b, 0x08)
        self.mpu.set_config_raw(0x1c, 0x10)
        self.mpu.set_config_raw(0x1a, 0x03)

        self.angle_pitch=self.angle_roll=self.angle_yaw=Constants.READING_UNKNOWN
        ##self.gyro_roll_input = self.gyro_pitch_input = self.gyro_yaw_input = 0.0

        self.acc_x = self.acc_y = self.acc_z = 0 #int_16_t
        self.gyro_pitch =self.gyro_roll =self.gyro_yaw= 0 #int_16_t

        self.compass_x = self.compass_y = self.compass_z = Constants.READING_UNKNOWN
        self.compass_x_input = self.compass_y_input = self.compass_z_input = 0

        self.load() #load IMU calibration data

    def start(self):
        if self.read(Constants.LOOP_TIMER_S):
            self.angle_pitch=self.angle_roll=self.angle_yaw=0

        if self.read_compass():
            self.compass_x = self.compass_y = self.compass_z = 0

        self.compass_x_input = self.compass_x
        self.compass_y_input = self.compass_y
        self.compass_z_input = self.compass_z
            
    def reset_calibrations(self, imu, compass):
        if imu:
            self.acc_x_base = 0
            self.acc_y_base = 0
            self.acc_z_base = 0

            self.gyro_roll_base  = 0
            self.gyro_pitch_base = 0
            self.gyro_yaw_base   = 0
        if compass:
            self.compass_scale_x = 1
            self.compass_scale_y = 1
            self.compass_scale_z = 1
            self.compass_offset_x = 0
            self.compass_offset_y = 0
            self.compass_offset_z = 0

    def read(self, dt):
        #Read from the sensor, update all the values
        data = None
        try:
            data = self.mpu.get_data_raw()
        except Exception as e:
            logging.error('An exception occurred when read the acce/gyro: {}'.format(e))

        if data is None:
            return False

        self.acc_y = data[0] #switch x - y
        self.acc_x = data[1]
        self.acc_z = data[2]

        self.gyro_roll = data[3]
        self.gyro_pitch= 0 - data[4] #Invert the direction of the axis.
        self.gyro_yaw  = 0 - data[5] #Invert the direction of the axis.

        self.acc_x -= self.acc_x_base  #Subtact the manual accelerometer roll calibration value.
        self.acc_y -= self.acc_y_base  #Subtact the manual accelerometer pitch calibration value.

        self.gyro_roll -= self.gyro_roll_base #Subtact the manual gyro roll calibration value.
        self.gyro_pitch-= self.gyro_pitch_base #Subtact the manual gyro pitch calibration value.
        self.gyro_yaw  -= self.gyro_yaw_base   #Subtact the manual gyro yaw calibration value.

        #65.5 = 1 deg/sec (check the datasheet of the MPU-6050 for more information).
        ##self.gyro_roll_input = self.gyro_roll_input * 0.7 + self.gyro_roll * 0.00458015 #/ 65.5 * 0.3)   #Gyro pid input is deg/sec.
        ##self.gyro_pitch_input= self.gyro_pitch_input* 0.7 + self.gyro_pitch* 0.00458015 #Gyro pid input is deg/sec.
        ##self.gyro_yaw_input  = self.gyro_yaw_input  * 0.7 + self.gyro_yaw  * 0.00458015      #Gyro pid input is deg/sec.

        #Gyro angle calculations
        #0.0000611 = 1 / 250Hz / 65.5 if dt=4ms
        c1 = dt / 65.5
        c2 = c1 * 0.01745328 #3.14159 / 180

        self.angle_pitch += self.gyro_pitch* c1  #Calculate the traveled pitch angle and add this to the angle_pitch variable.
        self.angle_roll  += self.gyro_roll * c1    #Calculate the traveled roll angle and add this to the angle_roll variable.
        self.angle_yaw   += self.gyro_yaw  * c1     #Calculate the traveled yaw angle and add this to the angle_yaw variable.

        if self.angle_yaw < -180:
            self.angle_yaw += 360
        elif self.angle_yaw > 180:
            self.angle_yaw -= 360

        #0.000001066 = 0.0000611 * (3.142(PI) / 180degr) The sin function is in radians and not degrees.
        transfer = math.sin(self.gyro_yaw * c2)
        self.angle_pitch-= self.angle_roll  * transfer                  #If the IMU has yawed transfer the roll angle to the pitch angel.
        self.angle_roll += self.angle_pitch * transfer                  #If the IMU has yawed transfer the pitch angle to the roll angel.

        #//self.angle_yaw -= course_deviation(angle_yaw, actual_compass_heading) / 1200.0;  #Calculate the difference between the gyro and compass heading and make a small correction.
        #//if (angle_yaw < 0) angle_yaw += 360;                                             #If the compass heading becomes smaller then 0, 360 is added to keep it in the 0 till 360 degrees range.
        #//else if (angle_yaw >= 360) angle_yaw -= 360;                                     #If the compass heading becomes larger then 360, 360 is subtracted to keep it in the 0 till 360 degrees range.

        #Accelerometer angle calculations
        if self.filter_on:
            angle_roll_acc = angle_pitch_acc = 0
            acc_total_vector = math.sqrt((self.acc_x * self.acc_x) + (self.acc_y * self.acc_y) + (self.acc_z * self.acc_z))    #Calculate the total accelerometer vector.
            if abs(self.acc_y) < acc_total_vector:                                             #Prevent the asin function to produce a NaN.
                angle_pitch_acc = math.asin(self.acc_y / acc_total_vector) * 57.296       #Calculate the pitch angle.
            if abs(self.acc_x) < acc_total_vector:                                             #Prevent the asin function to produce a NaN.
                angle_roll_acc = math.asin(self.acc_x / acc_total_vector) * 57.296        #Calculate the roll angle.

            self.angle_pitch= self.angle_pitch * 0.9997 + angle_pitch_acc * 0.0003         #Correct the drift of the gyro pitch angle with the accelerometer pitch angle.
            self.angle_roll = self.angle_roll  * 0.9997 + angle_roll_acc  * 0.0003         #Correct the drift of the gyro roll angle with the accelerometer roll angle.

        return True

    def read_compass(self):
        readings = None
        if self.compass is None:
            return False
        else:
            try:
                readings = self.compass.readMagnet()
            except Exception as e:
                logging.error('An exception occurred when read the magnetometer: {}'.format(e))

            if readings is None:
                return False

            x = readings['x']
            y = readings['y']
            z = readings['z']

            #refer to https://github.com/kevinmcaleer/mpu9250/blob/main/ak8963.py
            x -= self.compass_offset_x
            x *= self.compass_scale_x

            y -= self.compass_offset_y
            y *= self.compass_scale_y

            z -= self.compass_offset_z
            z *= self.compass_scale_z

            self.compass_x = x
            self.compass_y = y
            self.compass_z = z

            self.compass_x_input = self.compass_x_input * 0.85 + self.compass_x * 0.15
            self.compass_y_input = self.compass_y_input * 0.85 + self.compass_y * 0.15
            self.compass_z_input = self.compass_z_input * 0.85 + self.compass_z * 0.15

            return True

    def calibrate(self):
        self.reset_calibrations(True, False)

        acc_x_cal = acc_y_cal = acc_z_cal = 0
        gyro_roll_cal = gyro_pitch_cal = gyro_yaw_cal = 0
        #Let's take multiple gyro data samples so we can determine the average gyro offset (calibration).
        total = 8000
        for _i in range(total):
            self.read(False)

            acc_x_cal += self.acc_x
            acc_y_cal += self.acc_y
            acc_z_cal += self.acc_z

            gyro_roll_cal  += self.gyro_roll
            gyro_pitch_cal += self.gyro_pitch
            gyro_yaw_cal   += self.gyro_yaw

            time.sleep(Constants.LOOP_TIMER_S)

        self.acc_x_base = acc_x_cal / total
        self.acc_y_base = acc_y_cal / total
        self.acc_z_base = acc_z_cal / total

        self.gyro_roll_base = gyro_roll_cal / total
        self.gyro_pitch_base= gyro_pitch_cal/ total
        self.gyro_yaw_base  = gyro_yaw_cal  / total

        self.save_calibrations()

    def calibrate_compass(self):
        if self.compass is None:
            return False

        self.reset_calibrations(False, True)

        compass_cal_values=[0,0,0,0,0,0]
        total = 1500
        for _i in range(total):
            if self.read_compass():
                x = self.compass_x
                y = self.compass_y
                z = self.compass_z
                if _i <= 0:
                    compass_cal_values[0] = compass_cal_values[1] = x
                    compass_cal_values[2] = compass_cal_values[3] = y
                    compass_cal_values[4] = compass_cal_values[5] = z
                else:
                    if x < compass_cal_values[0]:
                        compass_cal_values[0] = x
                    if x > compass_cal_values[1]:
                        compass_cal_values[1] = x
                    if y < compass_cal_values[2]:
                        compass_cal_values[2] = y
                    if y > compass_cal_values[3]:
                        compass_cal_values[3] = y
                    if z < compass_cal_values[4]:
                        compass_cal_values[4] = z
                    if z > compass_cal_values[5]:
                        compass_cal_values[5] = z
            time.sleep(Constants.LOOP_TIMER_S)
        self.calculate_compass_calibration(compass_cal_values)
        self.save_calibrations()

    def get_config_file_name(self):
        return os.path.dirname(__file__) + "/imu.ini" #same folder as this script

    def save_calibrations(self):
        #save to the file
        file = open(self.get_config_file_name(), "w")
        file.write(str(self.acc_x_base) +"," + str(self.acc_y_base)+","+str(self.acc_z_base)+","+str(self.gyro_roll_base)+","+str(self.gyro_pitch_base)+","+str(self.gyro_yaw_base))
        file.write("\n")
        file.write(str(self.compass_offset_x) +"," + str(self.compass_offset_y)+","+str(self.compass_offset_z) +"," + str(self.compass_scale_x)+","+str(self.compass_scale_y)+","+str(self.compass_scale_z))
        file.write("\n")
        file.close()

    def load(self):
        self.reset_calibrations(True, True)
        try:
            with open(self.get_config_file_name()) as file:
                line = file.readline()
                options = line.split(",")
                if len(options) == 6:
                    self.acc_x_base = float(options[0])
                    self.acc_y_base = float(options[1])
                    self.acc_z_base = float(options[2])

                    self.gyro_roll_base = float(options[3])
                    self.gyro_pitch_base = float(options[4])
                    self.gyro_yaw_base = float(options[5])

                    line = file.readline()
                    options = line.split(",")
                    if len(options) == 6:
                        self.compass_offset_x = float(options[0])
                        self.compass_offset_y = float(options[1])
                        self.compass_offset_z = float(options[2])
                        self.compass_scale_x = float(options[3])
                        self.compass_scale_y = float(options[4])
                        self.compass_scale_z = float(options[5])
                    print("IMU configuration loaded")
                else:
                    print("Malformed IMU configuration file")
        except FileNotFoundError:
            pass

    #Calculate the calibration offset and scale values
    def calculate_compass_calibration(self, compass_cal_values):
        minx = compass_cal_values[0]
        maxx = compass_cal_values[1]
        miny = compass_cal_values[2]
        maxy = compass_cal_values[3]
        minz = compass_cal_values[4]
        maxz = compass_cal_values[5]

        # Hard iron correction
        self.compass_offset_x = (maxx + minx) / 2
        self.compass_offset_y = (maxy + miny) / 2
        self.compass_offset_z = (maxz + minz) / 2

        # Soft iron correction
        avg_delta_x = (maxx - minx) / 2
        avg_delta_y = (maxy - miny) / 2
        avg_delta_z = (maxz - minz) / 2

        avg_delta = (avg_delta_x + avg_delta_y + avg_delta_z) / 3

        self.compass_scale_x = avg_delta / avg_delta_x
        self.compass_scale_y = avg_delta / avg_delta_y
        self.compass_scale_z = avg_delta / avg_delta_z
