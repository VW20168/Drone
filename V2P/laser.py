import time
import logging
import qwiic_i2c
import qwiic_vl53l1x

#https://github.com/sparkfun/Qwiic_VL53L1X_Py?tab=readme-ov-file
class LaserDistance:
    NA = 65535

    def __init__(self):
        self.tof = qwiic_vl53l1x.QwiicVL53L1X(i2c_driver=qwiic_i2c.get_i2c_driver(iBus = 6))
        self.tof.sensor_init()
        self.tof.set_inter_measurement_in_ms(25) # >= timing_budget
        self.tof.set_timing_budget_in_ms(20) #20, 33, 50, 100, 200, 500
        self.tof.set_distance_mode(1)  #1-short, 2-long
        self.altitude = self.raw_altitude = 0
        self.last_distance_mm = -1
        self.home_alt = self.NA

    def start(self):
        self.home_alt = self.NA
        self.last_distance_mm = -1
        self.tof.start_ranging()
        self.read()

    def reset(self):
        self.home_alt = self.raw_altitude = self.NA

    def ready(self):
        self.home_alt = self.raw_altitude

    def stop(self):
        self.tof.stop_ranging()
        pass

    def read(self):
        distance_in_mm = None
        #time.sleep(0.005)
        rangestatus = 0
        try:
            distance_in_mm = self.tof.get_distance()
            rangestatus = self.tof.get_range_status()
            if rangestatus > 0:
                distance_in_mm = None
            else:
                if self.last_distance_mm < 0:
                    self.last_distance_mm = distance_in_mm
                else:
                    mm = distance_in_mm
                    distance_in_mm = distance_in_mm * 0.7 + self.last_distance_mm * 0.3
                    self.last_distance_mm = mm
        except Exception as e:
            logging.error('An exception occurred when read the laser distance sensor: {}'.format(e))
        #logging.info('Laser reading: ' + str(distance_in_mm)+", status="+str(rangestatus))
        
        if rangestatus == 4:
            distance_in_mm = 1399 #short range 1.3m
            
        if distance_in_mm is None:
            return False
        if distance_in_mm < 0:
            return False

        alt_input = round(distance_in_mm / 1000, 3) #to meters
        self.raw_altitude = alt_input
        
        if self.home_alt >= self.NA:
            self.home_alt = alt_input
        self.altitude = alt_input - self.home_alt
        if self.altitude < 0:
            self.altitude = 0
        return True

    #rangestatus = ToF.get_range_status()
	#if rangestatus == 0:
		#print("Good ")
	#elif rangestatus == 1:
		#print("Signal Fail ")
	#elif rangestatus == 2:
		#print("Sigma Fail ")
	#elif rangestatus == 7:
		#print("Wrapped Target Fail ")
	#else:
		#print("Unknown (code: %s) ", rangestatus)
