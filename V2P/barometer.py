import logging
from BMP280 import BMP280
from ms5611 import MS5611

'''
 Enabling Multiple I2c, https://medium.com/@mileperuma/enable-multiple-i2c-ports-on-raspberry-pi-5a8807471737
 sudo vim boot/config.txt
 dtparam=i2c_arm=on #add after this line
 dtoverlay=i2c-gpio,bus=4,i2c_gpio_sda=10,i2c_gpio_scl=9
'''
class Barometer:
    NA          = 65535
    BUF_SIZE_P  = 20
    BUF_SIZE_T  = 5

    def __init__(self, sensor):
        self.barometer = None
        if sensor == "MS5611":
            self.barometer = MS5611(I2C_bus_number=4)
            self.barometer.setup()
        elif sensor == "BMP280":
            self.barometer = BMP280(bus=4)
            self.barometer.setup()

        self.pressures = [0] * self.BUF_SIZE_P
        self.temperatures = [0] * self.BUF_SIZE_T

        self.reset()

    def start(self):
        pass

    def reset(self):
        self.altitude = self.raw_altitude = 0
        self.home_alt = self.NA

        self.header_p = self.counter_p = self.sum_p = 0
        self.header_t = self.counter_t = self.sum_t = 0
        self.pressure_slow = 0
        for i in range(self.BUF_SIZE_P):
            self.pressures[i]=0
        for i in range(self.BUF_SIZE_T):
            self.temperatures[i]=0

    def ready(self):
        self.home_alt = self.raw_altitude  #reset home altitude

    def read(self):
        altitude_input = None
        try:
            self.barometer.update()
            altitude_input =  self.get_altitude(self.barometer.TEMP, self.barometer.PRES)
        except Exception as e:
            logging.error('An exception occurred when read the barometer: {}'.format(e))

        if altitude_input is None:
            return False
        self.raw_altitude = altitude_input

        if self.home_alt >= self.NA:
            self.home_alt = altitude_input
        self.altitude = altitude_input - self.home_alt
        if self.altitude < 0:
            self.altitude = 0

        return True

    def stop(self):
        pass

    def get_altitude(self, temp, pressure):
        if pressure <= 0:
            return None

        #moving average of temperatures
        self.counter_t += 1
        self.sum_t = self.sum_t - self.temperatures[self.header_t] + temp
        self.temperatures[self.header_t] = temp
        self.header_t += 1
        if self.header_t >= self.BUF_SIZE_T:
            self.header_t = 0
        applied = self.counter_t
        if applied > self.BUF_SIZE_T:
            applied = self.BUF_SIZE_T
        avg_temp = self.sum_t / applied

        #moving average of pressures
        self.counter_p += 1
        self.sum_p = self.sum_p - self.pressures[self.header_p] + pressure
        self.pressures[self.header_p] = pressure
        self.header_p += 1
        if self.header_p >= self.BUF_SIZE_P:
            self.header_p = 0
        applied = self.counter_p
        if applied > self.BUF_SIZE_P:
            applied = self.BUF_SIZE_P
        avg_pressure = self.sum_p / applied

        '''
        if self.pressure_slow > 0:
            self.pressure_slow = self.pressure_slow * 0.985 + avg_pressure * 0.015
        else:
            self.pressure_slow = avg_pressure
        pressure_diff = self.pressure_slow - avg_pressure
        if pressure_diff > 8:
            pressure_diff = 8
        elif pressure_diff < -8:
            pressure_diff = -8
        if pressure_diff > 1 or pressure_diff < -1:
            self.pressure_slow -= pressure_diff / 6.0
        actual_pressure = self.pressure_slow
        '''
        
        qnh = 1013.25
        altitude = ((pow((qnh / avg_pressure), (1.0 / 5.257)) - 1) * (avg_temp + 273.15)) / 0.0065
        #print("=====>", altitude, pressure_diff)
        return altitude
