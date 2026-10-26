import logging
from INA219 import INA219
from constants import Constants

#Enabling Multiple I2c, https://medium.com/@mileperuma/enable-multiple-i2c-ports-on-raspberry-pi-5a8807471737
class Battery:

    def __init__(self):
        self.ina219 = INA219(busnum=5)
        try: 
            self.ina219.configure(INA219.RANGE_16V)
        except Exception as e: #if INA219 is not connected
            print(e)
        self.voltage = Constants.READING_UNKNOWN
        self.last_reading = Constants.READING_UNKNOWN

    def start(self):
        self.last_reading = Constants.READING_UNKNOWN
        self.read()

    def read(self):
        available = False
        try:
            this_reading = self.ina219.voltage()
            if self.last_reading < Constants.READING_UNKNOWN:
                self.voltage = round(self.last_reading * 0.3 + this_reading * 0.7, 1)
            else:
                self.voltage = this_reading
            self.last_reading = this_reading    
            available = True
        except Exception as e:
            logging.error('An exception occurred when read the battery voltage: {}'.format(e))

        return available
