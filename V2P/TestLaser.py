import time
import pigpio
from laser import LaserDistance

pi = pigpio.pi()
#I2C power, pin 26
pi.set_mode(26, pigpio.OUTPUT)
pi.write(26, True)

laser = LaserDistance()
laser.start()

while True:
    if laser.read():
        print(laser.distance)
    else:
        print("Failed to read")
    time.sleep(0.5)
