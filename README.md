# Drone
Flight controller system that can be run on a Raspberry Pi to control a drone and use the Flying Hawk PID tuning algorithm, as well as visual odometry for positional calculations.

# Layout
V2P is the first generation of the drone project, designed to make the drone fly and collect data to compare between the complementary and Kalman filters. All code is in Python.

V3P is the second iteration of the drone project, designed to implement the Flying Hawk algorithm to effectively tune PID gains. It contains a dedicated folder, bt, that holds all code relating to the Bluetooth connection to the drone. All code is in C++ to reduce loop times on the Raspberry Pi, allowing for faster PID response.

V4P is the third iteration of the drone project, designed to implement the POSIT (visual odometry) algorithm for accurate navigation by detecting planar objects such as stop signs. All code is still in C++, and the code has the bt folder from the previous iteration, as well as two more folders. The cv folder holds the code related to the computer vision parts of the project, and the lccv folder contains driver code allowing the camera on the drone to run on a Raspberry Pi 5 (used to provide more compute to process images with CNNs).

# Credits
The motor system via PWM control is based off the source code avaliable here: https://www.instructables.com/Driving-an-ESCBrushless-Motor-Using-Raspberry-Pi/
The drone's flight controller system is also based off a project by Joop Brooking on running a drone with a Raspberry Pi: https://www.brokking.net/ymfc-32_main.html
The drone's Bluetooth system is inspired off https://github.com/Douglas6/cputemp

Additional sensor drivers:
MS5611- https://github.com/emlid/Navio/blob/master/Python/navio/ms5611.py
BMP280- https://github.com/pimoroni/bmp280-python/blob/main/bmp280/__init__.py
MPU9250- https://github.com/FaBoPlatform/FaBo9AXIS-MPU9250-Python/tree/master/FaBo9Axis_MPU9250
INA219- https://github.com/chrisb2/pi_ina219
VL53L1X- https://github.com/pimoroni/vl53l1x-python/blob/master/python/VL53L1X.py
GPS- https://github.com/Knio/pynmea2
