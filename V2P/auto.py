import time
import threading
import math
import logging
from IMU import IMU
#to reenable it https://sparklers-the-makers.github.io/blog/robotics/use-neo-6m-module-with-raspberry-pi/
#modify config.txt and cmdline.txt
from GPS import GPS
from barometer import Barometer
from laser import LaserDistance
from battery import Battery
from pid import PID
from altpid import AltPID
from altassistant import AltPIDAssistant
from constants import Constants

class AutoPilot:
    PC_OK                   = 0 #preflight check result
    PC_WARNING_GPS_NA       = 1
    PC_WARNING_COMPASS_NA   = 2
    PC_WARNING_MAX_         = 50

    PC_ERR_IMU          = 100
    PC_ERR_ALT          = 101
    PC_ERR_BATTERY      = 102

    AP_ERR_OK           = 0  #000000
    AP_ERR_IMU          = 1  #000001
    AP_ERR_COMPASS      = 2  #000010
    AP_ERR_ALTIMETER    = 4  #000100
    AP_ERR_BATTERY      = 16 #010000
    AP_ERR_GPS          = 32 #100000

    DECLINATION     = -10.6

    def __init__(self, configuration):
        self.configuration = configuration
        self.configuration_did_change() #create PIDs
        self.err_code = 0
        self.imu = IMU(self.configuration.imu, self.configuration.compass, self.configuration.imu_filter_on)
        if configuration.altimeter == "VL53L1X":
            self.altimeter = LaserDistance()
        else:
            self.altimeter = Barometer(self.configuration.altimeter)
        self.gps = GPS()
        self.battery = Battery()

        self.altitude = self.latitude = self.longitude= Constants.READING_UNKNOWN
        self.heading  = Constants.READING_UNKNOWN
        self.position_updated = False

        self.home_latitude = self.home_longitude = Constants.READING_UNKNOWN
        self.hovering_altitude = 0
        self.last_altitude = self.altitude_setpoint = self.pid_output_altitude = self.yaw_setpoint = 0

        self.battery_voltage = Constants.READING_UNKNOWN

        self.more_info = False
        self.counter = 0
        self.alt_failures = 0

        self.started = False

        if self.configuration.is_mode_training():
            self.assistant = AltPIDAssistant()
        else:
            self.assistant = None

    def start(self):
        if self.started:
            return
        self.started = True
        self.counter = 0
        self.alt_failures = 0 #consecutive altimeter errors
        self.err_code = self.AP_ERR_OK
        self.reset(True, True)
        self.imu.start()
        self.altimeter.start()
        self.battery.start()

        threading.Thread(target = self.position_daemon).start()

    def stop(self):
        self.started = False

    #reset PIDs
    def reset(self, reset_mpu=True, reset_altimeter=True):
        if reset_mpu:
            self.pid_roll.reset()
            self.pid_pitch.reset()
            self.pid_yaw.reset()
        if reset_altimeter:
            self.pid_altitude.reset()

    def set_err_code(self, code):
        self.err_code = self.err_code | code

    def format_preflight_code(self, code):
        msg = ""
        if code == self.PC_WARNING_GPS_NA:
            msg = "Warning: GPS not available"
        elif code == self.PC_WARNING_COMPASS_NA:
            msg = "Warning: Compass not available"
        elif code == self.PC_ERR_IMU:
            msg = "Error: IMU"
        elif code == self.PC_ERR_BATTERY:
            msg = "Error: Battery"
        return msg

    def format_err_code(self):
        msg = ""
        if self.err_code & self.AP_ERR_IMU > 0:
            msg += "IMU,"
        if self.err_code & self.AP_ERR_COMPASS > 0:
            msg += "Compass,"
        if self.err_code & self.AP_ERR_ALTIMETER > 0:
            msg += "Altimeter,"
        if self.err_code & self.AP_ERR_BATTERY > 0:
            msg += "Battery,"
        if self.err_code & self.AP_ERR_GPS > 0:
            msg += "GPS,"
        if msg == "":
            msg = "None"
        else:
            msg = msg.rstrip(",")
        return msg

    def calibrate_imu(self):
        self.imu.calibrate()

    def calibrate_compass(self):
        self.imu.calibrate_compass()

    def is_compass_available(self):
        return self.heading < Constants.READING_UNKNOWN

    def is_gps_available(self):
        return self.latitude < Constants.READING_UNKNOWN and self.longitude < Constants.READING_UNKNOWN

    def preflight_check(self):
        self.altimeter.reset()
        time.sleep(0.2) #waiting for the barameter reading to be available
        self.altimeter.ready()

        #GPS location
        self.home_latitude = self.latitude
        self.home_longitude= self.longitude

        #errors
        if self.imu.angle_pitch >= Constants.READING_UNKNOWN or self.imu.angle_roll >= Constants.READING_UNKNOWN:
            return self.PC_ERR_IMU
        #altitude
        if self.altitude >= Constants.READING_UNKNOWN:
            return self.PC_ERR_ALT
        if self.battery_voltage >= Constants.READING_UNKNOWN or self.battery_voltage < Constants.MIN_BATTERY_VOLTAGE:
           return self.PC_ERR_BATTERY

        #warnings
        if not self.is_gps_available():
            return self.PC_WARNING_GPS_NA
        if not self.is_compass_available():
            return self.PC_WARNING_COMPASS_NA

        return self.PC_OK

    def fc_state_did_change(self, previous_status, new_status):
        self.reset(True, True)
        if new_status == Constants.FS_TAKING_OFF:
            self.imu.start() #reset imu readings
            self.yaw_setpoint = 0
        elif new_status == Constants.FS_HOVERING:
            self.hovering_altitude = self.altitude
        self.last_altitude = self.altitude_setpoint = self.pid_output_altitude = 0

    def configuration_did_change(self):
        self.pid_roll = PID(self.configuration.p_gain, self.configuration.i_gain, self.configuration.d_gain, Constants.PID_MAX_IMU)
        self.pid_pitch= PID(self.configuration.p_gain, self.configuration.i_gain, self.configuration.d_gain, Constants.PID_MAX_IMU)
        self.pid_yaw  = PID(self.configuration.yaw_p_gain, self.configuration.yaw_i_gain, self.configuration.yaw_d_gain, Constants.PID_MAX_IMU)
        self.pid_altitude = AltPID(self.configuration.alt_p_gain, self.configuration.alt_i_gain, self.configuration.alt_d_gain, Constants.PID_MAX_ALT)

    def loop(self, fc_state, dt):
        self.more_info = self.counter <= 0
        self.counter += 1
        if self.counter >= 50: #every 200ms
            self.counter = 0

        #compass and IMU are in the same sensor, so read compass here
        if (self.more_info or fc_state == Constants.FS_HOME):
            if self.imu.read_compass():
                self.determine_heading()
            else:
                self.set_err_code(self.AP_ERR_COMPASS)

        if not self.imu.read(dt):
            self.set_err_code(self.AP_ERR_IMU)

        roll_setpoint = pitch_setpoint = 0
        if fc_state >= Constants.FS_TAKING_OFF:
            if fc_state == Constants.FS_MOVE_FORWARD:
                pitch_setpoint = self.configuration.movement_angle
            elif fc_state == Constants.FS_MOVE_BACKWARD:
                pitch_setpoint = 0 - self.configuration.movement_angle
            elif fc_state == Constants.FS_MOVE_LEFT:
                roll_setpoint = self.configuration.movement_angle
            elif fc_state == Constants.FS_MOVE_RIGHT:
                roll_setpoint = 0 - self.configuration.movement_angle
            elif fc_state == Constants.FS_HOME:
                home_angle_correction = 0
                if self.longitude != self.home_longitude:
                    home_angle_correction = int(math.degrees(math.atan2((self.latitude-self.home_latitude)/(self.longitude-self.home_longitude))))
                if home_angle_correction < 0:
                    home_angle_correction += 360
                home_angle_correction -= self.heading
                if home_angle_correction > 180:
                    home_angle_correction = 180 - home_angle_correction
                if abs(home_angle_correction) <= 6:
                    pitch_setpoint = self.configuration.movement_angle #start to move forward
                if home_angle_correction >= self.configuration.turning_yaw:
                    self.yaw_setpoint -= self.configuration.turning_yaw * Constants.LOOP_TIMER_S
                    if self.yaw_setpoint < -180:
                        self.yaw_setpoint += 360
                elif home_angle_correction <= 0 - self.configuration.turning_yaw:
                    self.yaw_setpoint = self.configuration.turning_yaw * Constants.LOOP_TIMER_S
                    if self.yaw_setpoint > 180:
                        self.yaw_setpoint -= 360
            elif fc_state == Constants.FS_TURN_LEFT:
                self.yaw_setpoint -= self.configuration.turning_yaw * Constants.LOOP_TIMER_S
                if self.yaw_setpoint < -180:
                    self.yaw_setpoint += 360
            elif fc_state == Constants.FS_TURN_RIGHT:
                self.yaw_setpoint += self.configuration.turning_yaw * Constants.LOOP_TIMER_S
                if self.yaw_setpoint > 180:
                    self.yaw_setpoint -= 360
        pid_output_roll = self.pid_roll.compute(roll_setpoint, self.imu.angle_roll)
        pid_output_pitch= self.pid_pitch.compute(pitch_setpoint, self.imu.angle_pitch)
        pid_output_yaw  = pid_output_yaw = self.pid_yaw.compute(self.yaw_setpoint, self.imu.angle_yaw)

        esc_1 = int(0 + pid_output_pitch - pid_output_roll - pid_output_yaw) #esc 1 (front-right - CCW).
        esc_2 = int(0 - pid_output_pitch - pid_output_roll + pid_output_yaw) #esc 2 (rear-right - CW).
        esc_3 = int(0 - pid_output_pitch + pid_output_roll - pid_output_yaw) #esc 3 (rear-left - CCW).
        esc_4 = int(0 + pid_output_pitch + pid_output_roll + pid_output_yaw) #esc 4 (front-left - CW).
        ret = [esc_1, esc_2, esc_3, esc_4]

        #if self.more_info:
        #    logging.info("PID_ESC"+str(ret)+",ROLL="+str(self.imu.angle_roll)+",PITCH="+str(self.imu.angle_pitch)+",YAW="+str(self.imu.angle_yaw))

        return ret

    #altitude hold
    def hold(self):
        if self.position_updated:
            self.position_updated = False
            self.last_altitude = self.altitude
            self.pid_output_altitude = 0 - int(self.pid_altitude.compute(self.hovering_altitude, self.last_altitude))
        output = int(self.pid_output_altitude)
        if self.more_info:
            diff = round(self.last_altitude - self.hovering_altitude, 3)
            symbol = ""
            if diff > 0:
                symbol = "+"
            logging.info("PID_ALT_HOVERING,"+str(self.pid_output_altitude)+", "+symbol+str(diff)+",alt="+str(self.last_altitude))
        if self.assistant is not None:
            self.assistant.input(self.hovering_altitude, self.last_altitude, output)
        return output

    #delta time in seconds
    def vertical(self, dt, descending = True):
        if dt <= 0:
            if descending:
                self.pid_output_altitude = -5
            else:
                self.pid_output_altitude = 5
        elif self.position_updated:
            self.position_updated = False

            self.last_altitude = self.altitude
            if descending:  #target deceleration speed is -0.25m/s
                self.altitude_setpoint = self.hovering_altitude - dt / 4
            else:
                self.altitude_setpoint = self.hovering_altitude + dt / 4
                if self.altitude_setpoint < 0:
                    self.altitude_setpoint = 0
                elif (self.altitude_setpoint > Constants.MAX_ALTITUDE):
                    self.altitude_setpoint = Constants.MAX_ALTITUDE
            self.pid_output_altitude = 0 - int(self.pid_altitude.compute(self.altitude_setpoint, self.last_altitude))
        #if self.more_info:
        #    direction = "DESC" if descending else "ASC"
        #    logging.info("PID_ALT_VERTICAL,"+direction+","+str(self.pid_output_altitude)+","+str(self.last_altitude)+","+str(self.altitude_setpoint))
        return int(self.pid_output_altitude)

    def is_close_to_home(self):
        #https://aviation.stackexchange.com/questions/97359/easy-way-to-calculate-distance-between-two-gps-coordinates
        lat_diff = self.latitude - self.home_latitude
        lon_diff = self.longitude- self.home_longitude
        distance = math.sqrt(lat_diff * lat_diff + lon_diff * lon_diff) * 111000 #degrees to meters
        return distance <= 2

    def is_landed(self):
        result = self.altitude <= 0.01
        if result:
            logging.info("READY2LAND_ALT,"+str(self.altitude))
        return result

    def is_altimeter_failed(self):
        return self.alt_failures > 10

    def get_drone_altitude(self):
        return self.altitude

    def determine_battery_compensation(self):
        if self.battery_voltage >= Constants.READING_UNKNOWN:
            return 0
        compensation = int((Constants.MAX_BATTERY_VOLTAGE - self.battery_voltage) * self.configuration.ideal_battery_factor)
        if compensation < 0:
            compensation = 0
        else:
            compensation &= 0xfe #operator (x // 2) * 2
        #print("=====================>", compensation)
        return compensation

    def reached_critical_battery_level(self):
        if self.battery_voltage >= Constants.READING_UNKNOWN:
            return False
        return self.battery_voltage <= Constants.CRITICAL_BATTERY_VOLTAGE

    def determine_heading(self):
        x = self.imu.compass_x_input
        y = self.imu.compass_y_input

        #ATAN2(y,x) returns the arc tangent of the two numbers x and y.
        heading_angle = math.degrees(math.atan2(y, x)) + self.DECLINATION
        heading_angle = 270 - heading_angle
        if heading_angle >= 360:
            heading_angle -= 360
        elif heading_angle < 0:
            heading_angle += 360
        self.heading = int(heading_angle)

    #read altitude, gps location and battery voltage
    def position_daemon(self):
        self.altitude = self.latitude = self.longitude = Constants.READING_UNKNOWN
        self.position_updated = False

        logging.info("*Position daemon started")
        loop_seconds = Constants.LOOP_TIMER_S
        total_idle = 0; total_counter = 0
        next_tm_battery = 0; next_tm_gps = 0; next_tm_altimeter = 0
        while self.started:
            now = time.time()
            if now >= next_tm_altimeter:
                next_tm_altimeter = now + 0.012 #every 12ms
                distance = Constants.READING_UNKNOWN
                if self.altimeter.read():
                    distance = self.altimeter.altitude
                else:
                    self.set_err_code(self.AP_ERR_ALTIMETER)
                if distance < Constants.READING_UNKNOWN:
                    self.altitude = distance
                    self.position_updated = True
                    self.alt_failures = 0
                else:
                    self.alt_failures += 1

                #logging.info("** ALT: "+str(distance))
                #step2: 0.00018 seconds

            if now >= next_tm_gps:
                next_tm_gps = now + 1  #every second, update GPS reading
                if self.gps.read():
                    self.latitude = self.gps.latitude
                    self.longitude= self.gps.longitude
                    logging.info("GPS,"+str(self.latitude)+","+str(self.longitude))
                else:
                    self.set_err_code(self.AP_ERR_GPS)

            if now >= next_tm_battery + 10:
                next_tm_battery = now + 10 #every 10s, update battery reading
                if self.battery.read():
                    self.battery_voltage = self.battery.voltage
                else:
                    self.set_err_code(self.AP_ERR_BATTERY)

            idle = loop_seconds - (time.time() - now)
            total_idle += idle
            total_counter += 1
            if idle > 0:
                time.sleep(idle)

        self.altimeter.stop()
        self.position_updated = False
        if total_counter <= 0:
            total_counter = 1
        logging.info("*Position daemon stopped, averagle idle: " + str(int(total_idle / total_counter * 1000)) + " ms")

    def format_status(self):
        if self.started:
            return "heading="+str(self.heading)+";lat="+str(self.latitude)+";lon="+str(self.longitude)+";alt="+str(round(self.altitude, 1))+";bat="+str(self.battery_voltage)
        return ""
