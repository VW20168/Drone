import time
import threading
import pigpio
import logging
from auto import AutoPilot
from constants import Constants
from configuration import DroneConfiguration

class Motor:
    def __init__(self, no, pin, pi, _min, _max, _compensation = 0):
        self.MIN_VALUE=_min
        self.MAX_VALUE=_max
        self.compensation = _compensation
        self.no = "#"+str(no)
        self.pin = pin
        self.pi = pi
        self.speed = 0
        self.pi.set_mode(self.pin, pigpio.OUTPUT)
        self.pi.set_servo_pulsewidth(self.pin, 0)

    def get_pin(self):
        return self.pin

    def calibrate(self):
        logging.info("Calibrating motor " + self.no)
        self.pi.set_servo_pulsewidth(self.pin, 0)
        time.sleep(5)
        self.pi.set_servo_pulsewidth(self.pin, self.MIN_VALUE)
        time.sleep(5)
        logging.info("halfway there")
        self.pi.set_servo_pulsewidth(self.pin, self.MAX_VALUE)
        time.sleep(5)
        self.pi.set_servo_pulsewidth(self.pin, 0)
        logging.info("almost done")
        time.sleep(2)
        self.pi.set_servo_pulsewidth(self.pin, self.MIN_VALUE)# arming
        time.sleep(2)
        logging.info("done")

    def arm(self):
        #logging.info("Arming " + self.no)
        self.pi.set_servo_pulsewidth(self.pin, 0)
        time.sleep(0.5)
        self.pi.set_servo_pulsewidth(self.pin, self.MAX_VALUE)
        time.sleep(0.5)
        self.pi.set_servo_pulsewidth(self.pin, self.MIN_VALUE)
        time.sleep(0.8)
        self.speed = self.MIN_VALUE
        #logging.info("Armed " + self.no)

    def set_speed(self, _speed):
        new_speed = int(_speed) + self.compensation
        if abs(self.speed - new_speed) >= 2 or new_speed <= 0:
            self.speed = new_speed
            #pigpio.error: 'pulsewidth not 0 or 500-2500'
            if self.speed < self.MIN_VALUE:
                self.speed = 0
            elif self.speed > self.MAX_VALUE:
                self.speed = self.MAX_VALUE
            self.pi.set_servo_pulsewidth(self.pin, self.speed)

#Motor 1 (front-right - CCW)
#Motor 2 (rear-right - CW)
#Motor 3 (rear-left - CCW)
#Motor 4 (front-left - CW)
class FlightController:

    PIN_RED_LED     = 12
    PIN_GREEN_LED   = 20

    MIN_VALUE       = 1050
    MAX_VALUE       = 2200

    def __init__(self, mode):
        self.mode = mode

        self.pi = pigpio.pi()
        #I2C power, pin 26
        self.pi.set_mode(26, pigpio.OUTPUT)
        self.pi.write(26, True)
        self.pi.set_mode(18, pigpio.OUTPUT)
        self.pi.write(18, True)
        #LED
        self.pi.set_mode(self.PIN_RED_LED, pigpio.OUTPUT)
        self.pi.set_mode(self.PIN_GREEN_LED, pigpio.OUTPUT)
        self.state = 0
        self.speed = 0
        self.motor_compensation = [0, 0, 0, 0]
        self.red_led_on = False
        self.green_led_on = False
        self.turn_on_red_led()
        self.turn_on_green_led()
        self.configuration = DroneConfiguration(mode)
        self.motor1 = Motor(1, 22, self.pi, self.MIN_VALUE, self.MAX_VALUE, self.configuration.compensations[0])
        self.motor2 = Motor(2, 17, self.pi, self.MIN_VALUE, self.MAX_VALUE, self.configuration.compensations[1])
        self.motor3 = Motor(3, 19, self.pi, self.MIN_VALUE, self.MAX_VALUE, self.configuration.compensations[2])
        self.motor4 = Motor(4, 13, self.pi, self.MIN_VALUE, self.MAX_VALUE, self.configuration.compensations[3])
        self.motor_compensation[0] = self.motor_compensation[1] = self.motor_compensation[2] = self.motor_compensation[3] = 0
        self.last_stop_time = 0
        self.auto_pilot_active = False
        self.auto_pilot = AutoPilot(self.configuration)
        self.sem = threading.BoundedSemaphore(1)
        self.nav_to_state(Constants.FS_POWERED_ON)
        self.turn_off_red_led()
        self.turn_off_green_led()
        self.interrupted = False
        logging.info("Flight Controller initialized")

    def turn_on_red_led(self):
        if not self.red_led_on:
            self.pi.write(self.PIN_RED_LED, True)
            self.red_led_on = True

    def turn_off_red_led(self):
        if self.red_led_on:
            self.pi.write(self.PIN_RED_LED, False)
            self.red_led_on = False

    def turn_on_green_led(self):
        if not self.green_led_on:
            self.pi.write(self.PIN_GREEN_LED, True)
            self.green_led_on = True

    def turn_off_green_led(self):
        if self.green_led_on:
            self.pi.write(self.PIN_GREEN_LED, False)
            self.green_led_on = False

    def turn_on_all_leds(self):
        self.turn_on_red_led()
        self.turn_on_green_led()

    def turn_off_all_leds(self):
        self.turn_off_red_led()
        self.turn_off_green_led()

    #calibrate ESCs
    def calibrate(self):
        self.nav_to_state(Constants.FS_CALIBRATING)
        escs = [self.motor1.get_pin(), self.motor2.get_pin(), self.motor3.get_pin(), self.motor4.get_pin()]
        for pin in escs:
            self.pi.set_servo_pulsewidth(pin, 0)
        time.sleep(5)
        for pin in escs:
            self.pi.set_servo_pulsewidth(pin, self.MIN_VALUE)
        time.sleep(5)
        for pin in escs:
            self.pi.set_servo_pulsewidth(pin, self.MAX_VALUE)
        time.sleep(5)
        for pin in escs:
            self.pi.set_servo_pulsewidth(pin, 0)
        time.sleep(2)
        for pin in escs:
            self.pi.set_servo_pulsewidth(pin, self.MIN_VALUE)# arming
        time.sleep(2)
        self.speed = 0
        for pin in escs:
            self.pi.set_servo_pulsewidth(pin, 0)
        self.nav_to_state(Constants.FS_CALIBRATED)

    def calibrate_imu(self):
        self.nav_to_state(Constants.FS_CALIBRATING)
        self.auto_pilot.calibrate_imu()
        self.nav_to_state(Constants.FS_CALIBRATED)

    def calibrate_compass(self):
        self.nav_to_state(Constants.FS_CALIBRATING)
        self.auto_pilot.calibrate_compass()
        self.nav_to_state(Constants.FS_CALIBRATED)

    def activate_speed(self):
        self.sem.acquire()
        self.motor1.set_speed(self.speed + self.motor_compensation[0])
        self.motor3.set_speed(self.speed + self.motor_compensation[2])
        self.motor2.set_speed(self.speed + self.motor_compensation[1])
        self.motor4.set_speed(self.speed + self.motor_compensation[3])
        self.sem.release()

    def start(self):
        if self.state >= Constants.FS_STARTED:
            return
        if time.time() - self.last_stop_time <= 5:
            return
        self.nav_to_state(Constants.FS_STARTING)
        if not self.configuration.silent_esc:
            self.motor1.arm()
            self.motor2.arm()
            self.motor3.arm()
            self.motor4.arm()
        self.speed = self.configuration.idle_value
        self.reset_compensation()
        self.activate_speed()

        self.ping_received = False
        self.nav_to_state(Constants.FS_STARTED)
        threading.Thread(target = self.auto_pilot_daemon).start()
        if self.configuration.is_mode_training():
            threading.Thread(target = self.training_daemon).start()
        time.sleep(1) #waiting for the sensor to be ready
        threading.Thread(target = self.watchdog).start()

    def takeoff(self):
        if self.state == Constants.FS_STARTED:
            self.nav_to_state(Constants.FS_PREFLIGHT_CHK)

    def inc_speed(self):
        if self.state >= Constants.FS_HOVERING:
            self.nav_to_state(Constants.FS_FLY_HIGH)
        elif self.state == Constants.FS_STARTED:
            self.speed = self.speed + self.configuration.nudge_value
            if self.speed > self.MAX_VALUE:
                self.speed = self.MAX_VALUE
            self.activate_speed()

    def dec_speed(self):
        if self.state >= Constants.FS_HOVERING:
            self.nav_to_state(Constants.FS_FLY_LOW)
        elif self.state == Constants.FS_STARTED:
            self.speed = self.speed - self.configuration.nudge_value
            if self.speed < self.MIN_VALUE:
                self.stop(True)
            else:
                self.activate_speed()

    def landing(self):
        if self.allow_maneuver() or self.state == Constants.FS_HOME:
            self.nav_to_state(Constants.FS_LANDING)

    def turn_left(self):
        if self.allow_maneuver():
            self.nav_to_state(Constants.FS_TURN_LEFT)

    def turn_right(self):
        if self.allow_maneuver():
            self.nav_to_state(Constants.FS_TURN_RIGHT)

    def move_forward(self):
        if self.allow_maneuver():
            self.nav_to_state(Constants.FS_MOVE_FORWARD)

    def move_backward(self):
        if self.allow_maneuver():
            self.nav_to_state(Constants.FS_MOVE_BACKWARD)

    def move_left(self):
        if self.allow_maneuver():
            self.nav_to_state(Constants.FS_MOVE_LEFT)

    def move_right(self):
        if self.allow_maneuver():
            self.nav_to_state(Constants.FS_MOVE_RIGHT)

    def home(self):
        if self.allow_maneuver():
            if self.auto_pilot.is_gps_available() and self.auto_pilot.is_compass_available():
                self.nav_to_state(Constants.FS_HOME)
            else:
                self.nav_to_state(Constants.FS_LANDING)

    def hover(self):
        if self.allow_maneuver() or self.state == Constants.FS_TAKING_OFF:
            self.nav_to_state(Constants.FS_HOVERING)

    #may be a command from the client
    def stop(self, force=False, user_interactive = True, on_shutdown = False):
        if user_interactive:
            self.interrupted = True
        if (not on_shutdown) and (self.state == Constants.FS_POWERED_ON or self.state == Constants.FS_STOPPED):
            return
        if force or self.state == Constants.FS_STARTED:
            self.nav_to_state(Constants.FS_STOPPING)
            self.speed = 0 if on_shutdown or not self.configuration.silent_esc else self.MIN_VALUE
            self.reset_compensation()
            self.activate_speed()
            self.nav_to_state(Constants.FS_STOPPED)
            self.last_stop_time = time.time()
        else:
            if self.state == Constants.FS_TAKING_OFF:
                self.hover()
            elif self.state == Constants.FS_LANDING or self.state == Constants.FS_EMERGENCY:
                self.stop(True)
            elif self.state >= Constants.FS_HOME:
                self.landing()

    def fall(self):
        self.reset_compensation()
        self.speed = self.configuration.idle_value + (self.configuration.takeoff_value - self.configuration.idle_value) // 2
        self.activate_speed()

    def allow_maneuver(self):
        return self.state >= Constants.FS_HOVERING

    def nav_to_state(self, status):
        if self.state == status:
            return

        self.state = status
        if self.state != Constants.FS_SHUTDOWN:
            if self.state == Constants.FS_CALIBRATING:
                self.turn_on_all_leds()
            elif self.state == Constants.FS_CALIBRATED:
                self.turn_off_all_leds()

        msg = "--- to state " + str(self.state)
        if self.state == Constants.FS_TAKING_OFF:
            msg += ", in the air now"
        elif self.state == Constants.FS_HOVERING:
            msg += ", hovering"
        elif self.state == Constants.FS_LANDING:
            msg += ", landing now"
        elif self.state == Constants.FS_TURN_LEFT:
            msg += ", turning left"
        elif self.state == Constants.FS_TURN_RIGHT:
            msg += ", turning right"
        msg += " ---"
        logging.info(msg)

    def reset_compensation(self):
        self.motor_compensation[0] = self.motor_compensation[1] = self.motor_compensation[2] = self.motor_compensation[3] = 0

    def ping(self):
        self.ping_received = True

    def set_config(self, options):
        self.configuration.parse(options)
        self.auto_pilot.configuration_did_change()

    def get_status(self):
        info = "status=" + str(self.state) + ";pwm="+str(self.speed)
        info += ";"+self.auto_pilot.format_status() #location, altitude etc
        info += ";config="+self.configuration.to_string()
        #logging.info("[TX]"+info)
        return info

    def shutdown(self):
        if not self.state == Constants.FS_SHUTDOWN:
            self.stop(True, True, True)
            self.turn_off_red_led()
            self.turn_off_green_led()
            self.pi.stop()
            self.nav_to_state(Constants.FS_SHUTDOWN)

    def watchdog(self):
        logging.info("Watchdog started")
        self.ping_received = self.auto_pilot_active = False
        time.sleep(2)
        counter = 0
        while self.state >= Constants.FS_STARTED:
            if self.ping_received:
                counter = 0
            else:
                counter += 1
                logging.info("Ping not received: " + str(counter) +", auto:" + str(self.auto_pilot_active) +", state: " + str(self.state))
                if counter > 1:
                    if self.auto_pilot_active:
                        self.stop(False, True)
                    elif counter > 5:
                        self.stop(True, True)
                    else:
                        self.fall() #hard stop
            self.ping_received = self.auto_pilot_active = False
            time.sleep(2)
        logging.info("Watchdog stopped")

    def auto_pilot_daemon(self):
        logging.info("Auto pilot started")

        self.auto_pilot_active = True
        self.auto_pilot.start()
        time.sleep(1) #for the compass to be ready
        last_read_time = now_time = time.time()
        current_state = self.state
        sub_state = event_time = 0
        led_timer = sum_idle = counter = negative = 0
        loop_seconds = Constants.LOOP_TIMER_S

        self.turn_on_all_leds()
        while self.state >= Constants.FS_STARTED:
            self.auto_pilot_active = True

            if current_state != self.state:
                self.auto_pilot.fc_state_did_change(current_state, self.state)
                current_state = self.state
                event_time = time.time()
                sub_state = 0

            left = loop_seconds - (time.time() - last_read_time)
            sum_idle += left; counter += 1
            if left > 0:
                time.sleep(left)
            elif left < 0:
                negative += 1

            #flashing leds
            if self.state > Constants.FS_STARTED:
                if led_timer == 0:
                    self.turn_on_red_led()
                    self.turn_off_green_led()
                elif led_timer == 250: #1s as the loop time is 4ms
                    self.turn_on_green_led()
                    self.turn_off_red_led()
                led_timer += 1
                if led_timer >= 500:
                    led_timer = 0

            now_time = time.time()
            compensations = self.auto_pilot.loop(current_state, now_time - last_read_time)
            if current_state >= Constants.FS_PREFLIGHT_CHK:
                adjustment = 0
                if current_state > Constants.FS_HOME and current_state < Constants.FS_EMERGENCY and self.auto_pilot.reached_critical_battery_level():
                    self.home()
                if current_state == Constants.FS_PREFLIGHT_CHK:
                    compensations[0] = compensations[1] = compensations[2] = compensations[3] = 0
                    if sub_state == 0:
                        result = self.auto_pilot.preflight_check()
                        logging.info("Preflight check: " + self.auto_pilot.format_preflight_code(result))
                        if result <= AutoPilot.PC_WARNING_MAX_: #warnings only
                            sub_state += 1
                            self.speed += (self.configuration.takeoff_value - self.speed) * 3 // 4
                        else: #critical error, for example, altitude not available
                            self.nav_to_state(Constants.FS_STARTED)
                    elif sub_state == 1 and (now_time - event_time >= 0.1):
                        sub_state += 1
                        self.speed = self.configuration.takeoff_value
                        self.nav_to_state(Constants.FS_TAKING_OFF)
                elif current_state == Constants.FS_TAKING_OFF:
                    if (sub_state == 0):
                        alt = self.auto_pilot.get_drone_altitude()
                        #reached expected height
                        if (alt < Constants.READING_UNKNOWN) and (alt >= Constants.MAX_ALTITUDE):
                            sub_state += 1
                            self.hover()
                        elif self.configuration.is_mode_outdoor() and (now_time - event_time >= 1.5):
                            #maybe altimeter malfunction
                            sub_state += 1
                            self.hover()
                elif current_state >= Constants.FS_HOVERING and current_state <= Constants.FS_MOVE_RIGHT:
                    adjustment = self.auto_pilot.hold()
                elif current_state == Constants.FS_FLY_HIGH:
                    adjustment = self.auto_pilot.vertical(now_time - event_time, False)
                elif current_state == Constants.FS_FLY_LOW:
                    adjustment = self.auto_pilot.vertical(now_time - event_time, True)
                elif current_state == Constants.FS_LANDING:
                    if sub_state == 0:
                        if self.auto_pilot.is_landed():
                            sub_state = 1
                        else:
                            adjustment = self.auto_pilot.vertical(now_time - event_time, True)
                    else:
                        sub_state += 1
                        compensations[0] = compensations[1] = compensations[2] = compensations[3] = 0
                        if sub_state > 10:
                            self.speed = self.configuration.idle_value #this will chnage the state to FS_STARTED
                        else:
                            self.speed = (self.configuration.idle_value + self.configuration.takeoff_value) // 2
                elif current_state == Constants.FS_HOME:
                    #The above loop() call may have applied a yaw on it
                    if self.auto_pilot.is_close_to_home(): #shall raise the height first?
                        self.landing()
                        adjustment = self.auto_pilot.hold()
                
                if current_state == Constants.FS_EMERGENCY:
                    self.speed = self.configuration.takeoff_value - int((now_time - event_time) * 20) // 2 * 2 #decrease 20 pwm every second                    
                elif self.auto_pilot.is_altimeter_failed():
                    self.nav_to_state(Constants.FS_EMERGENCY)

                #activate the new speed
                if self.speed <= self.MIN_VALUE:
                    self.stop(True)
                elif self.speed <= self.configuration.idle_value:
                    self.speed = self.configuration.idle_value
                    self.reset_compensation()
                    self.activate_speed()
                    self.nav_to_state(Constants.FS_STARTED)

                    self.turn_on_all_leds()
                else:
                    adjustment += self.auto_pilot.determine_battery_compensation()
                    self.motor_compensation[0] = compensations[0] + adjustment
                    self.motor_compensation[1] = compensations[1] + adjustment
                    self.motor_compensation[2] = compensations[2] + adjustment
                    self.motor_compensation[3] = compensations[3] + adjustment
                    if self.speed > self.configuration.takeoff_value:
                        self.speed = self.configuration.takeoff_value
                    self.activate_speed()
                    #if self.auto_pilot.more_info:
                    #    logging.info("Speed: " + str(self.speed)+","+str(self.motor_compensation))
                                
            last_read_time = now_time #while

        self.auto_pilot.stop()
        self.auto_pilot_active = False
        self.turn_off_all_leds()

        if counter <= 0:
            counter = 1
        logging.info("Auto pilot finished, avg idle time: " + str(int(sum_idle/counter*1000)) +"ms, overtime: " + str(negative)+" of " + str(counter) + "(" + str(round(100 * negative / counter, 2))  +"%), errors: " + self.auto_pilot.format_err_code())

    def training_daemon(self):
        logging.info("Training started")
        self.interrupted = False

        self.configuration.alt_p_gain = self.auto_pilot.assistant.p_gain
        self.configuration.alt_i_gain = self.auto_pilot.assistant.i_gain
        self.configuration.alt_d_gain = self.auto_pilot.assistant.d_gain
        self.auto_pilot.configuration_did_change()
        self.auto_pilot.assistant.will_start()
        #waiting for the drone to take off
        while self.state == Constants.FS_STARTED:
            time.sleep(1)
        while self.state >= Constants.FS_STARTED and not self.interrupted:
            time.sleep(1)
            if self.auto_pilot.assistant.shall_land():
                self.landing()
                self.auto_pilot.assistant.finish_round()
                if self.auto_pilot.assistant.shall_stop():
                    time.sleep(6)
                    self.stop(force=False, user_interactive=False)
                    time.sleep(2)
                else:
                    self.configuration.alt_p_gain = self.auto_pilot.assistant.p_gain
                    self.configuration.alt_i_gain = self.auto_pilot.assistant.i_gain
                    self.configuration.alt_d_gain = self.auto_pilot.assistant.d_gain
                    self.auto_pilot.configuration_did_change()
                    #in started state, leds are on
                    time.sleep(16)
                    for i in range(2):
                        self.turn_off_red_led()
                        self.turn_off_green_led()
                        time.sleep(1)
                        self.turn_on_red_led()
                        self.turn_on_green_led()
                        time.sleep(1)
                    self.takeoff()
        self.turn_off_red_led()
        self.turn_off_green_led()
        logging.info("Training finished")
