#!/usr/bin/python3

import pigpio
import os
import sys
import time

class Motor:
    def __init__(self, no, pin, pi):
        self.MIN_VALUE=1050
        self.MAX_VALUE=2200
        self.compensation = 0
        self.no = "#"+str(no)
        self.pin = pin
        self.pi = pi
        self.speed = 0
        self.pi.set_servo_pulsewidth(self.pin, 0)
        self.aborted = False

    def get_pin(self):
        return self.pin

    def calibrate(self):
        self.aborted = False
        self.pi.set_servo_pulsewidth(self.pin, 0)
        self.wait(5)
        self.pi.set_servo_pulsewidth(self.pin, self.MIN_VALUE)
        self.wait(5)
        if not self.aborted:
            self.pi.set_servo_pulsewidth(self.pin, self.MAX_VALUE)
        self.wait(5)
        self.pi.set_servo_pulsewidth(self.pin, 0)
        self.wait(2)
        self.pi.set_servo_pulsewidth(self.pin, self.MIN_VALUE)# arming
        self.wait(2)

    def arm(self):
        self.pi.set_servo_pulsewidth(self.pin, 0)
        time.sleep(0.5)
        self.pi.set_servo_pulsewidth(self.pin, self.MAX_VALUE)
        time.sleep(0.5)
        self.pi.set_servo_pulsewidth(self.pin, self.MIN_VALUE)
        time.sleep(0.8)
        self.speed = self.MIN_VALUE

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

    def abort(self):
        self.aborted = True

    def wait(self, seconds):
        while not self.aborted and seconds > 0:
            time.sleep(1)
            seconds -= 1

the_pi = pigpio.pi()
the_motor = Motor(1, 14, the_pi)

stop = False; motor_running = False;
step=10; pwm = 1130 #1630: take off; 1350: idle
print("===>Idle speed 1350, take off speed 1630. Range 0 or 500-2500")
try:
    while not stop:
        print("\n***Current speed setting:", str(pwm)+".", "Please select one cmd:")
        print(" (c)Calibrate")
        print(" (r)Run, start the motor")
        print(" (s)Stop the motor")
        print(" (+)Increase speed")
        print(" (-)Decrease speed")
        print(" (q)Quit")
        cmd = input("Enter your cmd:")
        cmd = cmd.strip()
        if cmd == "c":
            the_motor.set_speed(0)
            motor_running = False
            confirm = input("Please make sure the properller is NOT installed, enter 'yes' to continue:")
            if confirm == "yes":
                the_motor.calibrate()
        elif cmd == "r":
            if not motor_running:
                the_motor.set_speed(pwm)
                motor_running = True
        elif cmd == "s":
            the_motor.set_speed(0)
            motor_running = False
        elif cmd == "+" or cmd == "=":
            pwm += step
            if motor_running:
                the_motor.set_speed(pwm)
        elif cmd == "-":
            pwm -= step
            if motor_running:
                the_motor.set_speed(pwm)
        elif cmd == "q":
            stop = True
            the_motor.set_speed(0)
            motor_running = False
        else:
            print ("Unknown cmd:", cmd)
except KeyboardInterrupt:
    the_motor.abort()
    the_motor.set_speed(0)
