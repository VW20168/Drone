#include "drone.h"
#include "motor.h"

Motor::Motor (int pin) {
    mPin = pin;
    mSpeed = 0;
    
    gpioSetMode(mPin, PI_OUTPUT);
}

Motor::~Motor() {
}

void Motor::calibrate() {
    gpioServo(mPin, 0);
    Utils::delayMs(5000);
    gpioServo(mPin, PWM_MIN);
    Utils::delayMs(5000);
    gpioServo(mPin, PWM_MAX);
    Utils::delayMs(5000);
    gpioServo(mPin, 0);
    Utils::delayMs(2000);
    gpioServo(mPin, PWM_MIN);
    Utils::delayMs(2000);
}

void Motor::arm() {
    gpioServo(mPin, 0);
    Utils::delayMs(500);
    gpioServo(mPin, PWM_MAX);
    Utils::delayMs(500);
    gpioServo(mPin, PWM_MIN);
    Utils::delayMs(500);
    mSpeed = PWM_MIN;
}

void Motor::setSpeed(int newSpeed) {
    if (newSpeed <= 0 || abs(newSpeed - mSpeed) >= 2) {
        mSpeed = newSpeed;
        //pigpio.error: 'pulsewidth not 0 or 500-2500'
        if (mSpeed < PWM_MIN)
            mSpeed = 0;
        else if (mSpeed > PWM_MAX)
            mSpeed = PWM_MAX;
        gpioServo(mPin, mSpeed);
    }
}

