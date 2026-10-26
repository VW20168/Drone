#ifndef __MOTOR_H__
#define __MOTOR_H__

#include "pipwm.h"

class Motor : public PiPWM {
    public:    
    Motor(int pin);
    Motor(int pin, int compensation);
    virtual ~Motor();
    void calibrate();
    void arm();
    void setSpeed(int speed);
    
    private:
    int mPin;
    int mSpeed;
    int mCompensation;
};



#endif

