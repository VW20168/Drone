#ifndef __MOTOR_H__
#define __MOTOR_H__

class Motor {
    public:    
    Motor(int pin);
    virtual ~Motor();
    void calibrate();
    void arm();
    void setSpeed(int speed);
    
    private:
    int mPin;
    int mSpeed;
    
};



#endif

