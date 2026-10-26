#ifndef __PI_PWM_H__
#define __PI_PWM_H__

#include <iostream>
#include <fstream>
#include <string>
#include <unistd.h>


class PiPWM {
    public:
    PiPWM(int pin);
    ~PiPWM();
    
    void gpioServo(int pin, int pulseWidth);
    
    private:    
    int chip_;
    int channel_;
    std::string basePath_;
    
    bool writeToFile(const std::string& path, const std::string& value);
    bool exportPWM();
    bool unexportPWM();
    bool setPeriod(int period_ns);
    bool setDutyCycle(int duty_cycle_ns);
    bool enablePWM(bool enable);
    
};

#endif
