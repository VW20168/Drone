#ifndef __ALT_H__
#define __ALT_H__

#include "pii2c.h"

//base class for all altimeters, such as BMP280, MS5611
class ALT : public PiI2C {
    public:
    ALT(int bus);
    virtual ~ALT();

    virtual bool initialize();
    virtual void start();
    virtual void stop();
    virtual bool readDistance(double *distance);
    virtual bool readPressureAndTemperature(double *pressure, double *temperature);

};


#endif

