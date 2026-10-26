#ifndef __BATTERY_H__
#define __BATTERY_H__

#include "configuration.h"
#include "ina219.h"

class Battery {
    public:
    Battery(Configuration *config);
    virtual ~Battery();
    
    void start();
    bool read(float *dta);
    
    private:
    INA219 *pINA219;
    float voltage, lastReading;
};


#endif

