#ifndef __MS5611_H__
#define __MS5611_H__

#include "alt.h"

class MS5611 : public ALT {
public:
    MS5611(uint8_t i2c_bus = 1);
    ~MS5611();
    bool initialize();
    bool readPressureAndTemperature(float *pressure, float *temperature);

private:
    uint8_t i2c_bus_;
    uint8_t i2c_address_;
    int i2cHandle;
    
    float C1, C2, C3, C4, C5, C6, D1, D2;
    float TEMP, PRES;
    bool readDigitalValues();
    
};

#endif

