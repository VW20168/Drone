#ifndef __MPU9250_H__
#define __MPU9250_H__

#include "mpu.h"

class MPU9250 : public MPU {
public:
    MPU9250(int bus);
    ~MPU9250();

    bool readAccGyro(int16_t *buffer);
    bool readMagnetometer(double *mx, double *my, double *mz);
private:
    int i2cHandle, magHandle;
    double mres, magXcoef, magYcoef, magZcoef;
    
    void initialize();
    int16_t assemble(uint8_t high, uint8_t low);
    void configAK8963(uint8_t mode, uint8_t mfs);
};

#endif
