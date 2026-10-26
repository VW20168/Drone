#ifndef __MPU_H__
#define __MPU_H__

#include <cstdint>
#include "pii2c.h"

class MPU : public PiI2C {
    public:    
    MPU(int bus);
    virtual ~MPU();

    virtual bool readAccGyro(int16_t *buffer);
    virtual bool readMagnetometer(double *mx, double *my, double *mz);
};


#endif

