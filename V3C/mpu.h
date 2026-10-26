#ifndef __MPU_H__
#define __MPU_H__

#include <cstdint>

class MPU {
    public:    
    MPU(int bus);
    virtual ~MPU();

    virtual bool readAccGyro(int16_t *buffer);
    virtual bool readMagnetometer(float *mx, float *my, float *mz);
};


#endif

