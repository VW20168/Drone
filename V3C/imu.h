#ifndef __IMU_H__
#define __IMU_H__

#include "configuration.h"
#include "mpu.h"

class IMU {
    public:
    IMU(Configuration *config);
    virtual ~IMU();
    
    void start();
    void prepare2takeoff();
    bool read(long dtMs, float *dta1, float *dta2, float *dta3, float *dta4);
    bool readCompass(float *dta1, float *dta2, float *dta3);
    void calibrate();
    void calibrateCompass();
    
    private:
    MPU *pMPU;
    
    float anglePitch, angleRoll, angleYaw;
    float gyroPitchInput, gyroYawInput;
    float accX, accY, accZ;
    float gyroPitch, gyroRoll, gyroYaw;

    float compassX, compassY, compassZ;
    float compassXInput,compassYInput, compassZInput;
    

    float accXBase, accYBase, accZBase;
    float gyroRollBase, gyroPitchBase, gyroYawBase;
    float compassScaleX, compassScaleY, compassScaleZ;
    float compassOffsetX, compassOffsetY, compassOffsetZ;
    
    void resetCalibrations(bool, bool);
    void saveCalibrations();
    void load();
    void calculateCompassCalibration(float minx, float maxx, float miny, float maxy, float minz, float maxz);
};


#endif

