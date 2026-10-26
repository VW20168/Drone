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
    bool read(long dtMs, double *dta1, double *dta2, double *dta3, double *dta4);
    bool readCompass(double *dta1, double *dta2, double *dta3);
    void calibrate();
    void calibrateCompass();
    
    private:
    MPU *pMPU;
    
    double anglePitch, angleRoll, angleYaw;
    double gyroPitchInput, gyroYawInput;
    double accX, accY, accZ;
    double gyroPitch, gyroRoll, gyroYaw;

    double compassX, compassY, compassZ;
    double compassXInput,compassYInput, compassZInput;
    

    double accXBase, accYBase, accZBase;
    double gyroRollBase, gyroPitchBase, gyroYawBase;
    double compassScaleX, compassScaleY, compassScaleZ;
    double compassOffsetX, compassOffsetY, compassOffsetZ;
    
    void resetCalibrations(bool, bool);
    void saveCalibrations();
    void load();
    void calculateCompassCalibration(double minx, double maxx, double miny, double maxy, double minz, double maxz);
};


#endif

