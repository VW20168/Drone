#ifndef __AUTOPILOT_H__
#define __AUTOPILOT_H__

#include <pthread.h>
#include "imu.h"
#include "battery.h"
#include "altimeter.h"
#include "pid.h"
#include "altpid.h"
#include "neo6m.h"

//preflight check result
#define PC_OK                   0
#define PC_WARNING_GPS_NA       1
#define PC_WARNING_COMPASS_NA   2
#define PC_WARNING_MAX_         50

#define PC_ERR_IMU          100
#define PC_ERR_ALT          101
#define PC_ERR_BATTERY      102

//runtime errors
#define AP_ERR_OK           0  //000000
#define AP_ERR_IMU          1  //000001
#define AP_ERR_COMPASS      2  //000010
#define AP_ERR_ALTIMETER    4  //000100
#define AP_ERR_BATTERY      16 //010000
#define AP_ERR_GPS          32 //100000

class AutoPilot {
    public:
    AutoPilot(Configuration *pConfig);
    virtual ~AutoPilot();
    
    void configurationDidChange();
    char* getStatus();
    
    void start();
    void stop();

    void calibrateImu();
    void calibrateCompass();
    int preflightCheck();
    void prepare2takeoff();
    
    string formatPreflight(int code);
    string formatErrCode();
    
    void fcStateDidChange(int previousStatus, int newState);
    void loop(int fc_state, long dtMs, long elspsed, int *esc1, int *esc2, int *esc3, int *esc4);
    int hold();
    int vertical(float dt, bool descending);
    bool isAutoNavigationAvailable();
    bool isCloseToHome();
    bool isLanded();
    bool isAltimeterFailed();
    float getDroneAltitude();
    int determineBatteryCompensation();
    bool reachedCriticalBatteryLevel();
    bool isFlightAttitudeGood();

    void trainingWillStart(int round);
    void finishTrainingRound();
    void finishTraining();
    
    private:
    bool mStarted;
    int errCode;
    IMU *pImu;
    Altimeter *pAltimeter;
    Battery *pBattery;
    GPS_NEO6M *pGps;
    
    Configuration *pConfig;
    PID *pidRoll, *pidPitch, *pidYaw;
    PID *pidAnchor;
    AltPID *pidAlt;
    
    float anglePitch, angleRoll, angleYaw;
    float maxPitch, maxRoll;
    float temperature;
    bool altUpdated, gpsUpdated;
    int heading;
    double longitude, latitude;
    float altitude, voltage;
    float hoveringAltitude;
    float yawSetpoint;
    float lastMovingAngle;
    double homeLatitude, homeLongitude;
    double waypointLatitude, waypointLongitude;
    int pidOutputAltitude;
        
    bool moreInfo;
    int counter;
    int altFailures;
    
    char szBuf[128];
    
    void reset(bool, bool);
    void setErrCode (int code);
    
    bool isCompassAvailable();
    bool isGpsAvailable();
    void determineHeading(float x, float y);
    float towards(double targetLatitude, double targetLongitude, bool enabled);
    float getDistance(double targetLatitude, double targetLongitude);
    static void* startDaemon(void *arg);
    pthread_t theDaemonThread;
    void positionDaemon();
    
    double radian2degree;

    int trainingRound;
    int *ptrTrainingData;
    int pointer;
};


#endif

