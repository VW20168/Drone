#ifndef __AUTOPILOT_H__
#define __AUTOPILOT_H__

#include <pthread.h>
#include "imu.h"
#include "battery.h"
#include "altimeter.h"
#include "pid.h"
#include "altpid.h"
#include "neo6m.h"
#include "cv/vision.h"

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
#define AP_ERR_CAMERA       8  //001000
#define AP_ERR_BATTERY      16 //010000
#define AP_ERR_GPS          32 //100000

class AutoPilot {
    public:
    AutoPilot(Configuration *pConfig);
    virtual ~AutoPilot();

    void configurationDidChange();
    char* getStatus();
    void *getLastFrame(long *data_size);

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
    int vertical(double dt, bool descending);
    bool isAutoNavigationAvailable();
    bool isCloseToHome();
    bool isLanded();
    bool isAltimeterFailed();
    double getDroneAltitude();
    int determineBatteryCompensation();
    bool reachedCriticalBatteryLevel();
    bool reachedCriticalCpuTemperature();
    bool isFlightAttitudeGood();

    void trainingWillStart(int round);
    void finishTrainingRound();
    void finishTraining();

    private:
    bool mStarted;
    int errCode;
    IMU *pImu;
    Altimeter *pAltimeter, *pBackupAltimeter;
    Battery *pBattery;
    GPS_NEO6M *pGps;

    Configuration *pConfig;
    PID *pidRoll, *pidPitch, *pidYaw;
    PID *pidAnchor, *pidVert;
    AltPID *pidAlt;

    double anglePitch, angleRoll, angleYaw;
    double maxPitch, maxRoll;
    double imuTemperature;
    double cpuTemperature;
    bool altUpdated, gpsUpdated, voUpdated;
    int heading;
    double longitude, latitude;
    double altitude, voltage, backupAltitude;
    double hoveringAltitude, objectAltitude;
    double yawSetpoint;
    double lastMovingAngle;
    double homeLatitude, homeLongitude;
    double waypointLatitude, waypointLongitude;
    int pidOutputAltitude;

    double xVoOffset,yVoOffset,zVoOffset;

    bool moreInfo;
    int counter;
    int altFailures;

    char szBuf[128];

    void reset(bool, bool);
    void setErrCode (int code);

    bool isCompassAvailable();
    bool isGpsAvailable();
    void determineHeading(double x, double y);
    double towards(double targetLatitude, double targetLongitude, bool enabled);
    double getDistance(double targetLatitude, double targetLongitude);
    static void* startDaemon(void *arg);
    static void* startCameraDaemon(void *arg);
    pthread_t theDaemonThread, theCameraThread;
    void positionDaemon();
    void cameraDaemon();

    double radian2degree;

    int trainingRound;
    int *ptrTrainingData;
    int pointer;

    Vision *pVision;
};


#endif

