#ifndef __FLIGHT_CONTROLLER_H__
#define __FLIGHT_CONTROLLER_H__

#include <pthread.h>
#include <mutex>
#include "motor.h"
#include "autopilot.h"
#include "configuration.h"
#include "pipin.h"

#define CAL_ESC         1
#define CAL_IMU         2
#define CAL_COMPASS     3

class FlightController {
    public:
    FlightController(int mode = MODE_INDOOR);
    virtual ~FlightController();
    bool onCommand(char *cmd);
    void onConfig(char *configurations);
    void cmdStop();

    char* getStatus(long *data_size);
    
    private:
    Motor *pMotor1;
    Motor *pMotor2;    
    Motor *pMotor3;
    Motor *pMotor4;
    PiPIN *pRedLed;
    PiPIN *pGreenLed;
    Configuration *pConfig;
    AutoPilot *pAutoPilot;
    char szBuf[512];
    bool enhancedStatus;
    
    char *frameDataPtr;
    long frameDataLen, frameDataLeft;

    int mSpeed;
    int mMotorCompensations[4];
    bool mInterrupted, mFinishingTrainingRound;
    
    int mState, mActiveState; //flight controller/drone state
    void nav2State(int state);
    
    bool pingReceived, autoPilotActive;
    long lastStopTime;
    long counter;

    void stop();
    void resetCompensation();
    int onCalibrating;
    void doCalibration();
    void autoPilotDaemon();
    void watchdog();
    void trainingDaemon();
    pthread_t theCalibrationThread, theAutoPilotThread, theWatchDogThread, theTrainingThread;
    static void* startCalibration(void *arg);
    static void* startAutoPilot(void *arg);
    static void* startWatchDog(void *arg);
    static void* startTraining(void *arg);
    
    bool mRedLedOn, mGreenLedOn;
    void turnOnRedLed();
    void turnOffRedLed();
    void turnOnGreenLed();
    void turnOffGreenLed();
    void turnOnAllLeds();
    void turnOffAllLeds();
    void activateSpeed();
    bool allowManeuver();
    
    void cmdCalibrate(int);
    void cmdPing();
    void cmdStart();
    void cmdTakeoff();
    void cmdIncSpeed();
    void cmdDecSpeed();
    void cmdLanding();
    void cmdTurnLeft();
    void cmdTurnRight();
    void cmdMoveForward();
    void cmdMoveBackward();
    void cmdMoveLeft();
    void cmdMoveRight();
    void cmdHome();
    void cmdHover();
    void cmdShutdown();
    void cmdReset();//reset drone configuration
    
    string formatDroneState(int state);
    
    std::mutex *pMutex;
};

#endif


