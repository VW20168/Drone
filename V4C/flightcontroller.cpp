#include "drone.h"
#include "flightcontroller.h"

FlightController::FlightController(int mode) {
    pMotor1 = new Motor(PIN_MOTOR1); //was -100
    pMotor2 = new Motor(PIN_MOTOR2);
    pMotor3 = new Motor(PIN_MOTOR3);
    pMotor4 = new Motor(PIN_MOTOR4);

    //i2c power
    PiPIN::powerUp(26);
    PiPIN::powerUp(17);

    pRedLed = new PiPIN(PIN_RED_LED, PI_OUTPUT);
    pGreenLed = new PiPIN(PIN_GREEN_LED, PI_OUTPUT);

    mState = FS_POWERED_ON;
    mSpeed = 0;
    mMotorCompensations[0]=mMotorCompensations[1]=mMotorCompensations[2]=mMotorCompensations[3]=0;
    pMutex = new mutex();

    mRedLedOn = mGreenLedOn = false;
    turnOnAllLeds();
    pConfig = new Configuration(mode);
    pAutoPilot = new AutoPilot(pConfig);

    pingReceived = false;
    lastStopTime = 0;
    autoPilotActive = false;
    turnOffAllLeds();

    frameDataPtr = NULL;
    frameDataLen = frameDataLeft = 0;

    counter = 0;
    enhancedStatus = false;
    log_info(LOGTAG, "******Flight Controller started, mode %d******", mode);
}

FlightController::~FlightController() {
    delete pMotor1;
    delete pMotor2;
    delete pMotor3;
    delete pMotor4;
    delete pMutex;

    delete pRedLed;
    delete pGreenLed;

    delete pAutoPilot;
    delete pConfig;
}

void FlightController::turnOnRedLed(){
    if (!mRedLedOn) {
        mRedLedOn = true;
        pRedLed->gpioWrite(HIGH);
    }
}

void FlightController::turnOffRedLed() {
    if (mRedLedOn) {
        mRedLedOn = false;
        pRedLed->gpioWrite(LOW);
    }
}

void FlightController::turnOnGreenLed() {
    if (!mGreenLedOn) {
        mGreenLedOn = true;
        pGreenLed->gpioWrite(HIGH);
    }
}

void FlightController::turnOffGreenLed() {
    if (mGreenLedOn) {
        mGreenLedOn = false;
        pGreenLed->gpioWrite(LOW);
    }
}

void FlightController::turnOnAllLeds() {
    turnOnRedLed();
    turnOnGreenLed();
}

void FlightController::turnOffAllLeds() {
    turnOffRedLed();
    turnOffGreenLed();
}

bool FlightController::onCommand(char *value) {
    bool pingCmd = (strcmp(value, "ping") == 0);
    if (!pingCmd) //not ping command
        log_info("DroneCmd", "%s", value);

    bool processed = true;
    if (pingCmd)
        cmdPing();
    else if (strcmp(value, "calibrate") == 0)
        cmdCalibrate(CAL_ESC);
    else if (strcmp(value, "cali_imu") == 0)
        cmdCalibrate(CAL_IMU);
    else if (strcmp(value, "cali_mag") == 0)
        cmdCalibrate(CAL_COMPASS);
    else if (strcmp(value, "start") == 0)
        cmdStart();
    else if (strcmp(value, "takeoff") == 0)
        cmdTakeoff();
    else if (strcmp(value, "landing") == 0)
        cmdLanding();
    else if (strcmp(value, "turn_left") == 0)
        cmdTurnLeft();
    else if (strcmp(value, "turn_right") == 0)
        cmdTurnRight();
    else if (strcmp(value, "move_forward") == 0)
        cmdMoveForward();
    else if (strcmp(value, "move_backward") == 0)
        cmdMoveBackward();
    else if (strcmp(value, "move_left") == 0)
        cmdMoveLeft();
    else if (strcmp(value, "move_right") == 0)
        cmdMoveRight();
    else if (strcmp(value, "hover") == 0)
        cmdHover();
    else if (strcmp(value, "home") == 0)
        cmdHome();
    else if (strcmp(value, "+") == 0)
        cmdIncSpeed();
    else if (strcmp(value, "-") == 0)
        cmdDecSpeed();
    else if (strcmp(value, "stop") == 0) {
        mInterrupted = true;
        cmdStop();
    }
    else if (strcmp(value, "reset") == 0)
        cmdReset();
    else if (strcmp(value, "shutdown") == 0)
        cmdShutdown();
    else if (strcmp(value, "cfg") == 0) {//Client requested to download the current configuration
        enhancedStatus = true;
    }
    else {
        processed = false;
    }
    return processed;
}

void FlightController::onConfig(char *configurations) {
    string options(configurations);

    bool pidChanged = pConfig->parse(options);
    if (pidChanged)
        pAutoPilot->configurationDidChange();
}

//create a new thread to calibrate ESCs, IMU or the compass
void FlightController::cmdCalibrate(int target) {
    if (mState == FS_POWERED_ON || mState == FS_CALIBRATED || mState == FS_STOPPED) {
        nav2State(FS_CALIBRATING);
        onCalibrating = target;
        pthread_create(&theCalibrationThread, NULL, startCalibration, this);
    }
}

void FlightController::cmdPing() {
    pingReceived = true;
}

void FlightController::doCalibration() {
    turnOnAllLeds();
    if (onCalibrating == CAL_ESC) {
        if (!pConfig->isModeOutdoor()) {
            //calibrate all motors at the same time
            Motor* motors[]={pMotor1, pMotor2, pMotor3, pMotor4};
            for (int i = 0; i < 4; ++i)
                motors[i]->gpioServo(0, 0);
            Utils::delayMs(5000);
            for (int i = 0; i < 4; ++i)
                motors[i]->gpioServo(0, PWM_MIN);
            Utils::delayMs(5000);
            for (int i = 0; i < 4; ++i) {
                motors[i]->gpioServo(0, PWM_MAX);
            }
            Utils::delayMs(5000);
            for (int i = 0; i < 4; ++i)
                motors[i]->gpioServo(0, 0);
            Utils::delayMs(2000);
            for (int i = 0; i < 4; ++i)
                motors[i]->gpioServo(0, PWM_MIN);
            Utils::delayMs(2000);
        }
    }
    else if (onCalibrating == CAL_IMU) {
        pAutoPilot->calibrateImu();
    }
    else if (onCalibrating == CAL_COMPASS) {
        pAutoPilot->calibrateCompass();
    }
    nav2State(FS_CALIBRATED);
    turnOffAllLeds();
}

void FlightController::nav2State(int state) {
    if (mState == state)
        return;
    if ((mState == FS_STOPPED) && (state == FS_STOPPING))
        return;
    mState = state;
}

/*static*/ void* FlightController::startCalibration(void *arg) {
    FlightController *theController = (FlightController*)arg;
    theController->doCalibration();
    return NULL;
}

/*static*/ void* FlightController::startAutoPilot(void *arg) {
    FlightController *theController = (FlightController*)arg;
    theController->autoPilotDaemon();
    return NULL;
}

/*static*/ void* FlightController::startWatchDog(void *arg) {
    FlightController *theController = (FlightController*)arg;
    theController->watchdog();
    return NULL;
}

/*static*/ void* FlightController::startTraining(void *arg) {
    FlightController *theController = (FlightController*)arg;
    theController->trainingDaemon();
    return NULL;
}

char* FlightController::getStatus(long *data_size) {
    if (frameDataLeft <= 0) {
        char *imgData = (char*)pAutoPilot->getLastFrame(&frameDataLen);
        if (imgData && (frameDataLen > 0)) {
            frameDataPtr = (char*)malloc (frameDataLen);
            memcpy (frameDataPtr, imgData, frameDataLen);
            frameDataLeft = frameDataLen;
        }
    }
    if (enhancedStatus) {
        sprintf(szBuf, "cfg=%s", pConfig->toString());
        enhancedStatus = false;
    }
    else {
        sprintf(szBuf, "sts=%d;pwm=%d;%s",  mState, mSpeed, pAutoPilot->getStatus());
    }
    long taken = strlen(szBuf) + 2;
    long available = sizeof(szBuf) - taken;
    if (frameDataLeft > 0 && available > 0) {
        if (available > frameDataLeft)
            available = frameDataLeft;
        long pos = frameDataLen - frameDataLeft;
        memcpy(&szBuf[taken], &frameDataPtr[pos], available);
        frameDataLeft -= available;
        if (frameDataLeft <= 0) {
            frameDataLen = frameDataLeft = 0;
            free(frameDataPtr);
            frameDataPtr = NULL;
            szBuf[taken-1]=(char)2;
        }
        else {
            szBuf[taken-1]=(char)1;
        }
        *data_size = taken + available;
    }
    else {
        *data_size = taken;
        szBuf[taken-1]=(char)0;
    }
    return szBuf;
}

void FlightController::activateSpeed() {
    pMutex->lock();

    pMotor1->setSpeed(mSpeed + mMotorCompensations[0]);
    pMotor3->setSpeed(mSpeed + mMotorCompensations[2]);
    pMotor2->setSpeed(mSpeed + mMotorCompensations[1]);
    pMotor4->setSpeed(mSpeed + mMotorCompensations[3]);

    pMutex->unlock();
}

void FlightController::cmdStart() {
    if (mState != FS_POWERED_ON && mState != FS_CALIBRATED && mState != FS_STOPPED)
        return;
    if (Utils::currentTimeMs() - lastStopTime <= 5000)
        return;

    counter = 0;
    nav2State(FS_STARTING);
    //arm ESCs
    if ((pConfig->getQuirk() & QUIRK_BEEPING) == 0) {
        Motor* motors[]={pMotor1, pMotor2, pMotor3, pMotor4};
        for (int i = 0; i < 4; ++i)
            motors[i]->gpioServo(0, 0);
        Utils::delayMs(500);
        for (int i = 0; i < 4; ++i)
            motors[i]->gpioServo(0, PWM_MAX);
        Utils::delayMs(500);
        for (int i = 0; i < 4; ++i)
            motors[i]->gpioServo(0, PWM_MIN);
        Utils::delayMs(2000);
    }

    mSpeed = pConfig->getIdleSpeed();
    resetCompensation();
    activateSpeed();

    pingReceived = false;
    nav2State(FS_STARTED);
    mActiveState = mState;

    //start the auto pilot and watchdog threads
    pthread_create(&theAutoPilotThread, NULL, startAutoPilot, this);
    Utils::delayMs(1000); //waiting for the sensor to be ready
    pthread_create(&theWatchDogThread, NULL, startWatchDog, this);
    if (pConfig->isModeTraining()) {
        pthread_create(&theTrainingThread, NULL, startTraining, this);
    }
}

void FlightController::cmdTakeoff() {
    if (mState == FS_STARTED)
        nav2State(FS_PREFLIGHT_CHK);
}

void FlightController::cmdIncSpeed() {
    if (mState >= FS_HOVERING)
        nav2State(FS_FLY_HIGH);
    else if (mState == FS_STARTED) {
        mSpeed += pConfig->getNudgeSpeed();
        if (mSpeed > pConfig->getTakeoffSpeed())
            mSpeed = pConfig->getTakeoffSpeed();
        activateSpeed();
    }
}

void FlightController::cmdDecSpeed() {
    if (mState >= FS_HOVERING)
        nav2State(FS_FLY_LOW);
    else if (mState == FS_STARTED) {
        mSpeed -= pConfig->getNudgeSpeed();
        if (mSpeed < PWM_MIN)
            stop();
        else
            activateSpeed();
    }
}
//the received command will be processed by the auto pilot
void FlightController::cmdLanding() {
    if (allowManeuver() || mState == FS_HOME)
        nav2State(FS_LANDING);
}

void FlightController::cmdTurnLeft() {
    if (allowManeuver())
        nav2State(FS_TURN_LEFT);
}

void FlightController::cmdTurnRight() {
    if (allowManeuver())
        nav2State(FS_TURN_RIGHT);
}

void FlightController::cmdMoveForward() {
    if (allowManeuver())
        nav2State(FS_MOVE_FORWARD);
}

void FlightController::cmdMoveBackward() {
    if (allowManeuver())
        nav2State(FS_MOVE_BACKWARD);
}

void FlightController::cmdMoveLeft() {
    if (allowManeuver())
        nav2State(FS_MOVE_LEFT);
}

void FlightController::cmdMoveRight() {
    if (allowManeuver())
        nav2State(FS_MOVE_RIGHT);
}

void FlightController::cmdHome() {
    if (allowManeuver()) {
        if (pAutoPilot->isAutoNavigationAvailable())
            nav2State(FS_HOME);
        else
            nav2State(FS_LANDING);
    }
}

void FlightController::cmdHover() {
    if (allowManeuver() || mState == FS_TAKING_OFF)
        nav2State(FS_HOVERING);
}

//unconditionally stop the drone
void FlightController::stop() {
    nav2State(FS_STOPPED);
    mSpeed = 0;
    resetCompensation();
    activateSpeed();
    lastStopTime = Utils::currentTimeMs();
}

void FlightController::cmdStop() {
    if (mState == FS_POWERED_ON || mState == FS_STOPPED || mState == FS_STOPPING)
        return;

    if (mState == FS_STARTED)
        stop();
    else if (mState == FS_TAKING_OFF)
        cmdHover();
    else if (mState == FS_LANDING)
        nav2State(FS_STOPPING);
    else if (mState >= FS_HOME)//hovering etc
        cmdLanding();
}

bool FlightController::allowManeuver() {
    return mState >= FS_HOVERING;
}

void FlightController::resetCompensation() {
    mMotorCompensations[0]=mMotorCompensations[1]=mMotorCompensations[2]=mMotorCompensations[3]=0;
}

//reset done configuration, such as the pid settings
void FlightController::cmdReset() {
    pConfig->reset();
    pAutoPilot->configurationDidChange();
}

//The pi zero is going to shutdown, here stop all motors
void FlightController::cmdShutdown() {
    if (mState != FS_SHUTDOWN) {
        nav2State(FS_SHUTDOWN);
        mSpeed = 0;
        resetCompensation();
        activateSpeed();
        turnOffAllLeds();
   }
}

//pings are expected to be received every second. If not received, abort the flight
void FlightController::watchdog() {
    log_info(LOGTAG, "Watchdog started");
    pingReceived = autoPilotActive = false;
    Utils::delayMs(2000);
    long counter = 0;
    while (mState >= FS_STARTED) {
        if (pingReceived)
            counter = 0;
        else {
            counter += 1;
            log_error(LOGTAG, "Ping not received: %d, state: %d", counter, mState);
            if (counter > 1) {
                if (autoPilotActive)
                    cmdStop();
                else if (counter > 5) { //hard stop, expect user to shutdown the drone
                    resetCompensation();
                    mSpeed = pConfig->getIdleSpeed() + (pConfig->getTakeoffSpeed() - pConfig->getIdleSpeed()) / 2;
                    activateSpeed();
                    break;
                }
            }
        }
        pingReceived = autoPilotActive = false;
        Utils::delayMs(2000);
    }
    log_info(LOGTAG, "*Watchdog stopped");
}

void FlightController::autoPilotDaemon() {
    log_info(LOGTAG, "Auto pilot started");

    autoPilotActive = true;
    pAutoPilot->start();//start all sensors, reset all PIDs
    Utils::delayMs(1000); //for the compass to be ready

    int takeOffSpeed = pConfig->getTakeoffSpeed();
    long lastReadTime, nowTime;
    long emergencyTime = 0;
    int subState = 0; long eventTime = 0;
    long ledTimer = 0, sumIdle = 0, counter = 0, negative = 0;
    long loopMs = LOOP_TIMER_MS;
    int compensations[4];

    mActiveState = mState;
    lastReadTime = nowTime = Utils::currentTimeMs();
    turnOnAllLeds();
    while (mState >= FS_STARTED || mState == FS_STOPPING) {
        autoPilotActive = true;

        if (mActiveState != mState) {
            //reset PIDs if needed, or save the current drone location as home location
            pAutoPilot->fcStateDidChange(mActiveState, mState);
            mActiveState = mState;
            eventTime = Utils::currentTimeMs();//when the drone state changed
            subState = 0;
            log_info(LOGTAG, "to state %d(%s)", mActiveState, formatDroneState(mActiveState).c_str());
        }

        long left = loopMs - (Utils::currentTimeMs() - lastReadTime);
        if (left > 0)
            Utils::delayMs(left);
        if (mActiveState > FS_PREFLIGHT_CHK) {
            sumIdle += left;
            ++counter;
            if (left < 0) {
               ++negative;
           }
        }
        nowTime = Utils::currentTimeMs();
        //flashing leds
        if (mState > FS_STARTED) {
            //flashing red and green leds
            if (ledTimer == 0) {
                turnOnRedLed();
                turnOffGreenLed();
            }
            else if (ledTimer == 250) { //1s as the loop time is 4ms
                turnOffRedLed();
                turnOnGreenLed();
            }
            if (++ledTimer >= 500)
                ledTimer = 0;
        }
        //outputs of pitch, roll and yaw PIDs to stabilize the drone
        pAutoPilot->loop(mActiveState, nowTime - lastReadTime, nowTime - eventTime, &compensations[0], &compensations[1], &compensations[2], &compensations[3]);
        if (mActiveState >= FS_PREFLIGHT_CHK) {
            int adjustment = 0;//vertical speed compensation to hold the drone altitude
            //make sure IMU and altimeter are working properly. Others may be warnings, for example, GPS is not working
            if (mActiveState == FS_PREFLIGHT_CHK) {
                compensations[0] = compensations[1] = compensations[2] = compensations[3] = 0;
                takeOffSpeed = pConfig->getTakeoffSpeed(); //refresh the take off speed
                if (subState == 0) {
                    log_info(LOGTAG, "Taking off with battery compensation %d", pAutoPilot->determineBatteryCompensation());

                    int result = pAutoPilot->preflightCheck();
                    log_error(LOGTAG, "Preflight check result: %s", pAutoPilot->formatPreflight(result).c_str());
                    if (result <= PC_WARNING_MAX_) { //warnings only
                        ++subState;
                        mSpeed += (takeOffSpeed - mSpeed) * 3 / 4;
                        //there is a 500ms delay in AutoPilot.preflightCheck()
                        eventTime = nowTime;
                    }
                    else { //critical errors, for example, altitude not available, abort the take off
                        nav2State(FS_STARTED);
                    }
                }
                else if (subState == 1 && (nowTime - eventTime >= 100)) {
                    //100ms after the speed was increased a little bit, increase it to take off speed
                    ++subState;
                    mSpeed = takeOffSpeed;
                    //save the drone home altitude. Once the drone is in the air, we use the (drone_altitude - home_altitude) as the drone altitude
                    pAutoPilot->prepare2takeoff();
                    nav2State(FS_TAKING_OFF);//wait to reach the max altitude allowed
                }
            }
            else if (mActiveState == FS_TAKING_OFF) {
                if (subState == 0) {
                    double alt = pAutoPilot->getDroneAltitude();
                    //reached expected height
                    if (alt >= pConfig->getMaxAltitude()) {
                        ++subState;
                        cmdHover();
                    }
                    else if (nowTime - eventTime >= 8000) { //milliseconds
                        ++subState;
                        if (alt > 0.08) { //8cm
                            cmdHover();
                        }
                        else {//abort the takeoff, altimeter malfunction
                            nav2State(FS_STOPPING);
                        }
                     }
                }
            }
            else if (mActiveState >= FS_HOVERING && mActiveState <= FS_MOVE_RIGHT)
                adjustment = pAutoPilot->hold();
            else if (mActiveState == FS_FLY_HIGH)
                adjustment = pAutoPilot->vertical(nowTime - eventTime, false);
            else if (mActiveState == FS_FLY_LOW)
                adjustment = pAutoPilot->vertical(nowTime - eventTime, true);
            else if (mActiveState == FS_LANDING) {
                if (pAutoPilot->isLanded()) {
                    mSpeed = pConfig->isModeOutdoor() ? PWM_MIN : pConfig->getIdleSpeed();//this will change the state to FS_STARTED
                }
                else {
                    adjustment = pAutoPilot->vertical(nowTime - eventTime, true);
                }
            }
            else if (mActiveState == FS_HOME) {
                //The above loop() call may have applied a yaw on it
                if  (pAutoPilot->isCloseToHome()) { //shall raise the height first?
                    cmdLanding();
                    adjustment = pAutoPilot->hold();
                }
            }

            //activate the new speed
            if (mSpeed <= PWM_MIN)
                stop();
            else if (mSpeed <= pConfig->getIdleSpeed()) {
                mSpeed = pConfig->getIdleSpeed();
                resetCompensation();
                activateSpeed();
                nav2State(FS_STARTED);
                turnOnAllLeds();
            }
            else {
                adjustment += pAutoPilot->determineBatteryCompensation();
                mMotorCompensations[0] = compensations[0] + adjustment;
                mMotorCompensations[1] = compensations[1] + adjustment;
                mMotorCompensations[2] = compensations[2] + adjustment;
                mMotorCompensations[3] = compensations[3] + adjustment;

                if (mSpeed > takeOffSpeed)
                    mSpeed = takeOffSpeed;
                activateSpeed();
                //log_info("MOTOR", "%d:%d,%d,%d,%d", mSpeed, mMotorCompensations[0], mMotorCompensations[1], mMotorCompensations[2], mMotorCompensations[3]);
            }
        }
        else if (mActiveState == FS_STOPPING) {
            if (subState == 0) {
                mSpeed += (mMotorCompensations[0] + mMotorCompensations[1] + mMotorCompensations[2] + mMotorCompensations[3]) / 4;
                ++subState;
            }
            long timeElapsed = ((nowTime - eventTime) / 50 * 50); //every 50ms
            int adjustment = (int)(timeElapsed / 50.0f); //reduce 20 pwm every second

            mMotorCompensations[0] = compensations[0] - adjustment;
            mMotorCompensations[1] = compensations[1] - adjustment;
            mMotorCompensations[2] = compensations[2] - adjustment;
            mMotorCompensations[3] = compensations[3] - adjustment;

            int minimal = mMotorCompensations[0];
            if (mMotorCompensations[1] < minimal)
                minimal = mMotorCompensations[1];
            if (mMotorCompensations[2] < minimal)
                minimal = mMotorCompensations[2];
            if (mMotorCompensations[3] < minimal)
                minimal = mMotorCompensations[3];
            if (mSpeed + minimal <= pConfig->getIdleSpeed()) {
                stop();
            }
            else {
                activateSpeed();
            }
        }
        //exception checks
        if ((mActiveState > FS_PREFLIGHT_CHK || mActiveState == FS_STOPPING) && !pAutoPilot->isFlightAttitudeGood()) {
            stop(); //emergency stop to prevent more damages, for example, the drone flipped over
            log_error(LOGTAG, "Failed to stabilize the drone, flight aborted");
        }
        else if (pAutoPilot->reachedCriticalCpuTemperature() || pAutoPilot->reachedCriticalBatteryLevel()) {
            if (emergencyTime <= 0) {
                emergencyTime = Utils::currentTimeMs();
                if (mActiveState > FS_HOME)
                    cmdHome();
                else
                    cmdStop();
                log_error(LOGTAG, "CPU temperture too high or battery too low");
            }
        }
        else if (pAutoPilot->isAltimeterFailed())
            cmdHome();
        else if (mActiveState == FS_HOVERING && pConfig->isModeTraining() && (nowTime - eventTime >= TRAINING_ROUNDS_DURATION_MS))//training mode
            mFinishingTrainingRound = true;

        lastReadTime = nowTime;
    }//while drone started
    log_info(LOGTAG, "final state %d(%s)", mState, formatDroneState(mState).c_str());

    pAutoPilot->stop();
    autoPilotActive = false;
    turnOffAllLeds();

    if (counter <= 0)
        counter = 1;
    char szBuf[1024];
    sprintf(szBuf, "*Auto pilot finished, avg idle time: %dms, overtime %ld(%.3f%%), error %s",  int(sumIdle/counter),  negative, (100.0f * negative / counter), pAutoPilot->formatErrCode().c_str());
    log_info(LOGTAG, szBuf);
    fprintf(stderr, "%s\n", szBuf);
}

void FlightController::trainingDaemon() {
    log_info(LOGTAG, "Training daemon started");
    mInterrupted = mFinishingTrainingRound = false;

    //waiting for the drone to take off
    while (mActiveState == FS_STARTED)
        Utils::delayMs(200);

    int trainingRounds = 0;
    pAutoPilot->trainingWillStart(trainingRounds);
    while (mActiveState >= FS_STARTED && !mInterrupted) {
        Utils::delayMs(200);
        if (mFinishingTrainingRound) {
            cmdLanding();
            Utils::delayMs(10);
            pAutoPilot->finishTrainingRound();
            if (++trainingRounds >= TRAINING_ROUNDS) {
                Utils::delayMs(6000);//enough time to land
                cmdStop();
                Utils::delayMs(2000);
            }
            else {
                for (int i = 0; i < 16 && !mInterrupted; ++i) {
                    Utils::delayMs(1000);
                }
                //flash leds, the training will start soon
                for (int i = 0; i < 2 && !mInterrupted; ++i) {
                    turnOffAllLeds();
                    Utils::delayMs(1000);
                    turnOnAllLeds();
                    Utils::delayMs(1000);
                }
                pAutoPilot->trainingWillStart(trainingRounds);
                if (!mInterrupted)
                    cmdTakeoff();
            }
            mFinishingTrainingRound = false;
        }
    }
    pAutoPilot->finishTraining();
    turnOffAllLeds();
    log_info(LOGTAG, "*Training daemon stopped");
}

string FlightController::formatDroneState(int state) {
    string info = "Unknown";
    switch (state) {
        case FS_SHUTDOWN: info="Shutdown"; break;
        case FS_POWERED_ON: info="Powered on"; break;
        case FS_CALIBRATING: info="Calibrating"; break;
        case FS_CALIBRATED: info="Calibrated"; break;
        case FS_STOPPING: info="Stopping"; break;
        case FS_STOPPED: info="Stopped"; break;
        case FS_STARTING: info="Starting"; break;
        case FS_STARTED: info="Started"; break;
        case FS_PREFLIGHT_CHK: info="Preflight Check"; break;
        case FS_TAKING_OFF: info="Taking Off"; break;
        case FS_LANDING: info="Landing"; break;
        case FS_HOME: info="Home"; break;
        case FS_HOVERING: info="Hovering"; break;
        case FS_TURN_LEFT: info="Turn Left"; break;
        case FS_TURN_RIGHT: info="Turn Right"; break;
        case FS_MOVE_FORWARD: info="Move Forward"; break;
        case FS_MOVE_BACKWARD: info="Move Backward"; break;
        case FS_MOVE_LEFT: info="Move Left"; break;
        case FS_MOVE_RIGHT: info="Move Right"; break;
        case FS_FLY_HIGH: info="Fly High"; break;
        case FS_FLY_LOW: info="Fly Low"; break;
    }
    return info;
}
