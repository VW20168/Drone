#include <cmath>
#include <fstream>
#include "drone.h"
#include "autopilot.h"


#define DECLINATION  -10.6
#define MD_NA        -720 //moving direction is not applicable
#define TRAINING_DATA_NA      65535

//course lock
#define CL_NA                   0
#define CL_MV_LRFB              1  //move left, right, forward or backward
#define CL_HOME                 3
#define CL_HOVERING             4

AutoPilot::AutoPilot (Configuration *p) {
    pConfig = p;
    errCode = 0;

    pidRoll = pidPitch = pidYaw = pidAnchor = NULL;
    pidAlt = NULL;
    configurationDidChange();

    pImu = new IMU(pConfig);
    pAltimeter = new Altimeter(pConfig);
    pBattery = new Battery(pConfig);
    pGps = new GPS_NEO6M("/dev/serial0", 9600); //M100(115200) or NEO6M(9600)
    pGps->initialize();

    anglePitch = angleRoll = angleYaw = READING_UNKNOWN;
    latitude = longitude= READING_UNKNOWN;
    heading  = READING_UNKNOWN;
    temperature = READING_UNKNOWN;
    altUpdated = gpsUpdated = false;

    homeLatitude = homeLongitude = READING_UNKNOWN;
    hoveringAltitude = 0; pidOutputAltitude = 0;
    yawSetpoint = 0;

    voltage = READING_UNKNOWN;

    moreInfo = false;
    counter = 0;
    altFailures = 0;

    radian2degree =  180.0 / M_PI;
    mStarted = false;

    ptrTrainingData = NULL;
}

AutoPilot::~AutoPilot() {
    delete pImu;
    delete pBattery;
    delete pAltimeter;

    delete pidPitch;
    delete pidRoll;
    delete pidYaw;
    delete pidAlt;

    pGps->closePort();
    delete pGps;

    if (ptrTrainingData)
        free(ptrTrainingData);
}

void AutoPilot::start() {
    if (mStarted)
        return;
    mStarted = true;
    counter = 0;
    altFailures = 0; //consecutive altimeter errors
    maxPitch = maxRoll = 0;
    errCode = AP_ERR_OK;
    reset(true, true);

    pImu->start();
    pAltimeter->start();
    pBattery->start();

    pthread_create(&theDaemonThread, NULL, startDaemon, this);
}

//PID settings may have changed
void AutoPilot::configurationDidChange() {
    if (pidRoll)
        delete pidRoll;
    if (pidPitch)
        delete pidPitch;
    if (pidYaw)
        delete pidYaw;
    if (pidAlt)
        delete pidAlt;
    if (pidAnchor)
        delete pidAnchor;

    float p, i, d;
    pConfig->getPitchRollPid(&p, &i, &d);
    pidRoll = new PID(p, i, d, PID_MAX_IMU);
    pidPitch = new PID(p, i, d, PID_MAX_IMU);

    pConfig->getYawPid(&p, &i, &d);
    pidYaw = new PID(p, i, d, PID_MAX_IMU);

    pConfig->getAltPid(&p, &i, &d);
    pidAlt = new AltPID(p, i, d, PID_MAX_ALT);

    pConfig->getAnchorPid(&p, &i, &d);
    pidAnchor = new PID(p, i, d, pConfig->getMovementAngle() * 1.6f); //5 => 8 degrees
}

//drone heading, location and battery voltage etc
char* AutoPilot::getStatus() {
    if (mStarted) {
        sprintf(szBuf, "heading=%d;lat=%f;lon=%f;alt=%.1f;bat=%.1f", heading, latitude, longitude, altitude, voltage);
    }
    else {
        szBuf[0] = '\0';
    }
    return szBuf;
}

void AutoPilot::stop() {
    mStarted = false;//this will stop the background thread
}

//reset PIDs
void AutoPilot::reset(bool resetMpu, bool resetAltimeter) {
    if (resetMpu) {
        pidRoll->reset();
        pidPitch->reset();
        pidYaw->reset();
    }
    if (resetAltimeter)
        pidAlt->reset();
        
    pidAnchor->reset();        
}

void AutoPilot::setErrCode (int code) {
    errCode |= code;
}

string AutoPilot::formatPreflight(int code) {
    string msg = "OK";
    if (code == PC_WARNING_GPS_NA)
        msg = "Warning: GPS not available";
    else if (code == PC_WARNING_COMPASS_NA)
        msg = "Warning: Compass not available";
    else if (code == PC_ERR_IMU)
        msg = "Error: IMU";
    else if (code == PC_ERR_BATTERY)
        msg = "Error: Battery";
    else if (code == PC_ERR_ALT)
        msg = "Error: Altimeter";
    return msg;
}

string AutoPilot::formatErrCode() {
    string msg = "";
    if (errCode & AP_ERR_IMU)
        msg += "IMU,";
    if (errCode & AP_ERR_COMPASS)
        msg += "Compass,";
    if (errCode & AP_ERR_ALTIMETER)
        msg += "Altimeter,";
    if (errCode & AP_ERR_BATTERY)
        msg += "Battery,";
    if (errCode & AP_ERR_GPS)
        msg += "GPS,";

    if (msg == "")
        msg = "None";
    else if (msg.back() == ',')
        msg = msg.substr(0, msg.length() - 1);
    return msg;
}

void AutoPilot::calibrateImu() {
    pImu->calibrate();
}

void AutoPilot::calibrateCompass() {
    pImu->calibrateCompass();
}

bool AutoPilot::isCompassAvailable() {
    return heading < READING_UNKNOWN;
}

bool AutoPilot::isGpsAvailable() {
    return homeLatitude < READING_UNKNOWN && homeLongitude < READING_UNKNOWN;
}

bool AutoPilot::isAutoNavigationAvailable() {
    return isCompassAvailable() && isGpsAvailable();
}

int AutoPilot::preflightCheck() {
    altUpdated = gpsUpdated = false;
    Utils::delayMs(500);//waiting for the barameter reading to be available

    maxRoll = maxPitch = 0;
    //GPS location
    homeLatitude = latitude;
    homeLongitude= longitude;

    //errors: imu readings
    if (anglePitch >= READING_UNKNOWN || angleRoll >= READING_UNKNOWN)
        return PC_ERR_IMU;
    //altitude
    if (!altUpdated)
        return PC_ERR_ALT;
    //battery voltage
    if (voltage >= READING_UNKNOWN || voltage < MIN_BATTERY_VOLTAGE)
       return PC_ERR_BATTERY;

    //warnings
    if (!isGpsAvailable())
        return PC_WARNING_GPS_NA;
    if (!isCompassAvailable())
        return PC_WARNING_COMPASS_NA;

    return PC_OK;
}

void AutoPilot::prepare2takeoff() {
    pAltimeter->prepare2takeoff();
    pImu->prepare2takeoff(); //reset imu readings
    yawSetpoint = 0;
}

void AutoPilot::fcStateDidChange(int previousStatus, int newState) {
    reset(true, true);

    pidOutputAltitude = 0;
    if (newState == FS_HOVERING || newState == FS_LANDING) {
        hoveringAltitude = altitude;
    }
    if (isAutoNavigationAvailable()) {
        lastMovingAngle = pConfig->getMovementAngle();
        //current drone location
        waypointLatitude = latitude;
        waypointLongitude= longitude;
    }
}

//calculate compensations from pitch, roll and yaw
void AutoPilot::loop(int fcState, long dtMs, long elapsed, int *esc1, int *esc2, int *esc3, int *esc4) {
    bool moreInfo = (counter <= 0);
    if (++counter >= 50) //every 200ms
        counter = 0;

    //compass and IMU are in the same sensor, so read compass here
    if (moreInfo || fcState == FS_HOME) {
        float xInput, yInput, zInput;
        if (pImu->readCompass(&xInput, &yInput, &zInput)) {
            determineHeading(xInput, yInput);
        }
        else {
            setErrCode(AP_ERR_COMPASS);
        }
    }

    if (!pImu->read(dtMs, &anglePitch, &angleRoll, &angleYaw, &temperature)) {
        setErrCode(AP_ERR_IMU);
    }
    if (abs(anglePitch) > maxPitch)
        maxPitch = abs(anglePitch);
    if (abs(angleRoll) > maxRoll)
        maxRoll = abs(angleRoll);

    float rollSetpoint = 0, pitchSetpoint = 0;
    if (fcState >= FS_TAKING_OFF) {
        int turningYaw = pConfig->getTurningYaw();
        float movingDirection = MD_NA; //0-360
        float movingAngle = pConfig->getMovementAngle();
        double targetLatitude = 0, targetLongitude = 0;
        int course_lock = CL_NA;

        if (fcState == FS_MOVE_FORWARD) {
            movingDirection = 0;
            if (isAutoNavigationAvailable())
                course_lock = CL_MV_LRFB;
        }
        else if (fcState == FS_MOVE_BACKWARD) {
            movingDirection = 180;
            if (isAutoNavigationAvailable())
                course_lock = CL_MV_LRFB;
        }
        else if (fcState == FS_MOVE_LEFT) {
            movingDirection = 270;
            if (isAutoNavigationAvailable())
                course_lock = CL_MV_LRFB;
        }
        else if (fcState == FS_MOVE_RIGHT) {
            movingDirection = 90;
            if (isAutoNavigationAvailable())
                course_lock = CL_MV_LRFB;
        }
        else if (fcState == FS_HOME) {
            if (isAutoNavigationAvailable())
                course_lock = CL_HOME;
        }
        else if (fcState == FS_TURN_LEFT) {
            yawSetpoint -= turningYaw * LOOP_TIMER_MS / 1000.0f;
            if (yawSetpoint < -180)
                yawSetpoint += 360;
        }
        else if (fcState == FS_TURN_RIGHT) {
            yawSetpoint += turningYaw * LOOP_TIMER_MS / 1000.0f;
            if (yawSetpoint > 180)
                yawSetpoint -= 360;
        }
        else if (fcState == FS_HOVERING) {
            if (isAutoNavigationAvailable())
                course_lock = CL_HOVERING;
        }
        /*run this python script as a unit test for the movement waypoint calcuation
            import math

            radian2degree =  180.0 / 3.14159265
            #current location
            latitude,longitude = 39.31386719764081,-77.3336316806884
            #home position
            waypointLatitude,waypointLongitude=39.30868185025992, -77.36303616969464
            horizontal_speed = 0.5 #meters per second

            print(waypointLatitude,",",waypointLongitude)
            print("=================>")

            movement = 0 #0-farward; 90-move right;270-left;180-backward
            heading = 45 #in degrees
            for tm in range(10):
                elapsed = (tm + 1) * 1000 #every second
                away = elapsed / 1000.0 * horizontal_speed / 111000
                targetLatitude = waypointLatitude  + away * math.cos((movement + heading)/radian2degree)
                targetLongitude= waypointLongitude + away * math.sin((movement + heading)/radian2degree)
                
                latDiff = waypointLatitude - targetLatitude
                lonDiff = waypointLongitude- targetLongitude
                distance = math.sqrt(latDiff * latDiff + lonDiff * lonDiff) * 111000 #degrees to meters    
                print(targetLatitude,",",targetLongitude, ", distance", round(distance, 1))

        */
        if (course_lock > CL_NA) {
            if (gpsUpdated) {
                gpsUpdated = false;
                if (course_lock == CL_MV_LRFB) {
                    //1 degree of latitude or longitude is ~111000m
                    float away = elapsed / 1000.0f * pConfig->getHorizonalSpeed() / 111000;
                    double radian = (movingDirection+heading)/radian2degree;
                    targetLatitude = waypointLatitude  + away * cos(radian);
                    targetLongitude= waypointLatitude  + away * sin(radian);
                }
                else if (course_lock == CL_HOME) {
                    targetLatitude = homeLatitude;
                    targetLongitude= homeLongitude;                    
                }
                else if (course_lock == CL_HOVERING) {
                    targetLatitude = waypointLatitude;
                    targetLongitude= waypointLongitude;                    
                }
                float _distance = getDistance(targetLatitude, targetLongitude);
                movingAngle = pidAnchor->compute(0, _distance);
                if (movingAngle < 0)
                    movingAngle = 0;
                lastMovingAngle = movingAngle;
            }
            else {
                movingAngle = lastMovingAngle;
            }
            movingDirection = towards(targetLatitude, targetLongitude, pConfig->isTryingGpsHolding());
        }
        //if trial mode, movingDirection will be MD_NA(-720)
        if (moreInfo && movingDirection >= 0) {
            float allowed = 1.0f * elapsed / 1000; //1 degree per second
            if (movingAngle > allowed)
                movingAngle = allowed;
            //set pitch setpoint and roll setpoint
            //radian2degree =  180.0 / M_PI;
            double yaw  = movingDirection / radian2degree; //degree to radian
            double pitch= movingAngle / radian2degree; //degree to radian
            double x = cos(-yaw) * sin(pitch);
            double y = sin(-yaw) * sin(pitch);
            //double z = cos(pitch);
            double _roll = asin(y);
            double _pitch= asin(x / cos(_roll));
            pitchSetpoint= (float)(_pitch * radian2degree);
            rollSetpoint = (float)(_roll  * radian2degree); //Drone roll is counterclockwise

            log_info("DroneDIR", "direction %.3f, angle %.3f, pitch %.3f, roll %.3f", movingDirection, movingAngle, pitchSetpoint, rollSetpoint);
        }
    }

    float pidOutputRoll = pidRoll->compute(rollSetpoint, angleRoll);
    float pidOutputPitch= pidPitch->compute(pitchSetpoint, anglePitch);
    float pidOutputYaw  = pidYaw->compute(yawSetpoint, angleYaw);

    *esc1 = (int)(    pidOutputPitch - pidOutputRoll - pidOutputYaw); //esc 1 (front-right - CCW).
    *esc2 = (int)(0 - pidOutputPitch - pidOutputRoll + pidOutputYaw); //esc 2 (rear-right - CW).
    *esc3 = (int)(0 - pidOutputPitch + pidOutputRoll - pidOutputYaw); //esc 3 (rear-left - CCW).
    *esc4 = (int)(    pidOutputPitch + pidOutputRoll + pidOutputYaw); //esc 4 (front-left - CW).
    log_info("PIDOUTPUT",  "%d,%d,%d,%d, IMU readings %.3f,%.3f,%.3f", *esc1, *esc2, *esc3, *esc4, anglePitch, angleRoll, angleYaw);
}

//the direction(angle, relative to north, clockwise) to the target position
/*run this python script as a unit test
    import math
    #current location
    latitude,longitude = 39.31386719764081,-77.3336316806884
    #home position
    targetLatitude,targetLongitude=39.30868185025992, -77.36303616969464
    # Return the arc tangent of y/x in radians
    angleCorrection = int(180.0 * math.atan2((targetLongitude-longitude), (targetLatitude - latitude)) / 3.14);
    if angleCorrection < 0:
        angleCorrection += 360
    print(angleCorrection)
*/
float AutoPilot::towards(double targetLatitude, double targetLongitude, bool enabled) {
    float angleCorrection = 0;
    if (latitude != targetLatitude) {
        angleCorrection = int(atan2(targetLongitude-longitude, targetLatitude - latitude) * radian2degree);
    }
    else if (longitude > targetLongitude) {
        angleCorrection = 270;
    }
    else if (longitude < targetLongitude) {
        angleCorrection = 90;
    }
    else {
        return MD_NA;
    }

    float movingDir = angleCorrection - heading;
    if (movingDir < 0)
        movingDir += 360;
    else if (movingDir >= 360)
        movingDir -= 360;

    //log_info("DroneDIR", "target %f,%f,from %f,%f,heading=%d,angleCorrection=%f,moving=%f",
    //                        targetLatitude, targetLongitude, latitude, longitude, heading, angleCorrection, movingDir);

    return enabled ? movingDir : MD_NA;
}

//altitude hold
int AutoPilot::hold() {
    if (altUpdated) {
        altUpdated = false;
        float lastAltitude = altitude;
        pidOutputAltitude = 0 - int(pidAlt->compute(hoveringAltitude, lastAltitude));
        if (pConfig->isModeTraining() && ptrTrainingData) {
            ptrTrainingData[trainingRound * TRAINING_MAX_ROUND_DATA + pointer++] = (int)((lastAltitude - hoveringAltitude) * 100); //meter to centimeter
            log_info("PIDALT", "PID output,%d, current alt,%.3f, target,%.3f", pidOutputAltitude, lastAltitude, hoveringAltitude);
        }
    }
    /*
     if more_info:
        diff = round(last_altitude - hovering_altitude, 3)
        symbol = ""
        if diff > 0:
            symbol = "+"
        logging.info("PID_ALT_HOVERING,"+str(pid_output_altitude)+", "+symbol+str(diff)+",alt="+str(last_altitude))
    */
    return pidOutputAltitude;
}

//@param dt: delta time in milliseconds
int AutoPilot::vertical(float dt, bool descending) {
    if (dt <= 0) {
        if (descending)
            pidOutputAltitude = -5;
        else
            pidOutputAltitude = 5;
    }
    else if (altUpdated) {
        altUpdated = false;

        float lastAltitude = altitude, altitudeSetpoint = 0;
        if (descending)  //target deceleration speed is -0.25m/s
            altitudeSetpoint = hoveringAltitude - dt / 1000 * pConfig->getVerticalSpeed();
        else
            altitudeSetpoint = hoveringAltitude + dt / 1000 * pConfig->getVerticalSpeed();

        if (altitudeSetpoint > pConfig->getMaxAltitude())
            altitudeSetpoint = pConfig->getMaxAltitude();
        pidOutputAltitude = 0 - int(pidAlt->compute(altitudeSetpoint, lastAltitude));

        log_info("PIDVertical", "PID output,%d, current alt,%.3f, target, %.3f", pidOutputAltitude, lastAltitude, altitudeSetpoint);
    }
    return pidOutputAltitude;
}

bool AutoPilot::isFlightAttitudeGood() {
    return maxPitch < 20 && maxRoll < 20;
}

bool AutoPilot::isCloseToHome() {
    return getDistance(homeLatitude, homeLongitude) <= 2;
}

float AutoPilot::getDistance(double targetLatitude, double targetLongitude) {
    //https://aviation.stackexchange.com/questions/97359/easy-way-to-calculate-distance-between-two-gps-coordinates
    double latDiff = latitude - targetLatitude;
    double lonDiff = longitude- targetLongitude;
    float distance = (float)(sqrt(latDiff * latDiff + lonDiff * lonDiff) * 111000); //degrees to meters
    return distance;
}

bool AutoPilot::isLanded() {
    return (altitude <= 0.05); //5 cm
}

bool AutoPilot::isAltimeterFailed() {
    return altFailures > 10;
}

float AutoPilot::getDroneAltitude() {
    return altitude;
}

int AutoPilot::determineBatteryCompensation() {
    if (voltage >= MAX_BATTERY_VOLTAGE) //12.5v or reading_unknown
        return 0;

    int compensation = (int)((MAX_BATTERY_VOLTAGE - voltage) * pConfig->getIdealBatteryFactor());
    compensation &= 0xfe; //operator (x // 2) * 2

    //print("=====================>", compensation)
    return compensation;
}

bool AutoPilot::reachedCriticalBatteryLevel() {
    if (voltage >= READING_UNKNOWN)
        return false;
    return (voltage <= CRITICAL_BATTERY_VOLTAGE);
}

//drone heading: angle, relative to north, clockwise
void AutoPilot::determineHeading(float x, float y) {
    //ATAN2(y,x) returns the arc tangent of the two numbers x and y.
    double headingAngle = atan2(y, x) * 57.29578 + DECLINATION; //1 radian = 57.29578 degrees
    headingAngle = 270 - headingAngle;
    if (headingAngle >= 360)
        headingAngle -= 360;
    else if (headingAngle < 0)
        headingAngle += 360;
    heading = (int)(headingAngle);
}

void* AutoPilot::startDaemon(void *arg) {
    AutoPilot *thePilot = (AutoPilot*)arg;
    thePilot->positionDaemon();
    return NULL;
}

//background thread to read altitude, gps location and battery voltage
void AutoPilot::positionDaemon() {
    latitude = longitude = READING_UNKNOWN;
    altitude = READING_UNKNOWN;
    altUpdated = gpsUpdated = false;

    log_info(LOGTAG, "*Position daemon started");
    long loopMs = 20; //ms
    long nextTmBattery = 0,  nextTmGps = 0;
    while (mStarted) {
        long now = Utils::currentTimeMs();
        {//read drone altitude every 20ms
            //long at = Utils::currentTimeMs();
            if (pAltimeter->read(&altitude)) {
                altUpdated = true;
                altFailures = 0;
            }
            else {
                setErrCode(AP_ERR_ALTIMETER);
                ++altFailures;
            }
            //cout << "Step 1=" << (Utils::currentTimeMs() - at) << endl;
            //logging.info("** ALT: "+str(distance))
            //step2: 0.00018 seconds
        }
        if (now >= nextTmGps) {
            //long at = Utils::currentTimeMs();
            nextTmGps = now + 500; //ms, update GPS reading
            if (pGps->readData(&latitude, &longitude)) {
                gpsUpdated = true;
                log_info("GPS", "%f, %f", latitude, longitude);
            }
            else {
                setErrCode(AP_ERR_GPS);
            }
            //cout << "Step 2=" << (Utils::currentTimeMs() - at) << endl;
        }

        if (now >= nextTmBattery + 10) {
            //long at = Utils::currentTimeMs();
            nextTmBattery = now + 10000; //every 10s, update battery reading
            float reading = 0;
            if (pBattery->read(&reading))
                voltage = reading;
            else
                setErrCode(AP_ERR_BATTERY);
            //cout << "Step 3=" << (Utils::currentTimeMs() - at) << endl;
        }
        int idle = loopMs - (Utils::currentTimeMs() - now);
        if (idle > 0)
            Utils::delayMs(idle);
    }

    altUpdated = gpsUpdated = false;
    pAltimeter->stop();

    log_info(LOGTAG, "*Position daemon stopped, interval %ldms", loopMs);
}

//PID training support
void AutoPilot::trainingWillStart(int round) {
    cout << "Training started: round " << (round + 1) << endl;

    if (ptrTrainingData == NULL) {
        ptrTrainingData = (int*)malloc (TRAINING_MAX_ROUND_DATA * TRAINING_ROUNDS * sizeof (int));
    }
    for (int i = 0; i < TRAINING_MAX_ROUND_DATA; ++i)
        ptrTrainingData[round*TRAINING_MAX_ROUND_DATA+i] = TRAINING_DATA_NA;
    trainingRound = round;
    pointer = 0;
}

void AutoPilot::finishTrainingRound() {
    cout << "Training finished: round " << (trainingRound + 1) << endl;

    //current PID values as the output file name
    float _p = 0, _i = 0, _d = 0;
    pConfig->getAltPid(&_p, &_i, &_d);
    char szFileName[255];
    sprintf(szFileName, "P%dI%.1fD%d.csv", (int)_p,_i,(int)_d);
    //save to the file
    ofstream file(szFileName);
    if (!file.is_open()) {
        cout << "Failed to create the training result file: " << szFileName << endl;
        return;
    }
    cout << "Saving results to file: " << szFileName << endl;

    //R1,R2,R3,R4,R5
    //0, 0, 0, 0, 0,
    //0, 1, 0, 1, 1,
    //1, 1, 1, 2, 2,
    char szData[125];
    for (int j = 0; j < TRAINING_MAX_ROUND_DATA; ++j) {
        int data = 0;
        for (int i = 0; i <= trainingRound; ++i) {
            int err = ptrTrainingData[i * TRAINING_MAX_ROUND_DATA + j];
            if (err == TRAINING_DATA_NA) {
                strcpy (szData, ",");
            }
            else {
                ++data;
                sprintf(szData, "%d,", err);
            }
            file.write(szData, strlen(szData));
        }
        strcpy (szData, "\n");
        file.write(szData, strlen(szData));
        if (data < trainingRound)
            break;
     }

     file.close();
}

void AutoPilot::finishTraining() {
    cout << "Training finished." << endl;

    if (ptrTrainingData) {
        free(ptrTrainingData);
        ptrTrainingData = NULL;
    }
    trainingRound = pointer = 0;
}
