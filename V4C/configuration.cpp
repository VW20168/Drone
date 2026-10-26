
#include <algorithm>
#include <cctype>
#include <locale>
#include <iostream>
#include <fstream>
#include "drone.h"
#include "configuration.h"

#define LINE_DELIMITER      "\n"
#define PID_VALUE_SEP       ","
#define NAME_VALUE_SEP      ":" //do not use '=' here as commands are in format of "cmd=ping"


// trim from start (in place)
inline void ltrim(std::string &s) {
    s.erase(s.begin(), std::find_if(s.begin(), s.end(), [](unsigned char ch) {
        return !std::isspace(ch);
    }));
}

// trim from end (in place)
inline void rtrim(std::string &s) {
    s.erase(std::find_if(s.rbegin(), s.rend(), [](unsigned char ch) {
        return !std::isspace(ch);
    }).base(), s.end());
}

Configuration::Configuration(int mode) {
    mMode = mode;
    reset();
}

Configuration::~Configuration() {
}


void Configuration::reset() {
    //when the battery voltage reaches the MIN_BATTERY_VOLTAGE, the compensation is 40
    mIdealBatteryFactor = 40.0 / (MAX_BATTERY_VOLTAGE - MIN_BATTERY_VOLTAGE);

    mTakeoffSpeed   = 1480; //was 1450
    mIdleSpeed      = 1220; //was 1200
    mNudgeSpeed     = 10;
    mMovementAngle  = 6;
    mTurningYaw     = 10;

    pGain = 1.48 * SCALE;
    iGain = 0.04 * SCALE;
    dGain = 100.0 * SCALE;

    pGainYaw = 5.0 * SCALE;
    iGainYaw = 0.01 * SCALE;
    dGainYaw = 0;

    pGainAlt = 66 * SCALE; //was 50, 40
    iGainAlt = 0.02 * SCALE;
    dGainAlt = 230 * SCALE;

    pGainAnchor = 1 * SCALE;
    iGainAnchor = 0;//always be zero
    dGainAnchor = 0.2 * SCALE;
    //trial: 1 - try gps location holding when hovering
    mQuirk = mTrial = 0;
    load();

    //hardware configuration
    mAltimeter = (isModeTraining() || isModeIndoor()) ? "VL53L1X" : "MS5611";
    mMaxAlt = (isModeTraining() || isModeIndoor()) ? 0.3 : 2.0; //in meters

    cout << "!!!Mode " << mMode << ", max altitude " << mMaxAlt << "m, altimeter " << mAltimeter << endl;
}

bool Configuration::isModeIndoor() {
    return mMode == MODE_INDOOR;
}

bool Configuration::isModeOutdoor() {
    return mMode == MODE_OUTDOOR;
}

bool Configuration::isModeTraining() {
    return mMode == MODE_TRAINING;
}

int Configuration::getIdleSpeed() {
    return mIdleSpeed;
}

int Configuration::getTakeoffSpeed() {
    return mTakeoffSpeed;
}

int Configuration::getNudgeSpeed() {
    return mNudgeSpeed;
}

void Configuration::getPitchRollPid(double *p, double *i, double *d) {
    *p = pGain;
    *i = iGain;
    *d = dGain;
}

void Configuration::getYawPid(double *p, double *i, double *d) {
    *p = pGainYaw;
    *i = iGainYaw;
    *d = dGainYaw;
}

void Configuration::getAltPid(double *p, double *i, double *d) {
    *p = pGainAlt;
    *i = iGainAlt;
    *d = dGainAlt;
}

void Configuration::getAnchorPid(double *p, double *i, double *d) {
    *p = pGainAnchor;
    *i = iGainAnchor;
    *d = dGainAnchor;
}

double Configuration::getMovementAngle() {
    return mMovementAngle;
}

int Configuration::getTurningYaw() {
    return mTurningYaw;
}

double Configuration::getMaxAltitude() {
    return mMaxAlt;
}

double Configuration::getIdealBatteryFactor() {
    return mIdealBatteryFactor;
}

double Configuration::getHorizonalSpeed() {
    return 0.5; //meters per second
}

double Configuration::getVerticalSpeed() {
    return 0.25; //meters per second
}

bool Configuration::isTryingGpsHolding() {
    return (mTrial & TRIAL_GPS_HOLDING) != 0;
}

bool Configuration::isTryingVisualOdometry() {
    return (mTrial & TRIAL_VISUAL_ODOMETRY) != 0;
}


char* Configuration::toString() {
    sprintf(szBuf, "trial:%d\nmvangle:%.1f\ntakeoff:%d", mTrial, mMovementAngle, mTakeoffSpeed);
    return szBuf;
}

string Configuration::getAltimeter() {
    return mAltimeter;
}

void Configuration::setTrial(int trial) {
    mTrial = trial;
}

int Configuration::getQuirk() {
    return mQuirk;
}

//returns true if pid settings changed
bool Configuration::parse(string options) {
    bool pidChanged = false;

    try {
        std::vector<std::string> lines = Utils::split(options, LINE_DELIMITER);
        for (unsigned int i = 0; i < lines.size(); ++i) {
            string line = lines[i];
            int pos = line.find(NAME_VALUE_SEP);
            if (pos > 0) {
                string name = line.substr(0, pos);
                string value= line.substr(pos+1);
                if (name == "idle")  //idle speed
                    mIdleSpeed = stoi(value);
                else if (name == "takeoff") //take off speed
                    mTakeoffSpeed = stoi(value);
                else if (name == "trial") //traial features
                    mTrial = stoi(value);
                else if (name =="mvangle")
                    mMovementAngle = stof(value);
                else if (name == "quirk") //esc type
                    mQuirk = stoi(value);
                else if (parsePid(name, value)) {
                    pidChanged = true;
                }
            }
        }
    }
    catch (...) {
    }

    return pidChanged;
}

bool Configuration::parsePid(string name, string value) {
    bool pidChanged = false;

    int pos1 = value.find(PID_VALUE_SEP);
    int pos2 = value.find(PID_VALUE_SEP, pos1 + 1);
    if (pos2 > pos1 && pos1 > 0) {
        string value1 = value.substr(0, pos1);
        string value2 = value.substr(pos1 + 1, pos2 - pos1 - 1);
        string value3 = value.substr(pos2 + 1);
        ltrim(value1); rtrim(value1);
        ltrim(value2); rtrim(value2);
        ltrim(value3); rtrim(value3);
        if (name == "pid") {
            pGain = stof(value1);
            iGain = stof(value2);
            dGain = stof(value3);
            pidChanged = true;
        }
        else if (name == "yaw_pid") {
            pGainYaw = stof(value1);
            iGainYaw = stof(value2);
            dGainYaw = stof(value3);
            pidChanged = true;
        }
        else if (name == "alt_pid") {
            pGainAlt = stof(value1);
            iGainAlt = stof(value2);
            dGainAlt = stof(value3);
            pidChanged = true;
        }
        else if (name == "training") {
            if (isModeTraining()) {
                pGainAlt = stof(value1);
                iGainAlt = stof(value2);
                dGainAlt = stof(value3);
                
                cout << "@Loading training configurations: " << pGainAlt << "," << iGainAlt << "," << dGainAlt << endl;
            }
        }
    }

    return pidChanged;
}

void Configuration::load() {
    ifstream file("drone.ini");
    string line;

    if (file.is_open()) {
        while (getline(file, line)) {
            int pos = line.find("#");
            if (pos >= 0)
                line = line.substr(0, pos);
            if (!line.empty()) {
                cout << "@" << line << endl;
                parse(line);
            }
        }
        file.close();
    }
}
/*
**configurations for Drone #2**

quirk:1
idle:1380
takeoff:1600
pid:1.48, 0.04, 200
alt_pid:180, 0.02, 300

*/
