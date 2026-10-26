#ifndef __CONFIGURATION_H__
#define __CONFIGURATION_H__

#define SCALE           1

#define PID_MAX_IMU     (60 * SCALE)
#define PID_MAX_ALT     (15 * SCALE) //was 25

///ESC beeps if no signal
#define QUIRK_BEEPING           1

#define TRIAL_GPS_HOLDING               1
#define TRIAL_VISUAL_ODOMETRY           2

using namespace std;

class Configuration {
    public:
    Configuration(int mode);
    virtual ~Configuration();

    bool isModeIndoor();
    bool isModeOutdoor();
    bool isModeTraining();
    double getMovementAngle();
    int getTurningYaw();
    double getMaxAltitude();
    double getIdealBatteryFactor();
    string getAltimeter();
    int getQuirk();

    void reset();
    bool parse(string);
    char *toString();

    int getTakeoffSpeed();
    int getIdleSpeed();
    int getNudgeSpeed();
    void getPitchRollPid(double *p, double *i, double *d);
    void getYawPid(double *p, double *i, double *d);
    void getAltPid(double *p, double *i, double *d);
    void getAnchorPid(double *p, double *i, double *d);

    double getHorizonalSpeed(); //move left, right etc
    double getVerticalSpeed(); //landing etc

    void setTrial(int trial);
    bool isTryingGpsHolding();
    bool isTryingVisualOdometry();

    private:
    int mMode;
    double mMaxAlt;
    string gps;

    bool mQuirk, mTrial;
    int mTakeoffSpeed;
    int mIdleSpeed;
    int mNudgeSpeed;
    double mMovementAngle;
    int mTurningYaw;
    double mIdealBatteryFactor;
    double pGain, iGain, dGain;
    double pGainYaw, iGainYaw, dGainYaw;
    double pGainAlt, iGainAlt, dGainAlt;
    double pGainAnchor, iGainAnchor, dGainAnchor;
    char szBuf[128];

    std::string mAltimeter;

    void load();
    bool parsePid(string name, string value);
};

#endif

