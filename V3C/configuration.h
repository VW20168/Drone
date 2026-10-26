#ifndef __CONFIGURATION_H__
#define __CONFIGURATION_H__

///ESC beeps if no signal
#define QUIRK_BEEPING           1

class Configuration {
    public:
    Configuration(int mode);
    virtual ~Configuration();

    bool isModeIndoor();
    bool isModeOutdoor();
    bool isModeTraining();
    float getMovementAngle();
    int getTurningYaw();
    float getMaxAltitude();
    float getIdealBatteryFactor();
    string getAltimeter();
    int getQuirk();

    void reset();
    bool parse(string);
    char *toString();

    int getTakeoffSpeed();
    int getIdleSpeed();
    int getNudgeSpeed();
    void getPitchRollPid(float *p, float *i, float *d);
    void getYawPid(float *p, float *i, float *d);
    void getAltPid(float *p, float *i, float *d);
    void getAnchorPid(float *p, float *i, float *d);

    float getHorizonalSpeed(); //move left, right etc
    float getVerticalSpeed(); //landing etc

    bool isTryingGpsHolding();

    private:
    int mMode;
    float mMaxAlt;
    string gps;

    bool mQuirk, mTrial;
    int mTakeoffSpeed;
    int mIdleSpeed;
    int mNudgeSpeed;
    float mMovementAngle;
    int mTurningYaw;
    float mIdealBatteryFactor;
    float pGain, iGain, dGain;
    float pGainYaw, iGainYaw, dGainYaw;
    float pGainAlt, iGainAlt, dGainAlt;
    float pGainAnchor, iGainAnchor, dGainAnchor;
    char szBuf[128];

    std::string mAltimeter;

    void load();
    bool parsePid(string name, string value);
};

#endif

