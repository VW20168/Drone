#ifndef __ALT_PID_H__
#define __ALT_PID_H__

#define BUF_SIZE    18 //was 30

class AltPID {
    public:
    AltPID(float p, float i, float d, float max);
    virtual ~AltPID();
    void reset();
    float compute(float pidSetpoint, float pidInput);

    private:
    float pGain, iGain, dGain;
    float min, max;
    float iSum, lastDError;

    float parachuteThrottle;
    int parachuteRotatingMemLocation;
    float parachuteAltitudePrevious;
    float parachuteBuffer[BUF_SIZE+5];

};

#endif

