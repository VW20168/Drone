#ifndef __ALT_PID_H__
#define __ALT_PID_H__

#define BUF_SIZE    18 //was 30

class AltPID {
    public:
    AltPID(double p, double i, double d, double max);
    virtual ~AltPID();
    void reset();
    double compute(double pidSetpoint, double pidInput);

    private:
    double pGain, iGain, dGain;
    double min, max;
    double iSum, lastDError;

    double parachuteThrottle;
    int parachuteRotatingMemLocation;
    double parachuteAltitudePrevious;
    double parachuteBuffer[BUF_SIZE+5];

};

#endif

