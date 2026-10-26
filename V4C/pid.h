#ifndef __PID_H__
#define __PID_H__

class PID {
    public:    
    PID(double p, double i, double d, double max);
    virtual ~PID();
    void reset();
    double compute(double pidSetpoint, double pidInput);
    
    private:
    double pGain, iGain, dGain;
    double min, max;
    double iSum, lastDError;
    bool started;
};


#endif

