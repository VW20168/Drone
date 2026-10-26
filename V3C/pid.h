#ifndef __PID_H__
#define __PID_H__

class PID {
    public:    
    PID(float p, float i, float d, float max);
    virtual ~PID();
    void reset();
    float compute(float pidSetpoint, float pidInput);
    
    private:
    float pGain, iGain, dGain;
    float min, max;
    float iSum, lastDError;
    bool started;
};


#endif

