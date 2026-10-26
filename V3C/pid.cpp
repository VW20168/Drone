
#include "pid.h"

PID::PID (float _p, float _i, float _d, float _max) {
    pGain = _p;
    iGain = _i;
    dGain = _d;
    max = _max;
    min = 0 - max;
    
    reset();
}

PID::~PID() {
}

void PID::reset() {
    iSum = 0;
    lastDError = 0;
    started = false;
}

float PID::compute(float pidSetpoint, float pidInput) {
    float pidError = pidInput - pidSetpoint;
    if (!started) {
        started = true;
        lastDError = pidError;
    }
    iSum += iGain * pidError;
    if (iSum > max)
        iSum = max;
    else if (iSum < min)
        iSum = min;

    float pidOutput = pGain * pidError + iSum + dGain * (pidError - lastDError);
    if (pidOutput > max)
        pidOutput = max;
    else if (pidOutput < min)
        pidOutput = min;

    lastDError = pidError;
    return pidOutput;
}
