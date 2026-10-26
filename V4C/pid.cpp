
#include "pid.h"

PID::PID (double _p, double _i, double _d, double _max) {
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

double PID::compute(double pidSetpoint, double pidInput) {
    double pidError = pidInput - pidSetpoint;
    if (!started) {
        started = true;
        lastDError = pidError;
    }
    iSum += iGain * pidError;
    if (iSum > max)
        iSum = max;
    else if (iSum < min)
        iSum = min;

    double pidOutput = pGain * pidError + iSum + dGain * (pidError - lastDError);
    if (pidOutput > max)
        pidOutput = max;
    else if (pidOutput < min)
        pidOutput = min;

    lastDError = pidError;
    return pidOutput;
}
