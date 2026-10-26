
#include "altpid.h"

AltPID::AltPID (float _p, float _i, float _d, float _max) {
    pGain = _p;
    iGain = _i;
    dGain = _d;
    max = _max;
    min = 0 - max;

    reset();
}

AltPID::~AltPID() {
}

void AltPID::reset() {
    iSum = 0;
    lastDError = 0;

    parachuteThrottle = 0;
    parachuteRotatingMemLocation = 0;
    parachuteAltitudePrevious = -100;
    for (int i = 0; i < BUF_SIZE; ++i)
        parachuteBuffer[i] = 0;
}

float AltPID::compute(float pidSetpoint, float pidInput) {
    if (parachuteAltitudePrevious < 0)
        parachuteAltitudePrevious = pidInput;
    parachuteThrottle -= parachuteBuffer[parachuteRotatingMemLocation];
    parachuteBuffer[parachuteRotatingMemLocation] = pidInput - parachuteAltitudePrevious;
    parachuteThrottle += parachuteBuffer[parachuteRotatingMemLocation];
    parachuteAltitudePrevious = pidInput;
    if (++parachuteRotatingMemLocation >= BUF_SIZE)
        parachuteRotatingMemLocation = 0;

    float pidError = pidInput - pidSetpoint;
    iSum += iGain * pidError;
    if (iSum > max)
        iSum = max;
    else if (iSum < min)
        iSum = min;
    float pidOutput = pGain * pidError + iSum + dGain * parachuteThrottle;
    if (pidOutput > max)
        pidOutput = max;
    else if (pidOutput < min)
        pidOutput = min;

    lastDError = pidError;

    return pidOutput;
}
