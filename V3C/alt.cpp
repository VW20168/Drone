#include "alt.h"

ALT::ALT (int bus){
}

ALT::~ALT() {
}

bool ALT::initialize() {
    return false;
}

void ALT::start() {
}

void ALT::stop() {
}

bool ALT::readDistance(float *distance) {
    return false;
}

bool ALT::readPressureAndTemperature(float *pressure, float *temperature) {
    return false;
}
