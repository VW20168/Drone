#include "alt.h"

ALT::ALT (int bus) : PiI2C() {
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

bool ALT::readDistance(double *distance) {
    return false;
}

bool ALT::readPressureAndTemperature(double *pressure, double *temperature) {
    return false;
}
