#include "mpu.h"

MPU::MPU (int bus) : PiI2C() {
}

MPU::~MPU() {
}

bool MPU::readAccGyro(int16_t *buffer) {
    return false;
}

bool MPU::readMagnetometer(double *mx, double *my, double *mz) {
    return false;
}

