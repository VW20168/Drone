#include "mpu.h"

MPU::MPU (int bus) {
}

MPU::~MPU() {
}

bool MPU::readAccGyro(int16_t *buffer) {
    return false;
}

bool MPU::readMagnetometer(float *mx, float *my, float *mz) {
    return false;
}

