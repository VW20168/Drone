#ifndef __PI_I2C_H__
#define __PI_I2C_H__

#include <iostream>
#include <vector>
#include <chrono>
#include <thread>
#include <cmath>
#include <gpiod.h>
#include <unistd.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <linux/i2c-dev.h>


#define PI_I2C_WRITE_FAILED        -82
#define PI_I2C_READ_FAILED         -83

class PiI2C {
    public:
    PiI2C();
    ~PiI2C();

    int i2cOpen(unsigned i2cBus, unsigned i2cAddr, unsigned i2cFlags);
    int i2cClose(int handle);
    int i2cWriteByte(int handle, uint8_t bVal);
    int i2cWriteDevice(int handle, char *buf, unsigned count);
    int i2cReadByteData(int handle, uint8_t i2cReg);
    int i2cReadDevice(int handle, char *buf, unsigned count);
    
    int i2cWriteByteData(int handle, uint8_t i2cReg, uint8_t bVal);
    int i2cReadI2CBlockData(int handle, uint8_t i2cReg, char *buf, unsigned count);
};

#endif
