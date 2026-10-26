#include <iostream>
#include "pii2c.h"

PiI2C::PiI2C() {
}

PiI2C::~PiI2C() {
}

int PiI2C::i2cOpen(unsigned i2cBus, unsigned i2cAddr, unsigned i2cFlags) {
    char i2cDevice_[128];    
    sprintf(i2cDevice_, "/dev/i2c-%d", i2cBus);
    int handle = open(i2cDevice_, O_RDWR);
    if (handle >= 0) {
        if (ioctl(handle, I2C_SLAVE, i2cAddr) < 0) {
            close(handle);
            handle = -1;
        }
    }
    if (handle < 0)
        std::cerr << "Failed to open I2C device " << i2cDevice_ << std::endl;
    return handle;//9250 will open two handles
}

int PiI2C::i2cClose(int handle) {
    if (handle >= 0)
        close(handle);
    return 0;
}

int PiI2C::i2cWriteByte(int handle, uint8_t bVal) {
    return i2cWriteDevice(handle, (char*)&bVal, 1);
}

int PiI2C::i2cWriteDevice(int handle, char *buf, unsigned count) {
    return write(handle, buf, count) == count ? 0 : PI_I2C_WRITE_FAILED;
}

//Returns 0 if OK, otherwise PI_BAD_HANDLE, PI_BAD_PARAM, or PI_I2C_WRITE_FAILED.
int PiI2C::i2cWriteByteData(int handle, uint8_t i2cReg, uint8_t bVal) {
    uint8_t buffer[2] = {i2cReg, bVal};
    return write(handle, buffer, sizeof(buffer)) == sizeof (buffer) ? 0 : PI_I2C_WRITE_FAILED;
}

//Returns the byte read (>=0) if OK, otherwise PI_BAD_HANDLE, PI_BAD_PARAM, or PI_I2C_READ_FAILED.
int PiI2C::i2cReadByteData(int handle, uint8_t i2cReg) {
    if (write(handle, &i2cReg, 1) == 1) {
        uint8_t readBuffer = 0;
        if (read(handle, &readBuffer, 1) == 1)
            return readBuffer;
    }
    //something went wrong
    return PI_I2C_READ_FAILED;
}

//Returns count (>0) if OK, otherwise PI_BAD_HANDLE, PI_BAD_PARAM, or PI_I2C_READ_FAILED.
int PiI2C::i2cReadDevice(int handle, char *buf, unsigned count) {
    int got = read(handle, buf, count);
    return got > 0 ? got : PI_I2C_READ_FAILED;
}

//Returns the number of bytes read (>0) if OK, otherwise PI_BAD_HANDLE, PI_BAD_PARAM, or PI_I2C_READ_FAILED.
int PiI2C::i2cReadI2CBlockData(int handle, uint8_t i2cReg, char *buf, unsigned count){
    if (write(handle, &i2cReg, 1) == 1) {
        int got = read(handle, buf, count);
        if (got > 0)
            return got;
    }
    //something went wrong
    return PI_I2C_READ_FAILED;
}
