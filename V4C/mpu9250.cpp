#include "drone.h"
#include "mpu9250.h"

#define MPU9250_ADDR 0x68
#define ACCEL_CONFIG 0x1C
#define GYRO_CONFIG 0x1B
#define PWR_MGMT_1 0x6B
#define ACCEL_XOUT_H 0x3B
#define GYRO_XOUT_H 0x43
#define MAG_XOUT_L 0x03
#define MAG_XOUT_H 0x04
#define MAG_YOUT_L 0x05
#define MAG_YOUT_H 0x06
#define MAG_ZOUT_L 0x07
#define MAG_ZOUT_H 0x08
#define MAG_CNTL1 0x0A
#define MAG_ADDR 0x0C

#define SMPLRT_DIV     0x19
#define CONFIG         0x1A
#define GYRO_CONFIG    0x1B
#define ACCEL_CONFIG   0x1C
#define ACCEL_CONFIG_2 0x1D
#define INT_PIN_CFG    0x37

#define ACCEL_XOUT_H       0x3B
#define ACCEL_XOUT_L       0x3C
#define ACCEL_YOUT_H       0x3D
#define ACCEL_YOUT_L       0x3E
#define ACCEL_ZOUT_H       0x3F
#define ACCEL_ZOUT_L       0x40

#define GYRO_XOUT_H        0x43
#define GYRO_XOUT_L        0x44
#define GYRO_YOUT_H        0x45
#define GYRO_YOUT_L        0x46
#define GYRO_ZOUT_H        0x47
#define GYRO_ZOUT_L        0x48

#define AK8963_ADDRESS  0x0C
#define WHO_AM_I   0x75


//AK8963 Register Addresses
#define AK8963_ST1        0x02
#define AK8963_MAGNET_OUT 0x03
#define AK8963_CNTL1      0x0A
#define AK8963_CNTL2      0x0B
#define AK8963_ASAX       0x10

// CNTL1 Mode select
// Power down mode
#define AK8963_MODE_DOWN   0x00
// One shot data output
#define AK8963_MODE_ONE    0x01

// Continous data output 8Hz
#define AK8963_MODE_C8HZ   0x02
// Continous data output 100Hz
#define AK8963_MODE_C100HZ 0x06

//Continous data output 8Hz
#define AK8963_MODE_C8HZ    0x02
#define AK8963_BIT_16       0x01
#define AK8963_BIT_14       0x00


MPU9250::MPU9250(int bus) : MPU(bus) {
    i2cHandle = i2cOpen(bus, MPU9250_ADDR, 0);
    if (i2cHandle >= 0)
        initialize();
    // Magnetometer is on a different I2C address        
    magHandle = i2cOpen(bus, AK8963_ADDRESS, 0);
    if (magHandle >= 0)
        configAK8963(AK8963_MODE_C8HZ, AK8963_BIT_16);
}

MPU9250::~MPU9250() {
    if (i2cHandle >= 0)
        i2cClose(i2cHandle);
    if (magHandle >= 0)
        i2cClose(magHandle);
}

void MPU9250::initialize() {
    // Wake up MPU9250
    i2cWriteByteData(i2cHandle, PWR_MGMT_1, 0x00);
    Utils::delayMs(100); // Sleep for 100ms
    i2cWriteByteData(i2cHandle, PWR_MGMT_1, 0x01);
    Utils::delayMs(100);

    i2cWriteByteData(i2cHandle, CONFIG, 0x03);
    i2cWriteByteData(i2cHandle, SMPLRT_DIV, 0x04);//sample rate divider
    i2cWriteByteData(i2cHandle, GYRO_CONFIG, 0x08);
    i2cWriteByteData(i2cHandle, ACCEL_CONFIG, 0x10);
    i2cWriteByteData(i2cHandle, ACCEL_CONFIG_2, 0x03);
    i2cWriteByteData(i2cHandle, INT_PIN_CFG, 0x02);
}

int16_t MPU9250::assemble(uint8_t high, uint8_t low) {
    return ((int16_t)high << 8) | low;
}

bool MPU9250::readAccGyro(int16_t *buffer) {
    if (i2cHandle < 0)
        return false;

    uint8_t data[20];//expect 14 bytes
    if (i2cReadI2CBlockData(i2cHandle, ACCEL_XOUT_H, (char*)data, 14) != 14)
        return false;
    buffer[0] = assemble(data[0], data[1]);
    buffer[1] = assemble(data[2], data[3]);
    buffer[2] = assemble(data[4], data[5]);
    buffer[3] = assemble(data[8], data[9]);
    buffer[4] = assemble(data[10], data[11]);
    buffer[5] = assemble(data[12], data[13]);
    //... temperature 6&7...    
    buffer[6] = assemble(data[6], data[7]);
    return true;
}

void MPU9250::configAK8963(uint8_t mode, uint8_t mfs) {
    mres = (mfs == AK8963_BIT_16) ? 4912.0/32760.0 : 4912.0/8190.0;

    i2cWriteByteData(magHandle, AK8963_CNTL1, 0x00);
    Utils::delayMs(10);

    // set read FuseROM mode
    i2cWriteByteData(magHandle, AK8963_CNTL1, 0x0F);
    Utils::delayMs(10);

    //read coef data
    uint8_t data[10]; //expect 3 bytes
    i2cReadI2CBlockData(magHandle, AK8963_ASAX, (char*)&data[0], 3);
    
    magXcoef = (data[0] - 128) / 256.0 + 1.0;
    magYcoef = (data[1] - 128) / 256.0 + 1.0;
    magZcoef = (data[2] - 128) / 256.0 + 1.0;

    magXcoef *= mres;
    magYcoef *= mres;
    magZcoef *= mres;
    
    //set power down mode
    i2cWriteByteData(magHandle, AK8963_CNTL1, 0x00);
    Utils::delayMs(10);

    //set scale&continous mode
    i2cWriteByteData(magHandle, AK8963_CNTL1, ((mfs << 4) | mode));
    Utils::delayMs(10);
}

bool MPU9250::readMagnetometer(double *mx, double *my, double *mz) {
    if (magHandle < 0)
        return false;

    bool ready = false;
    // check data ready
    int drdy = i2cReadByteData(magHandle, AK8963_ST1);
    if (drdy & 0x01) {
        uint8_t data[20]; //expect 7 bytes
        int bytes = i2cReadI2CBlockData(magHandle, AK8963_MAGNET_OUT, (char*)&data[0], 7);
        if ((bytes >= 7) && ((data[6] & 0x08) != 0x08)) { // check overflow
            double x = assemble(data[1], data[0]); //high - low
            double y = assemble(data[3], data[2]);
            double z = assemble(data[5], data[4]);

            x *= magXcoef;
            y *= magYcoef;
            z *= magZcoef;
            
            *mx = x;
            *my = y;
            *mz = z;
            ready = true;
         }
         else {
             cout << "Failed to read magnetometer  " << bytes << endl;
         }
    }
    return ready;
}
