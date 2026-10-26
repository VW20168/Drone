#include <unistd.h>  // for usleep
#include <cmath>
#include "drone.h"
#include "ms5611.h"


#define    __MS5611_ADDRESS_CSB_LOW   0x76
#define    __MS5611_ADDRESS_CSB_HIGH  0x77
#define    __MS5611_DEFAULT_ADDRESS   0x77

#define    __MS5611_RA_ADC            0x00
#define    __MS5611_RA_RESET          0x1E

#define    __MS5611_RA_C0             0xA0
#define    __MS5611_RA_C1             0xA2
#define    __MS5611_RA_C2             0xA4
#define    __MS5611_RA_C3             0xA6
#define    __MS5611_RA_C4             0xA8
#define    __MS5611_RA_C5             0xAA
#define    __MS5611_RA_C6             0xAC
#define    __MS5611_RA_C7             0xAE

#define    __MS5611_RA_D1_OSR_256     0x40
#define    __MS5611_RA_D1_OSR_512     0x42
#define    __MS5611_RA_D1_OSR_1024    0x44
#define    __MS5611_RA_D1_OSR_2048    0x46
#define    __MS5611_RA_D1_OSR_4096    0x48

#define    __MS5611_RA_D2_OSR_256     0x50
#define    __MS5611_RA_D2_OSR_512     0x52
#define    __MS5611_RA_D2_OSR_1024    0x54
#define    __MS5611_RA_D2_OSR_2048    0x56
#define    __MS5611_RA_D2_OSR_4096    0x58

MS5611::MS5611(uint8_t bus) : ALT(bus) {
    i2c_bus_ = bus;
    i2c_address_ = __MS5611_DEFAULT_ADDRESS;
    i2cHandle = -1;
    C1 = C2 = C3 = C4 = C5 = C6 = 0;
    D1 = D2 = 0;
    TEMP = PRES = 0;
}

MS5611::~MS5611() {
    if (i2cHandle >= 0) {
        i2cClose(i2cHandle);
    }
}

bool MS5611::initialize() {
    i2cHandle = i2cOpen(i2c_bus_, i2c_address_, 0);
    if (i2cHandle < 0) {
        return false;
    }

    //The MS6511 Sensor stores 6 values in the EPROM memory that we need in order to calculate the actual temperature and pressure
    //These values are calculated/stored at the factory when the sensor is calibrated.
    //I probably could have used the read word function instead of the whole block, but I wanted to keep things consistent.
    uint8_t data[20]; //expect 12 bytes
    i2cReadI2CBlockData(i2cHandle, __MS5611_RA_C1, (char*)&data[0], 2);
    i2cReadI2CBlockData(i2cHandle, __MS5611_RA_C2, (char*)&data[2], 2);
    i2cReadI2CBlockData(i2cHandle, __MS5611_RA_C3, (char*)&data[4], 2);
    i2cReadI2CBlockData(i2cHandle, __MS5611_RA_C4, (char*)&data[6], 2);
    i2cReadI2CBlockData(i2cHandle, __MS5611_RA_C5, (char*)&data[8], 2);
    i2cReadI2CBlockData(i2cHandle, __MS5611_RA_C6, (char*)&data[10], 2);

    // Again here we are converting the 2 8bit packages into a single decimal
    C1 = data[0] * 256.0 + data[1];
    C2 = data[2] * 256.0 + data[3];
    C3 = data[4] * 256.0 + data[5];
    C4 = data[6] * 256.0 + data[7];
    C5 = data[8] * 256.0 + data[9];
    C6 = data[10]* 256.0 + data[11];

    return true;
}

bool MS5611::readDigitalValues() {
    if (i2cHandle < 0)
        return false;
        
    uint8_t data[10]; //expect 3 bytes
    //refresh pressure
    i2cWriteByte(i2cHandle, __MS5611_RA_D1_OSR_4096);
    //OSR_256:delay 1ms; OSR_512: 2ms; OSR_1024:3ms; OSR_2048:5ms; OSR_4096:9ms
    usleep(9000);
    //read pressure
    int bytes1 = i2cReadI2CBlockData(i2cHandle, __MS5611_RA_ADC, (char*)data, 3);
    D1 = data[0] * 65536 + data[1] * 256.0 + data[2];
    
    //refresh temperature
    i2cWriteByte(i2cHandle, __MS5611_RA_D2_OSR_4096);
    usleep(9000);
    //read temperature
    int bytes2 = i2cReadI2CBlockData(i2cHandle, __MS5611_RA_ADC, (char*)data, 3);
    D2 = data[0] * 65536 + data[1] * 256.0 + data[2];

    return (bytes1 == 3) && (bytes2 == 3);
}

bool MS5611::readPressureAndTemperature(double *pressure, double *temperature) {
    if (!readDigitalValues()) {
        return false;
    }

    double dT = D2 - C5 * 256;
    TEMP = 2000 + dT * C6 / 8388608;
    
    double OFF = C2 * 65536 + (C4 * dT) / 128;
    double SENS = C1 * 32768 + (C3 * dT) / 256;
    double T2 = 0, OFF2 = 0, SENS2 = 0;
    if (TEMP >= 2000) {
        T2 = 0;
        OFF2 = 0;
        SENS2 = 0;
    }
    else if (TEMP < 2000) {
        T2 = dT * dT / 2147483648;
        OFF2 = 5 * pow((TEMP - 2000), 2) / 2;
        SENS2 = OFF2 / 2;
        if (TEMP < -1500) {
            OFF2 = OFF2 + 7 * pow((TEMP + 1500), 2);
            SENS2 = SENS2 + 11 * pow((TEMP + 1500), 2) / 2;
        }
    }
    TEMP -= T2;
    OFF -= OFF2;
    SENS-= SENS2;

    PRES = (D1 * SENS  / 2097152 - OFF) / 32768;

    TEMP = TEMP / 100; // Temperature updated
    PRES = PRES / 100; // Pressure updated

    *temperature = TEMP;
    *pressure = PRES;
    //cout << TEMP << "," << PRES << endl;
    return true;
}

/*
int main() {
    MS5611 ms5611;
    if (!ms5611.initialize()) {
        std::cerr << "MS5611 initialization failed." << std::endl;
        return 1;
    }

    double pressure, temperature;
    if (ms5611.readPressureAndTemperature(pressure, temperature)) {
        std::cout << "Pressure: " << pressure << " hPa, Temperature: " << temperature << " °C" << std::endl;
    } else {
        std::cerr << "Failed to read pressure and temperature." << std::endl;
    }

    return 0;
}
*/
