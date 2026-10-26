#ifndef __INA219_H__
#define __INA219_H__

#define INA219_ADDRESS      0x40  // Default I2C address for INA219

class INA219 {
    public:
    INA219(int i2c_bus);
    ~INA219();
    
    bool readBusVoltage(float *dta);
    
    private:
    int i2c_bus;
    int i2c_address;
    int handle;

    void calibrate();
    void writeRegister(uint8_t reg, uint16_t value);
    uint16_t readRegister(uint8_t reg);
    
    
    bool readShuntVoltage(float *dta);
    bool readCurrent(float *dta);
    bool readPower(float *dta);
};

#endif

