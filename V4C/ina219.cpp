#include "ina219.h"

// Registers
#define INA219_REG_CONFIG           0x00
#define INA219_REG_SHUNTVOLTAGE     0x01
#define INA219_REG_BUSVOLTAGE       0x02
#define INA219_REG_POWER            0x03
#define INA219_REG_CURRENT          0x04
#define INA219_REG_CALIBRATION      0x05

INA219::INA219(int _i2c_bus) : PiI2C() {
    i2c_bus = _i2c_bus;
    i2c_address = INA219_ADDRESS;
    //Returns a handle (>=0) if OK, otherwise PI_BAD_I2C_BUS, PI_BAD_I2C_ADDR, PI_BAD_FLAGS, PI_NO_HANDLE, or PI_I2C_OPEN_FAILED.
    handle = i2cOpen(i2c_bus, i2c_address, 0);
    if (handle >= 0)
        calibrate();
}

INA219::~INA219() {
    if (handle >= 0)
        i2cClose(handle);
}

void INA219::calibrate() {
    // Calibration value for 16V, 2A range
    // Calibration formula: Cal = 0.04096 / (Current_LSB * R_shunt)
    // For 2A range and 0.1 ohm shunt resistor:
    float Current_LSB = 2.0f / 32768.0f;  // 2A range, 15-bit resolution
    float Cal = 0.04096f / (Current_LSB * 0.1f);
    uint16_t calibrationValue = static_cast<uint16_t>(Cal);
    writeRegister(INA219_REG_CALIBRATION, calibrationValue);

    // Set config register for 16V, 2A range
    // BRNG = 0 (16V range), PGA = 11 (320mV shunt range), BADC = 1111 (12-bit), SADC = 1111 (12-bit), MODE = 111 (continuous)
    uint16_t config = 0x199F;  // 16V, 2A, 12-bit ADC, continuous mode
    writeRegister(INA219_REG_CONFIG, config);
}

bool INA219::readBusVoltage(double *dta) {
    if (handle < 0)
        return false;
    uint16_t busVoltage = readRegister(INA219_REG_BUSVOLTAGE);
    *dta = (busVoltage >> 3) * 0.004;  // LSB = 4mV
    return true;
}

/*
bool INA219::readShuntVoltage(float *dta) {
    if (handle < 0)
        return false;
    int16_t shuntVoltage = readRegister(INA219_REG_SHUNTVOLTAGE);
    *dta = shuntVoltage * 0.01f;  // LSB = 10uV
    return true;
}

bool INA219::readCurrent(float *dta) {
    if (handle < 0)
        return false;
    int16_t current = readRegister(INA219_REG_CURRENT);
    *dta = current * (2.0f / 32768.0f);  // Current_LSB = 2A / 32768
    return true;
}

bool INA219::readPower(float *dta) {
    if (handle < 0)
        return false;
    uint16_t power = readRegister(INA219_REG_POWER);
    *dta = power * (2.0f * 0.04096f / 0.1f);  // Power_LSB = 20 * Current_LSB
    return true;
}
*/

void INA219::writeRegister(uint8_t reg, uint16_t value) {
    uint8_t data[3];
    data[0] = reg;
    data[1] = (value >> 8) & 0xFF;  // MSB
    data[2] = value & 0xFF;         // LSB
    i2cWriteDevice(handle, (char*)&data[0], 3);
}

uint16_t INA219::readRegister(uint8_t reg) {
    uint8_t data[2];
    i2cWriteByte(handle, reg);
    i2cReadDevice(handle, (char*)&data[0], 2);
    return (data[0] << 8) | data[1];
}
/*
int main() {
    INA219 ina219(5, INA219_ADDRESS);

    while (true) {
        float shuntVoltage = ina219.readShuntVoltage();
        float busVoltage = ina219.readBusVoltage();
        float current = ina219.readCurrent();
        float power = ina219.readPower();

        std::cout << "Shunt Voltage: " << shuntVoltage << " mV" << std::endl;
        std::cout << "Bus Voltage: " << busVoltage << " V" << std::endl;
        std::cout << "Current: " << current << " A" << std::endl;
        std::cout << "Power: " << power << " W" << std::endl;
        std::cout << "-----------------------------" << std::endl;

        usleep(500000);  // Sleep for 500ms
    }

    return 0;
}
*/
