
#include "pipwm.h"


PiPWM::PiPWM(int pin) {
    if (pin == 12) {
        chip_    = 0;
        channel_ = 0;
    }
    else if (pin == 13) {
        chip_    = 0;
        channel_ = 1;
    }
    else if (pin == 18) {
        chip_    = 0;
        channel_ = 2;
    }
    else if (pin == 19) {
        chip_    = 0;
        channel_ = 3;
    }
    basePath_ = "/sys/class/pwm/pwmchip" + std::to_string(chip_) + "/pwm" + std::to_string(channel_);

    exportPWM();
    setPeriod(20000000); // 20 ms period (50 Hz)
    setDutyCycle(0);
    enablePWM(true);
}

PiPWM::~PiPWM() {
    enablePWM(false);
}

void PiPWM::gpioServo(int pin, int pulseWidth) {
    setDutyCycle(pulseWidth * 1000);
}

bool PiPWM::exportPWM() {
    return writeToFile("/sys/class/pwm/pwmchip" + std::to_string(chip_) + "/export", std::to_string(channel_));
}

bool PiPWM::unexportPWM() {
    return writeToFile("/sys/class/pwm/pwmchip" + std::to_string(chip_) + "/unexport", std::to_string(channel_));
}

bool PiPWM::setPeriod(int period_ns) {
    return writeToFile(basePath_ + "/period", std::to_string(period_ns));
}

bool PiPWM::setDutyCycle(int duty_cycle_ns) {
    return writeToFile(basePath_ + "/duty_cycle", std::to_string(duty_cycle_ns));
}

bool PiPWM::enablePWM(bool enable) {
    return writeToFile(basePath_ + "/enable", enable ? "1" : "0");
}

bool PiPWM::writeToFile(const std::string& path, const std::string& value) {
    //std::cerr << "Writing:" << path << ", with value " << value << std::endl;
    std::ofstream file(path);
    if (file.is_open()) {
        file << value;
        file.close();
        return true;
    }
    else {
        std::cerr << "Error opening file: " << path << std::endl;
        return false;
    }
    //usleep(10000); // Small delay
}
