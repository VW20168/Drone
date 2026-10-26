#include "drone.h"
#include "battery.h"

Battery::Battery (Configuration *config) {
    pINA219 = new INA219(BUS_INA219);
    lastReading = READING_UNKNOWN;
}

Battery::~Battery() {
    delete pINA219;
}

void Battery::start() {
}

bool Battery::read(double *dta) {
    double reading;
    if (!pINA219->readBusVoltage(&reading))
        return false;

    if (lastReading < READING_UNKNOWN)
        voltage = lastReading * 0.3 + reading * 0.7;
    else
        voltage = reading;
    lastReading = voltage;
    //the result
    *dta = voltage;

    return true;
}
