
#include "drone.h"
#include "testrunner.h"
#include "neo6m.h"
#include "battery.h"
#include "utils.h"

using namespace std;

TestRunner::TestRunner(std::string comp) {
    component = comp;
    stopped = false;
    
    //i2c power
    gpioSetMode(26, PI_OUTPUT);
    gpioWrite(26, 1);
    gpioSetMode(18, PI_OUTPUT);
    gpioWrite(18, 1);    
}

TestRunner::~TestRunner() {
}

void TestRunner::stop() {
    stopped = true;
}

void TestRunner::run() {
    if (component == "gps") {
        testGPS();
    }
    else if (component == "battery") {
        testBattery();
    }
}

void TestRunner::testGPS() {
    cout << "Testing GPS..." <<endl;

    GPS_NEO6M *pGps = new GPS_NEO6M("/dev/serial0", 115200);
    pGps->initialize();
    pGps->setVerbose(true);

    double longitude, latitude;
    while (!stopped) {
        if (pGps->readData(&latitude, &longitude)) {
            cout << "****GPS: " << latitude << "," << longitude << endl;
        }
        Utils::delayMs(1000);
    }
    
    pGps->closePort();
    delete pGps;
}

void TestRunner::testBattery() {
    cout << "Testing Battery..." <<endl;

    Battery *pBattery = new Battery(NULL);
    pBattery->start();

    float voltage;
    while (!stopped) {
        if (pBattery->read(&voltage)) {
            cout << "****Voltage: " << voltage << endl;
        }
        Utils::delayMs(1000);
    }
    
    delete pBattery;
}
