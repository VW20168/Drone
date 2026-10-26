#include "drone.h"
#include "testrunner.h"
#include "motor.h"
#include "neo6m.h"
#include "battery.h"
#include "ms5611.h"
#include "vl53l1x.h"
#include "pipin.h"
#include "utils.h"
#include "cv/vision.h"

using namespace std;

TestRunner::TestRunner(std::string comp) {
    component = comp;
    stopped = false;

    //i2c power
    PiPIN::powerUp(26);
    PiPIN::powerUp(17);
}

TestRunner::~TestRunner() {
}

void TestRunner::stop() {
    stopped = true;
}

void TestRunner::run() {
    if (component == "led") {
        testLed();
    }
    else if (component == "gps") {
        testGPS();
    }
    else if (component == "battery") {
        testBattery();
    }
    //else if (component == "esc") {
    //    testEsc();
    //}
    else if (component == "opencv") {
        testOpenCV();
    }
    else if (component == "alt") {
        testAltimeter();
    }
    cout << "\n***Finished testing***\n" << endl;
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

    double voltage;
    while (!stopped) {
        if (pBattery->read(&voltage)) {
            cout << "****Voltage: " << voltage << endl;
        }
        Utils::delayMs(1000);
    }

    delete pBattery;
}

void TestRunner::testLed() {
    cout << "Testing LEDs..." <<endl;

    PiPIN *pRedLed = new PiPIN(PIN_RED_LED, PI_OUTPUT);
    PiPIN *pGreenLed = new PiPIN(PIN_GREEN_LED, PI_OUTPUT);
    while (!stopped) {
        pRedLed->gpioWrite(HIGH);
        pGreenLed->gpioWrite(HIGH);
        Utils::delayMs(1000);
        pRedLed->gpioWrite(LOW);
        pGreenLed->gpioWrite(LOW);
        Utils::delayMs(1000);
    }
    pRedLed->gpioWrite(LOW);
    pGreenLed->gpioWrite(LOW);
    delete pRedLed;
    delete pGreenLed;
}

void TestRunner::testEsc() {
    cout << "Testing ESCs..." <<endl;

    Motor *motor = new Motor(PIN_MOTOR1);
    motor->calibrate();
    motor->arm();
    motor->setSpeed(1200);
    Utils::delayMs(8000);
    motor->setSpeed(0);
    delete motor;
}

void TestRunner::testOpenCV() {
    cout << "Testing OpenCV..." <<endl;

    Vision *vision = new Vision();
    if (vision->start()) {
        Utils::delayMs(2000); //Waiting for the camera to be focused
        double px, py, pz, maxy = 0; long seq;
        while (!stopped) {
            long timestamp = Utils::currentTimeMs();
            int ret = vision->loop(0, &px, &py, &pz, &seq);//~120ms
            long now = Utils::currentTimeMs();
            if (ret == VO_OK) {
                cout << "[" << seq << "]" << px << "," << py << "," << pz << endl;
                double diffy = abs(py);
                if (diffy > maxy)
                    maxy = diffy;
            }
            else {
                cout << "Failed to capture/process one frame" << endl;
            }
            Utils::delayMs(1000);
        }
        vision->stop();
        cout << "Max Y: " << maxy << endl;
    }
    delete vision;
}

void TestRunner::testAltimeter() {
    Vl53l1x *vl53l1x = new Vl53l1x(BUS_VL53L1X);
    MS5611 *ms5611   = new MS5611(BUS_MS5611);

    vl53l1x->initialize();
    ms5611->initialize();

    vl53l1x->start();
    ms5611->start();
    while (!stopped) {
        double distance = 0;
        if (vl53l1x->readDistance(&distance)) {
            cout << "TOF reading " << distance << endl;
        }
        else {
            cout << "TOF error" << endl;
        }

        double pressure, temperature;
        if (ms5611->readPressureAndTemperature(&pressure, &temperature)) {
            cout << "MS5611 reading, pressure " << pressure << ", temperature " << temperature << endl;
        }
        else {
            cout << "MS5611 error" << endl;
        }

        Utils::delayMs(1000);
    }
    vl53l1x->stop();
    ms5611->stop();
    delete vl53l1x;
    delete ms5611;
}


