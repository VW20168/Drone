#include <math.h>
#include "drone.h"
#include "altimeter.h"

#define NA          0xffffff

Altimeter::Altimeter(Configuration *config, bool opposite){
    altimeterType = ALTIMETER_UNKNOWN;
    qnh = 1013.25;

    bool tof = false;
    if (config->getAltimeter() == "VL53L1X") {
        tof = !opposite;
    }
    else if (config->getAltimeter() == "MS5611") {
        tof = opposite;
    }
    if (tof) {
        pAlt = new Vl53l1x(BUS_VL53L1X);
        altimeterType = ALTIMETER_TOF;
    }
    else {
        pAlt = new MS5611(BUS_MS5611); //bus no 4
        altimeterType = ALTIMETER_BAROMETER;
    }
    pAlt->initialize();
    reset();
}

Altimeter::~Altimeter() {
    delete pAlt;
}

void Altimeter::reset() {
    resetBuf();
    homeAlt = lastAltitude = altitude = NA;
}

void Altimeter::resetBuf() {
    headerP = counterP = 0; sumP = 0;
    headerT = counterT = 0; sumT = 0;
    avgPressure = avgAlt = 0;

    for (int i = 0; i < BUF_SIZE_P; ++i)
        pressures[i] = 0;
    for (int i = 0; i < BUF_SIZE_T; ++i)
        temperatures[i] = 0;
}

void Altimeter::start() {
    pAlt->start();
    reset();
}

void Altimeter::stop() {
    pAlt->stop();
}

void Altimeter::prepare2takeoff() {
    homeAlt = altitude;  //reset home altitude
}

bool Altimeter::read(double *dta) {
    if (altimeterType == ALTIMETER_BAROMETER) {
        double pressure, temperature;
        if (!pAlt->readPressureAndTemperature(&pressure, &temperature))
            return false;
        double altReading = 0;
        if (!getAltitude(pressure, temperature, &altReading)) {
            log_error(LOGTAG, "Wrong barometer reading %.3f", pressure);
            return false;
        }
        if (lastAltitude < NA) {
            altitude = lastAltitude * 0.9f + altReading * 0.1f;
            /*
            float altitudeSlow = lastAltitude * 0.985f + altReading * 0.015f;
            //try to catch the fast altitude change
            float diff = altitudeSlow - altReading;
            altDiff = diff;
            if (diff > 0.2f) {
                diff = 0.2f;
            }
            else if (diff < -0.2f) {
                diff = -0.2f;
            }
            if (diff >= -0.1f && diff <= 0.1f)
                diff = 0;
            altitudeSlow -= diff / 2;

            altitude = altitudeSlow;
            */
        }
        else {
            altitude = altReading;
        }
        //log_info("MS5611", "Alt,%.3f,P,%.3f,T,%.3f", altitude, pressure, temperature);
    }
    else if (altimeterType == ALTIMETER_TOF) {
        double altReading = 0;
        if (!pAlt->readDistance(&altReading))
            return false;
        if (!validateAltitude(altReading)) {
            log_error(LOGTAG, "Wrong TOF reading %.3f", altReading);
            return false;
        }
        if (lastAltitude < NA) {
            altitude = lastAltitude * 0.3f + altReading * 0.7f;
        }
        else {
            altitude = altReading;
        }
    }
    lastAltitude = altitude;

    double alt = (homeAlt >= NA) ? 0 : altitude - homeAlt;
    //relative altitude
    if (alt < 0)
        alt = 0;
    *dta = alt;

    return true;
}

bool Altimeter::getAltitude(double pressure, double temp, double *output) {
    if (pressure <= 0)
        return false;
    //impossible drone speed: 0.5 * 50(reading interval is 20ms) * 8.5 = 212m/s
    if (counterP >= 50 && abs(pressure - avgPressure) >= 0.5) //was 1.0
        return false;

    //moving average of temperatures
    /*
    ++counterT;
    sumT = sumT - temperatures[headerT] + temp;
    temperatures[headerT] = temp;
    if (++headerT >= BUF_SIZE_T)
        headerT = 0;
    int applied = counterT;
    if (applied > BUF_SIZE_T)
        applied = BUF_SIZE_T;
    double avgTemp = sumT / applied;
    */

    //moving average of pressures
    ++counterP;
    sumP = sumP - pressures[headerP] + pressure;
    pressures[headerP] = pressure;
    if (++headerP >= BUF_SIZE_P)
        headerP = 0;
    int applied = counterP;
    if (applied > BUF_SIZE_P)
        applied = BUF_SIZE_P;
    avgPressure = sumP / applied;

    //*output = (float)( ((pow((qnh / avgPressure), (1.0 / 5.257)) - 1) * (avgTemp + 273.15)) / 0.0065);
    //The standard air pressure at sea level is 1013.25 millibars (mb)
    //xxx *output = (1013.25f-avgPressure) * 8.5f;


    //use pressure, not the average pressure
    *output = (1013.25-pressure) * 8.5;

    return true;
}

bool Altimeter::validateAltitude(double alt) {
    //impossible speed: 0.5/0.02 (reading interval is 20ms) = 25m/s
    if (counterP >= 50 && abs(alt - avgAlt) >= 0.50)
        return false;

    //using the pressure buffer
    ++counterP;
    sumP = sumP - pressures[headerP] + alt;
    pressures[headerP] = alt;
    if (++headerP >= BUF_SIZE_P)
        headerP = 0;
    int applied = counterP;
    if (applied > BUF_SIZE_P)
        applied = BUF_SIZE_P;
    avgAlt = sumP / applied;

    return true;
}

