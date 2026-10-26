#ifndef __ALTIMETER_H__
#define __ALTIMETER_H__

#include "configuration.h"
#include "alt.h"
#include "ms5611.h"
#include "vl53l1x.h"

#define BUF_SIZE_P  20 //was 20
#define BUF_SIZE_T  2 //was 5, not used anymore

#define ALTIMETER_UNKNOWN   0
#define ALTIMETER_BAROMETER 1
#define ALTIMETER_TOF       2

class Altimeter {
public:
    Altimeter(Configuration *config);
    ~Altimeter();
    void start();
    bool read(float *dta);
    void reset();
    void prepare2takeoff();
    void stop();
    
private:
    ALT *pAlt;
    int altimeterType;
    bool getAltitude(float pressure, float temperature, float *output);
    bool validateAltitude(float alt);
    void resetBuf();
    
    float avgPressure, avgAlt;
    //moving average buffers
    double sumP, sumT;
    double pressures[BUF_SIZE_P];
    double temperatures[BUF_SIZE_T];
    int headerP, counterP, headerT, counterT;

    float altitude, lastAltitude, homeAlt;    
    float qnh;
};

#endif

