#ifndef __ALT_H__
#define __ALT_H__

//base class for all altimeters, such as BMP280, MS5611
class ALT {
    public:
    ALT(int bus);
    virtual ~ALT();

    virtual bool initialize();
    virtual void start();
    virtual void stop();
    virtual bool readDistance(float *distance);
    virtual bool readPressureAndTemperature(float *pressure, float *temperature);

};


#endif

