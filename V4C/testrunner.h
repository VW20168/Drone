#ifndef __TESTRUNNER_H__
#define __TESTRUNNER_H__

#include <string>
#include <iostream>

class TestRunner {
    public:    
    TestRunner(std::string comp);
    virtual ~TestRunner();
    void run();
    void stop();
    
    private:
    std::string component;
    bool stopped;
    
    void testGPS();
    void testBattery();
    void testLed();
    void testEsc();
    void testOpenCV();
    void testAltimeter();
};


#endif

