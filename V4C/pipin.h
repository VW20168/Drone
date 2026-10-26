#ifndef __PI_PIN_H__
#define __PI_PIN_H__

#include <gpiod.h>
#include <stdio.h>
#include <unistd.h>

#ifndef HIGH
#define HIGH       1
#endif

#ifndef LOW
#define LOW        0
#endif

#ifndef PI_OUTPUT
#define PI_OUTPUT       1
#endif

#ifndef PI_INPUT
#define PI_INPUT        0
#endif

#ifndef CONSUMER
#define CONSUMER        "Consumer"
#endif

class PiPIN {
    public:
    PiPIN(int pin, int mode);
    ~PiPIN();
    
    void gpioWrite(int value);
    int gpioRead();

    static void powerUp(int pin);
    
    private:
    struct gpiod_chip *chip;
    struct gpiod_line *line;    

};

#endif
