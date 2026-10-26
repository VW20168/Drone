#include "pipin.h"

PiPIN::PiPIN(int pin, int mode) {
    chip = NULL; line = NULL;
    
    chip = gpiod_chip_open_by_name("gpiochip0");
    line = gpiod_chip_get_line(chip, pin);
    if (line) {
        if (mode == PI_OUTPUT) {
            gpiod_line_request_output(line, CONSUMER, 0); //default value zero
        }
        else if (mode == PI_INPUT)
            gpiod_line_request_input(line, CONSUMER);
    }
}

PiPIN::~PiPIN() {
    if (line)
        gpiod_line_release(line);
    if (chip)
        gpiod_chip_close(chip);
}

void PiPIN::gpioWrite(int value) {
    if (line) {
        gpiod_line_set_value(line, value);
    }
}

int PiPIN::gpioRead() {    
    return gpiod_line_get_value(line);
}

void PiPIN::powerUp(int pin) {
    PiPIN *thePin = new PiPIN(pin, PI_OUTPUT);
    thePin->gpioWrite(HIGH);
    delete thePin;
}

