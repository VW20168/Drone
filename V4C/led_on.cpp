#include <iostream>
#include <wiringPi.h>

int main() {
    // Initialize WiringPi
    if (wiringPiSetup() == -1) {
        std::cerr << "WiringPi setup failed" << std::endl;
        return 1;
    }

    // Set GPIO pin 0 (physical pin 11) as output
    pinMode(0, OUTPUT);

    // Turn on the LED
    digitalWrite(0, HIGH);

    std::cout << "LED is now on" << std::endl;

    // Keep the program running to keep the LED on
    while (true) {
        // Infinite loop to prevent program exit
    }

    return 0;
}
