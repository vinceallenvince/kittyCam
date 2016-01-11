#include <stdio.h>
#include <wiringPi.h>

class LED {

    int pin;

public:
    LED(int pin);

    int getPin() { return this->pin; };

    void on();
    void off();
};