#include "LED.h"

LED::LED(int pin) {
    this->pin = pin;
};

void LED::on() {
    digitalWrite(this->pin, HIGH);
};

void LED::off() {
    digitalWrite(this->pin, LOW);
};