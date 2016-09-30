#include <Arduino.h>
#include <avr/interrupt.h>
#include <avr/power.h>
#include <avr/sleep.h>
#include <avr/wdt.h>
#include <pins_arduino.h>
#include "rng.h"
#include "clock.h"
#include "switch.h"


Switch::Switch(uint8_t pwrPin, uint8_t inpPin, bool invert)
{
    this->pwrPin = pwrPin;
    this->inpPin = inpPin;
    this->invert = invert;
    for (uint8_t i=0; i<this->samples; i++) this->Poll();
    this->lastState = this->sample[this->samples-1];
}

Switch::Switch(uint8_t inpPin, bool invert)
{
    this->pwrPin = 0xff;
    this->inpPin = inpPin;
    this->invert = invert;
    for (uint8_t i=0; i<this->samples; i++) this->Poll();
    this->lastState = this->sample[this->samples-1];
}

void Switch::Poll()
{
    // Power sensor
    if (pwrPin == 0xff) {
        // No explicit power pin, using input's pull-up resistor to power it
        pinMode(inpPin, INPUT_PULLUP);
    } else {
        pinMode(pwrPin, OUTPUT);
        pinMode(inpPin, INPUT);
        digitalWrite(pwrPin, HIGH);
    }
    // Shift the state up
    memcpy(&sample[1], &sample[0], sizeof(sample[0]) * (samples-1));
    // Poll
    sample[samples-1] = (digitalRead(inpPin)==HIGH)?true:false;

    // If last N are the same, assume we've been debounced and change state
    bool same = true;
    for (uint8_t i=1; i<samples; i++)
        if (sample[i] != sample[0]) same = false;
    if (same)
        lastState = sample[0];

    if (pwrPin == 0xff) {
        pinMode(inpPin, OUTPUT);
        digitalWrite(inpPin, LOW);
    } else {
        digitalWrite(pwrPin, LOW);
    }       
}

bool Switch::Value()
{
    return invert ? ~lastState : lastState;
}
