#include <avr/interrupt.h>
#include <avr/power.h>
#include <avr/sleep.h>
#include <avr/wdt.h>
#include <pins_arduino.h>
#include <SPI.h>
#include "clock.h"
#include "rng.h"


#define debug 0

static uint16_t clkFreq = 1000;
static bool isClockSlow = true; // We start @ 1MHz INTOSC/8

static bool lowBatt=true; // If we're below 8mhz operating voltage

void ClockSetLowBatt(bool state)
{
    if (debug && (lowBatt != state)) {
        Serial.print("LOWBATT=");
        Serial.println(lowBatt?"true":"false");
        Serial.flush();
    }
    lowBatt = state;
}


void ClockDelay(uint16_t ms)
{
    uint32_t done = FromNowMS(ms);
    while (millis() < done) { /* nada */ }
}


static void SetClockPrescaler(uint8_t clockPrescaler);


void ClockSlow()
{
    if (isClockSlow) return; // Already in slow mode
    if (debug) {
        Serial.println("CLOCKSLOW");
        Serial.flush();
    }
    RandEnable(false);
    isClockSlow = true;
    SetClockPrescaler(0x04); // /16, 0.5MHz
    clkFreq = 500;
}

void ClockNormal()
{
    if (!isClockSlow) return; // Already in normal mode

    if (!lowBatt) {
        SetClockPrescaler(0x00); // /1, 8MHz
        clkFreq = 8000;
    } else {
        SetClockPrescaler(0x01); // /2, 4 MHz
        clkFreq = 4000;
    }
    RandEnable(true);
    isClockSlow = false;
    if (debug) {
        Serial.println("CLOCKFAST");
        Serial.flush();
    }
}

uint16_t GetClockFreq()
{
    return clkFreq;
}

uint32_t FromNowMS(uint32_t delta)
{
    uint32_t now = millis();
    if (isClockSlow) {
        // Divide by 16 (since we're at .5MHz, not 8)
        delta = (delta >> 4) & (0x0fff);
    } else if (lowBatt) {
        // Divide by 2 (since we're at 4MHz, not 8)
        delta = (delta >> 1) & (0x7fff);
    }
    if (delta==0) delta = 1;
    now += delta;
    return now;
}

static void SetClockPrescaler(uint8_t clockPrescaler)
{
    cli();
    CLKPR = _BV(CLKPCE);    // Enable writes to the clock prescale register
    CLKPR = clockPrescaler; // Write new value
    sei();
}
