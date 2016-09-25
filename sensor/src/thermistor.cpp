#include <Arduino.h>
#include <avr/interrupt.h>
#include <avr/power.h>
#include <avr/sleep.h>
#include <avr/wdt.h>
#include <pins_arduino.h>
#include "rng.h"
#include "clock.h"
#include "thermistor.h"

static inline uint8_t nibble(const uint8_t *ary, uint8_t idx)
{
    if (idx&0x01) {
        return ary[idx>>1] & 0x0f;
    } else {
        return (ary[idx>>1]>>4) & 0x0f;
    }
}

// Takes ADC reading and returns temp in C * 10
int16_t ADCtoTempCx10(uint16_t adc)
{
    // Following taken from spreadsheet for
    // B=3800K, r0 = 10k, t0=25, upper-r=10k
    // Ignores self-heating which means temps over 40C may be low

    const int16_t startTemp = -27 + 273; // Keep it in K so no sign issues to worry about
    const uint16_t base = 65;
    const uint8_t delta[] PROGMEM = { 0x44, 0x45, 0x45, 0x55, 0x66, 0x66, 0x67,
        0x67, 0x87, 0x88, 0x88, 0x98, 0x9a, 0x99, 0xaa, 0xaa, 0xba, 0xba, 0xbb,
        0xbb, 0xbb, 0xbc, 0xbb, 0xbb, 0xbc, 0xbb, 0xab, 0xbb, 0xaa, 0xba, 0xaa,
        0xa9, 0xa9, 0x99, 0x98, 0x98, 0x88, 0x88, 0x77, 0x77, 0x77, 0x67, 0x66,
        0x65, 0x65, 0x56, 0x54, 0x55, 0x44, 0x54, 0x44, 0x34, 0x43, 0x34, 0x33,
        0x33, 0x32, 0x33, 0x23, 0x22, 0x32, 0x22, 0x22, 0x20 };

    if (adc < base) return (startTemp - 273) * 10; // We don't know, but it's cold!

    // Linear scan across the delta array.  If the adc value is >= last
    // and <= (last+delta) we found it andcan interpolate.
    int16_t val = base;
    for (uint8_t idx = 0; idx < 128; idx++) {
        uint16_t off = nibble(delta, idx);
        uint16_t top = val + off;
        if ( (adc >= val) && (adc < (top)) ) {
            int16_t fract = ((adc-val) * 10 ) / (off);
            int16_t temp = (startTemp + idx)*10  + fract - 273 * 10;
            return temp;
        }
        val += off;
    }
    return (startTemp + 127 - 273) * 10; // We don't know, but it's hot
}

int16_t ReadThermistor(uint8_t pwrPin, uint8_t adcPin)
{
    uint16_t sum = 0;

    bool oldRE = RandEnable(false);

    pinMode(adcPin, INPUT);
    pinMode(pwrPin, OUTPUT);
    digitalWrite(pwrPin, HIGH);

    power_adc_enable();  // Need to be powered on before we write any registers

    ADCSRA = 0x80 | ((GetClockFreq()==4000)?0x03:0x04); //  0x84 = /16 clock = 512khz

    // Set input and use VCC as VRef
    ADMUX = 0x40 | ((adcPin-A0) & 0x07);
    ClockDelay(5);

    for (uint8_t i=0; i<4; i++) {
        // And...start ADC and busy wait until it's done
        ADCSRA |= _BV(ADSC);
        while (bit_is_set(ADCSRA,ADSC)) {/*busy wait */ }

        // Read it out
        uint16_t res = ADC;
        sum += res;
    }
    sum = (sum + 2) >> 2; // Divide by 4 and round

    digitalWrite(pwrPin, LOW);
    pinMode(adcPin, OUTPUT);

    // Put ADC back to sleep
    ADMUX = 0x00;
    ADCSRA = 0x07;
    power_adc_disable();

    RandEnable(oldRE);

    return ADCtoTempCx10(sum);
}

