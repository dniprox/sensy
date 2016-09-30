#include <Arduino.h>
#include <avr/interrupt.h>
#include <avr/power.h>
#include <avr/sleep.h>
#include <avr/wdt.h>
#include <pins_arduino.h>
#include "rng.h"
#include "clock.h"
#include "internalsensors.h"

uint16_t InternalSensors::TempValue()
{
    return Value(tempSample);
}

uint16_t InternalSensors::VoltValue()
{
    return Value(voltSample);
}

uint16_t InternalSensors::Value(uint16_t *sample)
{
    uint16_t sum = 0;
    uint8_t cnt = (polls < samples)?polls : samples;
    for (uint8_t i=0; i<cnt; i++) sum += sample[i];
    sum /= cnt;
    return sum;
}

void InternalSensors::Poll()
{
    // Shift out old value
    memcpy(voltSample, voltSample+1, sizeof(voltSample[0]) * (samples-1));
    memcpy(tempSample, tempSample+1, sizeof(tempSample[0]) * (samples-1));

    uint16_t temp;

    bool oldRE = RandEnable(false); // We're using the ADC, don't interrupt us!

    power_adc_enable();  // Need to be powered on before we write any registers

    // Try to keep 512khz frequency
    ADCSRA = 0x80 | ((GetClockFreq()==4000)?0x03:0x04);

    // Set bandgap input
    ADMUX = _BV(REFS0) | _BV(MUX3) | _BV(MUX2) | _BV(MUX1);
    ClockDelay(5);

    // And...start ADC and busy wait until it's done
    ADCSRA |= _BV(ADSC);
    while (bit_is_set(ADCSRA,ADSC)) {/*busy wait */ }

    // Read it out
    uint16_t res = ADC;

    // Scale results:  VCC=1023L, Vref=1100mV;
    // Vref(mV)/Vref(cnt) = VCC(mV)/1023 => Vref(mV)*1023/Vref(cnt) = VCC(mV)
    voltSample[samples-1] = (1100L*1023L) / res; // 1100mV, 2^10 as max

    // Set the internal reference and mux.
    ADMUX = (_BV(REFS1) | _BV(REFS0) | _BV(MUX3));
    ClockDelay(5);

    // And...start ADC and busy wait until it's done
    ADCSRA |= _BV(ADSC);
    while (bit_is_set(ADCSRA,ADSC)) {/*busy wait */ }

    // Read it out
    res = ADC;

    // Offset taken from Arduino website, may not be very accurate
    // Keep everything in integer by scaling by 50 on both sides
    temp = (50*res - 16205 /*324.31*50*/ ) / 61 /*1.22*50*/;
    if (temp > 255) temp = 255; // !!! We be hot

    tempSample[samples-1] = temp;

    // ADC back to sleep
    ADMUX = 0x00;
    ADCSRA = 0x07;
    power_adc_disable();

    RandEnable(oldRE);

    if (polls<255) polls++;
}


InternalSensors::InternalSensors()
{
    this->polls = 0;
}

