#ifndef _THERMISTOR_H
#define _THERMISTOR_H


class Thermistor {
public:
    Thermistor(uint8_t pwrPin, uint8_t adcPin); // Has controlled power up/down
    Thermistor(uint8_t adcPin); // Always powered on
    void Poll();
    int16_t Value();

private:
    uint8_t pwrPin;
    uint8_t adcPin;
    uint8_t polls;
    static const uint8_t samples = 4;
    uint16_t sample[samples];
    int16_t ADCtoTempCx10(uint16_t adc);
    int16_t ReadThermistorRaw();
};


#endif
