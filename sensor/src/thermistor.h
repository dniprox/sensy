#ifndef _THERMISTOR_H
#define _THERMISTOR_H

int16_t ADCtoTempCx10(uint16_t adc);
int16_t ReadThermistor(uint8_t pwrPin, uint8_t adcPin);

#endif
