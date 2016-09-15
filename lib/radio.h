#ifndef _RADIO_H
#define _RADIO_h

#include "RF24.h"

extern RF24 radio;

void DefineNetwork(uint32_t addr, uint8_t chan, rf24_datarate_e rate);
void RadioSetup(rf24_datarate_e rate, uint8_t channel, uint32_t addr );
void RadioSetupJoin();
void RadioSetupNormal();
void SendRadioMessage(uint8_t msg[16], uint8_t key[16], uint8_t *rawSent);
void SendRadioRawMessage(uint8_t bytes[19]);

#endif

