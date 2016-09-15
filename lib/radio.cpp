#include "RF24.h"
#include "radio.h"
#include "coding.h"


#ifndef __AVR__
    RF24 radio(22,0);
#else
    RF24 radio(7,8);
#endif

// To join, use these hardcoded settings
static const uint32_t        joinAddr = 0xabcd12;
static const uint8_t         joinChan = 0;
static const rf24_datarate_e joinRate = RF24_250KBPS;

// Real network configuration, dynamic from the gateway
static uint32_t        netAddr = 0;
static uint8_t         netChan = 0;
static rf24_datarate_e netRate = RF24_250KBPS;

void DefineNetwork(uint32_t addr, uint8_t chan, rf24_datarate_e rate)
{
    netAddr = addr;
    netChan = chan;
    netRate = rate;
}

void RadioSetup(rf24_datarate_e rate, uint8_t channel, uint32_t addr )
{
    radio.stopListening();
    radio.setPALevel(RF24_PA_MAX);  // Always yell - TODO use bit error rate to lower volume
    radio.setDataRate(rate);
    radio.setAddressWidth(3);       // Always 3
    radio.setChannel(channel);
    radio.disableCRC();
    radio.setAutoAck(false);
    radio.setPayloadSize(19);
    radio.openWritingPipe(addr);
    radio.openReadingPipe(1, addr);
    radio.startListening();
}

void RadioSetupJoin()
{
    RadioSetup(joinRate, joinChan, joinAddr);
}

void RadioSetupNormal()
{
    RadioSetup(netRate, netChan, netAddr);
}

void SendRadioMessage(uint8_t msg[16], uint8_t key[16], uint8_t *rawSent)
{
    uint8_t bytes[19];

    CodeMessage(bytes, msg, key);
    if (rawSent) memcpy(rawSent, bytes, 19); // Squirrel away in case it's a K2SACK

    radio.stopListening();
    radio.write( bytes, 19 );
    // TBD - listen before sending?  Check for error?
    radio.startListening();
}

void SendRadioRawMessage(uint8_t bytes[19])
{
    radio.stopListening();
    radio.write( bytes, 19 );
    // TBD - listen before sending?  Check for error?
    radio.startListening();
}


