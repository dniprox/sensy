/*
 Copyright (C) 2011 J. Coliz <maniacbug@ymail.com>

 This program is free software; you can redistribute it and/or
 modify it under the terms of the GNU General Public License
 version 2 as published by the Free Software Foundation.

 03/17/2013 : Charles-Henri Hallard (http://hallard.me)
              Modified to use with Arduipi board http://hallard.me/arduipi
						  Changed to use modified bcm2835 and RF24 library
TMRh20 2014 - Updated to work with optimized RF24 Arduino library

 */

/**
 * Example RF Radio Ping Pair
 *
 * This is an example of how to use the RF24 class on RPi, communicating to an Arduino running
 * the GettingStarted sketch.
 */

#include <cstdlib>
#include <iostream>
#include <sstream>
#include <string>
#include <unistd.h>
#include <RF24/RF24.h>
#include "/root/arduinolibs/libraries/Crypto/AES.h"
#include "/root/arduinolibs/libraries/Crypto/Curve25519.h"
#include <pthread.h>

//using namespace std;
RF24 radio(22,0);

// To join, use these hardcoded settings
const uint32_t        joinAddr = 0xabcd12;
const bool            joinHamm = true;
const uint8_t         joinChan = 0;
const rf24_datarate_e joinRate = RF24_250KBPS;

// Gateway will send up the real network config
uint32_t        netAddr = 0x21dcba;
bool            netHamm = true;
uint8_t         netChan = 0;
rf24_datarate_e netRate = RF24_2MBPS;


void RadioSetup(rf24_datarate_e rate, uint8_t channel, uint32_t addr, bool hamming )
{
    printf("RADIO rate=%d\n",rate);
    printf("RADIO chann=%d\n",channel);
    printf("RADIO addr=0x%0x\n",addr);
    printf("RADIO hamming=%d\n",hamming);
    radio.stopListening();
    radio.setPALevel(RF24_PA_MAX);  // Always yell
    radio.setDataRate(rate);
    radio.setAddressWidth(3);       // Always 3
    radio.setChannel(channel);
    radio.disableCRC();
    radio.setAutoAck(false);
    radio.setPayloadSize(hamming?19:16);
    radio.openWritingPipe(addr);
    radio.openReadingPipe(1, addr);
    radio.startListening();
}

void RadioSetupJoin()
{
    RadioSetup(joinRate, joinChan, joinAddr, joinHamm);
}

void RadioSetupNormal()
{
    RadioSetup(netRate, netChan, netAddr, netHamm);
}


// Hyper-naive parity in straight C.  Assembly it's probably 1 byte on x86 and AVR.
uint8_t inline parity(uint8_t p)
{
    p = ((p&0xf0)>>4) ^ (p&0x0f);
    p = ((p&0x0c)>>2) ^ (p&0x03);
    p = ((p&0x02)>>1) ^ (p&0x01);
    return p&1;
}


// CRC-CCITT(0xffff) 16-bit cyclic redundancy check, nibble-tabled
uint16_t crc16(uint8_t *data, uint8_t cnt)
{
    const uint16_t tc[16] PROGMEM = {
       0x0000, 0x1021, 0x2042, 0x3063, 0x4084, 0x50a5, 0x60c6, 0x70e7,
       0x8108, 0x9129, 0xa14a, 0xb16b, 0xc18c, 0xd1ad, 0xe1ce, 0xf1ef
    };
 
    uint16_t sum = 0xffff;  // CCITT starts at 0xffff
  
    while (cnt--) {
        sum = tc[(sum>>12)^(*data >> 4)]^(sum<<4);
        sum = tc[(sum>>12)^(*(data++) & 0xf)]^(sum<<4);
    }

    return sum;
}


// Return 6-bit hamming code for 32bit integer
uint8_t calc_hamming_32(const uint8_t a[4])
{
    // P0 = D0 + D1 + D3 + D4 + D6 + D8 + D10 + D11 + D13 + D15 + D17 + D19 + D21 + D23 + D25 + D26 + D28 + D30
    uint8_t p0 = (a[0]&(0x01|0x02|0x08|0x10|0x40))^(a[1]&(0x01|0x04|0x08|0x20|0x80))^(a[2]&(0x02|0x08|0x20|0x80))^(a[3]&(0x02|0x04|0x10|0x40));
    // P1 = D0 + D2 + D3 + D5 + D6 + D9 + D10 + D12 + D13 + D16 + D17 + D20 + D21 + D24 + D25 + D27 + D28 + D31
    uint8_t p1 = (a[0]&(0x01|0x04|0x08|0x20|0x40))^(a[1]&(0x02|0x04|0x10|0x20))^(a[2]&(0x01|0x02|0x10|0x20))^(a[3]&(0x01|0x02|0x08|0x10|0x80));
    // P2 = D1 + D2 + D3 + D7 + D8 + D9 + D10 + D14 + D15 + D16 + D17 + D22 + D23 + D24 + D25 + D29 + D30 + D31
    uint8_t p2 = (a[0]&(0x02|0x04|0x08|0x80))^(a[1]&(0x01|0x02|0x04|0x40|0x80))^(a[2]&(0x01|0x02|0x40|0x80))^(a[3]&(0x01|0x02|0x20|0x40|0x80));
    // P3 = D4 + D5 + D6 + D7 + D8 + D9 + D10 + D18 + D19 + D20 + D21 + D22 + D23 + D24 + D25
    uint8_t p3 = (a[0]&(0x10|0x20|0x40|0x80))^(a[1]&(0x01|0x02|0x04))^(a[2]&(0x04|0x08|0x10|0x20|0x40|0x80))^(a[3]&(0x01|0x02));
    // P4 = D11 + D12 + D13 + D14 + D15 + D16 + D17 + D18 + D19 + D20 + D21 + D22 + D23 + D24 + D25
    uint8_t p4 = (a[1]&(0x08|0x10|0x20|0x40|0x80))^(a[2]&(0x01|0x02|0x04|0x08|0x10|0x20|0x40|0x80))^(a[3]&(0x01|0x02));
    // P5 = D26 + D27 + D28 + D29 + D30 + D31
    uint8_t p5 = (a[3]&(0x04|0x08|0x10|0x20|0x40|0x80));
    
    uint8_t p = parity(p0) | (parity(p1)<<1) | (parity(p2)<<2) | (parity(p3)<<3) | (parity(p4)<<4) | (parity(p5)<<5);
    return p;
}

void encode_hamming(uint8_t a[5], uint8_t b[5], uint8_t c[5], uint8_t d[5], const uint8_t src[16])
{
    memcpy(a, src, 4);
    a[4] = calc_hamming_32(a);
    memcpy(b, src+4, 4);
    b[4] = calc_hamming_32(b);
    memcpy(c, src+8, 4);
    c[4] = calc_hamming_32(c);
    memcpy(d, src+12, 4);
    d[4] = calc_hamming_32(d);
}


void hamming_correct(uint8_t a[5], uint8_t syn)
{
    const uint8_t flipbit[39] PROGMEM = {
        /*0*/  37, 32, 33,  0, 34,  1,  2,  3,
        /*8*/  35,  4,  5,  6,  7,  8,  9, 10,
        /*16*/ 36, 11, 12, 13, 14, 15, 16, 17,
        /*24*/ 18, 19, 20, 21, 22, 23, 24, 25,
        /*32*/ 37, 26, 27, 28, 29, 30, 31 };

    if (0==syn || syn>38) return; // No errors or something weird

    uint8_t toflip = flipbit[syn];
    a[(toflip>>3)] ^= 1<<(toflip & 7);
}


void hamming_to_bytes(uint8_t src[19], const uint8_t ap[5], const uint8_t bp[5], const uint8_t cp[5], const uint8_t dp[5])
{
    memcpy(src, ap, 4);
    memcpy(src+4, bp, 4);
    memcpy(src+8, cp, 4);
    memcpy(src+12, dp, 4);
}


// Allow us to fix single burst error of up to 4 bits during xmission
// Only first 4 bytes(the original data) and lower 6 bits of parity used
void interleave_4x38(uint8_t dest[19], uint8_t a[5], uint8_t b[5], uint8_t c[5], uint8_t d[5])
{
    uint8_t cnt = 0; // output pointer
    uint8_t i;

    // 5th element only has 6 bits, so need to shift them a bit to keep math same
    // Then process as normally and don't store the last byte of the quartet and you're golden
    a[4] = ((a[4]&0x38)<<2) | ((a[4]&0x07)<<1);               // AAA0 AAA0
    b[4] = ((b[4]&0x38)<<1) | (b[4]&0x07);                    // 0BBB 0BBB
    c[4] = ((c[4]&0x20)<<2) | ((c[4]&0x1c)<<1) | (c[4]&0x03); // C0CC C0CC
    d[4] = ((d[4]&0x30)<<2) | ((d[4]&0x0e)<<1) | (d[4]&0x01); // DD0D DD0D

    for (i=0; i<5; i++) {
        uint8_t q;
        // Interleave while leaving bits in place to allow mask and or calculation
        q = (a[i]&(128|8)) | (b[i]&(64|4)) | (c[i]&(32|2)) | (d[i]&(16|1));
        dest[cnt ++] = q; // Don't need to rotate dest

        q = (a[i]&(64|4))  | (b[i]&(32|2))  | (c[i]&(16|1))  | (d[i]&(128|8));
        q = (q<<1) | (q&128?1:0); // ROL A bit to MSB to preserve distance between different codeword bits
        dest[cnt++] = q;

        q = (a[i]&(32|2))  | (b[i]&(16|1))  | (c[i]&(128|8)) | (d[i]&(64|4));
        q = (q<<2) | (q&128?2:0) | (q&64?1:0); // ROL A bit to MSB
        dest[cnt++] = q;

        if (i!=4) {
            q = (a[i]&(16|1))  | (b[i]&(128|8)) | (c[i]&(64|4))  | (d[i]&(32|2));
            q = (q<<3) | (q&128?4:0) | (q&64?2:0) | (q&32?1:0);  // ROL A bit to MSB
            dest[cnt++] = q;
        }
    }
}


void deinterleave_4x38(uint8_t a[5], uint8_t b[5], uint8_t c[5], uint8_t d[5], uint8_t src[20] /*1 byte bigger than data, last byte ignored */)
{
    uint8_t q = 0; // output pointer
    uint8_t i;
    for (i=0; i<5; i++) {
        src[q+1] = ((src[q+1]>>1)&127) | (src[q+1]&1?128:0);
        src[q+2] = ((src[q+2]>>2)&63) | (src[q+2]&2?128:0) | (src[q+2]&1?64:0);
        src[q+3] = ((src[q+3]>>3)&31) | (src[q+3]&4?128:0) | (src[q+3]&2?64:0) | (src[q+3]&1?32:0);
        // Deinterleave from bits in place
        a[i] = (src[q+0]&(128|8)) | (src[q+1]&(64|4)) | (src[q+2]&(32|2)) | (src[q+3]&(16|1));
        b[i] = (src[q+3]&(128|8)) | (src[q+0]&(64|4)) | (src[q+1]&(32|2)) | (src[q+2]&(16|1));
        c[i] = (src[q+2]&(128|8)) | (src[q+3]&(64|4)) | (src[q+0]&(32|2)) | (src[q+1]&(16|1));
        d[i] = (src[q+1]&(128|8)) | (src[q+2]&(64|4)) | (src[q+3]&(32|2)) | (src[q+0]&(16|1));
        q += 4;
    }
    // 5th element only has 6 bits, and they're in odd 0-padded format as shown above.  Multipass
    a[4] = 0x3f & ( ((a[4]&0xe0)>>2) | ((a[4]&0x0e)>>1) );               // AAA0 AAA0 -> 00AA AAAA
    b[4] = 0x3f & ( ((b[4]&0x70)>>1) | (b[4]&0x07) );                    // 0BBB 0BBB -> 00BB BBBB
    c[4] = 0x3f & ( ((c[4]&0x80)>>2) | ((c[4]&0x38)>>1) | (c[4]&0x03) ); // C0CC C0CC -> 00CC CCCC
    d[4] = 0x3f & ( ((d[4]&0xc0)>>2) | ((d[4]&0x1c)>>1) | (d[4]&0x01) ); // DD0D DD0D -> 00DD DDDD
}


// Deinterleave, hamming correct, and convert from hamming codewords to bytes
// Decryption is not performed, nor is CRC or any other sanity.  Return corrected bits
uint8_t CorrectMessage(uint8_t dst[16], uint8_t src[19])
{
    uint8_t inMsgAESHammA[5], inMsgAESHammB[5], inMsgAESHammC[5], inMsgAESHammD[5]; // Hamming encoded, AES encrypted, raw message
    uint8_t inMsgAESHamm[16]; // HAmming encoded, AES encrypted, raw message in byte form

    // Reorder back into codewords
    deinterleave_4x38(inMsgAESHammA, inMsgAESHammB, inMsgAESHammC, inMsgAESHammD, src);

    // Calculate Hamming for received data to ^ with transmitted to find syndrome
    uint8_t syna, synb, sync, synd;
    syna = inMsgAESHammA[4];
    synb = inMsgAESHammB[4];
    sync = inMsgAESHammC[4];
    synd = inMsgAESHammD[4];
    hamming_to_bytes(inMsgAESHamm, inMsgAESHammA, inMsgAESHammB, inMsgAESHammC, inMsgAESHammD);
    encode_hamming(inMsgAESHammA, inMsgAESHammB, inMsgAESHammC, inMsgAESHammD, inMsgAESHamm);
    syna ^= inMsgAESHammA[4];
    synb ^= inMsgAESHammB[4];
    sync ^= inMsgAESHammC[4];
    synd ^= inMsgAESHammD[4];
    int c=0;
    if (syna) c++;
    if (synb) c++;
    if (sync) c++;
    if (synd) c++;

    // Correct it and move from codewords to bytes
    hamming_correct(inMsgAESHammA, syna);
    hamming_correct(inMsgAESHammB, synb);
    hamming_correct(inMsgAESHammC, sync);
    hamming_correct(inMsgAESHammD, synd);
    hamming_to_bytes(dst, inMsgAESHammA, inMsgAESHammB, inMsgAESHammC, inMsgAESHammD);

    return c;
}



bool CheckMessageAES(uint8_t decMsg[16], const uint8_t aesKey[16], const uint8_t encMsg[16])
{
    if (aesKey) {
        AES128 aes;
        aes.setKey(aesKey, 16);
        aes.decryptBlock(decMsg, encMsg);
    } else {
        memcpy(decMsg, encMsg, 16);
    }
    uint16_t rec_crc = crc16(decMsg, 14);
    if ( ((rec_crc&0xff) != decMsg[14]) || (((rec_crc>>8)&0xff) != decMsg[15])) {
        return false;
    } else {
        return true;
    }
}





typedef enum { MSG_JOIN=0, MSG_K0G, MSG_K0S, MSG_K1G, MSG_K1S, MSG_K2G, MSG_K2S, MSG_REPORT, MSG_ACK, MSG_LOG=15 } messageType_t;


typedef int report_t;


typedef struct {
    int     id;    // SID
    char    name[10];

    uint8_t aesKey[16];
    time_t  joinTime;
    time_t  lastReport;

    int      reportEntries;
    report_t reportType[13];  // What fields we have
    uint8_t  reportState[13];   // The raw data

    int     lastSeqNo;      // Last sequence number we received
    uint8_t lastRecvMessage[16]; // Last message to come in from the sensor
    uint8_t outMessage[16];      // Message to sensor
    time_t  lastRetryTime;       // When we last resent it
    int     retriesLeft;         // If any retries left
} sensor_t;

int sensors = 0;
sensor_t *sensor = NULL;

messageType_t GetMessageType(const uint8_t msg[16])
{
    return (messageType_t)(0x0f & (msg[0]>>4));
}

int GetSequenceNum(const uint8_t msg[16])
{
    return (msg[0] & 0x0f);
}

volatile bool newJoin = 0; // external thread sets to 1 to initiate a join process
const int joinMaxTime = 120; // # of seconds to stay in join state before returning to listener
const int rekeyMaxTime = 120; // # of seconds to stay in join state before returning to listener
const int sleepTimeUs = 10000; // 1/2 of a message transmission time (1/250K * 19*8) in microseconds

int nextSensorID = 100; // != 0

void SetMessageTypeSeq(uint8_t msg[16], messageType_t type, int seq)
{
    msg[0] = ((type&0xf) << 4) | (seq & 0xf);
}


void CodeMessage(uint8_t done[19], uint8_t outMsg[16], uint8_t key[16])
{
    uint8_t outMsgAES[16];     // AES encrypted copy
    uint8_t outMsgAESHammA[5], outMsgAESHammB[5], outMsgAESHammC[5], outMsgAESHammD[5]; // Hamming encoded, AES encrypted, raw message

    // Set the CRC in raw form
    uint16_t crc = crc16(outMsg, 14);
    outMsg[14] = crc & 0xff;
    outMsg[15] = crc>>8;

    // Encrypt or pass-through plaintext
    if (key) {
        AES128 aes;
        aes.setKey(key, 16);
        aes.encryptBlock(outMsgAES, outMsg);
    } else {
        memcpy(outMsgAES, outMsg, 16);
    }

    // Get hamming codewords
    encode_hamming(outMsgAESHammA, outMsgAESHammB, outMsgAESHammC, outMsgAESHammD, outMsgAES);
  
     // Pack them interleaved as raw bytes for transmission
    interleave_4x38(done, outMsgAESHammA, outMsgAESHammB, outMsgAESHammC, outMsgAESHammD);
}

static uint8_t lastBytes[19];
void SendRadioMessage(uint8_t msg[16], uint8_t key[16])
{
    uint8_t bytes[19];

    CodeMessage(bytes, msg, key);
    memcpy(lastBytes, bytes, 19); // Squirrel away in case it's a K2SACK

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



void SendSensorAck(sensor_t *sensor, bool enc)
{
    time_t now = time(NULL);
    struct tm *t = localtime(&now);
    uint8_t outMsg[16];

    memset(outMsg, 0, 16);
    SetMessageTypeSeq(outMsg, MSG_ACK, sensor->lastSeqNo);
    outMsg[1] = (t->tm_hour << 3) | ((t->tm_min >> 3)&0x07);
    int year = t->tm_year + 1900 - 2016;
    outMsg[2] = ((t->tm_min&0x07)<<5) | (((t->tm_sec>>2)&0x0f)<<1) | ((year>>4)&1);
    outMsg[3] = ((year&0x0f)<<4) | ((t->tm_mon+1)&0x0f);
    outMsg[4] = ((t->tm_mday&0x1f)<<3) | ((t->tm_wday)&0x7);
    SendRadioMessage(outMsg, enc?sensor->aesKey:NULL);
}


void LogMessage(sensor_t *sensor, uint8_t *decMsg)
{
    decMsg[14] = 0;
    printf("LOG: Sensor %d: %s\n", sensor->id, decMsg+1);
}


void UpdateSensorState(sensor_t *sensor, uint8_t *decMsg)
{
    printf("SENSOR %d: ", sensor->id);
    for (int i=0; i<13; i++) { 
        sensor->reportState[i] = decMsg[i+1];
        printf("0x%02x(%d) ", sensor->reportState[i], sensor->reportState[i]);
    }
    printf("\n");
    sensor->lastSeqNo = GetSequenceNum(decMsg);
}

int GetSensorID(uint8_t *msg)
{
    int id = (msg[0]) | (msg[1]<<8) | (((msg[2]>>2)<<16)&0x3f0000);
    return id;
}

void logHex(const char *str, uint8_t *b, int c)
{
    printf("%s", str);
    for (int i=0; i<c; i++) {
        printf("%02x", (int)(b[i]&0xff));
    }
    printf("\n");
}


// Only 1 master state machine for the radio listener
// No interleaving of comms transaction.
// Join starts and finishes or times out before we look at any other kinds of message
// Reports get acks, identical reports get acks.  Reports with < last sequence invalid, no ack.

void ListenerLoop()
{
    typedef enum { WAITJOIN, WAITK0S, WAITK1S, WAITK2S, WAITREPORT } state_t;
    state_t state;
    time_t joinTimeout = 0;
    uint8_t rawMsg[19];
    uint8_t msg[16], decMsg[16];

    sensor_t newSensor, *rekeySensor;
    uint8_t fm[32], km[32], ky[32];

    uint8_t outMsg[16];
    uint8_t *retryKey = NULL;
    int retriesLeft = 0;
    time_t retryTime;

    // Hold the last rekey message/ack to handle case where sensor doesn't see my first ACK
    uint8_t lastK2S[16], lastK2SACK[19];

    // Statistics
    time_t dumpTime = time(NULL)+10;
    uint32_t corrBits = 0; // Total # of corrected bits for all time
    uint32_t packetsRec =  0; // Total # of packets seen by radio.read()
    uint32_t packetsGood = 0; // # that were successfully processed
    uint32_t packetsRepeat = 0; // # that were repeats of prior sequence #s
    uint32_t packetsOOS = 0; // # that were out of sequence
    uint32_t packetsBad = 0; // # that were not able to be processed
    uint32_t packetsSent = 0; // # packets GW transmitted initially
    uint32_t packetsResent = 0; // # packets GW re-transmitted (not including Sent above)

    radio.begin();
    RadioSetupNormal(); // Radio set to proper operational channel/etc.

    state = WAITREPORT; // Start up in default mode
    newJoin = true;
    rekeySensor = NULL;

    while (1) {

        if (time(NULL) > dumpTime) {
            printf("---------------------------------------------------------------\n");
            printf("Mode: %s - now = %ld jointimeout = (%ld)\n", joinTimeout?"JOIN":"NORMAL", time(NULL), joinTimeout);
            printf("Packets Received: %d\n", packetsRec);
            printf("    Good: %d, Repeat: %d, OutOfSeq: %d, Bad: %d\n", packetsGood, packetsRepeat, packetsOOS, packetsBad);
            printf("Packets Sent: %d, Resent: %d\n", packetsSent, packetsResent);
            printf("ECC corrected bits: %d\n", corrBits);
            printf("---------------------------------------------------------------\n");
            dumpTime = time(NULL) + 10;
        }

        // If we've gone past the timeout period for JOIN operation, cancel and revert to normal ops
        if (joinTimeout) {
            if (time(NULL) > joinTimeout) {
                state = WAITREPORT;
                RadioSetupNormal();
                joinTimeout = 0;
                rekeySensor = NULL;
                continue; // Go back to the loop with new state, short-circuit
            }
        }

        // Only start the new joining process once we're idle
        if (newJoin && (state == WAITREPORT)) {
            newJoin = false;
            RadioSetupJoin();
            state = WAITJOIN;
            joinTimeout = time(NULL) + joinMaxTime;
            continue;
        }

        // If no data, sleep unless we're going to add a new sensor
        if (!radio.available()) {
            if ( (retriesLeft > 0) && (time(NULL) > retryTime) ) {
                logHex("Resend: ", outMsg, 16);
                packetsResent++;
                SendRadioMessage(outMsg, retryKey);
                retriesLeft--;
                retryTime = time(NULL) + 1;
            }
            usleep(sleepTimeUs);
            continue;
        }
        

        if (radio.available()) {
            radio.read(rawMsg, 19);
            packetsRec++;
        } else {
            continue;
        }

        uint8_t corr = CorrectMessage(msg, rawMsg);

        switch (state) {
        case WAITREPORT: 
            // Special case of K2S repeat (they didn't hear my ACK)
            CheckMessageAES(decMsg, NULL, msg); // RAW AES encrypted bits we care about
            if (!memcmp(lastK2S, decMsg, 16)) {
                logHex("Repeated K2S, resending last ACK: ", lastK2SACK, 19);
                SendRadioRawMessage(lastK2SACK);
                packetsResent++;
                break;
            }

            // Need to determine which sensor this is from, sanity check, etc.
            for (int i=0; i<sensors; i++) {
                if (CheckMessageAES(decMsg, sensor[i].aesKey, msg)) {
                    corrBits += corr;
                    // Sensor = i 
                    if (GetMessageType(decMsg) == MSG_JOIN) {
                        packetsGood++;
                        // Same as JOIN but set retryKey to current AES
                        retryKey = sensor[i].aesKey;
                        rekeySensor = &sensor[i];
                        joinTimeout = time(NULL) + joinMaxTime;
                        Curve25519::dh1(km, fm);
                        SetMessageTypeSeq(outMsg, MSG_K0G, 1);
                        memcpy(outMsg+1, km, 13);
                        logHex(">REKEY-K0G : ", outMsg, 16);
                        SendRadioMessage(outMsg, retryKey);
                        packetsSent++;
                        retriesLeft = 5;
                        retryTime = time(NULL) + 1;
                        state = WAITK0S;
                    } else if (sensor[i].lastSeqNo == GetSequenceNum(decMsg)) {
                        packetsRepeat++;
                        logHex("Repeated Report, resending ack:", decMsg, 16);
                        // Sensor didn't see last ack, just re-send it
                        SendSensorAck(&sensor[i], true);
                        packetsResent++;
                    } else if (sensor[i].lastSeqNo > GetSequenceNum(decMsg)) {
                        packetsOOS++;
                        // Looks like replay attack, silently drop this
                        logHex("Out of seq: ", decMsg, 16);
                    } else {
                        if (GetMessageType(decMsg) == MSG_LOG) {
                            packetsGood++;
                            LogMessage(&sensor[i], decMsg);
                            SendSensorAck(&sensor[i], true);
                            packetsSent++;
                        } else if (GetMessageType(decMsg) == MSG_REPORT) {
                            packetsGood++;
                            logHex(">REPORT: ", decMsg, 16);
                            UpdateSensorState(&sensor[i], decMsg);
                            SendSensorAck(&sensor[i], true);
                            packetsSent++;
                        } else {
                            packetsBad++;
                            logHex("Unexpected: ", decMsg, 16);
                            // Weren't expecting this, drop it
                        }
                    }
                    continue;
                } else {
                    // Unknown bad packet
                    packetsBad++;
                }
            }
            // If we get here, then we weren't able to find a sensor for this code, drop it
            break;

        case WAITJOIN:
            if (!CheckMessageAES(decMsg, NULL, msg)) {
                // CRC mismatch, throw out
                packetsBad++;
                break;
            }
            corrBits += corr;
            if (GetMessageType(decMsg) == MSG_JOIN) {
                packetsGood++;
                logHex("<JOIN: ", decMsg, 16);
                memset(&newSensor, 0, sizeof(newSensor));
                if (GetSensorID(decMsg+2)) newSensor.id = GetSensorID(decMsg+2);
                else newSensor.id = nextSensorID++;
                newSensor.joinTime = time(NULL);
                memcpy(newSensor.name, decMsg+5, 9);
                newSensor.name[9] = 0;
                printf("Sensor: %s\n", newSensor.name);
                Curve25519::dh1(km, fm);
                SetMessageTypeSeq(outMsg, MSG_K0G, 1);
                memcpy(outMsg+1, km, 13);
                logHex(">K0G : ", outMsg, 16);
                retryKey = NULL;
                SendRadioMessage(outMsg, retryKey);
                packetsSent++;
                retriesLeft = 5;
                retryTime = time(NULL) + 1;
                state = WAITK0S;
            } else {
                // Unknown, drop it
                packetsBad++;
            }
            break;


        case WAITK0S:
            if (!CheckMessageAES(decMsg, retryKey, msg)) {
                // CRC mismatch, throw out
                packetsBad++;
                break;
            }
            corrBits += corr;
            if (GetMessageType(decMsg) == MSG_K0S) {
                packetsGood++;
                memcpy(ky, decMsg+1, 13);
                logHex("<K0S: ", decMsg, 16);
                SetMessageTypeSeq(outMsg, MSG_K1G, 2);
                memcpy(outMsg+1, km+13, 13);
                logHex(">K1G: ", outMsg, 16);
                SendRadioMessage(outMsg, retryKey);
                packetsSent++;
                retriesLeft = 5;
                retryTime = time(NULL) + 1;
                state = WAITK1S;
            } else if (GetMessageType(decMsg) == MSG_JOIN) {
                packetsRepeat++;
                // Didn't hear my last msg, resend it and hope
                SendRadioMessage(outMsg, retryKey);
                packetsResent++;
                retriesLeft = 5;
                retryTime = time(NULL) + 1;
                state = WAITK0S;
            } else {
                // Something weird, drop it
            }
            break;

        case WAITK1S:
            if (!CheckMessageAES(decMsg, retryKey, msg)) {
                // CRC mismatch, throw out
                packetsBad++;
                break;
            }
            corrBits += corr;
            if (GetMessageType(decMsg) == MSG_K1S) {
                packetsGood++;
                memcpy(ky+13, decMsg+1, 13);
                logHex("<K1S: ", decMsg, 16);
                SetMessageTypeSeq(outMsg, MSG_K2G, 2);
                memcpy(outMsg+1, km+26, 6);
                outMsg[7] = netAddr & 0xff;
                outMsg[8] = (netAddr>>8) & 0xff;
                outMsg[9] = (netAddr>>16) & 0xff;
                outMsg[10] = (netHamm?0x80:0x00) | (netChan & 0x7f);
                outMsg[11] = newSensor.id & 0xff;
                outMsg[12] = (newSensor.id>>8) & 0xff;
                outMsg[13] = ((newSensor.id>>16) & 0xfc) | (netRate & 0x03);
                logHex(">K2G: ", outMsg, 16);
                SendRadioMessage(outMsg, retryKey);
                packetsSent++;
                retriesLeft = 5;
                retryTime = time(NULL) + 1;
                state = WAITK2S;
            } else if (GetMessageType(decMsg) == MSG_K0S) {
                // Didn't hear my last msg, resend it and hope
                packetsRepeat++;
                SendRadioMessage(outMsg, retryKey);
                packetsResent++;
                retriesLeft = 5;
                retryTime = time(NULL) + 1;
                state = WAITK1S;
            } else {
                // Something weird, drop it
                packetsBad++;
            }
            break;
        
        case WAITK2S:
            if (!CheckMessageAES(decMsg, retryKey, msg)) {
                // CRC mismatch, throw out
                packetsBad++;
                break;
            }
            corrBits += corr;
            if (GetMessageType(decMsg) == MSG_K2S) {
                packetsGood++;
                memcpy(ky+26, decMsg+1, 6);

                logHex("<K2S: ", decMsg, 16);

                newSensor.reportEntries = (decMsg[7]>>4) & 0x0f;
                newSensor.reportType[0] = (decMsg[7]) & 0x0f;
                newSensor.reportType[1] = (decMsg[8]>>4) & 0x0f;
                newSensor.reportType[2] = (decMsg[8]) & 0x0f;
                newSensor.reportType[3] = (decMsg[9]>>4) & 0x0f;
                newSensor.reportType[4] = (decMsg[9]) & 0x0f;
                newSensor.reportType[5] = (decMsg[10]>>4) & 0x0f;
                newSensor.reportType[6] = (decMsg[10]) & 0x0f;
                newSensor.reportType[7] = (decMsg[11]>>4) & 0x0f;
                newSensor.reportType[8] = (decMsg[11]) & 0x0f;
                newSensor.reportType[9] = (decMsg[12]>>4) & 0x0f;
                newSensor.reportType[10] = (decMsg[12]) & 0x0f;
                newSensor.reportType[11] = (decMsg[13]>>4) & 0x0f;
                newSensor.reportType[12] = (decMsg[13]) & 0x0f;
                
                newSensor.lastSeqNo = -1;

                if (Curve25519::dh2(ky, fm)) {
                    // If we can get an AES key, we're done here!  OTW this was a dud,don't add
                    memcpy(newSensor.aesKey, ky, 16);
                    if (rekeySensor) {
                        SendSensorAck(rekeySensor, true);
                        // Store this K2S message.  If we see it again (in encrypted form, we don't
                        // keep the old key, we will resend the same ACK encoded w/the old key
                        memcpy(lastK2S, msg, 16);
                        memcpy(lastK2SACK, lastBytes, 19);

                        memcpy(rekeySensor, &newSensor, sizeof(sensor_t));
                    } else {
                        sensor = (sensor_t*)realloc(sensor, sizeof(sensor_t)*(++sensors));
                        memcpy(&sensor[sensors-1], &newSensor, sizeof(newSensor));
                        // This is a new sensor, so we're on join frequency and won't see any repeated requests.
                        // So spray out several ACKs in case the 1st one isn't heard (don't mind wasting power here)
                        for (int cnt=0; cnt<10; cnt++) {
                            usleep(sleepTimeUs*2);
                            SendSensorAck(&sensor[sensors], false); // Unencrypted ACK
                        }
                    }
                    logHex("Shared Secret = ", newSensor.aesKey, 16);
                }
                if (!rekeySensor) {
                    // Rekey is on main channel
                    RadioSetupNormal();
                }
                rekeySensor = NULL;
                joinTimeout = 0;
                retriesLeft = 0;
                retryKey = NULL;
                state = WAITREPORT;
            } else if (GetMessageType(decMsg) == MSG_K1S) {
                // Didn't hear my last msg, resend it and hope
                packetsRepeat++;
                SendRadioMessage(outMsg, retryKey);
                packetsResent++;
                retriesLeft = 5;
                retryTime = time(NULL) + 1;
                state = WAITK2S;
            } else {
                // Something weird, drop it
                packetsBad++;
            }
            break;

        }

    }

}



int main(int argc, char** argv)
{

    ListenerLoop();

    return 0;
}

