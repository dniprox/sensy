#include <cstdlib>
#include <iostream>
#include <sstream>
#include <string>
#include <unistd.h>
#include <pthread.h>
#include <MQTTClient.h>
#include "RF24.h"
#include "AES.h"
#include "Curve25519.h"

#include "coding.h"
#include "message.h"


RF24 radio(22,0);

// TODO:  Following come from .conf file

// Real network config
uint32_t        netAddr = 0x21dcba;
bool            netHamm = true;
uint8_t         netChan = 0;
rf24_datarate_e netRate = RF24_2MBPS;


// MQTT broker, topic
const char *mqttURL = "tcp://localhost:1883";
const char *mqttID = "sensy";
const char *mqttTopic = "sensy1";
int mqttQOS = 2;
long mqttTimeout = 10000;


void MQTTDelivered(void *ctx, MQTTClient_deliveryToken dt)
{
    // Nothing to do here, yet...
}


int MQTTArrived(void *ctx, char *topic, int topicLen, MQTTClient_message *msg)
{
    // Not subscribing to anything just yet
    return 1;
}

void MQTTDisconnect(void *ctx, char *cause)
{
    // This is bad....set flag to retry TODO
    printf("ERROR: Disconnected from MQTT, cause='%s'\n", cause);
}

MQTTClient mqtt;
MQTTClient_message mqttMsg = MQTTClient_message_initializer;
MQTTClient_deliveryToken mqttToken;

void MQTTInit()
{
    int rc;
    MQTTClient_connectOptions mqttOpts = MQTTClient_connectOptions_initializer;

    MQTTClient_create(&mqtt, mqttURL, mqttID, MQTTCLIENT_PERSISTENCE_DEFAULT, NULL);
    mqttOpts.keepAliveInterval = 20;
    mqttOpts.cleansession = 1;

    MQTTClient_setCallbacks(mqtt, NULL, MQTTDisconnect, MQTTArrived, MQTTDelivered);

    if ((rc = MQTTClient_connect(mqtt, &mqttOpts)) != MQTTCLIENT_SUCCESS)
    {
        printf("Failed to connect, return code %d\n", rc);
        exit(-1);       
    }
}



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


static uint8_t lastBytes[19];
void SendRadioMessage(uint8_t msg[16], uint8_t key[16])
{
    uint8_t bytes[19];

    CodeMessage(bytes, msg, key, true);
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

    // Publish them 
    MQTTClient_message msg = MQTTClient_message_initializer;
    char payload[64];
    char topic[64];
    int battCnt = 0;
    int swCnt = 0;
    int tempCnt = 0;
    int anCnt = 0;
    for (int i=0; i<sensor->reportEntries; i++) {
        switch (sensor->reportType[i]) {
        case 0:
            sprintf(payload, "%d %%", sensor->reportState[i]);
            sprintf(topic, "battery%d", battCnt++);
            break; 
        case 1:
            sprintf(payload, "%d", sensor->reportState[i]?1:0);
            sprintf(topic, "switch%d", swCnt++);
            break;
        case 2:
            sprintf(payload, "%d C", sensor->reportState[i]);
            sprintf(topic, "temp%d", tempCnt++);
            break;
        case 3:
            sprintf(payload, "%d", sensor->reportState[i]);
            sprintf(topic, "analog%d", anCnt++);
            break;
        }
        msg.payload = payload;
        msg.payloadlen = strlen(payload);
        msg.qos = mqttQOS;
        msg.retained = 0;
        char fullTopic[128];
        sprintf(fullTopic, "%s/%d/%s", mqttTopic, sensor->id, topic);
        printf("Publishing: '%s'='%s'\n", fullTopic, payload);
        MQTTClient_publishMessage(mqtt, fullTopic, &msg, NULL);
    }
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


    MQTTInit();


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

