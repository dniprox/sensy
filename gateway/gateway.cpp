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
#include "radio.h"
#include "web.h"

// My network 
static uint32_t        netAddr = 0x21dcba;
static uint8_t         netChan = 0;
static rf24_datarate_e netRate = RF24_2MBPS;

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



typedef int report_t;


typedef enum { WAITJOIN, WAITK0S, WAITK1S, WAITK2S, WAITREPORT } state_t;
typedef struct {
    bool joined;
    state_t state;
    uint32_t     id;
    char    name[10];

    uint8_t aesKey[16];

    uint8_t lastK2S[16]; // Last receive K2S to check if we re-hear it (they didn't hear our ACK)
    uint8_t lastK2SACK[19]; // if we need to resend the last ACK to a K2S

    time_t  joinTime;
    time_t  lastReport;
    time_t  retryTime;

    int      reportEntries;
    report_t reportType[13];  // What fields we have
    uint8_t  reportState[13];   // The raw data

    int     lastSeqNo;      // Last sequence number we received
    uint8_t lastRecvMessage[16]; // Last message to come in from the sensor
    uint8_t outMessage[16];      // Message to sensor
    time_t  lastRetryTime;       // When we last resent it
    int     retriesLeft;         // If any retries left


    uint8_t fm[32], km[32], ky[32];
    uint8_t outMsg[16];
} sensor_t;

int sensors = 0;
sensor_t *gSensor = NULL;

void SerializeSensors()
{
    FILE *fp = fopen("sensors.bin", "wb");
    if (fp) {
        fwrite(&sensors, sizeof(sensors), 1, fp);
        fwrite(gSensor, sizeof(sensor_t), sensors, fp);
        fclose(fp);
    }
}

void DeserializeSensors()
{
    FILE *fp = fopen("sensors.bin", "rb");
    if (fp) {
        fread(&sensors, sizeof(sensors), 1, fp);
        gSensor = (sensor_t*)malloc(sizeof(sensor_t) * sensors);
        fread(gSensor, sizeof(sensor_t), sensors, fp);
        fclose(fp);
    }
}

void DumpSensors()
{
    printf("%8s %-10s %-10s\n", "ID", "TYPE", "LASTREPORT");
    time_t now = time(NULL);
    for (int i=0; i<sensors; i++) {
        printf("%08X %-10s %-10ld\n", gSensor[i].id, gSensor[i].name, now - gSensor[i].lastReport);
    }
}


volatile bool newJoin = 0; // external thread sets to 1 to initiate a join process
const int joinMaxTime = 120; // # of seconds to stay in join state before returning to listener
const int rekeyMaxTime = 120; // # of seconds to stay in join state before returning to listener
const int sleepTimeUs = 10000; // 1/2 of a message transmission time (1/250K * 19*8) in microseconds


void SendSensorAck(sensor_t *sensor, bool enc, uint8_t *rawSent)
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
    SendRadioMessage(outMsg, enc?sensor->aesKey:NULL, rawSent);
}


void LogMessage(sensor_t *sensor, uint8_t *decMsg)
{
    decMsg[14] = 0;
    printf("LOG: Sensor %d: %s\n", sensor->id, decMsg+1);
}

void UpdateSensorState(sensor_t *sensor, uint8_t *decMsg)
{
    printf("SENSOR %08X: ", sensor->id);
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
        sprintf(fullTopic, "%s/%s/%08X/%s", mqttTopic, sensor->name, sensor->id, topic);
        printf("Publishing: '%s'='%s'\n", fullTopic, payload);
        MQTTClient_publishMessage(mqtt, fullTopic, &msg, NULL);
    }
}

uint32_t GetSensorID(uint8_t *msg)
{
    uint32_t id = (msg[0]) | (msg[1]<<8) | (msg[2]<<16) | (msg[3]<<24);
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

bool HandleSensor(uint8_t rawMsg[16], sensor_t *sensor);

    // Statistics
    uint32_t corrBits = 0; // Total # of corrected bits for all time
    uint32_t packetsRec =  0; // Total # of packets seen by radio.read()
    uint32_t packetsGood = 0; // # that were successfully processed
    uint32_t packetsRepeat = 0; // # that were repeats of prior sequence #s
    uint32_t packetsOOS = 0; // # that were out of sequence
    uint32_t packetsBad = 0; // # that were not able to be processed
    uint32_t packetsSent = 0; // # packets GW transmitted initially
    uint32_t packetsResent = 0; // # packets GW re-transmitted (not including Sent above)

time_t joinTimeout = 0;


void ListenerLoop()
{
    uint8_t rawMsg[19], msg[16];

    int retrySensor = 0;

    MQTTInit();


    radio.begin();
    RadioSetupNormal(); // Radio set to proper operational channel/etc.

    newJoin = false;

    while (1) {
        // If we've gone past the timeout period for JOIN operation, cancel and revert to normal ops
        if (joinTimeout) {
            if (time(NULL) > joinTimeout) {
                RadioSetupNormal();
                joinTimeout = 0;
                sensors--;
                continue; // Go back to the loop with new state, short-circuit
            }
        }

        // Only start the new joining process once we're idle
        if (newJoin) {
            bool idle = true;
            for (int i=0; idle && (i<sensors); i++) {
                if (gSensor[i].state != WAITREPORT) idle = false;  // Somebody waiting on something
                if (!gSensor[i].joined) idle=false; // Already joining
            }
            if (idle) {
                // Add empty sensor at end of list for this...
                newJoin = false;
                RadioSetupJoin();
                gSensor = (sensor_t*)realloc(gSensor, sizeof(sensor_t)*(++sensors));
                memset(&gSensor[sensors-1], 0, sizeof(sensor_t));
                gSensor[sensors-1].joined = false;
                gSensor[sensors-1].state = WAITJOIN;
                joinTimeout = time(NULL) + joinMaxTime;
                continue;
            }
        }

        // If no data, check for some retries pending
        if (!radio.available() && sensors) {
            retrySensor = (retrySensor+1) % sensors;
            if ( (gSensor[retrySensor].retriesLeft > 0) && (time(NULL) > gSensor[retrySensor].retryTime) ) {
                logHex("Resend: ", gSensor[retrySensor].outMsg, 16);
                packetsResent++;
                SendRadioMessage(gSensor[retrySensor].outMsg, gSensor[retrySensor].joined ? gSensor[retrySensor].aesKey : NULL, NULL);
                gSensor[retrySensor].retriesLeft--;
                gSensor[retrySensor].retryTime = time(NULL) + 1;
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
        bool handled = false;
        for (int i=0; (!handled) && (i < sensors); i++) {
            // Anyone got this?
            if (HandleSensor(msg, &gSensor[i])) {
                corrBits += corr;
                gSensor[i].lastReport = time(NULL);
                handled = true;
            }
        }
        if (!handled) {
            // Nobody got it.  Sounds like garbage to me...
            packetsBad++;
        }
    }
}


bool HandleSensor(uint8_t msg[16], sensor_t *sensor)
{
    uint8_t decMsg[16];

    if (sensor->state == WAITREPORT) { // The standard listening loop
        // Special case of K2S repeat (they didn't hear my ACK)
        if (CheckMessageAES(decMsg, NULL, msg)) {  // RAW AES encrypted bits we care about
            if (!memcmp(sensor->lastK2S, decMsg, 16)) {
                logHex("Repeated K2S, resending last ACK: ", sensor->lastK2SACK, 19);
                SendRadioRawMessage(sensor->lastK2SACK);
                packetsGood++;
                packetsResent++;
                return true;
            }
        }

        if (CheckMessageAES(decMsg, sensor->aesKey, msg)) {
            // Prior encoding key goes away now...
            memset(sensor->lastK2S, 0, 16);
            memset(sensor->lastK2SACK, 0, 19);

            if (GetMessageType(decMsg) == MSG_JOIN) {
                packetsGood++;
                // Same as JOIN but set retryKey to current AES
                Curve25519::dh1(sensor->km, sensor->fm);
                SetMessageTypeSeq(sensor->outMsg, MSG_K0G, 1);
                memcpy(sensor->outMsg+1, sensor->km, 13);
                logHex(">REKEY-K0G : ", sensor->outMsg, 16);
                SendRadioMessage(sensor->outMsg, sensor->aesKey, NULL);
                packetsSent++;
                sensor->retriesLeft = 5;
                sensor->retryTime = time(NULL) + 1;
                sensor->state = WAITK0S;
            } else if (sensor->lastSeqNo == GetSequenceNum(decMsg)) {
                packetsRepeat++;
                logHex("Repeated Report, resending ack:", decMsg, 16);
                // Sensor didn't see last ack, just re-send it
                SendSensorAck(sensor, true, NULL);
                packetsResent++;
            } else if (sensor->lastSeqNo > GetSequenceNum(decMsg)) {
                packetsOOS++;
                // Looks like replay attack, silently drop this
                logHex("Out of seq: ", decMsg, 16);
            } else {
                if (GetMessageType(decMsg) == MSG_LOG) {
                    packetsGood++;
                    LogMessage(sensor, decMsg);
                    SendSensorAck(sensor, true, NULL);
                    packetsSent++;
                } else if (GetMessageType(decMsg) == MSG_REPORT) {
                    packetsGood++;
                    logHex(">REPORT: ", decMsg, 16);
                    UpdateSensorState(sensor, decMsg);
                    SendSensorAck(sensor, true, NULL);
                    packetsSent++;
                } else {
                    packetsBad++;
                    logHex("Unexpected in WAITREPORT: ", decMsg, 16);
                    // Weren't expecting this, drop it
                }
            }
            return true;
        } 
        // If we get here, then we weren't able to find a sensor for this code, drop it
        return false;
    }

    if (!CheckMessageAES(decMsg, sensor->joined?sensor->aesKey:NULL, msg)) {
        // CRC mismatch, throw out
        return false;
    }

    // Join sequence state machine
    if (sensor->state == WAITJOIN) {
        if (GetMessageType(decMsg) == MSG_JOIN) {
            packetsGood++;
            logHex("<JOIN: ", decMsg, 16);
            sensor->id = GetSensorID(decMsg+2);
            sensor->joinTime = time(NULL);
            memcpy(sensor->name, decMsg+6, 8);
            sensor->name[8] = 0;
            printf("Sensor: MAC: %08X, NAME=%s\n", sensor->id, sensor->name);
            Curve25519::dh1(sensor->km, sensor->fm);
            SetMessageTypeSeq(sensor->outMsg, MSG_K0G, 1);
            memcpy(sensor->outMsg+1, sensor->km, 13);
            logHex(">K0G : ", sensor->outMsg, 16);
            SendRadioMessage(sensor->outMsg, NULL, NULL);
            packetsSent++;
            sensor->retriesLeft = 5;
            sensor->retryTime = time(NULL) + 1;
            sensor->state = WAITK0S;
        } else {
            // Unknown, drop it
            packetsBad++;
            logHex("Unexpected in JOIN: ", decMsg, 16);
        }
        return true;
    }


    if (sensor->state == WAITK0S) {
        if (GetMessageType(decMsg) == MSG_K0S) {
            packetsGood++;
            memcpy(sensor->ky, decMsg+1, 13);
            logHex("<K0S: ", decMsg, 16);
            SetMessageTypeSeq(sensor->outMsg, MSG_K1G, 2);
            memcpy(sensor->outMsg+1, sensor->km+13, 13);
            logHex(">K1G: ", sensor->outMsg, 16);
            SendRadioMessage(sensor->outMsg, sensor->joined?sensor->aesKey:NULL, NULL);
            packetsSent++;
            sensor->retriesLeft = 5;
            sensor->retryTime = time(NULL) + 1;
            sensor->state = WAITK1S;
        } else if (GetMessageType(decMsg) == MSG_JOIN) {
            packetsRepeat++;
            // Didn't hear my last msg, resend it and hope
            SendRadioMessage(sensor->outMsg, sensor->joined?sensor->aesKey:NULL, NULL);
            packetsResent++;
            sensor->retriesLeft = 5;
            sensor->retryTime = time(NULL) + 1;
            sensor->state = WAITK0S;
        } else {
            // Something weird, drop it
            packetsBad++;
            logHex("Unexpected in WAITK0S: ", decMsg, 16);
        }
        return true;
    }

    if (sensor->state == WAITK1S) {
        if (GetMessageType(decMsg) == MSG_K1S) {
            packetsGood++;
            memcpy(sensor->ky+13, decMsg+1, 13);
            logHex("<K1S: ", decMsg, 16);
            SetMessageTypeSeq(sensor->outMsg, MSG_K2G, 2);
            memcpy(sensor->outMsg+1, sensor->km+26, 6);
            sensor->outMsg[7] = netAddr & 0xff;
            sensor->outMsg[8] = (netAddr>>8) & 0xff;
            sensor->outMsg[9] = (netAddr>>16) & 0xff;
            sensor->outMsg[10] = (netChan & 0x7f);
            sensor->outMsg[11] = 0; //
            sensor->outMsg[12] = 0; //
            sensor->outMsg[13] = (netRate & 0x03);
            logHex(">K2G: ", sensor->outMsg, 16);
            SendRadioMessage(sensor->outMsg, sensor->joined?sensor->aesKey:NULL, NULL);
            packetsSent++;
            sensor->retriesLeft = 5;
            sensor->retryTime = time(NULL) + 1;
            sensor->state = WAITK2S;
        } else if (GetMessageType(decMsg) == MSG_K0S) {
            // Didn't hear my last msg, resend it and hope
            packetsRepeat++;
            SendRadioMessage(sensor->outMsg, sensor->joined?sensor->aesKey:NULL, NULL);
            packetsResent++;
            sensor->retriesLeft = 5;
            sensor->retryTime = time(NULL) + 1;
            sensor->state = WAITK1S;
        } else {
            // Something weird, drop it
            packetsBad++;
            logHex("Unexpected in WAITK1S: ", decMsg, 16);
        }
        return true;
    }
    
    if (sensor->state == WAITK2S) {
        if (GetMessageType(decMsg) == MSG_K2S) {
            packetsGood++;
            memcpy(sensor->ky+26, decMsg+1, 6);

            logHex("<K2S: ", decMsg, 16);

            sensor->reportEntries = (decMsg[7]>>4) & 0x0f;
            sensor->reportType[0] = (decMsg[7]) & 0x0f;
            sensor->reportType[1] = (decMsg[8]>>4) & 0x0f;
            sensor->reportType[2] = (decMsg[8]) & 0x0f;
            sensor->reportType[3] = (decMsg[9]>>4) & 0x0f;
            sensor->reportType[4] = (decMsg[9]) & 0x0f;
            sensor->reportType[5] = (decMsg[10]>>4) & 0x0f;
            sensor->reportType[6] = (decMsg[10]) & 0x0f;
            sensor->reportType[7] = (decMsg[11]>>4) & 0x0f;
            sensor->reportType[8] = (decMsg[11]) & 0x0f;
            sensor->reportType[9] = (decMsg[12]>>4) & 0x0f;
            sensor->reportType[10] = (decMsg[12]) & 0x0f;
            sensor->reportType[11] = (decMsg[13]>>4) & 0x0f;
            sensor->reportType[12] = (decMsg[13]) & 0x0f;
            
            sensor->lastSeqNo = -1;

            if (Curve25519::dh2(sensor->ky, sensor->fm)) {
                // If we can get an AES key, we're done here!  OTW this was a dud,don't add
                if (sensor->joined) {
                    uint8_t rawSent[19];
                    SendSensorAck(sensor, true, rawSent);
                    // Store this K2S message.  If we see it again (in encrypted form, we don't
                    // keep the old key, we will resend the same ACK encoded w/the old key
                    memcpy(sensor->lastK2S, msg, 16);
                    memcpy(sensor->lastK2SACK, rawSent, 19);
                } else {
                    // This is a new sensor, so we're on join frequency and won't see any repeated requests.
                    // So spray out several ACKs in case the 1st one isn't heard (don't mind wasting power here)
                    for (int cnt=0; cnt<10; cnt++) {
                        usleep(sleepTimeUs*2);
                        SendSensorAck(sensor, false, NULL); // Unencrypted ACK
                    }

                    // In the case of re-joining of an existing sensor, overwrite any internal
                    // state for the sensor w/the new one and remove the "joined" copy
                    for (int i=0; i<sensors-1; i++) {
                        if (gSensor[i].id == sensor->id) {
                            memcpy(&gSensor[i], sensor, sizeof(sensor_t));
                            sensors--;
                            sensor = &gSensor[i];
                        }
                    }
                }
                memcpy(sensor->aesKey, sensor->ky, 16);
                logHex("Shared Secret = ", sensor->aesKey, 16);
            }
            if (!sensor->joined) {
                // Rekey is on main channel
                RadioSetupNormal();
            }
            joinTimeout = 0;
            sensor->retriesLeft = 0;
            sensor->state = WAITREPORT;
            sensor->joined = true;
            SerializeSensors();
        } else if (GetMessageType(decMsg) == MSG_K1S) {
            // Didn't hear my last msg, resend it and hope
            packetsRepeat++;
            SendRadioMessage(sensor->outMsg, sensor->joined?sensor->aesKey:NULL, NULL);
            packetsResent++;
            sensor->retriesLeft = 5;
            sensor->retryTime = time(NULL) + 1;
            sensor->state = WAITK2S;
        } else {
            // Something weird, drop it
            packetsBad++;
            logHex("Unexpected in WAITK2S: ", decMsg, 16);
        }
        return true;
    }

    return false;
}


bool WebStatus(const char *uri, char **output)
{
    // Only handle main page for now
    if (strcmp(uri, "/") && strcmp(uri, "/index.html") && strcmp(uri, "index.html") && strcmp(uri, "/join.html"))
        return false;

    if (!strcmp(uri, "/join.html")) {
        newJoin = true;
        // Redirect to main page
        *output = strdup("<html><head><title>Join Enabled</title><meta http-equiv=\"refresh\" content=\"0;URL='index.html'\"/></head><body><a href=\"index.html\">Back to status page</a></body></html>");
        return true;
    }

    char *buff = (char*)malloc(65536);
    if (!buff) return false;

    buff[0] = 0;
    char line[8192];
    strcat(buff, "<html>\n");
    sprintf(line, "<head><meta http-equiv=\"refresh\" content=\"10\"><title>Sensy Gateway: %s</title></head>\n", mqttTopic);
    strcat(buff, line);
    strcat(buff, "<body>\n");
    strcat(buff, "<form action=\"join.html\"><input type=\"submit\" value=\"Join\" /></form>");
    strcat(buff, "<h1>Statistics</h1>\n");
    sprintf(line, "Mode: %s<br>\n", joinTimeout?"JOIN":"NORMAL");
    strcat(buff, line);
    sprintf(line, "Packets Received: %d (Good: %d, Repeat: %d, OutOfSeq: %d, Bad: %d)<br>\n",
                  packetsRec, packetsGood, packetsRepeat, packetsOOS, packetsBad);
    strcat(buff, line);
    sprintf(line, "Packets Sent: %d, Resent: %d<br>\n", packetsSent, packetsResent);
    strcat(buff, line);
    sprintf(line, "ECC corrected bits: %d<br><br>\n", corrBits);
    strcat(buff, line);
    strcat(buff, "<h1>Sensors</h1>\n");
    strcat(buff, "<table border=\"1\"><tr><th>ID</th><th>TYPE</th><th>LAST REPORT</th></tr>\n");
    time_t now = time(NULL);
    for (int i=0; i<sensors; i++) {
        sprintf(line, "<tr><td>%08X</td><td>%10s</td><td>%10ld</td></tr>\n", gSensor[i].id, gSensor[i].name, now - gSensor[i].lastReport);
        strcat(buff, line);
    }
    strcat(buff, "</table></body></html>\n");

    *output = buff;
    return true;
}

int main(int argc, char** argv)
{
    // TODO load settings from /etc/sensy.cfg

    DefineNetwork( netAddr, netChan, netRate );
    DeserializeSensors();

    StartWebserver(1000, WebStatus);

    ListenerLoop();

    return 0;
}

