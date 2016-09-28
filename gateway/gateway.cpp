#include <cstdlib>
#include <iostream>
#include <sstream>
#include <string>
#include <unistd.h>
#include <pthread.h>
#include <MQTTClient.h>
#include <json-c/json.h>
#include "RF24.h"
#include "AES.h"
#include "Curve25519.h"

#include "coding.h"
#include "message.h"
#include "radio.h"
#include "web.h"
#include "sensor.h"

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

MQTTClient mqtt;
MQTTClient_message mqttMsg = MQTTClient_message_initializer;
MQTTClient_deliveryToken mqttToken;

// Rotating log for web interface
const int logLines = 15;
char *logLine[logLines];
pthread_mutex_t mutexLog = PTHREAD_MUTEX_INITIALIZER;

// Known sensors
int sensors = 0;
sensor_t *gSensor = NULL;
pthread_mutex_t mutexSensor = PTHREAD_MUTEX_INITIALIZER; 


void logHex(const char *str, uint8_t *b, int c);
void AddLogLine(const char *c)
{
    pthread_mutex_lock( &mutexLog );
    free(logLine[0]);
    memcpy(&logLine[0], &logLine[1], (logLines-1) * sizeof(char *));
    logLine[logLines-1] = (char *)malloc(strlen(c) * 5); // Worst case everything is &amp;
    char *p = logLine[logLines-1];
    for (unsigned int i=0; i<strlen(c); i++) {
        if (c[i]=='>') { *(p++)='&'; *(p++)='g'; *(p++)='t'; *(p++)=';'; }
        else if (c[i]=='<') { *(p++)='&'; *(p++)='l'; *(p++)='t'; *(p++)=';'; }
        else if (c[i]=='&') { *(p++)='&'; *(p++)='a'; *(p++)='m'; *(p++)='p'; *(p++)=';'; }
        else *(p++) = c[i];
    }
    *(p) = 0;
    pthread_mutex_unlock( &mutexLog );
    printf(c);
}


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
    char buff[512];
    sprintf(buff, "ERROR: Disconnected from MQTT, cause='%s'\n", cause);
    AddLogLine(buff);
}


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

// Convert byte array to hex string, easier to parse than b64
const char *HexString(uint8_t *p, int c)
{
    static char s[513]; // 256 char limit
    if (c>256) return NULL; // Should SEGV app if we go rogue
    for (int i=0; i<c; i++) {
        sprintf(s+i*2, "%02X", p[i]);
    }
    return s;
}

// Scan a read string, fill byte array up to count
bool ParseHex(const char *s, int c, uint8_t *d)
{
    if ((int)strlen(s) != (int)(2*c)) return false; // Source string wrong length
    for (int i=0; i<c; i++) {
       char z[3];
       z[0] = s[i*2]; z[1] = s[1+i*2]; z[2] = 0;
       unsigned int q;
       if (sscanf(z, "%x", &q) != 1) return false; //invalid hex digit
       d[i] = (uint8_t)q;
    }
    return true;
}


// Write sensor states and keys to file
void SerializeSensors()
{
    FILE *fp = fopen("sensors.json", "w");
    if (fp) {
        json_object *jsonSensors = json_object_new_array();
        for (int i=0; i<sensors; i++) {
            if (!gSensor[i].joined) return; // Non-joined sensor, nothing to save yet
            json_object *jsonSensor = json_object_new_object();
            char buff[16];
            sprintf(buff, "%08X", gSensor[i].id);
            json_object_object_add(jsonSensor, "id", json_object_new_string(buff));
            json_object_object_add(jsonSensor, "name", json_object_new_string(gSensor[i].name));
            json_object_object_add(jsonSensor, "aesKey", json_object_new_string(HexString(gSensor[i].aesKey, 16)));
            json_object_object_add(jsonSensor, "joinTime", json_object_new_int(gSensor[i].joinTime));
            json_object_object_add(jsonSensor, "reportCount", json_object_new_int(gSensor[i].reportCount));
            json_object_object_add(jsonSensor, "reportEntries", json_object_new_int(gSensor[i].reportEntries));
            json_object *arr = json_object_new_array();
            for (int j=0; j<gSensor[i].reportEntries; j++) {
                json_object_array_add(arr, json_object_new_int(gSensor[i].reportType[j]));
            }
            json_object_object_add(jsonSensor, "reportType", arr);
            json_object *narr = json_object_new_array();
            for (int j=0; j<gSensor[i].reportEntries; j++) {
                json_object_array_add(narr, json_object_new_string(gSensor[i].reportName[j]));
            }
            json_object_object_add(jsonSensor, "reportName", narr);

            json_object_array_add(jsonSensors, jsonSensor);
        }
        json_object *jsonTop = json_object_new_object();
        json_object_object_add(jsonTop, "sensor", jsonSensors);
        fprintf(fp, "%s", json_object_to_json_string(jsonTop));
        json_object_put(jsonTop); // Free it all
        fclose(fp);
    }
}

// Clear existing sensors and load from file
void DeserializeSensors()
{
    FILE *fp = fopen("sensors.json", "r");
    sensors = 0;
    if (gSensor) free(gSensor);
    gSensor = NULL;

    if (fp) {
        long len;
        fseek(fp, 0, SEEK_END);
        len = ftell(fp);
        fseek(fp, 0, SEEK_SET);
        char *json = (char*)malloc(len+1);
        if (!json) { fclose(fp); return; }
        fread(json, 1, len, fp);
        fclose(fp);
        json[len] = 0;

        json_object *config = json_tokener_parse(json);
        json_object *sensorsAry;
        if (!json_object_object_get_ex(config, "sensor", &sensorsAry)) {
            printf("Missing sensors array in JSON\n");
            return;
        }
        if (!json_object_is_type(sensorsAry, json_type_array)) { 
            // Error!
            printf("Configuration file invalid, unable to parse.\n");
            json_object_put(config);
            return;
        }
        int cnt = json_object_array_length(sensorsAry);
        for (int i=0; i<cnt; i++) {
            json_object *s = json_object_array_get_idx(sensorsAry, i);
            sensor_t n;
            memset(&n, 0, sizeof(n));
            n.state = WAITREPORT;
            n.joined = true;
            json_object *jsonID;
            if (!json_object_object_get_ex(s, "id", &jsonID)) {
                printf("Missing sensor ID\n");
                continue;
            }
            if (sscanf(json_object_get_string(jsonID), "%x", &n.id) != 1) continue;
            json_object *jsonName;
            if (!json_object_object_get_ex(s, "name", &jsonName)) continue;
            strncpy(n.name, json_object_get_string(jsonName), sizeof(n.name));
            n.name[sizeof(n.name)-1] = 0;
            json_object *jsonAES;
            if (!json_object_object_get_ex(s, "aesKey", &jsonAES)) continue;
            if (!ParseHex(json_object_get_string(jsonAES), 16, n.aesKey)) continue;
            json_object *jsonJoin;
            if (!json_object_object_get_ex(s, "joinTime", &jsonJoin)) continue;
            if (sscanf(json_object_get_string(jsonJoin), "%ld", &n.joinTime) != 1) continue;
            json_object *jsonCnt;
            if (!json_object_object_get_ex(s, "reportCount", &jsonCnt)) continue;
            if (sscanf(json_object_get_string(jsonCnt), "%u", &n.reportCount) != 1) continue;
            json_object *jsonRE;
            if (!json_object_object_get_ex(s, "reportEntries", &jsonRE)) continue;
            if (sscanf(json_object_get_string(jsonRE), "%d", &n.reportEntries) != 1) continue;
            json_object *r;
            if (!json_object_object_get_ex(s, "reportType", &r)) continue;
            if (!json_object_is_type(r, json_type_array)) continue;
            int cnt = json_object_array_length(r);
            if ((cnt != n.reportEntries) || (cnt > 13)) continue;
            for (int j=0; j<cnt; j++) {
                json_object *q = json_object_array_get_idx(r, j);
                if (!json_object_is_type(q, json_type_int)) continue;
                n.reportType[j] = (report_t)json_object_get_int(q);
            }
            if (!json_object_object_get_ex(s, "reportName", &r)) continue;
            if (!json_object_is_type(r, json_type_array)) continue;
            cnt = json_object_array_length(r);
            if ((cnt != n.reportEntries) || (cnt > 13)) continue;
            for (int j=0; j<cnt; j++) {
                json_object *q = json_object_array_get_idx(r, j);
                if (!json_object_is_type(q, json_type_string)) continue;
                strcpy(n.reportName[j], json_object_get_string(q));
                n.reportName[j][13] = 0;
            }
            sensors++;
            gSensor=(sensor_t*)realloc(gSensor, sizeof(sensor_t)*sensors);
            memcpy(&gSensor[sensors-1], &n, sizeof(sensor_t));
        }
        json_object_put(config);

        free(json);
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
    logHex("<ACK: ", outMsg, 16);
}


void LogMessage(sensor_t *sensor, uint8_t *decMsg)
{
    decMsg[14] = 0;
    char buff[512];
    sprintf(buff, "LOG: Sensor %08X: %s\n", sensor->id, decMsg+1);
    AddLogLine(buff);
}

void Publish(sensor_t *sensor, const char *topic, const char *topicAppend, char *payload)
{
    MQTTClient_message msg = MQTTClient_message_initializer;
    msg.payload = payload;
    msg.payloadlen = strlen(payload);
    msg.qos = mqttQOS;
    msg.retained = 0;
    char fullTopic[128];
    sprintf(fullTopic, "%s/%s/%08X/%s%s", mqttTopic, sensor->name, sensor->id, topic, topicAppend);
    MQTTClient_publishMessage(mqtt, fullTopic, &msg, NULL);
    char report[128];
    sprintf(report, "'%s%s'='%s';", topic, topicAppend, payload);
    strcat(sensor->reportText, report);
}

void UpdateSensorState(sensor_t *sensor, uint8_t *decMsg)
{
    for (int i=0; i<13; i++) { 
        sensor->reportState[i] = decMsg[i+1];
    }
    sensor->lastSeqNo = GetSequenceNum(decMsg);
    sensor->reportText[0] = 0;

    // Publish them 
    char topic[64];
    char temp[64];
    char on[4];
    char off[4];
    int16_t i16;
    strcpy(on, "1");
    strcpy(off, "0");
    for (int i=0; i<sensor->reportEntries; i++) {
        if (sensor->reportName[i][0])
            strcpy(topic, sensor->reportName[i]);
        else
            sprintf(topic, "report_%d", i);
        switch (sensor->reportType[i]) {
        case ONEFLAG:
            Publish(sensor, topic, "", (sensor->reportState[i]&1)?on:off);
            break;
        case TWOFLAG:
            Publish(sensor, topic, "0", (sensor->reportState[i]&1)?on:off);
            Publish(sensor, topic, "1", (sensor->reportState[i]&2)?on:off);
            break;
        case FOURFLAG:
            Publish(sensor, topic, "0", (sensor->reportState[i]&1)?on:off);
            Publish(sensor, topic, "1", (sensor->reportState[i]&2)?on:off);
            Publish(sensor, topic, "2", (sensor->reportState[i]&4)?on:off);
            Publish(sensor, topic, "3", (sensor->reportState[i]&8)?on:off);
            break;
        case EIGHTFLAG:
            Publish(sensor, topic, "0", (sensor->reportState[i]&1)?on:off);
            Publish(sensor, topic, "1", (sensor->reportState[i]&2)?on:off);
            Publish(sensor, topic, "2", (sensor->reportState[i]&4)?on:off);
            Publish(sensor, topic, "3", (sensor->reportState[i]&8)?on:off);
            Publish(sensor, topic, "4", (sensor->reportState[i]&16)?on:off);
            Publish(sensor, topic, "5", (sensor->reportState[i]&32)?on:off);
            Publish(sensor, topic, "6", (sensor->reportState[i]&64)?on:off);
            Publish(sensor, topic, "7", (sensor->reportState[i]&128)?on:off);
            break;
        case UINT8:
        case PERCENT:
            sprintf(temp, "%u%s", (unsigned int)(sensor->reportState[i]), sensor->reportType[i]==PERCENT?"%":"");
            Publish(sensor, topic, "", temp);
            break;
        case INT8:
            sprintf(temp, "%d", (int)(sensor->reportState[i]));
            Publish(sensor, topic, "", temp);
            break;
        case UINT16:
            sprintf(temp, "%u", (unsigned int)(sensor->reportState[i]+256*sensor->reportState[i+1]));
            Publish(sensor, topic, "", temp);
            i++;
            break;
        case INT16:
            i16 = sensor->reportState[i]+256*sensor->reportState[i+1];
            sprintf(temp, "%d", (int)(i16)); i++;
            Publish(sensor, topic, "", temp);
            i++;
            break;
        case FIXPT16X10:
            i16 = sensor->reportState[i]+256*sensor->reportState[i+1];
            sprintf(temp, "%0.1f", ((double)i16)/10.0);
            Publish(sensor, topic, "", temp);
            i++;
            break;
        case FIXPT16X100:
            i16 = sensor->reportState[i]+256*sensor->reportState[i+1];
            sprintf(temp, "%0.2f", ((double)i16)/100.0);
            Publish(sensor, topic, "", temp);
            i++;
            break;
        case FRACT8:
            sprintf(temp, "%0.4f", ((double)sensor->reportState[i])/255.0);
            Publish(sensor, topic, "", temp);
            break;
        case FRACT16:
            uint16_t u16 = sensor->reportState[i]+256*sensor->reportState[i+1];
            sprintf(temp, "%0.6f", ((double)u16)/65535.0);
            Publish(sensor, topic, "", temp);
            i++;
            break;
        }
    }
}

void logHex(const char *str, uint8_t *b, int c)
{
    char buff[512];
    sprintf(buff, "%s", str);
    for (int i=0; i<c; i++) {
        char e[32];
        sprintf(e, "%02x", (int)(b[i]&0xff));
        strcat(buff, e);
    }
    strcat(buff, "\n");
    AddLogLine(buff);
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
    time_t serializeTimeout;

    MQTTInit();

    radio.begin();
    RadioSetupNormal(); // Radio set to proper operational channel/etc.

    newJoin = false;
    serializeTimeout = time(NULL) + 3600; // Dump state once an hour

    while (1) {

        if (time(NULL) > serializeTimeout) {
           pthread_mutex_lock( &mutexSensor );
           SerializeSensors();
           pthread_mutex_unlock( &mutexSensor );
           serializeTimeout = time(NULL) + 3600; // Dump state once an hour
        }

        // If we've gone past the timeout period for JOIN operation, cancel and revert to normal ops
        if (joinTimeout) {
            if (time(NULL) > joinTimeout) {
                RadioSetupNormal();
                joinTimeout = 0;
                pthread_mutex_lock( &mutexSensor );
                sensors--;
                pthread_mutex_unlock( &mutexSensor );
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
                pthread_mutex_lock( &mutexSensor );
                gSensor = (sensor_t*)realloc(gSensor, sizeof(sensor_t)*(++sensors));
                memset(&gSensor[sensors-1], 0, sizeof(sensor_t));
                gSensor[sensors-1].joined = false;
                gSensor[sensors-1].state = WAITJOIN;
                joinTimeout = time(NULL) + joinMaxTime;
                pthread_mutex_unlock( &mutexSensor );
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
                pthread_mutex_lock( &mutexSensor );
                gSensor[retrySensor].retriesLeft--;
                gSensor[retrySensor].retryTime = time(NULL) + 1;
                pthread_mutex_unlock( &mutexSensor );
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
            pthread_mutex_lock( &mutexSensor );
            if (HandleSensor(msg, &gSensor[i])) {
                corrBits += corr;
                gSensor[i].lastReport = time(NULL);
                handled = true;
            }
            pthread_mutex_unlock( &mutexSensor );
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
                    sensor->reportCount++;
                } else if (GetMessageType(decMsg) == MSG_SINFO) {
                    packetsGood++;
                    logHex(">SINFO: ", decMsg, 16);
                    memcpy(&sensor->reportName[GetSequenceNum(decMsg)], &decMsg[1], 13);
                    sensor->reportName[GetSequenceNum(decMsg)][13] = 0;
                    SendSensorAck(sensor, true, NULL);
                    packetsSent++;
                    SerializeSensors();
                    sensor->reportCount++;
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
            char buff[512];
            sprintf(buff, "Sensor: MAC: %08X, NAME=%s\n", sensor->id, sensor->name);
            AddLogLine(buff);
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
            sensor->reportType[0] = (report_t)((decMsg[7]) & 0x0f);
            sensor->reportType[1] = (report_t)((decMsg[8]>>4) & 0x0f);
            sensor->reportType[2] = (report_t)((decMsg[8]) & 0x0f);
            sensor->reportType[3] = (report_t)((decMsg[9]>>4) & 0x0f);
            sensor->reportType[4] = (report_t)((decMsg[9]) & 0x0f);
            sensor->reportType[5] = (report_t)((decMsg[10]>>4) & 0x0f);
            sensor->reportType[6] = (report_t)((decMsg[10]) & 0x0f);
            sensor->reportType[7] = (report_t)((decMsg[11]>>4) & 0x0f);
            sensor->reportType[8] = (report_t)((decMsg[11]) & 0x0f);
            sensor->reportType[9] = (report_t)((decMsg[12]>>4) & 0x0f);
            sensor->reportType[10] = (report_t)((decMsg[12]) & 0x0f);
            sensor->reportType[11] = (report_t)((decMsg[13]>>4) & 0x0f);
            sensor->reportType[12] = (report_t)((decMsg[13]) & 0x0f);
            
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
    if (!strcmp(uri, "/") || !strcmp(uri, "/index.html") || !strcmp(uri, "index.html") || !strcmp(uri, "/index.html?")) {
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
        strcat(buff, "<table border=\"1\"><tr><th>ID</th><th>TYPE</th><th>LAST REPORT</th><th>TOTAL REPORTS</th><th>REPORT</th><th>&nbsp;</th></tr>\n");
        time_t now = time(NULL);
        pthread_mutex_lock( &mutexSensor );
        for (int i=0; i<sensors; i++) {
            if (!gSensor[i].joined) continue;
            sprintf(line, "<tr><td>%08X</td><td>%s</td><td>%ld</td><td>%u</td><td>%s</td><td><form action=\"lastchance.html-id=%08X\"><input type=\"submit\" value=\"Remove\" /></form></tr>\n", gSensor[i].id, gSensor[i].name, now - gSensor[i].lastReport, gSensor[i].reportCount, gSensor[i].reportText, gSensor[i].id);
            strcat(buff, line);
        }
        pthread_mutex_unlock( &mutexSensor );
        strcat(buff, "</table></body></html>\n");
        pthread_mutex_lock( &mutexLog );
        strcat(buff, "<h1>Log</h1>\n<pre>");
        for (int i=0; i<logLines; i++) {
            strcat(buff, logLine[i]);
        }
        strcat(buff, "</pre>\n");
        pthread_mutex_unlock( &mutexLog );

        *output = buff;
        return true;
    } else if (!strcmp(uri, "/join.html")) {
        newJoin = true;
        // Redirect to main page
        *output = strdup("<html><head><title>Join Enabled</title><meta http-equiv=\"refresh\" content=\"0;URL='index.html'\"/></head><body><a href=\"index.html\">Back to status page</a></body></html>");
        return true;
    } else if (!strncmp(uri, "/lastchance.html-id=", 19)) {
        uint32_t id;
        if (sscanf(uri, "/lastchance.html-id=%x", &id) != 1) {
            return false;
        }

        char *buff = (char*)malloc(65536);
        if (!buff) return false;
        buff[0] = 0;

        char line[8192];
        strcat(buff, "<html>\n");
        sprintf(line, "<head><meta http-equiv=\"refresh\" content=\"10\"><title>Sensy Gateway: %s</title></head>\n", mqttTopic);
        strcat(buff, line);
        strcat(buff, "<body>\n");
        sprintf(line, "Do you really want to remove sensor %08X?<br>\n", id);
        strcat(buff, line);
        sprintf(line, "<table><tr><td><form action=\"remove.html-id=%08X\"><input type=\"submit\" value=\"Yes\" /></form></td>", id);
        strcat(buff, line);
        sprintf(line, "<td><form action=\"index.html\"><input type=\"submit\" value=\"No\" /></form></td></tr></table><br>\n");
        strcat(buff, line);
        strcat(buff, "</table></body></html>\n");
        
        *output = buff;
        return true;
    } else if (!strncmp(uri, "/remove.html-id=", 15)) {
        uint32_t id;
        if (sscanf(uri, "/remove.html-id=%x", &id) != 1) {
            return false;
        }
        pthread_mutex_lock( &mutexSensor );
        int j=0;
	for (int i=0; i<sensors; i++) {
            if (gSensor[i].id!=id) { j++; }
            else {
                sensors--;
                memcpy(&gSensor[j], &gSensor[i+1], sizeof(sensor_t)*sensors);
            }
        }
        pthread_mutex_unlock( &mutexSensor );
        *output = strdup("<html><head><title>Join Enabled</title><meta http-equiv=\"refresh\" content=\"0;URL='index.html'\"/></head><body><a href=\"index.html\">Back to status page</a></body></html>");
        return true;
    } else {
        return false;
    }
}
int main(int argc, char** argv)
{
    for (int i=0; i<logLines; i++) logLine[i] = strdup("");

    // TODO load settings from /etc/sensy.cfg

    DefineNetwork( netAddr, netChan, netRate );
    DeserializeSensors();
    SerializeSensors();

    StartWebserver(1000, WebStatus, "admin", "admin"); // TODO - from config

    ListenerLoop();

    return 0;
}

