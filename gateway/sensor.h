#ifndef _SENSOR_H
#define _SENSOR_H

// Sensor state machine state
typedef enum { WAITJOIN, WAITK0S, WAITK1S, WAITK2S, WAITREPORT } state_t;

// Sensor total state
typedef struct {
    bool     joined;     // Have we sync'd up with an AES key?
    state_t  state;      // Gateway state machine

    uint32_t id;         // MAC/ID of sensor
    char     name[10];   // Name/Type reported by sensor

    uint8_t  aesKey[16]; // AES encryption key we're using now
    int      lastSeqNo;  // Last sequence number we received (protect against replay attack)

    uint8_t  lastK2S[16];    // Last receive K2S to check if we re-hear it (they didn't hear our ACK)
    uint8_t  lastK2SACK[19]; // if we need to resend the last ACK to a K2S

    time_t   joinTime;    // When sensor was first added
    time_t   lastReport;  // Last time we got a dump from it
    time_t   retryTime;   // If we're sending and haven't heard an ACK by here, resend
    int      retriesLeft; // If any retries left

    int      reportEntries;   // How many fields 
    report_t reportType[13];  // Field type
    uint8_t  reportState[13]; // The raw data from last sensor report
    char     reportName[13][14]; // Friendly name for each sensor report
    uint32_t reportCount;     // Number of reports we've heard since joining
    char     reportText[1024]; // Text-format report for simple web iface
    uint8_t fm[32], km[32], ky[32]; // Used for key exchange/generation

    uint8_t outMsg[16]; // Last message sent to sensor
} sensor_t;

extern int sensors;
extern sensor_t *gSensor;
extern pthread_mutex_t mutexSensor;

#endif
