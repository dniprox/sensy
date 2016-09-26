#ifndef _MESSAGE_H
#define _MESSAGE_H

// Message defaults and handling utilities

typedef enum {
    MSG_JOIN = 0,
    MSG_K0G = 1,
    MSG_K0S = 1,
    MSG_K1G = 2,
    MSG_K1S = 2,
    MSG_K2G = 3,
    MSG_K2S = 3,
    MSG_REPORT = 4,
    MSG_ACK = 4,
    MSG_LOG = 7
} messageType_t;

messageType_t GetMessageType(const uint8_t msg[16]);
uint8_t GetSequenceNum(const uint8_t msg[16]);
void SetMessageTypeSeq(uint8_t msg[16], messageType_t type, int seq);
uint32_t GetSensorID(uint8_t *msg);


typedef enum { BATTERY=0, SWITCH, TEMP, ANALOG, ANALOG16X10 } report_t;

#endif
