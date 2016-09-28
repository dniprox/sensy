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
    MSG_SINFO = 5,
    MSG_LOG = 7
} messageType_t;

messageType_t GetMessageType(const uint8_t msg[16]);
uint8_t GetSequenceNum(const uint8_t msg[16]);
void SetMessageTypeSeq(uint8_t msg[16], messageType_t type, int seq);
uint32_t GetSensorID(uint8_t *msg);


// Types of returned values from sensor
typedef enum {
    ONEFLAG=0,   // Single bit (0) for switch
    TWOFLAG,     // Two single bits (1, 0) for 2 switches
    FOURFLAG,    // etc.
    EIGHTFLAG,   // etc.
    UINT8,       // 8-bit unsigned integer
    INT8,        // 8-bit signed integet
    UINT16,      // 16-bit unsigned integer
    INT16,       // 16-bit signed integer
    PERCENT,     // 0-100 in a byte representing %
    FIXPT16X10,  // Signed int16 * 10: value = (double)data / 10.0
    FIXPT16X100, // Signed int16 * 100: value = (double)data / 100.0
    FRACT8,      // Unsigned 8-bit fraction: value = (double)data / 255.0
    FRACT16      // Unsigned 16-bit fraction: value = (souble)data / 65535.0
} report_t;

#endif
