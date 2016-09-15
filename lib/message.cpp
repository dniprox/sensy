#include <stdint.h>
#include "RF24.h"
#include "message.h"

messageType_t GetMessageType(const uint8_t msg[16])
{
    return (messageType_t)(0x07 & (msg[0]>>5));
}

uint8_t GetSequenceNum(const uint8_t msg[16])
{
    return (msg[0] & 0x1f);
}

void SetMessageTypeSeq(uint8_t msg[16], messageType_t type, int seq)
{
    msg[0] = ((type&0x07) << 5) | (seq & 0x1f);
}

