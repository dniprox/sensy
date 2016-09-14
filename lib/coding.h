

uint8_t CorrectMessage(uint8_t dst[16], uint8_t src[19]);
bool CheckMessageAES(uint8_t decMsg[16], const uint8_t aesKey[16], const uint8_t encMsg[16]);
void CodeMessage(uint8_t done[19], uint8_t outMsg[16], uint8_t key[16]);


