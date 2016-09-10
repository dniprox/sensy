// Message and radio defaults and handling utilities

// To join, use these hardcoded settings
const uint32_t        joinAddr = 0xabcd12;
const bool            joinHamm = true;
const uint8_t         joinChan = 0;
const rf24_datarate_e joinRate = RF24_250KBPS;


typedef enum { MSG_JOIN=0, MSG_K0G, MSG_K0S, MSG_K1G, MSG_K1S, MSG_K2G, MSG_K2S, MSG_REPORT, MSG_ACK, MSG_LOG=15 } messageType_t;


