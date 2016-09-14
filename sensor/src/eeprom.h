// EEPROM storage definitions

typedef enum {
    // Skipping first few bytes as they may be used for OSCAL or others

    // "MAC" address, auto-generated from RNG on first power-up
    EEPROM_MAC_0 = 10,
    EEPROM_MAC_1 = 11,
    EEPROM_MAC_2 = 12,
    EEPROM_MAC_3 = 13,
    EEPROM_MAC_CK = 14,  // XOR of all values and 0xbe

} eepromAddress_t;

