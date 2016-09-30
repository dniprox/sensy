#ifndef _SWITCH_H
#define _SWITCH_H

class Switch {
public:
    Switch(uint8_t pwrPin, uint8_t inpPin, bool invert);
    Switch(uint8_t inpPin, bool invert);
    void Poll();
    bool Value();

private:
    uint8_t pwrPin;
    uint8_t inpPin;
    uint8_t invert;

    static const uint8_t samples = 4;
    bool sample[samples];
    bool lastState;
};

#endif
