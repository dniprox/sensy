#ifndef _INTERNALSENSORS_H
#define _INTERNALSENSORS_H


class InternalSensors {
public:
    InternalSensors(); // No pins to configure

    void Poll();
    uint16_t TempValue();
    uint16_t VoltValue();

private:
    uint16_t Value(uint16_t *sample);
    uint8_t polls;
    static const uint8_t samples = 4;
    uint16_t tempSample[samples];
    uint16_t voltSample[samples];
};


#endif
