#include <avr/interrupt.h>
#include <avr/power.h>
#include <avr/sleep.h>
#include <avr/wdt.h> 
#include <pins_arduino.h>
#include <SPI.h>
#include <EEPROM.h>
#include "AES.h"
#include "Curve25519.h"
#include "RNG.h"
#include "RF24.h"
#include "rng.h"
#include "clock.h"
#include "eeprom.h"
#include "thermistor.h"
#include "switch.h"
// INO has "some" issues, just include the source here...
#include "../lib/coding.h"
#include "../lib/coding.cpp"
#include "../lib/message.h"
#include "../lib/message.cpp"
#include "../lib/radio.h"
#include "../lib/radio.cpp"


// Serial debugging
#define debug 1 // Serial logging
#define debugrng 0 // RNG dump to check entropy
#define debugtemp 0 // Thermistor readout

const uint8_t retriesJoin = 20;
const uint8_t retriesReport = 15;

bool joined = false;
uint8_t seqNo = 0;
uint8_t aesKey[16] = {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};
uint8_t sensorMAC[4] = {0,0,0,0};

// Reported time at the gateway for last ACK
uint8_t  gwHour = 0;
uint8_t  gwMin = 0;
uint8_t  gwSec = 0;
uint16_t gwYear = 0;
uint8_t  gwMon = 0;
uint8_t  gwDay = 0;
uint8_t  gwWeekday = 0;

// Disable everything we can think of in the ADC block
void ShutdownADC()
{
    power_adc_enable();
    ADMUX = 0x00;
    ADCSRA = 0x07;
    ADCSRB = 0x00;
    DIDR0 = 0xff;
    DIDR1 = 0x03;
    ACSR = 0x80;
    power_adc_disable();
}


// Reads the 2 interesting internal sensors to determine, roughly, VCC(Vbatt) and Tambient
// Combined into 2 function to allow sharing of ADC power-up times
// Return a packed 16-bit quantity as I'm not sure if gcc-avr can return a struct w/o the stack
// MSB = (vcc-1.0) * 100 (i.e.  VCC=2.9V => return = (2.9-1.0) * 100 = 190
// LSB = (TEMP in C)
uint16_t ReadInternalBatteryTempFast()
{
    uint16_t batt;
    uint16_t temp;

    power_adc_enable();  // Need to be powered on before we write any registers

    ADCSRA = 0x83; //  ADCEN=1,PS=/8 (1MHz w/8MHz clock)...will be lower quality, but so is the input signal

    // Set bandgap input
    ADMUX = _BV(REFS0) | _BV(MUX3) | _BV(MUX2) | _BV(MUX1);
    ClockDelay(5);

    // And...start ADC and busy wait until it's done
    ADCSRA |= _BV(ADSC);
    while (bit_is_set(ADCSRA,ADSC)) {/*busy wait */ }

    // Read it out
    uint8_t l = ADCL;
    uint8_t h = ADCH;
    uint16_t res = (h<<8) | l;

    // Scale results:  VCC=1023L, Vref=1100mV;
    // Vref(mV)/Vref(cnt) = VCC(mV)/1023 => Vref(mV)*1023/Vref(cnt) = VCC(mV)
    batt = (1100L*1023L) / res; // 1100mV, 2^10 as max
    
    // If 2.4V or below need to run at 4MHz, not 8, so signal this
    ClockSetLowBatt((batt <= 2400)?true:false);

    batt -= 1000; // bat = bat - 1.0V
    batt /= 10; // 1000 => 100 
    if (batt > 255) batt = 255; // Can't return above 3.55V

    // Set the internal reference and mux.
    ADMUX = (_BV(REFS1) | _BV(REFS0) | _BV(MUX3));
    ClockDelay(5);

    // And...start ADC and busy wait until it's done
    ADCSRA |= _BV(ADSC);
    while (bit_is_set(ADCSRA,ADSC)) {/*busy wait */ }

    // Read it out
    l = ADCL;
    h = ADCH;
    res = (h<<8) | l;

    // Offset taken from Arduino website, may not be very accurate
    // Keep everything in integer by scaling by 50 on both sides
    temp = (50*res - 16205 /*324.31*50*/ ) / 61 /*1.22*50*/;
    if (temp > 255) temp = 255; // !!! We be hot

    ShutdownADC();

    return (batt<<8) | temp;
}



#if debug
void logHex(const char *header, const uint8_t *bytes, int cnt)
{
    char buff[256];

    Serial.print(gwHour);
    Serial.print(":");
    Serial.print(gwMin);
    Serial.print(":");
    Serial.print(gwSec);
    Serial.print("-");
    Serial.print(gwYear);
    Serial.print("-");
    Serial.print(gwMon);
    Serial.print("-");
    Serial.print(gwDay);
    Serial.print(" ");
    Serial.flush();
    const char hex[17] PROGMEM={"0123456789ABCDEF"};
    uint8_t c = 0;
    for (int i=0; i<cnt; i++) {
        buff[c++] = hex[0xf&(bytes[i]>>4)];
        buff[c++] = hex[bytes[i]&0xf];
    }
    buff[c++] = 0;
    Serial.flush();
    Serial.print(header);
    Serial.flush();
    Serial.print(buff);
    Serial.flush();
    Serial.println("");
    Serial.flush();
}
#else
#define logHex(a, b, c) {}
#endif


void SetJoinMessage(uint8_t *msg, uint8_t protoVersion, uint8_t mac[4],
                    uint8_t name[8])
{
    memset(msg, 0, 16);
    SetMessageTypeSeq(msg, MSG_JOIN, 0);
    msg[1] = protoVersion;
    msg[2] = mac[0];
    msg[3] = mac[1];
    msg[4] = mac[2];
    msg[5] = mac[3];
    memcpy(msg+6, name, 8);
}



bool CheckSensors();

// Is this report due to an event (interrupt)?  If so, send it out with
// shorter timeouts to make it get there sooner than standard report-ins.
bool irq = false;


// lowest power sleep for ~secs
// Assumes radio already powered down
void ZZZ(uint16_t secs)
{
    if (debug) {
        Serial.flush();
        Serial.println("Entering lowest power");
        Serial.flush();
    }

    // Cannot gather entropy in this mode, timers shutdown
    bool oldRand = RandEnable(false);

    noInterrupts();

    // Set the watchdog reset bit in the MCU status register to 0.
    MCUSR &= ~(1<<WDRF);
    // Set WDCE and WDE bits in the watchdog control register.
    WDTCSR |= (1<<WDCE) | (1<<WDE);
    // ~32ms WDT to allow for scanning many times a second...
    WDTCSR = 0x01;
    // Enable watchdog as interrupt only (no reset).
    WDTCSR |= (1<<WDIE);

    set_sleep_mode (SLEEP_MODE_PWR_DOWN);

    ShutdownADC(); // Should already be down, but just in case...

    secs = secs * 31; // ~1/32ms
    irq = false;

    while (secs && !irq) {
        noInterrupts ();           // timed sequence follows
        sleep_enable();
        // Brown-out detector disabled by fuses
        interrupts ();             // guarantees next instruction executed
        sleep_cpu ();              // sleep within 3 clock cycles of above
        secs--;
        irq = CheckSensors();
    }

    // Reset entropy WDT
    RandSetup();
    RandEnable(oldRand);
}

const int16_t sampleDelay = 10;;


// Any continuously powered sensors need to be set up here
void SetupSensors()
{
    // Nothing to do so far...
}

Thermistor thermistor(10, A0);
Switch sw0(2, false);

bool CheckSensors()
{
    bool wantIRQ = false;
    thermistor.Poll();

    bool old = sw0.Value();
    sw0.Poll();
    if (sw0.Value() != old) wantIRQ = true;

    return wantIRQ;
}


void SetupReportMsg(uint8_t *msg)
{
    const uint8_t report[] = { PERCENT, ONEFLAG, UINT8, EIGHTFLAG, FIXPT16X10 };
    const uint8_t reports = sizeof(report)/sizeof(report[0]);
    memset(msg+7, 7, 0);
    msg[7] = (reports<<4) & 0xf0;
    msg[7] |= report[0] & 0x0f;
    for (uint8_t p=0; p<12; p+=2) {
        if (reports>(p+1)) msg[8+(p>>1)]  = (report[p+1]<<4) & 0xf0;
        if (reports>(p+2)) msg[8+(p>>1)] |= (report[p+2]) & 0x0f;
    }
}

bool SetupSensorInfoMsg(uint8_t *msg)
{
    const char sensorName[][13] PROGMEM = {
        "Battery",
        "Switch",
        "IntTemp",
        "MultiSW",
        "Temp" };

    // Have we sent all that we can?
    if (GetSequenceNum(msg) > 5) {
        return false;
    }
    memcpy(msg+1, sensorName[GetSequenceNum(msg)], 13);
}


void UpdateReportMsg(uint8_t *msg)
{
    static uint8_t cnt = 0;
    uint16_t bt = ReadInternalBatteryTempFast();
    uint16_t battPct= ((bt>>8)& 0xff);
    const uint16_t minVolt = 90; // 1.9V due to NRF
    const uint16_t maxVolt = 200; // 3.0V from 2xAA 1.5V
    if (battPct < minVolt) battPct = 0;  // <1.9 = 0%
    else if (battPct > maxVolt) battPct = 100; // > 3.0V = 100%
    else battPct = ((battPct-minVolt)*100) / (maxVolt - minVolt);
                   //  batt * [ (100-0) / (max-min) ] 
    int16_t thermTemp = thermistor.Value();
    msg[1] = battPct & 0xff;
    msg[2] = sw0.Value()?0xff:0x00;
    msg[3] = bt & 0xff;
    msg[4] = cnt++;
    msg[5] = thermTemp&0xff;
    msg[6] = (thermTemp>>8) & 0xff;
}

void ProcessACK(uint8_t *dec)
{
    gwHour = (dec[1]>>3) & 0x1f;
    gwMin =  ((dec[1]<<3) | (dec[2]>>5)) & 0x3f;
    gwSec = ((dec[2]>>1) & 0x0f) << 2;
    gwYear = 2016 + ( ((dec[2]&1)<<4) | ((dec[3]>>4) & 0x0f) );
    gwMon = (dec[3]&0x0f);
    gwDay = (dec[4]>>3)& 0x1f;
    gwWeekday = dec[4] & 0x07;
    logHex("Got ACK:", dec, 16);
}

// Get and verify existing MAC from EEPROM.  If invalid/missing make one
void CreateOrGetMAC()
{
    ClockNormal();
    RandEnable(true);
    sensorMAC[0] = EEPROM.read(EEPROM_MAC_0);
    sensorMAC[1] = EEPROM.read(EEPROM_MAC_1);
    sensorMAC[2] = EEPROM.read(EEPROM_MAC_2);
    sensorMAC[3] = EEPROM.read(EEPROM_MAC_3);
    uint8_t ck = EEPROM.read(EEPROM_MAC_CK);
    ck ^= sensorMAC[0] ^ sensorMAC[1] ^ sensorMAC[2] ^ sensorMAC[3] ^ 0xbe;
    if (ck==0) return; // Checksum OK, we're done here
    do {
        sensorMAC[0] = RandGet();
        sensorMAC[1] = RandGet();
        sensorMAC[2] = RandGet();
        sensorMAC[3] = RandGet();
    } while (!sensorMAC[0] && !sensorMAC[1] && !sensorMAC[2] && !sensorMAC[3]);
    ck = sensorMAC[0] ^ sensorMAC[1] ^ sensorMAC[2] ^ sensorMAC[3] ^ 0xbe;
    EEPROM.write(EEPROM_MAC_0, sensorMAC[0]);
    EEPROM.write(EEPROM_MAC_1, sensorMAC[1]);
    EEPROM.write(EEPROM_MAC_2, sensorMAC[2]);
    EEPROM.write(EEPROM_MAC_3, sensorMAC[3]);
    EEPROM.write(EEPROM_MAC_CK, ck);
}


void SendLog(const char *str)
{
    uint8_t msg[16];
    SetMessageTypeSeq(msg, MSG_LOG, seqNo);
    for (int i=0; i<14; i++) {
        if (*str) msg[1+i] = *(str++);
        else msg[1+i] = 0;
    }
    SendRadioMessage(msg, aesKey, NULL);
}


// the setup routine runs once when you press reset:
void setup()
{
    // We start at INTOSC/8 = 1MHz, so let's check the battery
    // and move to proper speed
    ReadInternalBatteryTempFast();
    ReadInternalBatteryTempFast();
    ClockSlow();
    ClockNormal();

    if (debug) {
        Serial.begin(GetClockFreq()==8000?9600:19200);
        Serial.println("Initializing...");
        Serial.flush();
    }

#if debugrng
    RandSetup();
    RandEnable(true);
    while (1) {
        uint8_t x = RandGet();
        const char *hex="0123456789ABCDEF";
        Serial.print(hex[(x>>4)&0x0f]);
        Serial.print(hex[(x)&0x0f]);
        Serial.flush();
    }
#endif

#if debugtemp
    while (1) {
        int16_t a = thermistor.Read();
        Serial.print("TEMP = ");
        Serial.println(a);
    }
#endif

    // Initialize the digital pin as an input pullup, so they don't float
    // Later individual pins can be reassigned output if needed
    for (uint8_t i=0; i<=A7; i++) {
        pinMode(i, OUTPUT);
        digitalWrite(i, LOW);
    }
    
    RandSetup();
    RandEnable(true);

    // Who are we, again?
    CreateOrGetMAC();

    SetupSensors();
    radio.begin();
}

bool sentSensors = false;


// Main application
void loop()
{
    uint8_t msg[16];
    uint8_t raw[19], rec[16], dec[16];
    unsigned long timeout;
    uint8_t fm[32];
    uint8_t km[32], ky[32];
    bool gotResp;
    uint8_t retries;
    uint8_t *aesPtr;
    uint32_t now;
    uint32_t timeoutDelay = 1000L;
    uint8_t toggle = LOW;

    ClockNormal();
    if (debug) Serial.println("---------------------------");
   
    if (!joined) {
        if (debug) Serial.println("Calculating private key");
        // Make code, it takes a while so have it ready when we start talking
        ClockNormal();
        radio.powerDown();

        Curve25519::dh1(km, fm);
       
        RadioSetupJoin();
        aesPtr = NULL;
       
SENDJOIN:
        ClockNormal();
        SetJoinMessage(msg, 1, sensorMAC, (uint8_t*)"DOORWIND");
        logHex("Sending JOIN: ", msg, 16);
       
        radio.powerUp();
        SendRadioMessage(msg, aesPtr, NULL);
       
        // Wait for timeout then resend if nothing comes back
        timeoutDelay = 1000L + RandGet();
        ClockSlow();
        timeout = FromNowMS(timeoutDelay);
       
        gotResp = false;
        now = millis();
        while (now < timeout) {
            if (radio.available()) {
                ClockNormal();
                radio.read(raw, 19);
                CorrectMessage(rec, raw);
                if (!CheckMessageAES(rec, aesPtr, rec)) {
                    logHex("CRC mismatch, discarding ", rec, 16);
                    break;
                }
                if (GetMessageType(rec) != MSG_K0G) {
                    logHex("Want K0G, got, discarding:", rec, 16);
                    break;
                }
                logHex("Got K0G:", rec, 16);
                gotResp = true;
                break;
            } else {
                // Would like to flash LED, but SCK uses same pin
            }
            now = millis();
        }
        if (!gotResp) goto SENDJOIN;
       
        // Got K0G, stuff the code away
        memcpy(ky, rec+1, 13);
       
        SetMessageTypeSeq(msg, MSG_K0S, 1);
        memcpy(msg+1, km, 13);
        retries = retriesJoin;
        gotResp = false;
        
        // Send mine. If don't hear response then retry a few times
        // and finally give up and restart
SENDK0S:
        ClockNormal();
        logHex("Sending K0S: ", msg, 16);
        SendRadioMessage(msg, aesPtr, NULL);
        ClockSlow();
        timeout = FromNowMS(timeoutDelay);
        now = millis();
        while (now < timeout) {
            if (radio.available()) {
                ClockNormal();
                radio.read(raw, 19);
                CorrectMessage(rec, raw);
                if (!CheckMessageAES(rec, aesPtr, rec)) {
                    logHex("CRC mismatch, discarding ", rec, 16);
                    break;
                }
                if (GetMessageType(rec) != MSG_K1G) {
                    logHex("Want K1G, got, discarding:", rec, 16);
                    break;
                }
                logHex("Got K1G:", rec, 16);
                gotResp = true;
                break;
            } else {
                // Toggle LED?
            }
           now = millis();
        }
        if (!gotResp) {
            if (retries--) goto SENDK0S;
            else {
                if (debug)
                    Serial.println("Unable to get K0G, restarting protocl");
                goto SENDJOIN;
            }
        }
        // Got K1G, stuff the code away
        memcpy(ky+13, rec+1, 13);
        SetMessageTypeSeq(msg, MSG_K1S, 1);
        memcpy(msg+1, km+13, 13);
        retries = retriesJoin;
        gotResp = false;

SENDK1S:
        ClockNormal();
        logHex("Sending K1S: ", msg, 16);
        SendRadioMessage(msg, aesPtr, NULL);
        ClockSlow();
        timeout = FromNowMS(timeoutDelay);
        now = millis();
        while (now < timeout) {
            if (radio.available()) {
                ClockNormal();
                radio.read(raw, 19);
                CorrectMessage(rec, raw);
                if (!CheckMessageAES(rec, aesPtr, rec)) {
                    logHex("CRC mismatch, discarding ", rec, 16);
                    break;
                }
                if (GetMessageType(rec) != MSG_K2G) {
                    logHex("Want K2G, got, discarding:", rec, 16);
                    break;
                }
                logHex("Got K2G:", rec, 16);
                gotResp = true;
                break;
            } else {
                // Toggle LED?
            }
           now = millis();
        }
        if (!gotResp) {
            if (retries--) goto SENDK1S;
            else goto SENDJOIN;
        }
        // Got K2G, stuff the code away
        memcpy(ky+26, rec+1, 6);

        netAddr = ((uint32_t)rec[7]) | (((uint32_t)rec[8])<<8) | (((uint32_t)rec[9])<<16) & 0xffffff;
        netChan = rec[10]&0x7f;
        netRate = (rf24_datarate_e)(rec[13]&0x03);
       
        memset(msg, 0, 16);
        SetMessageTypeSeq(msg, MSG_K2S, 1);
        memcpy(msg+1, km+26, 6);
       
        SetupReportMsg(msg);

        retries = retriesJoin;
        gotResp = false;
SENDK2S:
        ClockNormal();
        logHex("Sending K2S: ", msg, 16);
        SendRadioMessage(msg, aesPtr, NULL);
        ClockSlow();
        timeout = FromNowMS(timeoutDelay);
        while (millis() < timeout) {
            if (radio.available()) {
                ClockNormal();
                radio.read(raw, 19);
                CorrectMessage(rec, raw);
                if (!CheckMessageAES(dec, aesPtr, rec)) {
                    logHex("CRC mismatch, discarding ", dec, 16);
                    break;
                }
                if (GetMessageType(dec) != MSG_ACK) {
                    logHex("Want ACK, got, discarding:", dec, 16);
                    break;
                }
                ProcessACK(dec);
                gotResp = true;
                break;
            }
        }
        ClockNormal(); // It'll be stopped right afterwards, anyways
        if (!gotResp) {
            // At this stage we're committed to the new key.  They may have heard my new secret and sent my ACK
            goto SENDK2S;  // 
        }

        radio.powerDown(); // At this point, no talking while we think...
        // If we can make the shared secret, we're joined
        ClockNormal();
        if (Curve25519::dh2(ky, fm)) {
            logHex("Secret: ", ky, 16);
            joined = true;
            memcpy(aesKey, ky, 16);
            RadioSetupNormal();
            seqNo = 0;
            return; // We're joined, go back to main....
        }
    } else if (joined && !sentSensors) {
        ClockNormal();
        radio.powerUp();
        memset(msg, 0, 16);
        SetMessageTypeSeq(msg, MSG_SINFO, seqNo);
        if (!SetupSensorInfoMsg(msg)) {
            sentSensors = true;
            return; // We're done, send normal reports now...
        }
        retries = retriesJoin;
        gotResp = false;
SENDSINFO:
        ClockNormal();
        logHex("Sending SINFO: ", msg, 16);
        SendRadioMessage(msg, aesKey, NULL);
        ClockSlow();
        timeout = FromNowMS(timeoutDelay);
        while (millis() < timeout) {
            if (radio.available()) {
                ClockNormal();
                radio.read(raw, 19);
                CorrectMessage(rec, raw);
                if (!CheckMessageAES(dec, aesKey, rec)) {
                    logHex("CRC mismatch, discarding ", dec, 16);
                    break;
                }
                if (GetMessageType(dec) != MSG_ACK) {
                    logHex("Want ACK, got, discarding:", dec, 16);
                    break;
                }
                ProcessACK(dec);
                gotResp = true;
                break;
            }
        }
        ClockNormal(); // It'll be stopped right afterwards, anyways
        if (!gotResp) {
            if (retries--) goto SENDSINFO;
            else {
                if (debug) Serial.println("No ACK received, ignoring");
            }
        }
        radio.powerDown();
        seqNo++; // SeqNo can never get above # sensors, and we start at 0, so no need to check for rollover here

    } else if (joined && sentSensors) {
        ClockNormal();
        radio.powerUp();
        memset(msg, 0, 16);
        SetMessageTypeSeq(msg, MSG_REPORT, seqNo);
        retries = retriesReport;
        gotResp = false;

SENDREPORT:
        SendLog(GetClockFreq()==4000?"4MHz clock":"8MHz clock");
        ClockNormal();
        UpdateReportMsg(msg);
        logHex("Sending report: ", msg, 16);
        SendRadioMessage(msg, aesKey, NULL);
        ClockSlow();
        // For IRQ (state changes) we want to send this ASAP, let other
        // check-ins back off for us
        timeout = FromNowMS(irq?timeoutDelay/10:timeoutDelay);
        while (millis() < timeout) {
            if (radio.available()) {
                ClockNormal();
                radio.read(raw, 19);
                CorrectMessage(rec, raw);
                if (!CheckMessageAES(dec, aesKey, rec)) {
                    logHex("CRC mismatch, discarding ", dec, 16);
                    break;
                }
                if (GetMessageType(dec) != MSG_ACK) {
                    logHex("Want ACK, got, discarding:", dec, 16);
                    break;
                }
                ProcessACK(dec);
                gotResp = true;
                break;
            }
        }
        ClockNormal(); // It'll be stopped right afterwards, anyways
        if (!gotResp) {
            if (retries--) goto SENDREPORT;
            else {
                if (debug) Serial.println("No ACK received, ignoring");
            }
        }
        radio.powerDown();
        seqNo++;
        if (seqNo>31) {
            if (debug) Serial.println("Starting rekey, generating new secrets");
            // Need to rekey, using current AES key
            aesPtr = aesKey;
            // Make new code first
            Curve25519::dh1(km, fm);
            seqNo = 0;
            retries = 0xff;
            goto SENDJOIN;
        }

        ZZZ(sampleDelay);
    }
  
}


