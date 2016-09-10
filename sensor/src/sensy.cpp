#include <avr/interrupt.h>
#include <avr/power.h>
#include <avr/sleep.h>
#include <avr/wdt.h> 
#include <SPI.h>
#include "Crypto/AES.h"
#include "Crypto/Curve25519.h"
#include "Crypto/RNG.h"
#include "RF24/RF24.h"
#include "rng.h"
#include "coding.h"
#include <pins_arduino.h>


// Serial debugging enabled
#define debug 0

// To join, use these hardcoded settings
const uint32_t        joinAddr = 0xabcd12;
const bool            joinHamm = true;
const uint8_t         joinChan = 0;
const rf24_datarate_e joinRate = RF24_250KBPS;

// Gateway will send up the real network config
uint32_t        netAddr = 0;
bool            netHamm = true;
uint8_t         netChan = 0;
rf24_datarate_e netRate = RF24_250KBPS;

const uint8_t retriesJoin = 20;
const uint8_t retriesReport = 15;

bool joined = false;
uint8_t seqNo = 0;
uint8_t aesKey[16] = {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};
uint8_t sensorID[3] = {0,0,0};

// Reported time at the gateway for last ACK
uint8_t  gwHour = 0;
uint8_t  gwMin = 0;
uint8_t  gwSec = 0;
uint16_t gwYear = 0;
uint8_t  gwMon = 0;
uint8_t  gwDay = 0;
uint8_t  gwWeekday = 0;

RF24 radio(7,8);
 
const int mySwitch0 = 2;

long readVcc();
void setClockPrescaler(uint8_t clockPrescaler);

volatile bool stopThePresses = false; // set by ISR to signal we need to send report back

#define CLOCK_PRESCALER_1   (0x0)
#define CLOCK_PRESCALER_2   (0x1)
#define CLOCK_PRESCALER_4   (0x2)
#define CLOCK_PRESCALER_8   (0x3)
#define CLOCK_PRESCALER_16  (0x4)
#define CLOCK_PRESCALER_32  (0x5)
#define CLOCK_PRESCALER_64  (0x6)
#define CLOCK_PRESCALER_128 (0x7)
#define CLOCK_PRESCALER_256 (0x8)


bool isClockSlow = false;
void ClockSlow()
{
    if (isClockSlow) return; // Already in slow mode
    if (debug) {
        Serial.println("CLOCKSLOW");
        Serial.flush();
    }
    RandEnable(false);
    isClockSlow = true;
    setClockPrescaler(CLOCK_PRESCALER_16);
}

void ClockNormal()
{
    if (!isClockSlow) return; // Already in normal mode

    setClockPrescaler(CLOCK_PRESCALER_1);
    RandEnable(true);
    isClockSlow = false;
    if (debug) {
        Serial.println("CLOCKFAST");
        Serial.flush();
    }
}

uint32_t FromNowMS(uint32_t delta)
{
    uint32_t now = millis();
    if (isClockSlow) {
        delta = (delta >> 4) & (0x0fff);
        if (delta==0) delta = 1;
    }
    now += delta;
    return now;
}

// Disable everything we can think of in the ADC block
void ShutdownADC()
{
    power_adc_enable();
    ADMUX = 0x00;
    ADCSRA = 0x07;
    ADCSRB = 0x00;
    DIDR0 = 0x00;
    DIDR1 = 0x00;
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

    ADCSRA = 0x87; //  ADCEN=1,PS=/8 (1MHz w/8MHz clock)...will be lower quality, but so is the input signal

    // Set bandgap input
    ADMUX = _BV(REFS0) | _BV(MUX3) | _BV(MUX2) | _BV(MUX1);
    delay(4);

    // And...start ADC and busy wait until it's done
    ADCSRA |= _BV(ADSC);
    while (bit_is_set(ADCSRA,ADSC)) {/*busy wait */ }

    // Read it out
    uint8_t l = ADCL;
    uint8_t h = ADCH;
    uint16_t res = (h<<8) | l;

    // Scale results.  VCC=1023L, Vref=1100mV; Vref(mV)/Vref(cnt) = VCC(mV)/1023; => Vref(mV)*1023/Vref(cnt) = VCC(mV)
    batt = (1100L*1023L) / res; // 1100mV, 2^10 as max
    batt -= 1000; // bat = bat - 1.0V
    batt /= 10; // 1000 => 100 
    if (batt > 255) batt = 255; // Can't return above 3.55V

    // Set the internal reference and mux.
    ADMUX = (_BV(REFS1) | _BV(REFS0) | _BV(MUX3));
    delay(4);

    // And...start ADC and busy wait until it's done
    ADCSRA |= _BV(ADSC);
    while (bit_is_set(ADCSRA,ADSC)) {/*busy wait */ }

    // Read it out
    l = ADCL;
    h = ADCH;
    res = (h<<8) | l;

    // The offset of 324.31 could be wrong. It is just an indication.
    temp = (50*res - 16205/*324.31*50*/ ) / 61/*1.22*50*/;
    if (temp > 255) temp = 255; // !!! We be hot

    ShutdownADC();

    return (batt<<8) | temp;
}



void setClockPrescaler(uint8_t clockPrescaler) {
  if (clockPrescaler <= CLOCK_PRESCALER_256) {
    // Disable interrupts.
    uint8_t oldSREG = SREG;
    cli();

    // Enable change.
    CLKPR = _BV(CLKPCE); // write the CLKPCE bit to one and all the other to zero

    // Change clock division.
    CLKPR = clockPrescaler; // write the CLKPS0..3 bits while writing the CLKPE bit to zero

    // Recopy interrupt register.
    SREG = oldSREG;
  }
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

void SetMessageTypeSeq(uint8_t msg[16], uint8_t type, uint8_t seq)
{
    msg[0] = ((type&0xf) << 4) | (seq & 0xf);
}

uint8_t GetMessageType(const uint8_t msg[16])
{
    return (uint8_t)(0x0f & (msg[0]>>4));
}


int GetSequenceNum(const uint8_t msg[16])
{
    return (msg[0] & 0x0f);
}

void SendRadioMessage(uint8_t msg[16], uint8_t key[16], bool useHamm)
{
    uint8_t bytes[19];

    logHex(key?"AES: ":"PLAINTEXT: ",  msg, 16);

    CodeMessage(bytes, msg, key, useHamm);

    radio.stopListening();
    radio.write( bytes, useHamm?19:16 );
    // TBD - listen before sending?  Check for error?
    radio.startListening();
}

typedef enum { MSG_JOIN=0, MSG_K0G, MSG_K0S, MSG_K1G, MSG_K1S, MSG_K2G, MSG_K2S, MSG_REPORT, MSG_ACK, MSG_REKEY, MSG_LOG=15 } messageType_t;


void SetJoinMessage(uint8_t *msg, uint8_t protoVersion, uint8_t oldID[3], uint8_t name[9])
{
    memset(msg, 0, 16);
    SetMessageTypeSeq(msg, MSG_JOIN, 0);
    msg[1] = protoVersion;
    msg[2] = oldID[0];
    msg[3] = oldID[1];
    msg[4] = oldID[2]<<2;
    memcpy(msg+5, name, 9);
}




void RadioSetup(rf24_datarate_e rate, uint8_t channel, uint32_t addr, bool hamming )
{
    if (debug) {
        Serial.print("RADIO rate="); Serial.println(rate);
        Serial.print("RADIO chann="); Serial.println(channel);
        Serial.print("RADIO addr="); Serial.println(addr, HEX);
        Serial.print("RADIO hamming="); Serial.println(hamming);
    }
    radio.stopListening();
    radio.setPALevel(RF24_PA_MAX);  // Always yell
    radio.setDataRate(rate);
    radio.setAddressWidth(3);       // Always 3
    radio.setChannel(channel);
    radio.disableCRC();
    radio.setAutoAck(false);
    radio.setPayloadSize(hamming?19:16);
    radio.openWritingPipe(addr);
    radio.openReadingPipe(1, addr);
    radio.startListening();
}

void RadioSetupJoin()
{
    RadioSetup(joinRate, joinChan, joinAddr, joinHamm);
}

void RadioSetupNormal()
{
    RadioSetup(netRate, netChan, netAddr, netHamm);
}

void PowerSensors(bool powered);
bool CheckSensors();

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
    PowerSensors(false); // Don't waste power when we're lot looking

    secs = secs * 31; // ~1/32ms
    bool irq = false;

    while (secs && !irq) {
        noInterrupts ();           // timed sequence follows
        sleep_enable();
        // Brown-out detector disabled by fuses
        interrupts ();             // guarantees next instruction executed
        sleep_cpu ();              // sleep within 3 clock cycles of above
        PowerSensors(true);
        secs--;
        irq = CheckSensors();
        PowerSensors(false);
    }

    // Reset entropy WDT
    RandSetup();
    RandEnable(oldRand);

    // Reset the radio registers
    radio.begin();
    RadioSetupNormal();
}

const int16_t sampleDelay = 10;;


typedef enum { BATTERY=0, SWITCH, TEMP, ANALOG } report_t;

// Any continuously powered sensors need to be set up here
void SetupSensors()
{
    // Ensure that we don't register power-on value as a change event
    PowerSensors(true);
    CheckSensors();
    CheckSensors();
    CheckSensors();
    PowerSensors(false);
}


// Sensors will be powered up as soon as WDT goes off, then scanned once
// some housekeeping done.  This gives them some time to charge/discharge
// input capacitance while we do useful work.
void PowerSensors(bool powered)
{
    if (powered) {
        pinMode(mySwitch0, INPUT_PULLUP);
    } else {
        pinMode(mySwitch0, OUTPUT);
        digitalWrite(mySwitch0, LOW);
    }
}

bool switch0 = false;
bool CheckSensors()
{
    static bool lastSwitch0 = false;
    static bool nextSwitch0 = false; // Debounce by only updating global switch state on 2 consecutive reads same
    bool irq = false;

    if (digitalRead(mySwitch0) == LOW) nextSwitch0 = false;
    else nextSwitch0 = true;
    if (lastSwitch0 == nextSwitch0) {
        if (switch0 != nextSwitch0) irq = true;
        switch0 = nextSwitch0;
    }
    lastSwitch0 = nextSwitch0;
    return irq;
}


void SetupReportMsg(uint8_t *msg)
{
    const uint8_t report[] = { BATTERY, SWITCH, TEMP, ANALOG };
    const uint8_t reports = sizeof(report)/sizeof(report[0]);
    memset(msg+7, 7, 0);
    msg[7] = (reports<<4) & 0xf0;
    msg[7] |= report[0] & 0x0f;
    for (uint8_t p=0; p<12; p+=2) {
        if (reports>(p+1)) msg[8+(p>>1)]  = (report[p+1]<<4) & 0xf0;
        if (reports>(p+2)) msg[8+(p>>1)] |= (report[p+2]) & 0x0f;
    }
}

void UpdateReportMsg(uint8_t *msg)
{
    static uint8_t cnt = 0;
    uint16_t bt = ReadInternalBatteryTempFast();
    uint16_t battPct= ((bt>>8)& 0xff);
    const uint16_t minVolt = 90; //1.9V due to NRF
    const uint16_t maxVolt = 200; //3.0V from 2xAA 1.5V
    if (battPct < minVolt) battPct = 0;  /* <1.9 -> 0% */
    else if (battPct > maxVolt) battPct = 100; /* > 3.0V = 100% */
    else battPct = ((battPct-minVolt)*100) / (maxVolt - minVolt);  /* batt * [ (100-0) / (max-min) ] */
    msg[1] = battPct & 0xff;
    msg[2] = switch0?0xff:0x00; //digitalRead(mySwitch0)==HIGH?255:0;digitalRead(mySwitch0)==HIGH?255:0;
    msg[3] = bt & 0xff;
    msg[4] = cnt++;
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

// the setup routine runs once when you press reset:
void setup()
{
    // Initialize the digital pin as an input pullup, so they don't float
    // Later individual pins can be reassigned output if needed
    for (uint8_t i=0; i<=A7; i++) {
        pinMode(i, OUTPUT);
        digitalWrite(i, LOW);
    }
    
    RandSetup();
    RandEnable(true);
    SetupSensors();
    if (debug) Serial.begin(9600);
    radio.begin();
}

// the loop routine runs over and over again forever:
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
    bool retryHamm = true;
    uint32_t now;
    uint32_t timeoutDelay = 1000L;
    uint8_t toggle = LOW;

    ClockNormal();
    if (debug) Serial.println("---------------------------");
   
    if (!joined) {
        if (debug) Serial.println("Calculating private key");
        // Make my code, it takes a while so have it ready when we start talking
        ClockNormal();
        radio.powerDown();

        Curve25519::dh1(km, fm);
       
        RadioSetupJoin();
        aesPtr = NULL;
        retryHamm = joinHamm;
       
SENDJOIN:
        ClockNormal();
        SetJoinMessage(msg, 1, sensorID, (uint8_t*)"SENSOR1  ");
        logHex("Sending JOIN: ", msg, 16);
       
        radio.powerUp();
        SendRadioMessage(msg, aesPtr, retryHamm);
       
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
        
        // Send mine, and if we don't hear back retry a few times and finally give up and restart
SENDK0S:
        ClockNormal();
        logHex("Sending K0S: ", msg, 16);
        SendRadioMessage(msg, aesPtr, retryHamm);
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
                if (debug) Serial.println("Unable to get K0G, restarting protocl");
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
        SendRadioMessage(msg, aesPtr, retryHamm);
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
        netHamm = (rec[10]&0x80)?true:false;
        netChan = rec[10]&0x7f;
        netRate = (rf24_datarate_e)(rec[13]&0x03);
        sensorID[0] = rec[11];
        sensorID[1] = rec[12];
        sensorID[2] = rec[13]>>2;
       
        memset(msg, 0, 16);
        SetMessageTypeSeq(msg, MSG_K2S, 1);
        memcpy(msg+1, km+26, 6);
       
        SetupReportMsg(msg);

        retries = retriesJoin;
        gotResp = false;
SENDK2S:
        ClockNormal();
        logHex("Sending K2S: ", msg, 16);
        SendRadioMessage(msg, aesPtr, netHamm);
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
            if (retries--) goto SENDK2S;
            else {
                if (debug) Serial.println("No ACK received, ignoring");
            }
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
            retryHamm = netHamm;
            return; // We're joined, go back to main....
        }
    } else /* joined */ {
        ClockNormal();
        stopThePresses = false;
        radio.powerUp();
        memset(msg, 0, 16);
        SetMessageTypeSeq(msg, MSG_REPORT, seqNo);
        retries = retriesReport;
        gotResp = false;

SENDREPORT:
        ClockNormal();
        UpdateReportMsg(msg);
        logHex("Sending report: ", msg, 16);
        SendRadioMessage(msg, aesKey, netHamm);
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
            if (retries--) goto SENDREPORT;
            else {
                if (debug) Serial.println("No ACK received, ignoring");
            }
        }
        radio.powerDown();
        seqNo++;
        if (seqNo>15) {
            if (debug) Serial.println("Starting rekey, generating new secrets");
            // Need to rekey, using current AES key
            aesPtr = aesKey;
            // Make new code first
            Curve25519::dh1(km, fm);
            seqNo = 0;
            retryHamm = netHamm;
            retries = 0xff;
            goto SENDJOIN;
        }

        ZZZ(sampleDelay);
    }
  
}

