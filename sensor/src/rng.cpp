#include <avr/interrupt.h>
#include <avr/power.h>
#include <avr/sleep.h>
#include <Arduino.h>
#include "Crypto/RNG.h"
#include "rng.h"

// WDT IRQ writes data to rand_tail, application reads from rand_head
#define MAXENTROPY 48
volatile bool rand_running = false;
volatile uint8_t rand_data[MAXENTROPY];
volatile uint8_t rand_head = 0;
volatile uint8_t rand_tail = 0;

// When all you need's just a tiny bit...
uint8_t RandGetBit()
{
    static uint8_t r;
    static uint8_t bit = 8;
    if (bit>7) {
        r = RandGet();
        bit = 0;
    }
    return ((1<<(bit++))&r)?1:0;
}

// Von Neumann deskewed random numbers, very slow but gives equal 1/0s.
uint8_t RandGetDeskew()
{
    uint8_t r = 0;
    uint8_t a = 0;
    uint8_t b = 0;
    for (uint8_t i=0; i<8; i++) {
        do {
            a = RandGetBit();
            b = RandGetBit();
        } while (a==b);
        r |= (a<<i);
    }
    return r;
}

// Get an 8-bit random
uint8_t RandGet()
{
    while (rand_head == rand_tail) {
        set_sleep_mode(SLEEP_MODE_PWR_SAVE);  // Keep peripheral running, will wake up on WDT IRQ
        sleep_enable();
        sleep_mode();
        sleep_disable();
    }
    uint8_t data = rand_data[rand_head];
    uint8_t next_head = (rand_head + 1) % MAXENTROPY;
    rand_head = next_head; // Make sure this is only written with proper updated value and not intermediates
    return data;
}

bool RandEnable(bool en)
{
    bool old = rand_running;
    rand_running = en;
    if (en) {
        power_adc_enable();  // Need to be powered on before we write any registers
        ADCSRA = 0x87; //  ADCEN=1,PS=/8 (1MHz w/8MHz clock)
        // Set the internal reference and mux.
        ADMUX = (_BV(REFS1) | _BV(REFS0) | _BV(MUX3));
    } else {
        ADCSRA = 0x00; // Go to sleep
        power_adc_disable();
    }
    return old;
}

// Implementation of RNGClass to use our generator for the EC system
RNGClass::RNGClass() {}
RNGClass::~RNGClass() {}
void RNGClass::begin(const char *tag, int eepromAddress) {}
void RNGClass::addNoiseSource(NoiseSource &source) {}
void RNGClass::setAutoSaveTime(uint16_t minutes) {}
void RNGClass::rand(uint8_t *data, size_t len)
{
    bool old = RandEnable(true);
    while (len--) { *(data++) = RandGet(); }
    RandEnable(old);
}
bool RNGClass::available(size_t len) const { return true; }
void RNGClass::stir(const uint8_t *data, size_t len, unsigned int credit) {}
void RNGClass::save() {}
void RNGClass::loop() {}
void RNGClass::destroy() {}
RNGClass RNG;

static uint8_t BitRev( uint8_t x ) 
{ 
    x = ((x >> 1) & 0x55) | ((x << 1) & 0xaa); 
    x = ((x >> 2) & 0x33) | ((x << 2) & 0xcc); 
    x = ((x >> 4) & 0x0f) | ((x << 4) & 0xf0); 
    return x;    
}


// Try and get 1 bit of entropy from LSB of internal voltage reference
static uint8_t ADCEntropy()
{
    // Start ADC and busy wait until it's done
    ADMUX = _BV(REFS0) | _BV(MUX3) | _BV(MUX2) | _BV(MUX1);  // bandgap
    ADCSRA |= _BV(ADSC);
    ADMUX = (_BV(REFS1) | _BV(REFS0) | _BV(MUX3));  // temp
    while (bit_is_set(ADCSRA, ADSC)) { }

    // Read it out
    uint8_t l = ADCL;

    ADCSRA |= _BV(ADSC);
    ADMUX = _BV(REFS0) | _BV(MUX3) | _BV(MUX2) | _BV(MUX1); // bandgap
    while (bit_is_set(ADCSRA, ADSC)) { }

    // Read it out
    uint8_t r = ADCL;

    return l ^ r; //(l&0x0f) | ((r & 0x0f)<<4);
}

// Watchdog Timer ISR
ISR(WDT_vect)
{
    uint8_t sample = 0;
    uint8_t adcSample = 0;
    static uint8_t curbit = 0;
    static uint8_t result = 0;
    static uint8_t overwrite = 0;

    if (!rand_running) return; // If we're not enabled, just ignore

    uint8_t next_tail = (rand_tail + 1) % MAXENTROPY;
  
    // Take the counter and optionally flip it on ADC noise
    sample = TCNT1L;
    adcSample = ADCEntropy();

    // Rotate and XOR in the new value
    result = ((result&0x80)?0x01:0x00) | (result << 1);  // ROL
    result ^= sample ^ adcSample;
    curbit++;

    if (curbit > 8) {
        curbit = 0;
        if (rand_head == next_tail) { // Buffer's full, XOR existing data
            rand_data[(rand_tail + overwrite) % MAXENTROPY] ^= result;
            overwrite = (overwrite + 1) % MAXENTROPY;
        } else { // Stuff new byte into the queue
            rand_data[rand_tail] = result;
            rand_tail = next_tail;
        }
    }
}
    
// Setup of the watchdog timer.
void RandSetup() {
    cli();
    MCUSR = 0;
  
    /* Start timed sequence */
    WDTCSR |= _BV(WDCE) | _BV(WDE);

    /* Put WDT into interrupt mode */
    /* Set shortest prescaler(time-out) value = 2048 cycles (~16 ms) */
    WDTCSR = _BV(WDIE);

    // Timer 1 count up for OCR1A ms, just longer than WDT
    TCCR1A = 0x00;
    TCCR1B = (1 << WGM12) | (1 << CS11);
    OCR1A = 17152L;
    TCNT1 = 0x00; 

    sei();
}

