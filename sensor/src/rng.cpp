#include <avr/interrupt.h>
#include <avr/power.h>
#include <avr/sleep.h>
#include <SPI.h>
#include "Crypto/RNG.h"
#include "rng.h"

#define MAXENTROPY 48
volatile bool rand_running = false;
volatile uint8_t rand_data[MAXENTROPY];
volatile uint8_t rand_head = 0;
volatile uint8_t rand_tail = 0;
// WDT IRQ writes data to rand_tail, application reads from rand_head

uint8_t RandGet()
{
    while (rand_head == rand_tail) {
        set_sleep_mode(SLEEP_MODE_PWR_SAVE);  // Keep peripheral running, will wake up on WDT IRQ
        power_adc_disable();
        sleep_enable();
        sleep_mode();
        sleep_disable();
        power_all_enable();
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
    return old;
}

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

// Watchdog Timer ISR
ISR(WDT_vect)
{
    static uint8_t sample = 0;
    static uint8_t curbit = 0;
    static uint8_t result = 0;
  
    if (!rand_running) return; // If we're not enabled, just ignore

    uint8_t next_tail = (rand_tail + 1) % MAXENTROPY;
    if (rand_head == next_tail) return; // Buffer's full, ignore
  
    sample = TCNT1L; // Ignore higher bits
    result = (result << 1) | (result >> 7); // Spread randomness around
    result ^= sample; // XOR preserves randomness

    curbit++;
    if (curbit > 7) {
      rand_data[rand_tail] = result;
      curbit = 0;
      rand_tail = next_tail;
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

    sei();
}

