#include <avr/interrupt.h>
#include <avr/power.h>
#include <avr/sleep.h>
#include <SPI.h>
#include "Crypto/AES.h"
#include "Crypto/Curve25519.h"
#include "Crypto/RNG.h"
#include "RF24/RF24.h"
#include "rng.h"
#include "coding.h"


// Hyper-naive parity in straight C.  Assembly it's probably 1 byte on x86 and AVR.
uint8_t inline parity(uint8_t p)
{
    p = ((p&0xf0)>>4) ^ (p&0x0f);
    p = ((p&0x0c)>>2) ^ (p&0x03);
    p = ((p&0x02)>>1) ^ (p&0x01);
    return p&1;
}


// CRC-CCITT(0xffff) 16-bit cyclic redundancy check, nibble-tabled
uint16_t crc16(uint8_t *data, uint8_t cnt)
{
    const uint16_t tc[16] PROGMEM = {
       0x0000, 0x1021, 0x2042, 0x3063, 0x4084, 0x50a5, 0x60c6, 0x70e7,
       0x8108, 0x9129, 0xa14a, 0xb16b, 0xc18c, 0xd1ad, 0xe1ce, 0xf1ef
    };

    uint16_t sum = 0xffff;  // CCITT starts at 0xffff

    while (cnt--) {
        sum = tc[(sum>>12)^(*data >> 4)]^(sum<<4);
        sum = tc[(sum>>12)^(*(data++) & 0xf)]^(sum<<4);
    }

    return sum;
}


// Return 6-bit hamming code for 32bit integer
uint8_t calc_hamming_32(const uint8_t a[4])
{
    // P0 = D0 + D1 + D3 + D4 + D6 + D8 + D10 + D11 + D13 + D15 + D17 + D19 + D21 + D23 + D25 + D26 + D28 + D30
    uint8_t p0 = (a[0]&(0x01|0x02|0x08|0x10|0x40))^(a[1]&(0x01|0x04|0x08|0x20|0x80))^(a[2]&(0x02|0x08|0x20|0x80))^(a[3]&(0x02|0x04|0x10|0x40));
    // P1 = D0 + D2 + D3 + D5 + D6 + D9 + D10 + D12 + D13 + D16 + D17 + D20 + D21 + D24 + D25 + D27 + D28 + D31
    uint8_t p1 = (a[0]&(0x01|0x04|0x08|0x20|0x40))^(a[1]&(0x02|0x04|0x10|0x20))^(a[2]&(0x01|0x02|0x10|0x20))^(a[3]&(0x01|0x02|0x08|0x10|0x80));
    // P2 = D1 + D2 + D3 + D7 + D8 + D9 + D10 + D14 + D15 + D16 + D17 + D22 + D23 + D24 + D25 + D29 + D30 + D31
    uint8_t p2 = (a[0]&(0x02|0x04|0x08|0x80))^(a[1]&(0x01|0x02|0x04|0x40|0x80))^(a[2]&(0x01|0x02|0x40|0x80))^(a[3]&(0x01|0x02|0x20|0x40|0x80));
    // P3 = D4 + D5 + D6 + D7 + D8 + D9 + D10 + D18 + D19 + D20 + D21 + D22 + D23 + D24 + D25
    uint8_t p3 = (a[0]&(0x10|0x20|0x40|0x80))^(a[1]&(0x01|0x02|0x04))^(a[2]&(0x04|0x08|0x10|0x20|0x40|0x80))^(a[3]&(0x01|0x02));
    // P4 = D11 + D12 + D13 + D14 + D15 + D16 + D17 + D18 + D19 + D20 + D21 + D22 + D23 + D24 + D25
    uint8_t p4 = (a[1]&(0x08|0x10|0x20|0x40|0x80))^(a[2]&(0x01|0x02|0x04|0x08|0x10|0x20|0x40|0x80))^(a[3]&(0x01|0x02));
    // P5 = D26 + D27 + D28 + D29 + D30 + D31
    uint8_t p5 = (a[3]&(0x04|0x08|0x10|0x20|0x40|0x80));

    uint8_t p = parity(p0) | (parity(p1)<<1) | (parity(p2)<<2) | (parity(p3)<<3) | (parity(p4)<<4) | (parity(p5)<<5);
    return p;
}

void encode_hamming(uint8_t a[5], uint8_t b[5], uint8_t c[5], uint8_t d[5], const uint8_t src[16])
{
    memcpy(a, src, 4);
    a[4] = calc_hamming_32(a);
    memcpy(b, src+4, 4);
    b[4] = calc_hamming_32(b);
    memcpy(c, src+8, 4);
    c[4] = calc_hamming_32(c);
    memcpy(d, src+12, 4);
    d[4] = calc_hamming_32(d);
}


void hamming_correct(uint8_t a[5], uint8_t syn)
{
    const uint8_t flipbit[39] PROGMEM = {
        /*0*/  37, 32, 33,  0, 34,  1,  2,  3,
        /*8*/  35,  4,  5,  6,  7,  8,  9, 10,
        /*16*/ 36, 11, 12, 13, 14, 15, 16, 17,
        /*24*/ 18, 19, 20, 21, 22, 23, 24, 25,
        /*32*/ 37, 26, 27, 28, 29, 30, 31 };

    if (0==syn || syn>38) return; // No errors or something weird

    uint8_t toflip = flipbit[syn];
    a[(toflip>>3)] ^= 1<<(toflip & 7);
}


void hamming_to_bytes(uint8_t src[19], const uint8_t ap[5], const uint8_t bp[5], const uint8_t cp[5], const uint8_t dp[5])
{
    memcpy(src, ap, 4);
    memcpy(src+4, bp, 4);
    memcpy(src+8, cp, 4);
    memcpy(src+12, dp, 4);
}


// Allow us to fix single burst error of up to 4 bits during xmission
// Only first 4 bytes(the original data) and lower 6 bits of parity used
void interleave_4x38(uint8_t dest[19], uint8_t a[5], uint8_t b[5], uint8_t c[5], uint8_t d[5])
{
    uint8_t cnt = 0; // output pointer
    uint8_t i;

    // 5th element only has 6 bits, so need to shift them a bit to keep math same
    // Then process as normally and don't store the last byte of the quartet and you're golden
    a[4] = ((a[4]&0x38)<<2) | ((a[4]&0x07)<<1);               // AAA0 AAA0
    b[4] = ((b[4]&0x38)<<1) | (b[4]&0x07);                    // 0BBB 0BBB
    c[4] = ((c[4]&0x20)<<2) | ((c[4]&0x1c)<<1) | (c[4]&0x03); // C0CC C0CC
    d[4] = ((d[4]&0x30)<<2) | ((d[4]&0x0e)<<1) | (d[4]&0x01); // DD0D DD0D

    for (i=0; i<5; i++) {
        uint8_t q;
        // Interleave while leaving bits in place to allow mask and or calculation
        q = (a[i]&(128|8)) | (b[i]&(64|4)) | (c[i]&(32|2)) | (d[i]&(16|1));
        dest[cnt ++] = q; // Don't need to rotate dest

        q = (a[i]&(64|4))  | (b[i]&(32|2))  | (c[i]&(16|1))  | (d[i]&(128|8));
        q = (q<<1) | (q&128?1:0); // ROL A bit to MSB to preserve distance between different codeword bits
        dest[cnt++] = q;

        q = (a[i]&(32|2))  | (b[i]&(16|1))  | (c[i]&(128|8)) | (d[i]&(64|4));
        q = (q<<2) | (q&128?2:0) | (q&64?1:0); // ROL A bit to MSB
        dest[cnt++] = q;

        if (i!=4) {
            q = (a[i]&(16|1))  | (b[i]&(128|8)) | (c[i]&(64|4))  | (d[i]&(32|2));
            q = (q<<3) | (q&128?4:0) | (q&64?2:0) | (q&32?1:0);  // ROL A bit to MSB
            dest[cnt++] = q;
        }
    }
}


void deinterleave_4x38(uint8_t a[5], uint8_t b[5], uint8_t c[5], uint8_t d[5], uint8_t src[20] /*1 byte bigger than data, last byte ignored */)
{
    uint8_t q = 0; // output pointer
    uint8_t i;
    for (i=0; i<5; i++) {
        src[q+1] = ((src[q+1]>>1)&127) | (src[q+1]&1?128:0);
        src[q+2] = ((src[q+2]>>2)&63) | (src[q+2]&2?128:0) | (src[q+2]&1?64:0);
        src[q+3] = ((src[q+3]>>3)&31) | (src[q+3]&4?128:0) | (src[q+3]&2?64:0) | (src[q+3]&1?32:0);
        // Deinterleave from bits in place
        a[i] = (src[q+0]&(128|8)) | (src[q+1]&(64|4)) | (src[q+2]&(32|2)) | (src[q+3]&(16|1));
        b[i] = (src[q+3]&(128|8)) | (src[q+0]&(64|4)) | (src[q+1]&(32|2)) | (src[q+2]&(16|1));
        c[i] = (src[q+2]&(128|8)) | (src[q+3]&(64|4)) | (src[q+0]&(32|2)) | (src[q+1]&(16|1));
        d[i] = (src[q+1]&(128|8)) | (src[q+2]&(64|4)) | (src[q+3]&(32|2)) | (src[q+0]&(16|1));
        q += 4;
    }
    // 5th element only has 6 bits, and they're in odd 0-padded format as shown above.  Multipass
    a[4] = 0x3f & ( ((a[4]&0xe0)>>2) | ((a[4]&0x0e)>>1) );               // AAA0 AAA0 -> 00AA AAAA
    b[4] = 0x3f & ( ((b[4]&0x70)>>1) | (b[4]&0x07) );                    // 0BBB 0BBB -> 00BB BBBB
    c[4] = 0x3f & ( ((c[4]&0x80)>>2) | ((c[4]&0x38)>>1) | (c[4]&0x03) ); // C0CC C0CC -> 00CC CCCC
    d[4] = 0x3f & ( ((d[4]&0xc0)>>2) | ((d[4]&0x1c)>>1) | (d[4]&0x01) ); // DD0D DD0D -> 00DD DDDD
}



// Deinterleave, hamming correct, and convert from hamming codewords to bytes
// Decryption is not performed, nor is CRC or any other sanity.  Return corrected bits
uint8_t CorrectMessage(uint8_t dst[16], uint8_t src[19])
{
    uint8_t inMsgAESHammA[5], inMsgAESHammB[5], inMsgAESHammC[5], inMsgAESHammD[5]; // Hamming encoded, AES encrypted, raw message
    uint8_t inMsgAESHamm[16]; // HAmming encoded, AES encrypted, raw message in byte form

    // Reorder back into codewords
    deinterleave_4x38(inMsgAESHammA, inMsgAESHammB, inMsgAESHammC, inMsgAESHammD, src);

    // Calculate Hamming for received data to ^ with transmitted to find syndrome
    uint8_t syna, synb, sync, synd;
    syna = inMsgAESHammA[4];
    synb = inMsgAESHammB[4];
    sync = inMsgAESHammC[4];
    synd = inMsgAESHammD[4];
    hamming_to_bytes(inMsgAESHamm, inMsgAESHammA, inMsgAESHammB, inMsgAESHammC, inMsgAESHammD);
    encode_hamming(inMsgAESHammA, inMsgAESHammB, inMsgAESHammC, inMsgAESHammD, inMsgAESHamm);
    syna ^= inMsgAESHammA[4];
    synb ^= inMsgAESHammB[4];
    sync ^= inMsgAESHammC[4];
    synd ^= inMsgAESHammD[4];
    int c=0;
    if (syna) c++;
    if (synb) c++;
    if (sync) c++;
    if (synd) c++;

    // Correct it and move from codewords to bytes
    hamming_correct(inMsgAESHammA, syna);
    hamming_correct(inMsgAESHammB, synb);
    hamming_correct(inMsgAESHammC, sync);
    hamming_correct(inMsgAESHammD, synd);
    hamming_to_bytes(dst, inMsgAESHammA, inMsgAESHammB, inMsgAESHammC, inMsgAESHammD);

    return c;
}



bool CheckMessageAES(uint8_t decMsg[16], const uint8_t aesKey[16], const uint8_t encMsg[16])
{
    if (aesKey) {
        AES128 aes;
        aes.setKey(aesKey, 16);
        aes.decryptBlock(decMsg, encMsg);
    } else {
        memcpy(decMsg, encMsg, 16);
    }
    uint16_t rec_crc = crc16(decMsg, 14);
    if ( ((rec_crc&0xff) != decMsg[14]) || (((rec_crc>>8)&0xff) != decMsg[15])) {
        return false;
    } else {
        return true;
    }
}


void CodeMessage(uint8_t done[19], uint8_t outMsg[16], uint8_t key[16], bool useHamm)
{
    uint8_t outMsgAES[16];     // AES encrypted copy
    uint8_t outMsgAESHammA[5], outMsgAESHammB[5], outMsgAESHammC[5], outMsgAESHammD[5]; // Hamming encoded, AES encrypted, raw message

    // Set the CRC in raw form
    uint16_t crc = crc16(outMsg, 14);
    outMsg[14] = crc & 0xff;
    outMsg[15] = crc>>8;

    // Encrypt or pass-through plaintext
    if (key) {
        AES128 aes;
        aes.setKey(key, 16);
        aes.encryptBlock(outMsgAES, outMsg);
    } else {
        memcpy(outMsgAES, outMsg, 16);
    }

    if (useHamm) {
        // Get hamming codewords
        encode_hamming(outMsgAESHammA, outMsgAESHammB, outMsgAESHammC, outMsgAESHammD, outMsgAES);

        // Pack them interleaved as raw bytes for transmission
        interleave_4x38(done, outMsgAESHammA, outMsgAESHammB, outMsgAESHammC, outMsgAESHammD);
    } else {
        memcpy(done, outMsgAES, 16);
    }
}

