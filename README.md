# sensy
Secure sensor network with Elliptic Curve cryptography, AES-128 encryption, Hamming code ECC.

##Work in progress
This is still a work in progress.  Encrypted communications are working
and AVR power demands are at arund the 9ua static level with polled
switches, but the gateway software still merely handles comms and does
implement any needed REST API for the controller (which is not even
started and may end up being replaced with some other standard one).

This writeup is alsonot even a first draft, sorry.

## About
This code and schematics implement a home-area sensor network of sensors,
gateways, and a master control with web interface.

What sets this apart from other is the emphasis on security and reliability.

### Linking up sensors to the gateway (public key exchange)
Sensors pair with the gateway and exchange a public key pair to begin
communication.  There is no global master key or backdoor required.

### Communicating reports securely (AES, replay attacks)
Each report is encrypted with a nonce and AES-128 encryption based off of
the shared key generated at link-up.  After the nonce is cycled a new key
handshake occurs.  I believe this implements perfect forward secrecy.
This nonce is actually a sequence number, and it is used to prevent
replay attacks as the gateway remembers the last message sequence for
each node.  Every sensor<->gateway connection has a unique key.

### Over-the-air information protection
Hamming codes are used to allow for correction of up to 4 bits in
the message of 16 bytes.  The hamming encoded, AES encrypted message
is also interleaved to better protect against burst errors.  A
standard CRC-16 (CCITT) is used to ensure message integrity on the
receiving end.  This CRC is part of the AES-encrypted message.

### Implicit addressing, every OTA bit encrypted
Messages, once out of join mode, are completely encrypted.  There is
no sender source address and no receiver destination address required.
In place of a publicly visible (and unencrypted) address field in the
message, the actual AES key itself defines the address.  Sensors only
need the shared AES key for its link to the gateway, and the gateway
when it receives a message trys to decode it using all the active
sensor keys.  If the key successfully decrypts the message, that
identifies the sending sensor uniquely (and without requiring any
additional bits in the protocol).

On the sensor side this imposes no overhead, it only knows its
single shared key with the gateway.  On the gateway this does
require that for each received message it needs to perform up
to #-sensors AES-128 decryption attempts to find if a message is
valid or not, but these are phenomenally fast operations and
the actual message is a single 16-byte step.


## Data flow
(@ sensor)
[sensor 14-byte message]
  -> Append CRC16
  -> Update nonce (sequence number)
  -> AES-128 encrypt
  -> Hamming encode 32:38 (4 codewords)
  -> Interleave 4 codewords
  -> RF24 transmit 19 (no CRC, no auto-ack)

(@ gateway)
  -> RF24 receive 19 bytes 
  -> Deinterleave
  -> Hamming decode
  -> For all i in AES[i]:
        AES-128 decrypt(aes, decoded, decrypted);
        if (crc16(decrypted) == decrypted.crc] =>
          sensor ID = i
          if (decrypted.seq == lastSeq[i]) =>
               Sensor didn't hear last ACK, resend it
          if (decrypted.seq < lastSeq) => Replay attack, drop!
          Update internal state
          Send ACK (same dataflow as sensor message, encryption/hamming/etc.)

(@ sensor)
  -> Recevied, decode ACK
  -> Increment seqNo, if overflow perform new key creation/exchange
    


## Library Attribution
GPL Cryptographic routines from:
* Rhys Weather's libraries: https://github.com/rweather/arduinolibs

NRF24L01 interface code from:
* TMRh20's library: https://github.com/TMRh20/RF24

1MHz bootloader @ 9600bps Optiboot from:
http://forum.arduino.cc/index.php?topic=160647.15

-Earle F. Philhower, III
 earlephilhower@yahoo.com
