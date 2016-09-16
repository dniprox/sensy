LIBDIR=../lib
CCFLAGS=-g -Wall -pedantic
CC=g++
LDFLAGS=-g
RF24=$(LIBDIR)/RF24
CRYPTO=$(LIBDIR)/arduinocrypto/libraries/Crypto

INCLUDES=-I$(RF24) -I$(CRYPTO) -I$(LIBDIR)

all: gateway

SRC=gateway.cpp RNG-linux.cpp web.cpp ../lib/coding.cpp ../lib/message.cpp ../lib/radio.cpp $(CRYPTO)/Curve25519.cpp $(CRYPTO)/BigNumberUtil.cpp $(CRYPTO)/AES128.cpp $(CRYPTO)/AESCommon.cpp $(CRYPTO)/BlockCipher.cpp $(CRYPTO)/Crypto.cpp $(RF24)/RF24.cpp

OBJ=$(SRC:.cpp=.o)

gateway: $(OBJ)
	$(CC) $(GCCOPTS) $(INCLUDES) -o gateway $(OBJ) -lpthread -lrf24-bcm -lpaho-mqtt3c -ljson-c

clean:
	rm -f gateway $(OBJ)

.SUFFIXES: .cpp

.cpp.o:
	$(CC) $(INCLUDES) $(CCFLAGS) -c $< -o $@
 
