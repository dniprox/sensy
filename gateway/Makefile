LIBDIR=../lib
CCFLAGS=-g -Wall
CC=g++
LDFLAGS=-g
RF24=$(LIBDIR)/RF24
CRYPTO=$(LIBDIR)/arduinocrypto/libraries/Crypto

INCLUDES=-I$(RF24) -I$(CRYPTO) -I$(LIBDIR)

all: gateway

SRC=gateway.cpp RNG-linux.cpp ../lib/coding.cpp $(CRYPTO)/Curve25519.cpp $(CRYPTO)/BigNumberUtil.cpp $(CRYPTO)/AES128.cpp $(CRYPTO)/AESCommon.cpp $(CRYPTO)/BlockCipher.cpp $(CRYPTO)/Crypto.cpp $(RF24)/RF24.cpp
OBJ=$(SRC:.cpp=.o)

gateway: $(OBJ)
	$(CC) $(GCCOPTS) $(INCLUDES) -o gateway $(OBJ) -lpthread

.SUFFIXES: .cpp

.cpp.o:
	$(CC) $(INCLUDES) $(CCFLAGS) -c $< -o $@
 