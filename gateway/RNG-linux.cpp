#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "RNG.h"

RNGClass::RNGClass()
{
}

RNGClass::~RNGClass()
{
}

void RNGClass::rand(uint8_t *data, size_t len)
{
	FILE *fp;
        size_t rlen;
	fp = fopen("/dev/urandom", "rb");
	if (!fp) exit(-1);
        rlen = fread(data, 1, len, fp);
        fclose(fp);
	if (len != rlen) {
		printf("fread fail, read %d\n", (int)rlen);
		exit(-1);
	}
	return;
}

RNGClass RNG;

