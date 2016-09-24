#include <stdio.h>

// Convert 1 nibble per line into an array for use in the thermistor routine

int main(int argc, char **argv)
{
    int l, h;
    int first = 1;

    printf("uint8_t ary[] = { ");
    while (!feof(stdin)) {
        if (scanf("%d", &h)!=1) break;
        if (scanf("%d", &l)!=1) l = 0;
        if (!first) printf(", ");
        printf("0x%02x", (l & 0x0f) | ((h<<4)&0xf0));
        first = 0;
    }
    printf("};\n");
}

