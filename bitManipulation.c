#include <stdio.h>
/*
A scenario where bit 3:3 be checked where it SET or RESET.
If it is Set, the bit will be reset
*/
typedef unsigned char UInt8;        //
typedef unsigned short int UInt16;
typedef unsigned int UInt32;

#define BITMASK     0x08

void main()  // no return from function
{
UInt8 regVal = 0xDA;

if (BITMASK & regVal)
{
    printf("The bit is in SET state\n");
}

else
{
    printf("The bit is in RESET state\n");
}

regVal &= ~BITMASK;
printf("The register value after reset bit BITMASK is : %.2x\n", regVal);

}