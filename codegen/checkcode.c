#include <stdio.h>
#include <stdlib.h>

unsigned long rotate_left(unsigned long x, int n)
{
    return ((x << n) | (x >> (32 - n)));
}

int main(int argc, char* argv[])
{
    unsigned int shsid,givencode,days,seq=0,i;
    unsigned long long testcode;

    if(argc < 2) {
        fprintf(stderr,"Usage:   %s code\n",argv[0]);
        fprintf(stderr,"Example: %s 2240563039\n",argv[0]);
        exit(EXIT_FAILURE);
    }

    testcode = strtoull(argv[1],NULL,10);
    //printf("testcode=%Lu (0x%Lx)]\n",testcode,testcode);
    if(testcode > 0xffffffff) {
        printf("Error: given code is more than 32bits\n");
        exit(EXIT_FAILURE);
    }

    givencode = strtoul(argv[1],NULL,10);
    givencode = ~givencode; // invert all bits
    givencode = rotate_left(givencode,2);

    seq = (givencode&0x7f);
    days = ((givencode>>7)&0x3f);
    shsid = givencode>>13;
    //printf("given code=%u (0x%x)]\n",givencode,givencode);
    printf("[seq=%u days=%u id=%u]\n",seq,days,shsid);
}

