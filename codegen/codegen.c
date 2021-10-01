#include <stdio.h>
#include <stdlib.h>

unsigned int rotate_right(unsigned int x, int n) {
  return ((x >> n) | (x << (32 - n)));
}


int main(int argc, char* argv[])
{
    unsigned int shsid,code,days,seq=0,runs=100,i;

    if(argc == 4) {
        seq = atoi(argv[3]);
        runs=1;
    } else if(argc < 3) {
        fprintf(stderr,"Usage:   %s id days seq\n",argv[0]);
        fprintf(stderr,"Example: %s 478839 5 1\n",argv[0]);
        fprintf(stderr,"(if seq not given, it prints 100 codes)\n");

        exit(EXIT_FAILURE);
    }

    days = atoi(argv[2]);
    shsid = atoi(argv[1]); // my codeswitch. Use code *1# to readout

    for(i=0;i<runs;i++) {
        code = shsid << 13;
        code |= ((days&0x3f) << 7);
        code |= (seq&0x7f);
        code = rotate_right(code,2);
        code = ~code;
        printf("codegen id=%d days=%d seq=%d ==> code = %010u\n",shsid,days,seq,code);
        seq++;
    }
}

