/*
 * Kamworks Codeswitch firmware
 *
 * Author: Leon Hiemstra <l.hiemstra@gmail.com>
 */

void delay_ms(unsigned long ms)
{
#define DELAY_1ms (850)  // 1/F_CPU * 850   (F_CPU=8e6) (measured with oscilloscope)
	unsigned long cnt;
	for(cnt=0;cnt<(DELAY_1ms*ms);cnt++) asm volatile ("nop");
}

char is_digit(char c)
{
	if(c&0xc0) return 0;
	if(c<0x30) return 0;
	if(c>0x39) return 0;
	return 1;
}

unsigned long rotate_left(unsigned long x, int n)
{
	return ((x << n) | (x >> (32 - n)));
}

