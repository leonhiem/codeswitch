/*
 * Kamworks Codeswitch firmware
 *
 * Author: Leon Hiemstra <l.hiemstra@gmail.com>
 */

#ifndef _UTILS_H
#define _UTILS_H

void delay_ms(unsigned long ms);
char is_digit(char c);
unsigned long rotate_left(unsigned long x, int n);

#endif // _UTILS_H
