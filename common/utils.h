/*
 * utils.h
 *
 *  Created on: Oct 23, 2012
 *      Author: samuezih
 */

#ifndef UTILS_H_
#define UTILS_H_

#include <stdint.h>

#define WAIT_OP_US 15 // 48
void mydelay(volatile uint32_t count);
void wait_us(uint32_t us);
void wait_ms(uint32_t ms);

/* code from CodeForge.com */
char *ftoa(float f);
void itoa(char *buf, unsigned int i, int base);
void ltoa(char *buf, unsigned long i, int base);


void myhex(uint8_t v, char *buf);

void print_space();
void print_number(int32_t number, uint8_t new_line);
void print_numbers(uint32_t *numbers, uint8_t size, uint8_t new_line);
void print_byte(uint8_t b);
void print_string(char *s, int len);


#endif /* UTILS_H_ */
