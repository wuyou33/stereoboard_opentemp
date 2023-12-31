/*
 * utils.h
 *
 *  Created on: Oct 23, 2012
 *      Author: samuezih
 */

#ifndef STEREO_UTILS_H_
#define STEREO_UTILS_H_

#include <stdint.h>

void Delay(volatile uint32_t count);

#define WAIT_OP_US 15 // 48
void wait_us(uint32_t us);
void wait_ms(uint32_t ms);

/* code from CodeForge.com */
char *ftoa(float f);
void ltoa(char *buf, unsigned long i, int base);

void myhex(uint8_t v, char *buf);

void print_space(void);
void print_number(int32_t number, uint8_t new_line);
void print_numbers(uint32_t *numbers, uint8_t size, uint8_t new_line);
void print_byte(uint8_t b);
void print_string(char *s, int len);

#endif /* STEREO_UTILS_H_ */
