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

#endif /* UTILS_H_ */
