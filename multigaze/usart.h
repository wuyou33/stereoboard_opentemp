/*
 * usart.h
 *
 *  Created on: Feb 3, 2012
 *      Author: pixhawk
 */

#ifndef USART_H_
#define USART_H_

#include <stdint.h>

uint8_t usart_tx_ringbuffer_push(uint8_t *ch, uint8_t len);
uint8_t usart_rx_ringbuffer_push();
uint8_t usart_tx_ringbuffer_pop_to_usart();

uint8_t uart_tx_finished(void);


int usart_char_available();
uint8_t usart_rx_ringbuffer_pop();

void usart_init();
void uart_test();

void usart_isr(void);

void myhex(uint8_t v, char *buf);

void print_number(int32_t number, uint8_t new_line);
void print_numbers(uint32_t *numbers, uint8_t size, uint8_t new_line);


#endif /* USART_H_ */
