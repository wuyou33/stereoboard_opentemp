/*
 * usart.h
 *
 *  Created on: Feb 3, 2012
 *      Author: pixhawk
 */

#ifndef USART_H_
#define USART_H_

#include <stdint.h>

#define TXBUFFERSIZE    (64*64) // 4 KByte
#define RXBUFFERSIZE    (64*64)

#define UsartTx  Usart1Tx


struct UartDataStruct {
  void *device;
  int usart_tx_counter_read;
  int usart_tx_counter_write;
  int usart_rx_counter_read;
  int usart_rx_counter_write;
  uint8_t usart_tx_buffer[TXBUFFERSIZE];
  uint8_t usart_rx_buffer[RXBUFFERSIZE];
};

#define USE_USART1
#define USE_USART2
#define USE_USART3
#define USE_USART4
#define USE_USART5
#define USE_USART6


uint8_t usart_tx_ringbuffer_push(struct UartDataStruct *dev, uint8_t *ch, uint16_t len);

#ifdef USE_USART1
extern struct UartDataStruct USART1_Data;
static inline uint8_t Usart1Tx(uint8_t *ch, uint16_t len) { return usart_tx_ringbuffer_push(&USART1_Data, ch, len);}
#endif
#ifdef USE_USART2
extern struct UartDataStruct USART2_Data;
#endif
#ifdef USE_USART3
extern struct UartDataStruct USART3_Data;
#endif
#ifdef USE_USART4
extern struct UartDataStruct USART4_Data;
static inline uint8_t Usart4Tx(uint8_t *ch, uint16_t len) { return usart_tx_ringbuffer_push(&USART4_Data, ch, len);}
#endif
#ifdef USE_USART5
extern struct UartDataStruct USART5_Data;
#endif
#ifdef USE_USART6
extern struct UartDataStruct USART6_Data;
#endif

uint8_t usart_rx_ringbuffer_push(struct UartDataStruct *dev);
uint8_t usart_tx_ringbuffer_pop_to_usart(struct UartDataStruct *dev);

uint8_t uart_tx_finished(struct UartDataStruct *dev);


int usart_char_available(struct UartDataStruct *dev);
uint8_t usart_rx_ringbuffer_pop(struct UartDataStruct *dev);

void usart_init();

void usart_isr(struct UartDataStruct *dev);


#endif /* USART_H_ */
