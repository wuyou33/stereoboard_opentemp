/*
 * usart.h
 *
 *  Created on: Feb 3, 2012
 *      Author: pixhawk
 */

#ifndef USART_H_
#define USART_H_

#include <stdint.h>
#include PROJECT_FILE

#define TXBUFFERSIZE    (64*64) // 4 KByte
#define RXBUFFERSIZE    (64*64) // 4 KByte

#define USART_SUCCESS   1
#define USART_FAIL      0

#ifndef UsartTx
#warning No default Usart defined: Using USART4
//////////////////////////////////////////////////////
// The default communication via UsartTx must be connected to a Usart
// Stereoboard bottom = Usart1
// Stereoboard top(cpld) = Usart4
#define USE_USART4
#define UsartTx Usart4Tx
#define UsartRx Usart4Rx
#define UsartCh Usart4Ch
#endif

struct UartDataStruct {
  void *device;
  int usart_tx_counter_read;
  int usart_tx_counter_write;
  int usart_rx_counter_read;
  int usart_rx_counter_write;
  uint8_t (*tx)(uint8_t*, uint16_t);
  uint8_t (*rx)(void);
  uint8_t (*char_available)(void);
  uint8_t usart_tx_buffer[TXBUFFERSIZE];
  uint8_t usart_rx_buffer[RXBUFFERSIZE];
};

//#define USE_USART1  // StereoBoard bottom / multigaze cam1
//#define USE_USART1B  // Multigaze PC
//#define USE_USART1MUX
//#define USE_USART2
//#define USE_USART3
//#define USE_USART4  // StereoBoard normal
//#define USE_USART4B  // Multigaze 4
//#define USE_USART5
//#define USE_USART6


// Low-level interaction
void usart_init(void);
int usart_char_available(struct UartDataStruct *dev);
uint8_t usart_rx_ringbuffer_pop(struct UartDataStruct *dev);
uint8_t usart_tx_ringbuffer_push(struct UartDataStruct *dev, uint8_t *ch, uint16_t len);

// Convenience functions
#if (defined(USE_USART1) || defined(USE_USART1B) || defined(USE_USART1MUX))
extern struct UartDataStruct USART1_Data;
static inline uint8_t Usart1Tx(uint8_t *ch, uint16_t len) { return usart_tx_ringbuffer_push(&USART1_Data, ch, len);}
static inline uint8_t Usart1Ch(void) { return usart_char_available(&USART1_Data);}
static inline uint8_t Usart1Rx(void) { return usart_rx_ringbuffer_pop(&USART1_Data);}
#endif
#ifdef USE_USART2
extern struct UartDataStruct USART2_Data;
static inline uint8_t Usart2Tx(uint8_t *ch, uint16_t len) { return usart_tx_ringbuffer_push(&USART2_Data, ch, len);}
static inline uint8_t Usart2Ch(void) { return usart_char_available(&USART2_Data);}
static inline uint8_t Usart2Rx(void) { return usart_rx_ringbuffer_pop(&USART2_Data);}
#endif
#ifdef USE_USART3
extern struct UartDataStruct USART3_Data;
static inline uint8_t Usart3Tx(uint8_t *ch, uint16_t len) { return usart_tx_ringbuffer_push(&USART3_Data, ch, len);}
static inline uint8_t Usart3Ch(void) { return usart_char_available(&USART3_Data);}
static inline uint8_t Usart3Rx(void) { return usart_rx_ringbuffer_pop(&USART3_Data);}
#endif
#if defined(USE_USART4) || defined(USE_USART4B)
extern struct UartDataStruct USART4_Data;
static inline uint8_t Usart4Tx(uint8_t *ch, uint16_t len) { return usart_tx_ringbuffer_push(&USART4_Data, ch, len);}
static inline uint8_t Usart4Ch(void) { return usart_char_available(&USART4_Data);}
static inline uint8_t Usart4Rx(void) { return usart_rx_ringbuffer_pop(&USART4_Data);}
#endif
#ifdef USE_USART5
extern struct UartDataStruct USART5_Data;
static inline uint8_t Usart5Tx(uint8_t *ch, uint16_t len) { return usart_tx_ringbuffer_push(&USART5_Data, ch, len);}
static inline uint8_t Usart5Ch(void) { return usart_char_available(&USART5_Data);}
static inline uint8_t Usart5Rx(void) { return usart_rx_ringbuffer_pop(&USART5_Data);}
#endif
#ifdef USE_USART6
extern struct UartDataStruct USART6_Data;
static inline uint8_t Usart6Tx(uint8_t *ch, uint16_t len) { return usart_tx_ringbuffer_push(&USART6_Data, ch, len);}
static inline uint8_t Usart6Ch(void) { return usart_char_available(&USART6_Data);}
static inline uint8_t Usart6Rx(void) { return usart_rx_ringbuffer_pop(&USART6_Data);}
#endif



// ISR callback
void usart_isr(struct UartDataStruct *dev);


#endif /* USART_H_ */
