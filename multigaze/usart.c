/*
 * usart.c
 *
 *      Author: pixhawk
 */

#include "stm32f4xx_conf.h"
#include "stm32f4xx_usart.h"
#include "stm32f4xx_gpio.h"
#include "stm32f4xx_rcc.h"
//#include "misc.h"

#include "usart.h"


struct UartDataStruct USART1_Data = {USART1, 0, 0, 0, 0};
struct UartDataStruct USART2_Data = {USART2, 0, 0, 0, 0};
struct UartDataStruct USART3_Data = {USART3, 0, 0, 0, 0};
struct UartDataStruct USART4_Data = {UART4, 0, 0, 0, 0};
struct UartDataStruct USART5_Data = {UART5, 0, 0, 0, 0};
struct UartDataStruct USART6_Data = {USART6, 0, 0, 0, 0};


#define USE_UART4
//#define USE_USART1

#ifdef USE_USART1
#define MY_USART_NR  USART1
#else
#define MY_USART_NR  UART4
#endif

uint8_t uart_tx_finished(struct UartDataStruct *dev)
{
  if (dev->usart_tx_counter_read != dev->usart_tx_counter_write) {
    return 0;
  }
  return 1;
}

uint8_t usart_tx_ringbuffer_push(struct UartDataStruct *dev, uint8_t *ch, uint16_t len)
{
  USART_ITConfig(dev->device, USART_IT_TXE, DISABLE);

  /* if there is free space in buffer */
  if ((((dev->usart_tx_counter_read - dev->usart_tx_counter_write) - 1) + TXBUFFERSIZE) % TXBUFFERSIZE > len) {

    uint16_t i;
    for (i = 0; i < len; i++) {

      dev->usart_tx_buffer[dev->usart_tx_counter_write] = ch[i];
      dev->usart_tx_counter_write = (dev->usart_tx_counter_write + 1) % TXBUFFERSIZE;

    }

    USART_ITConfig(dev->device, USART_IT_TXE, ENABLE);
    return 1;
  }

  USART_ITConfig(dev->device, USART_IT_TXE, ENABLE);
  return 0;
}

int usart_char_available(struct UartDataStruct *dev)
{
  return (dev->usart_rx_counter_read != dev->usart_rx_counter_write);
}

uint8_t usart_rx_ringbuffer_pop(struct UartDataStruct *dev)
{

  USART_ITConfig(dev->device, USART_IT_RXNE, DISABLE);
//  if ((RxCounterRead != RxCounterWrite)) {

  uint8_t value = dev->usart_rx_buffer[dev->usart_rx_counter_read];
  dev->usart_rx_counter_read = (dev->usart_rx_counter_read + 1) % TXBUFFERSIZE;

  USART_ITConfig(dev->device, USART_IT_RXNE, ENABLE);
  return value;
//  }
//  USART_ITConfig(dev->device, USART_IT_RXNE, ENABLE);
//  return 0;
}

uint8_t usart_rx_ringbuffer_push_from_usart(struct UartDataStruct *dev)
{
  //USART_ITConfig(dev->device, USART_IT_TXE, DISABLE);
  dev->usart_rx_buffer[dev->usart_rx_counter_write] = USART_ReceiveData(dev->device);
  int temp = (dev->usart_rx_counter_write + 1) % TXBUFFERSIZE;

  if (temp == dev->usart_rx_counter_read) {
    return 0;
  }

  dev->usart_rx_counter_write = temp;
  return 1;
}

uint8_t usart_tx_ringbuffer_pop_to_usart(struct UartDataStruct *dev)
{


  if (dev->usart_tx_counter_read != dev->usart_tx_counter_write) {

    USART_SendData(dev->device, dev->usart_tx_buffer[dev->usart_tx_counter_read]);
    dev->usart_tx_counter_read = (dev->usart_tx_counter_read + 1) % TXBUFFERSIZE;
    return 1;
  }
  return 0;
}


void usart_isr(struct UartDataStruct *dev)
{
  if (USART_GetITStatus(dev->device, USART_IT_RXNE) != RESET) {
    if (usart_rx_ringbuffer_push_from_usart(dev) == 0) {
      /* Disable the Receive interrupt if buffer is full */
      USART_ITConfig(dev->device, USART_IT_RXNE, DISABLE);
    }
    return;
  }

  if (USART_GetITStatus(dev->device, USART_IT_TXE) != RESET) {
    if (usart_tx_ringbuffer_pop_to_usart(dev) == 0) {
      /* Disable the Transmit interrupt if buffer is empty */
      USART_ITConfig(dev->device, USART_IT_TXE, DISABLE);
    }

    return;
  }
}

void uart_init_hw(struct UartDataStruct *dev, int baudrate)
{
  /* Enable the USART OverSampling by 8 */
  USART_OverSampling8Cmd(dev->device, ENABLE);


  /* USARTx configured as follow:
    - BaudRate = XXX baud
    - Word Length = 8 Bits
    - One Stop Bit
    - No parity
    - Hardware flow control disabled (RTS and CTS signals)
    - Receive and transmit enabled
  */

  USART_InitTypeDef USART_InitStructure;
  USART_InitStructure.USART_WordLength = USART_WordLength_8b;
  USART_InitStructure.USART_StopBits = USART_StopBits_1;
  USART_InitStructure.USART_Parity = USART_Parity_No;
  USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
  USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
  USART_InitStructure.USART_BaudRate = baudrate;
  USART_Init(dev->device, &USART_InitStructure);

  USART_Cmd(dev->device, ENABLE);

  /* Enable the Transmit interrupt: this interrupt is generated when
  * the transmit data register is empty
  */
  USART_ITConfig(dev->device, USART_IT_TXE, ENABLE);

  /* Enable the Receive interrupt: this interrupt is generated when
  * the receive data register is not empty
  */
  USART_ITConfig(dev->device, USART_IT_RXNE, ENABLE);

}




void usart_init()
{

  // Configures the nested vectored interrupt controller.
  NVIC_InitTypeDef NVIC_InitStructure;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;

  // GPIO
  GPIO_InitTypeDef GPIO_InitStructure;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;    // Alternate Function
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;


#ifdef USE_USART1
  // USART1:
  // Tx1 = PA9
  // Rx1 = PA10

  // Enable the USARTx Interrupt
  NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;
  NVIC_Init(&NVIC_InitStructure);

  /* Enable the USART clocks */
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1, ENABLE);

  /* Enable GPIO clock */
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);

  /* Connect UART pins to PA9, PA10 */
  GPIO_PinAFConfig(GPIOA, GPIO_PinSource9, GPIO_AF_USART1);
  GPIO_PinAFConfig(GPIOA, GPIO_PinSource10, GPIO_AF_USART1);

  /* usart TX pin configuration */
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;
  GPIO_Init(GPIOA, &GPIO_InitStructure);

  /* usart RX pin configuration */
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;
  GPIO_Init(GPIOA, &GPIO_InitStructure);

  uart_init_hw(&USART1_Data, 9600);
#endif

#ifdef USE_USART4

  // Enable the USARTx Interrupt
  NVIC_InitStructure.NVIC_IRQChannel = UART4_IRQn;
  NVIC_Init(&NVIC_InitStructure);

  /* Enable the USART clocks */
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_UART4, ENABLE);

  /* Enable GPIO clock */
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);

  /* Connect UART pins to PA0, PA1 */
  GPIO_PinAFConfig(GPIOA, GPIO_PinSource0, GPIO_AF_UART4);
  GPIO_PinAFConfig(GPIOA, GPIO_PinSource1, GPIO_AF_UART4);

  /* usart TX pin configuration */
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0;
  GPIO_Init(GPIOA, &GPIO_InitStructure);

  /* usart RX pin configuration */
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_1;
  GPIO_Init(GPIOA, &GPIO_InitStructure);

  uart_init_hw(&USART4_Data, 9600);
#endif

}

