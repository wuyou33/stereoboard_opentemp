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

// FIXME calculate buffer size, now it's just an estimation
#define TXBUFFERSIZE    (64*64*8) // 4 KByte
#define RXBUFFERSIZE    (64*64)

uint8_t usart_tx_buffer[TXBUFFERSIZE] = "\n\rStereoCam\n\r";
uint8_t usart_rx_buffer[RXBUFFERSIZE] = "";

int usart_tx_counter_read = 0;
int usart_tx_counter_write = 13;
int usart_rx_counter_read = 0;
int usart_rx_counter_write = 0;

#define MY_USART_NR  UART4

uint8_t uart_tx_finished(void)
{
  if (usart_tx_counter_read != usart_tx_counter_write) {
    return 0;
  }
  return 1;
}

uint8_t usart_tx_ringbuffer_push(uint8_t *ch, uint8_t len)
{
  USART_ITConfig(MY_USART_NR, USART_IT_TXE, DISABLE);

  /* if there is free space in buffer */
  if ((((usart_tx_counter_read - usart_tx_counter_write) - 1) + TXBUFFERSIZE) % TXBUFFERSIZE > len) {

    uint8_t i;
    for (i = 0; i < len; i++) {

      usart_tx_buffer[usart_tx_counter_write] = ch[i];
      usart_tx_counter_write = (usart_tx_counter_write + 1) % TXBUFFERSIZE;

    }

    USART_ITConfig(MY_USART_NR, USART_IT_TXE, ENABLE);
    return 1;
  }

  USART_ITConfig(MY_USART_NR, USART_IT_TXE, ENABLE);
  return 0;
}

int usart_char_available()
{
  return (usart_rx_counter_read != usart_rx_counter_write);
}

uint8_t usart_rx_ringbuffer_pop()
{

  USART_ITConfig(MY_USART_NR, USART_IT_RXNE, DISABLE);
//  if ((RxCounterRead != RxCounterWrite)) {

  uint8_t value = usart_rx_buffer[usart_rx_counter_read];
  usart_rx_counter_read = (usart_rx_counter_read + 1) % TXBUFFERSIZE;

  USART_ITConfig(MY_USART_NR, USART_IT_RXNE, ENABLE);
  return value;
//  }
//  USART_ITConfig(MY_USART_NR, USART_IT_RXNE, ENABLE);
//  return 0;
}

uint8_t usart_rx_ringbuffer_push_from_usart()
{
  //USART_ITConfig(MY_USART_NR, USART_IT_TXE, DISABLE);
  usart_rx_buffer[usart_rx_counter_write] = USART_ReceiveData(MY_USART_NR);
  int temp = (usart_rx_counter_write + 1) % TXBUFFERSIZE;

  if (temp == usart_rx_counter_read) {
    return 0;
  }

  usart_rx_counter_write = temp;
  return 1;
}

uint8_t usart_tx_ringbuffer_pop_to_usart()
{


  if (usart_tx_counter_read != usart_tx_counter_write) {

    USART_SendData(MY_USART_NR, usart_tx_buffer[usart_tx_counter_read]);
    usart_tx_counter_read = (usart_tx_counter_read + 1) % TXBUFFERSIZE;
    return 1;
  }
  return 0;
}

void uart_test(void)
{
  USART_SendData(MY_USART_NR, 0x55);
}


void usart_isr(void)
{
  if (USART_GetITStatus(MY_USART_NR, USART_IT_RXNE) != RESET) {
    if (usart_rx_ringbuffer_push_from_usart() == 0) {
      /* Disable the Receive interrupt if buffer is full */
      USART_ITConfig(MY_USART_NR, USART_IT_RXNE, DISABLE);
    }
    return;
  }

  if (USART_GetITStatus(MY_USART_NR, USART_IT_TXE) != RESET) {
    if (usart_tx_ringbuffer_pop_to_usart() == 0) {
      /* Disable the Transmit interrupt if buffer is empty */
      USART_ITConfig(MY_USART_NR, USART_IT_TXE, DISABLE);
    }

    return;
  }
}


void usart_init()
{

  // Configures the nested vectored interrupt controller.
  NVIC_InitTypeDef NVIC_InitStructure;

  // Enable the USARTx Interrupt
  NVIC_InitStructure.NVIC_IRQChannel = UART4_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);

  GPIO_InitTypeDef GPIO_InitStructure;

  /* Enable the USART clocks */
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_UART4, ENABLE);

  /* Enable GPIO clock */
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);

  /* Connect UART pins to PA0, PA1 */
  GPIO_PinAFConfig(GPIOA, GPIO_PinSource0, GPIO_AF_UART4);
  GPIO_PinAFConfig(GPIOA, GPIO_PinSource1, GPIO_AF_UART4);

  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;    // Alternate Function
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;

  /* usart TX pin configuration */
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0;
  GPIO_Init(GPIOA, &GPIO_InitStructure);

  /* usart RX pin configuration */
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_1;
  GPIO_Init(GPIOA, &GPIO_InitStructure);

  /* Enable the USART OverSampling by 8 */
  USART_OverSampling8Cmd(UART4, ENABLE);


  /* USARTx configured as follow:
    - BaudRate = 115200 baud
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

  USART_InitStructure.USART_BaudRate = 1000000; //9600; //3000000; //921600; //19200; //6400 * 2; //9600 for APcomm;
  USART_Init(MY_USART_NR, &USART_InitStructure);

  USART_Cmd(MY_USART_NR, ENABLE);

  /* Enable the Transmit interrupt: this interrupt is generated when
  * the transmit data register is empty
  */
  USART_ITConfig(MY_USART_NR, USART_IT_TXE, ENABLE);

  /* Enable the Receive interrupt: this interrupt is generated when
  * the receive data register is not empty
  */
  USART_ITConfig(MY_USART_NR, USART_IT_RXNE, ENABLE);

}


void myhex(uint8_t v, char *buf)
{
  uint8_t a, b;
  a = v & 0x0f;
  b = v & 0xf0;
  b = b >> 4;
  if (a < 10) {
    a += '0';
  } else {
    a += 'A' - 10;
  }

  if (b < 10) {
    b += '0';
  } else {
    b += 'A' - 10;
  }

  buf[1] = a;
  buf[0] = b;

}

void print_number(int32_t number, uint8_t new_line)
{
  //usart_tx_ringbuffer_pop_to_usart();
#define BLEN 16
  char comm_buff[ BLEN ];
  int ii;
  for (ii = 0; ii < BLEN; ii++) {
    comm_buff[ii] = ' ';
  }

  if (number < 0) {
    number = -number;
  }

  itoa(comm_buff, number, 10);
  if (new_line) {
    comm_buff[BLEN - 2] = '\n';
    comm_buff[BLEN - 1] = '\r';
  } else {
    comm_buff[BLEN - 2] = ' ';
    comm_buff[BLEN - 1] = ' ';
  }

  usart_tx_ringbuffer_push((uint8_t *)&comm_buff, BLEN);
}

void print_numbers(uint32_t *numbers, uint8_t size, uint8_t new_line)
{
  //usart_tx_ringbuffer_pop_to_usart();

  uint8_t ii;

  for (ii = 0; ii < size - 1; ii++) {
    print_number(numbers[ii], 0);
  }

  print_number(numbers[size - 1], new_line);
}

