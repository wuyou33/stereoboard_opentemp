/*
 * cameras.c
 *
 *  Created on: May 11, 2015
 *      Author: mavlab
 *
 *
 *
 *  Tx1 = PA9
 *  Rx1 = PA10
 *
 *  Tx2 = PA2
 *  Rx2 = PA3
 *
 *  Tx3 = PB10
 *  Rx3 = PB11
 *
 *  Tx4 = PC10
 *  Rx4 = PC11
 *
 *  Tx5 = PC12
 *  Rx5 = PD2
 *
 *  Tx6 = PC6
 *  Rx6 = PC7
 *
 */



#include "tunnel.h"


#include "stm32f4xx_conf.h"


void tunnel_init(void)
{
  GPIO_InitTypeDef  GPIO_InitStructure;

  /* GPIOA Peripheral clock enable */
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);

  /* GPIOB Peripheral clock enable */
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);

  /* GPIOC Peripheral clock enable */
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);

  /* GPIOD Peripheral clock enable */
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE);

  // UART1

  /* Configure PA9 in output push-pull mode */
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
  GPIO_Init(GPIOA, &GPIO_InitStructure);

  /* Configure PA10 in input push-pull mode */
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
  GPIO_Init(GPIOA, &GPIO_InitStructure);


  // UART2

  /* Configure PA2 in output push-pull mode */
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
  GPIO_Init(GPIOA, &GPIO_InitStructure);

  /* Configure PA3 in input push-pull mode */
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
  GPIO_Init(GPIOA, &GPIO_InitStructure);


  // UART3

  /* Configure PB10 in output push-pull mode */
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
  GPIO_Init(GPIOB, &GPIO_InitStructure);

  /* Configure PB11 in input push-pull mode */
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
  GPIO_Init(GPIOB, &GPIO_InitStructure);


  // UART4

  /* Configure PC10 in output push-pull mode */
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
  GPIO_Init(GPIOC, &GPIO_InitStructure);

  /* Configure PC11 in input push-pull mode */
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
  GPIO_Init(GPIOC, &GPIO_InitStructure);


  // UART5

  /* Configure PC12 in output push-pull mode */
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
  GPIO_Init(GPIOC, &GPIO_InitStructure);

  /* Configure PD2 in input push-pull mode */
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
  GPIO_Init(GPIOD, &GPIO_InitStructure);


  // UART6

  /* Configure PC6 in output push-pull mode */
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
  GPIO_Init(GPIOC, &GPIO_InitStructure);

  /* Configure PC7 in input push-pull mode */
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_7;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
  GPIO_Init(GPIOC, &GPIO_InitStructure);


  // OUTPUT PC: UART1:
  // Tx1 = PB6
  // Rx1 = PB7

  /* Configure PC6 in output push-pull mode */
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
  GPIO_Init(GPIOB, &GPIO_InitStructure);

  /* Configure PC7 in input push-pull mode */
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_7;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
  GPIO_Init(GPIOB, &GPIO_InitStructure);

}

void tunnel_run()
{
  while (1) {
    // Read PC input
    if (GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_7)) {
      //GPIO_SetBits(GPIOB, GPIO_Pin_6);
      GPIO_SetBits(GPIOA, GPIO_Pin_9);     // UART1
      GPIO_SetBits(GPIOA, GPIO_Pin_2);      // UART2
      GPIO_SetBits(GPIOB, GPIO_Pin_10);     // UART3
      GPIO_SetBits(GPIOC, GPIO_Pin_10);     // UART4
      GPIO_SetBits(GPIOC, GPIO_Pin_12);     // UART5
      GPIO_SetBits(GPIOC, GPIO_Pin_6);      // UART6
      //led_clear();
    } else {
      //GPIO_ResetBits(GPIOB, GPIO_Pin_6);
      GPIO_ResetBits(GPIOA, GPIO_Pin_9);     // UART1
      GPIO_ResetBits(GPIOA, GPIO_Pin_2);      // UART2
      GPIO_ResetBits(GPIOB, GPIO_Pin_10);     // UART3
      GPIO_ResetBits(GPIOC, GPIO_Pin_10);     // UART4
      GPIO_ResetBits(GPIOC, GPIO_Pin_12);     // UART5
      GPIO_ResetBits(GPIOC, GPIO_Pin_6);      // UART6
      //led_set();
    }

    // Read Camera
#if defined(TUNNEL4)
    if (GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_10)) { // UART1
#elif defined(TUNNEL1)
    if (GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_3)) { // UART2
#elif defined(TUNNEL6)
    if (GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_11)) { // UART3
#elif defined(TUNNEL3)
    if (GPIO_ReadInputDataBit(GPIOC, GPIO_Pin_11)) { // UART4
#elif defined(TUNNEL2)
    if (GPIO_ReadInputDataBit(GPIOD, GPIO_Pin_2)) { // UART5
#elif defined(TUNNEL5)
    if (GPIO_ReadInputDataBit(GPIOC, GPIO_Pin_7)) { // UART6
#else
    if (0) {
#endif
      led_clear();
      GPIO_SetBits(GPIOB, GPIO_Pin_6);
    } else {
      led_set();
      GPIO_ResetBits(GPIOB, GPIO_Pin_6);
    }
  }
}
