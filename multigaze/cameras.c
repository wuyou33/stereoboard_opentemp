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



#include "cameras.h"


#include "stm32f4xx_conf.h"


void cameras_init(void)
{
  GPIO_InitTypeDef  GPIO_InitStructure;

  /* GPIOB Peripheral clock enable */
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);

  /* Configure PB12 in output push-pull mode */
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
  GPIO_Init(GPIOB, &GPIO_InitStructure);




}
