/***
 *
 */

#include "cpld.h"

#include "stm32f4xx_conf.h"
#include "main_parameters.h"

/**
 *    DCMI8 = PC10  = Manual
 *    DCMI9 = PC12  = PxMux
 *    DCMI10  =   PB5   = LnMux
 */



// Setup IO Pins for image merging configuration
void camera_cpld_stereo_init(void)
{
  GPIO_InitTypeDef  GPIO_InitStructure;

  /* GPIOB Periph clock enable */
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB | RCC_AHB1Periph_GPIOC, ENABLE);

  /* Configure in output push-pull mode */
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
  GPIO_Init(GPIOB, &GPIO_InitStructure);

#ifndef DCMI_TEN_BITS
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10 | GPIO_Pin_12;
  GPIO_Init(GPIOC, &GPIO_InitStructure);
#endif

  // Select Desired Default Camera
  CAMERA_CPLD_STEREO();
}

// Set Stereo Mode
void camera_cpld_stereo_left(void)
{
#ifndef DCMI_TEN_BITS
  GPIO_ResetBits(GPIOC, GPIO_Pin_10);
  GPIO_ResetBits(GPIOC, GPIO_Pin_12);
#endif
  GPIO_ResetBits(GPIOB, GPIO_Pin_5);
}

void camera_cpld_stereo_right(void)
{
#ifndef DCMI_TEN_BITS
  GPIO_SetBits(GPIOC, GPIO_Pin_10);
  GPIO_ResetBits(GPIOC, GPIO_Pin_12);
#endif
  GPIO_ResetBits(GPIOB, GPIO_Pin_5);
}

void camera_cpld_stereo_pixmux(void)
{
#ifndef DCMI_TEN_BITS
  GPIO_ResetBits(GPIOC, GPIO_Pin_10);
  GPIO_SetBits(GPIOC, GPIO_Pin_12);
#endif
  GPIO_ResetBits(GPIOB, GPIO_Pin_5);
}

void camera_cpld_stereo_linemux(void)
{
#ifndef DCMI_TEN_BITS
  GPIO_ResetBits(GPIOC, GPIO_Pin_10);
  GPIO_ResetBits(GPIOC, GPIO_Pin_12);
#endif
  GPIO_SetBits(GPIOB, GPIO_Pin_5);
}

void camera_cpld_stereo_framemux(void)
{
#ifndef DCMI_TEN_BITS
  GPIO_SetBits(GPIOC, GPIO_Pin_10);
  GPIO_SetBits(GPIOC, GPIO_Pin_12);
#endif
  GPIO_SetBits(GPIOB, GPIO_Pin_5);
}

