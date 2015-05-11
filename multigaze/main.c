/**
  ******************************************************************************
  * @file    main.c
  * @author  C. De Wagter
  * @version V1.0.0
  * @date    2013
  * @brief   Main program body
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "led.h"
#include "usart.h"
#include "stm32f4xx_conf.h"



/* Private function prototypes -----------------------------------------------*/
void Delay(__IO uint32_t nCount);
/* Private functions ---------------------------------------------------------*/


void init_timer2()
{
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);
  TIM_TimeBaseInitTypeDef TIM_InitStruct;
  TIM_InitStruct.TIM_Prescaler = 42000 - 1;                // This will configure the clock to 2 kHz
  TIM_InitStruct.TIM_CounterMode = TIM_CounterMode_Up;     // Count-up timer mode
  TIM_InitStruct.TIM_Period = 20000 - 1;                   // 10 seconds
  TIM_InitStruct.TIM_ClockDivision = TIM_CKD_DIV1;         // Divide clock by 1
  TIM_InitStruct.TIM_RepetitionCounter = 0;                // Set to 0, not used
  TIM_TimeBaseInit(TIM2, &TIM_InitStruct);
  TIM_Cmd(TIM2, ENABLE);
}

/**
  * @brief  Main program
  * @param  None
  * @retval None
  */
int main(void)
{
  /*
    At this stage the microcontroller clock setting is already configured,
    this is done through SystemInit() function which is called from startup
    file (startup_stm32f4xx.s) before to branch to application main.
    To reconfigure the default setting of SystemInit() function, refer to
    system_stm32f4xx.c file
  */

  // Initialize the LED
  led_init();
  led_set();

  // Initialize the serial communication (before the camera so we can print status)
  // usart_init();

  // Camera interface init
  cameras_init();

  // Print welcome message
  //char comm_buff[128] = " --- Stereo Camera --- \n\r";
  //usart_tx_ringbuffer_push((uint8_t *)&comm_buff, strlen(comm_buff));


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
    // Read Camera 1
    if (GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_11)) {
      led_clear();
      GPIO_SetBits(GPIOB, GPIO_Pin_6);
    } else {
      led_set();
      GPIO_ResetBits(GPIOB, GPIO_Pin_6);
    }
  }
}

/**
  * @brief  Delay Function.
  * @param  nCount:specifies the Delay time length.
  * @retval None
  */
void Delay(__IO uint32_t nCount)
{
  while (nCount--) {
  }
}

#ifdef  USE_FULL_ASSERT

/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */

  /* Infinite loop */
  while (1) {
  }
}
#endif

/**
  * @}
  */

