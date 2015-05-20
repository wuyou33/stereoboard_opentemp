/*
 * sys_time.h
 *
 *  Created on: May 20, 2015
 *      Author: mavlab
 */

#ifndef PROFILING_H_
#define SYS_TIME_H_

inline static void sys_time_init(void)
{
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);
  TIM_TimeBaseInitTypeDef TIM_InitStruct;
  TIM_InitStruct.TIM_Prescaler = 42000 - 1;                // This will configure the clock to 2 kHz
  TIM_InitStruct.TIM_CounterMode = TIM_CounterMode_Up;     // Count-up timer mode
  TIM_InitStruct.TIM_Period = 20000 - 1;                    // 10 seconds
  TIM_InitStruct.TIM_ClockDivision = TIM_CKD_DIV1;        // Divide clock by 1
  TIM_InitStruct.TIM_RepetitionCounter = 0;                // Set to 0, not used
  TIM_TimeBaseInit(TIM2, &TIM_InitStruct);
  TIM_Cmd(TIM2, ENABLE);
}
inline static uint32_t sys_time_get(void)
{
  // timer:
  return TIM_GetCounter ( TIM2 );
}


#endif /* SYS_TIME_H_ */
