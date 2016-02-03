/*
 * sys_time.h
 *
 *  Created on: May 20, 2015
 *      Author: mavlab
 */

#ifndef PROFILING_H_
#ifndef SYS_TIME_H_
#define SYS_TIME_H_

#include "stm32f4xx_conf.h"

#define TIMER_TICKS_PER_SEC 2000
#define TIMER_PERIOD 20000

inline static void sys_time_init(void)
{

  /*
   * APB1 timer when prescaler = 1 is 42 MHz, 84Mhz when not
   *
   * timer_tick_frequency = Timer_default_frequency / (prescaler_set + 1)
   * frequency = timer_tick_frequency / (TIM_Period + 1)
   *
   */

  RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);
  TIM_TimeBaseInitTypeDef TIM_InitStruct;
  TIM_InitStruct.TIM_Prescaler = 42000 - 1;                // This will configure the clock to 2 kHz
  TIM_InitStruct.TIM_CounterMode = TIM_CounterMode_Up;     // Count-up timer mode
  TIM_InitStruct.TIM_Period = TIMER_PERIOD - 1;                    // 10 seconds
  TIM_InitStruct.TIM_ClockDivision = TIM_CKD_DIV1;        // Divide clock by 1
  TIM_InitStruct.TIM_RepetitionCounter = 0;                // Set to 0, not used
  TIM_TimeBaseInit(TIM2, &TIM_InitStruct);
  TIM_Cmd(TIM2, ENABLE);
}

inline static uint32_t sys_time_get(void)
{
  // timer:
  return TIM_GetCounter(TIM2);
}

inline static uint32_t get_timer_interval(uint32_t prev_time)
{
  uint32_t now = sys_time_get();
  if (now > prev_time) {
    return (now - prev_time);
  }

  return (now + TIMER_PERIOD - prev_time);
}

inline static uint32_t calc_abs_time_interval(uint32_t recent_time, uint32_t older_time)
{
  if (recent_time > older_time) {
    return (recent_time - older_time);
  }

  return (recent_time + TIMER_PERIOD - older_time);
}

#endif /* SYS_TIME_H_ */
#endif /* PROFILING_H_ */
