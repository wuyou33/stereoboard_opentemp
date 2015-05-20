/*
 * profiling.h
 *
 *  Created on: May 20, 2015
 *      Author: mavlab
 */

#ifndef PROFILING_H_
#define PROFILING_H_

void sys_time_init(void);
inline static uint32_t sys_time_get(void)
{
  // timer:
  return TIM_GetCounter ( TIM2 );
}


#endif /* PROFILING_H_ */
