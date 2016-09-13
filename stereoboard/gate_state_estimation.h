/*
 * gate_state_estimation.h
 *
 *  Created on: Sep 6, 2016
 *      Author: Guido de Croon
 */


#ifndef GATE_STATE_ESTIMATION_H_
#define GATE_STATE_ESTIMATION_H_

#include <stdint.h>


  void update_filter(float x_center, float y_center, float radius, float fitness, float* turn_rate, volatile uint32_t sys_time);
	void reset_states(uint32_t sys_time);
	float deg2rad(float deg);


#endif 
