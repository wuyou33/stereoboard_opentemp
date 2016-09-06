/*
 * gate_state_estimation.h
 *
 *  Created on: Sep 6, 2016
 *      Author: Guido de Croon
 */


#ifndef GATE_STATE_ESTIMATION_H_
#define GATE_STATE_ESTIMATION_H_


  void update_filter(float x_center, float y_center, float radius, float fitness, float* turn_rate);
	void reset_states(void);
	float deg2rad(float deg);


#endif 
