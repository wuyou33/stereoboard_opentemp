/*
 * gate_state_estimation.c
 *
 *  Created on: Sep 6, 2016
 *      Author: Guido de Croon
 */


#include "gate_state_estimation.h"
#include "gate_detection.h"
#include <math.h>
#define PI 3.14159265359
#include "main_parameters.h"
#include <stdlib.h>
#include "sys_time.h"

float 	current_angle_gate = 0.0f;
float	predicted_x_gate = 0.0f;
float	previous_x_gate = 0.0f;
float	predicted_y_gate = 5.0f;;
float	previous_y_gate = 5.0f;
float	current_x_gate = 0.0f;
float	current_y_gate = 5.0f;
float	uncertainty_gate = 0.0f;
float max_fitness = 0.2f;
uint32_t prev_time;
uint32_t prev_FSM_time;
int first_time = 1;
#define TIME_RESOLUTION 1000
#define UNCERTAINTY_LIMIT 30
// States and their numbers:
#define SEARCH 0
#define TRACK 1
#define N_STATES 2
int state = 0;
float FOV_width = 58.0f;
float velocity_gate = -0.6f; // opposite to the speed of the DelFly -> would best be measured of course
float gate_size_meters = 1.0f;
float P_gain = 1.0f;

void update_filter(float x_center, float y_center, float radius, float fitness, float* turn_rate, volatile uint32_t sys_time)
{

  // First time: reset all states, timers, and exit:
  if(first_time)
  {
    // initialize the filter:
    reset_states(sys_time);
    first_time = 0;
    prev_time = sys_time;

		// start the FSM
		// started = 1;
		// start_FSM = 0;
		prev_FSM_time = sys_time;
		state = 0;

    return;
  }

	// ****************
	// State estimation
	// ****************

	// get the time and time difference with the last call:
  volatile uint32_t DT = (get_timer_interval(prev_time) * TIME_RESOLUTION) / TIMER_TICKS_PER_SEC;
  //volatile uint32_t DT = ((sys_time-prev_time) * TIME_RESOLUTION) / TIMER_TICKS_PER_SEC;
  float dt = ((float) DT) / TIME_RESOLUTION;
  prev_time = sys_time;

  // if the filter has not run for a long time, reset the states and exit:
	if (dt > 1.0f)
	{
		reset_states(sys_time);
		return;
	}
	
	// predict the new location:
	float gate_turn_rate = -(*turn_rate); 
	float current_distance = sqrtf(current_x_gate*current_x_gate+current_y_gate*current_y_gate);
	float dx_gate = dt * (cosf(current_angle_gate) * gate_turn_rate * current_distance);
	float dy_gate = dt * (velocity_gate - sinf(current_angle_gate) * gate_turn_rate * current_distance);
	predicted_x_gate = previous_x_gate + dx_gate;
	predicted_y_gate = previous_y_gate + dy_gate;

	float measured_x_gate, measured_y_gate;
	if (fitness < GOOD_FIT)
	{
		// Determine the measurement:
		float alpha = (radius / 128.0f) * FOV_width;
		float measured_distance_gate = (0.5f * gate_size_meters) / tan(deg2rad(alpha));
		float measured_angle_gate = ((x_center - 64.0f) / (128.0f)) * FOV_width;
		measured_x_gate = measured_distance_gate * sinf(deg2rad(measured_angle_gate));
		measured_y_gate = measured_distance_gate * cosf(deg2rad(measured_angle_gate));

		// Mix the measurement with the prediction:
		float weight_measurement;
		if (uncertainty_gate > UNCERTAINTY_LIMIT)
		{
		  // if the gate was out of sight for too long, accept any measurement as the new state:
				weight_measurement = 1.0f;
		}
		else
		{
		  // scale the fitness (lower is better) between 0 and 1, topping bad fits off:
		  fitness = fitness > max_fitness ? max_fitness : fitness;
				weight_measurement = (max_fitness - fitness) / max_fitness;
		}

		current_x_gate = weight_measurement * measured_x_gate + (1.0f - weight_measurement) * predicted_x_gate;
		current_y_gate = weight_measurement * measured_y_gate + (1.0f - weight_measurement) * predicted_y_gate;
		current_angle_gate = atan2(current_x_gate,current_y_gate);

		// reset uncertainty:
		uncertainty_gate = 0;
	}
	else
	{
		// just the prediction
		current_x_gate = predicted_x_gate;
		current_y_gate = predicted_y_gate;
		current_angle_gate = atan2(current_x_gate,current_y_gate);

		// increase uncertainty
		uncertainty_gate++;
	}

	// set the previous state for the next time:
	previous_x_gate = current_x_gate;
	previous_y_gate = current_y_gate;

	// ********************
	// Finite State Machine
	// ********************

  /*
  // External control of the FSM - noch nicht im Frage.

	// stop the FSM:
	if(stop_FSM)
	{
		start_FSM = 0;
		started = 0;
		state = 0;
		stop_FSM = 0;
		reset_states();
		printf("STOPPED.");
	}

	// start the FSM:
	if(start_FSM)
	{
		// start the FSM
		started = 1;
		start_FSM = 0;
		prev_FSM_time = sys_time_get();
		state = 0;
		reset_states();
		printf("START!\n");
	}
	else if(started)
	{   
  */

	// behavior and transitions:
	switch (state)
	{

	case SEARCH:
		// Behavior:
    // Just turn right:
		(*turn_rate) = 0.25;

    // Until we find a gate:
		if (fitness < GOOD_FIT)
		{
			state = TRACK;
			prev_FSM_time = sys_time;
		}

		break;

	case TRACK:

		// Behavior:
    // Only lateral control, fixed P-gain on the x-location:
		(*turn_rate) = current_x_gate * P_gain;
    
    // If we past the gate, switch to searching again
		if (current_y_gate < 0)
		{
			state = SEARCH;
			prev_FSM_time = sys_time;
		}

		break;
	}

  //uint32_t DT_FSM = (get_timer_interval(prev_time) * TIME_RESOLUTION) / TIMER_TICKS_PER_SEC;
	//printf(status_message, "S = %d, T = %f, x = %f", state, ((float) DT_FSM) / TIME_RESOLUTION, current_x_gate);

	//}
}

void reset_states(uint32_t sys_time)
{
	current_angle_gate = 0.0f;
	predicted_x_gate = 0.0f;
	previous_x_gate = 0.0f;
	predicted_y_gate = 5.0f;;
	previous_y_gate = 5.0f;
	current_x_gate = 0.0f;
	current_y_gate = 5.0f;
	uncertainty_gate = 0.0f;
  prev_time = sys_time;
  prev_FSM_time = sys_time;
}

float deg2rad(float deg)
{
  return ((deg * PI) / 180.0f);
}

