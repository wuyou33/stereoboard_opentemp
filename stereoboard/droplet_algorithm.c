/*
 * droplet_algorithm.h
 *
 *  Created on: Aug 21, 2015
 *      Author: Sjoerd
 */

#include "droplet_algorithm.h"



uint16_t run_droplet_algorithm(int disparities_high, uint64_t sys_time) {

	if (sys_time < prev_time)
		internal_time += 4294967296; // (2^32)
	prev_time = sys_time;
	sys_time += internal_time;

	// Control logic
	if ( phase == 1 ) // unobstructed flight
	{
		if ( disparities_high> obst_thr_1 ) //|| entropy < obst_entr) // if true, obstacle in sight
			obst_count_1++;
		else
			obst_count_1 = 0;

		if ( obst_count_1 > obst_cons_1 ) // if true, obstacle is consistent
		{
			phase = 2;
			obst_count_1 = 0; // set zero for later
		}
	} else if ( phase == 2 ) // obstacle detected, wait for action
	{
		if ( obst_time == 0 ) // when entering phase, set start time
			obst_time = sys_time;

		if ( (sys_time - obst_time) > obst_wait_2*2 ) // wait (2 clocks per ms)
		{
			phase = 3;
			obst_time = 0; // set zero for later
		}
	} else if ( phase == 3 ) // avoid
	{
		// Turn command signal for AutoPilot ???
		if ( disparities_high < obst_thr_3 ) // if true, flight direction is safe
			obst_free_3++;
		else
			obst_free_3 = 0;

		if ( obst_free_3 > obst_cons_3 ) // if true, consistently no obstacles
		{
				phase = 4;
				obst_free_3 = 0; // set zero for later
		}
	} else if ( phase == 4 ) // fly straight, but be aware of undetected obstacles
	{
		if ( obst_time_4 == 0 ) // when entering phase, set start time
			obst_time_4 =  sys_time;

		if ( disparities_high> obst_thr_4) // if true, obstacle in sight
			obst_dect_4++;
		else
			obst_dect_4 = 0;

		if ( obst_dect_4 > obst_cons_5 ) // if true, obstacle is consistent
		{
			phase = 3; // go back to phase 3
			obst_time_4 = 0; // set zero for later
			obst_dect_4 = 0; // set zero for later

		} else if ( (sys_time - obst_time_4) > obst_wait_4*2 ) // wait (2 clocks per ms)
		{
			phase = 1;
			obst_time_4 = 0; // set zero for later
			obst_dect_4 = 0;
		}
	}

	return phase;

}
