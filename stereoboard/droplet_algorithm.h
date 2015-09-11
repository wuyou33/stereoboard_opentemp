/*
 * droplet_algorithm.h
 *
 *  Created on: Aug 21, 2015
 *      Author: Sjoerd
 */

#ifndef DROPLET_ALGORITHM_H_
#define DROPLET_ALGORITHM_H_

#include <arm_math.h>
//#include "../multigaze/stereoboard_parameters.h"
//#include "main_parameters.h"


// parameter setting
int obst_thr_1 = 10; 		// obstacle threshold for phase 1
int obst_wait_2 = 3000; 	// wait time for phase 2
int obst_thr_3 = 10;		// obstacle threshold for phase 3
int obst_thr_4 = 10;		// obstacle threshold for phase 4
int obst_wait_4 = 500;		// wait time for phase 4

int obst_cons_1 = 5;		// obstacle consistency threshold for phase 1
int obst_cons_3 = 1;		// no-obstacle consistency threshold for phase 3
int obst_cons_5 = 2;		// obstacle consistency threshold for phase 4

// init
int obst_count_1 = 0;		// counter for sequential obstacle detections in phase 1
int obst_free_3 = 0;		// counter for sequential no-obstacle detections in phase 3
int obst_dect_4 = 0;			// counter for sequential obstacle detections in phase 4
uint64_t obst_time = 0; 			// timer for phase 2
uint64_t obst_time_4 = 0;			// timer for phase 4

int phase = 1;
uint64_t internal_time = 0;
uint32_t prev_time = 0;


uint16_t run_droplet_algorithm(int disparities_high, uint64_t sys_time );



#endif /* DROPLET_ALGORITHM_H_ */
