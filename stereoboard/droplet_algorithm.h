/*
 * droplet_algorithm.h
 *
 *  Created on: Aug 21, 2015
 *      Author: Sjoerd
 */

#ifndef DROPLET_ALGORITHM_H_
#define DROPLET_ALGORITHM_H_

#include <arm_math.h>

/* parameter setting
extern int obst_thr_1;      // obstacle threshold for phase 1
extern uint32_t obst_wait_2;  // wait time for phase 2
extern int obst_thr_3;      // obstacle threshold for phase 3
extern int obst_thr_4;      // obstacle threshold for phase 4
extern int obst_wait_4;     // wait time for phase 4

extern int obst_cons_1;     // obstacle consistency threshold for phase 1
extern int obst_cons_3;     // no-obstacle consistency threshold for phase 3
extern int obst_cons_5;     // obstacle consistency threshold for phase 4

// init
extern int obst_count_1;    // counter for sequential obstacle detections in phase 1
extern int obst_free_3;     // counter for sequential no-obstacle detections in phase 3
extern int obst_dect_4;     // counter for sequential obstacle detections in phase 4
extern uint32_t obst_time;    // timer for phase 2 and 4

extern int phase;
extern uint32_t prev_tim;
extern uint32_t passed_time;
extern uint32_t clock_period;
*/
uint16_t run_droplet_algorithm(int disparities_high, uint16_t disparities_total, volatile uint32_t sys_time);

#endif /* DROPLET_ALGORITHM_H_ */
