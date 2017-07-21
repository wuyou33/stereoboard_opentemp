/*
 * droplet_algorithm.h
 *
 *  Created on: Aug 21, 2015
 *      Author: Sjoerd
 */

#include "droplet_algorithm.h"
#include "led.h"
#include "sys_time.h"

// parameter setting
uint16_t obst_thr_1 = 7;       // obstacle threshold for phase 1
uint16_t disp_thr_1 = 20;   // obstacle count minimum threshold for phase 1
uint32_t obst_wait_2 = 1300;  // -->1800<-- wait time for phase 2
uint16_t obst_thr_3 = 10;      // obstacle threshold for phase 3
uint16_t obst_thr_4 = 10;      // obstacle threshold for phase 4
uint16_t obst_wait_4 = 500;      // wait time for phase 4

uint16_t obst_cons_1 = 3;      // obstacle consistency threshold for phase 1
uint16_t obst_cons_3 = 1;      // no-obstacle consistency threshold for phase 3
uint16_t obst_cons_5 = 2;      // obstacle consistency threshold for phase 4

// init
uint16_t obst_count_1 = 0;     // counter for sequential obstacle detections in phase 1
uint16_t obst_free_3 = 0;      // counter for sequential no-obstacle detections in phase 3
uint16_t obst_dect_4 = 0;      // counter for sequential obstacle detections in phase 4
uint32_t obst_time = 0;     // timer for phase 2 and 4

int phase = 1;
uint32_t prev_time = 0;
uint32_t passed_time = 0;
uint32_t histogram_bin = 0;
uint32_t clock_period = 20000;

uint16_t run_droplet_algorithm(int disparities_high, uint16_t disparities_total)
{
  passed_time = get_timer_interval(obst_time) / TIMER_TICKS_PER_MSEC;

  // Control logic
  if (phase == 1) { // unobstructed flight
    if (disparities_high > obst_thr_1 || disparities_total < disp_thr_1) {  //|| entropy < obst_entr) // if true, obstacle in sight
      obst_count_1++;
    } else if (obst_count_1 > 0) {
      obst_count_1--;
    }

    if (obst_count_1 > obst_cons_1) { // if true, obstacle is consistent
      phase = 2;
      obst_count_1 = 0; // set zero for later
      obst_time = sys_time_get();
    }
  } else if (phase == 2) { // obstacle detected, wait for action
    //if ( obst_time == 0 ) // when entering phase, set start time


    if (passed_time > obst_wait_2) {
      phase = 3;
      //obst_time = 0; // set zero for later
    }
  } else if (phase == 3) { // avoid
    // Turn command signal for AutoPilot ???
    if (disparities_high < obst_thr_3 && disparities_total > disp_thr_1) { // if true, flight direction is safe
      obst_free_3++;
    } else {
      obst_free_3 = 0;
    }

    if (obst_free_3 > obst_cons_3) { // if true, consistently no obstacles
      phase = 4;
      obst_free_3 = 0; // set zero for later
      obst_time = sys_time_get();
    }
  } else if (phase == 4) { // fly straight, but be aware of undetected obstacles
    if (disparities_high > obst_thr_4 || disparities_total < disp_thr_1) { // if true, obstacle in sight
      obst_dect_4++;
    } else {
      obst_dect_4 = 0;
    }

    if (obst_dect_4 > obst_cons_5) { // if true, obstacle is consistent
      phase = 3; // go back to phase 3
      obst_dect_4 = 0; // set zero for later

    } else if (passed_time > obst_wait_4) {
      phase = 1;
      obst_dect_4 = 0;
    }
  }

  if (disparities_high > obst_thr_1 || disparities_total < disp_thr_1) {
    led_set();
  } else {
    led_clear();
  }

  return phase;
}


uint16_t run_droplet_algorithm_low_texture(int disparities_high, uint16_t disparities_total, volatile uint32_t histogram_obs,
    int count_disps_left, int count_disps_right)
{
  passed_time = get_timer_interval(obst_time) / TIMER_TICKS_PER_MSEC;

  disp_thr_1 = 5;
  obst_thr_1 = 60;

  // => max_Y = 500 mm
  // 35 too sensitive, many turns
  // 50 little bit less sensitive, but when approaching rich textured wall, still turns too early (cyberzoo sides also detected!)
  // 60 seems to work better, richly textured stuff is approached more closely. Orange poles are also detected. crashes into the net when background is blue doors

  // => max_Y = 400 mm
  // 60 works perfectly if sufficient space is available
  // 40 gives better performance with orange pole in middle of the space
  // 5 works for pole in middle

  // Control logic
  if (phase == 1) { // unobstructed flight
    if (histogram_obs > obst_thr_1 || count_disps_left < disp_thr_1 || count_disps_right < disp_thr_1) {  // if true, obstacle in sight
      phase = 3;
      obst_time =  sys_time_get();
    }
  } else if (phase == 3) { // avoid
    // Turn command signal for AutoPilot ???
    if (disparities_high < obst_thr_3  && count_disps_left > disp_thr_1 && count_disps_right > disp_thr_1) { // if true, flight direction is safe
      obst_free_3++;
    } else {
      obst_free_3 = 0;
    }

    if (obst_free_3 > obst_cons_3) { // if true, consistently no obstacles
      phase = 4;
      obst_free_3 = 0; // set zero for later
      obst_time =  sys_time_get();
    }
  } else if (phase == 4) { // fly straight, but be aware of undetected obstacles
    if (disparities_high > obst_thr_4 || disparities_total < disp_thr_1) { // if true, obstacle in sight
      obst_dect_4++;
    } else {
      obst_dect_4 = 0;
    }

    if (obst_dect_4 > obst_cons_5) { // if true, obstacle is consistent
      phase = 3; // go back to phase 3
      obst_dect_4 = 0; // set zero for later
    } else if (passed_time > obst_wait_4) {
      phase = 1;
      obst_dect_4 = 0;
    }
  }

  if (disparities_high > obst_thr_1 || disparities_total < disp_thr_1) {
    led_set();
  } else {
    led_clear();
  }

  return phase;
}
