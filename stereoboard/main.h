
/**
 * @brief This file contains global includes for all parameters computed in the main loop
 */


#ifndef _MAIN_H
#define _MAIN_H

#include "inttypes.h"

extern uint32_t frame_dt;     // Time between previous camera frame and current frame in ms
extern uint32_t frame_rate;   // Frequency of main loop time averaged over one second

extern struct image_t disparity_image;
extern struct image_t current_image_pair;

#endif // _MAIN_H
