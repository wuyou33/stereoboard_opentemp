/*
 * window_detection.h
 *
 *  Created on: Sep 9, 2013
 *      Author: mavlab
 */

#ifndef FILTER_COLOR_H_
#define FILTER_COLOR_H_

#include <arm_math.h>

uint16_t filter_red_color(uint8_t *in, uint8_t *out, uint32_t image_width, uint32_t image_height, uint8_t min_U,
                          uint8_t max_U, uint8_t min_V, uint8_t max_V);

#endif