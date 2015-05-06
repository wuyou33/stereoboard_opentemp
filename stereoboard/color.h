
/*
 * main_parameters.h
 *
 *  Created on: ?
 *      Author: mavlab
 */

#ifndef COLOR_H_
#define COLOR_H_

#include "image.h"
#include <stdint.h>

uint32_t colorfilt_uyvy(struct img_struct *input, struct img_struct *output, uint8_t y_m, uint8_t y_M, uint8_t u_m,
                        uint8_t u_M, uint8_t v_m, uint8_t v_M, uint32_t *results);


#endif /* COLOR_H_ */