/*
 * learning.h
 *
 *  Created on: 16 feb. 2016
 *      Author: Kevin
 */

#include <stdint.h>
#include <inttypes.h>
#include <arm_math.h>

#include "textons.h"

void learning_collisions_init(void);
void learning_collisions_run(uint8_t *image);
