/**
 *  \file ov2640.h OV2640 Driver in I2C
 *
 *  Copyright (c) See main file for copyright and liability
 */

#ifndef __MY_OV2640_HEADER__
#define __MY_OV2640_HEADER__

#include <stdint.h>


#include "ov2640_reg.h"

// Functions
void camera_ov2640_i2c_init(void);
void camera_ov2640_read(void);
void camera_ov2640_config(void);


#endif

