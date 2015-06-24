/**
 *  \file ov7670.h OV7670 Driver in I2C
 *
 *  Copyright (c) See main file for copyright and liability
 */

#ifndef __MY_OV7670_HEADER__
#define __MY_OV7670_HEADER__

#include <stdint.h>


#include "ov7670_reg.h"

// Functions
void camera_ov7670_read(void);
void camera_ov7670_config(void);


#endif

