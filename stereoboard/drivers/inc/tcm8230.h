/**
 *  \file tcm8230.h TCM8230 Driver in I2C
 *
 *  Copyright (c) See main file for copyright and liability
 */

#ifndef __MY_TCM8230_HEADER__
#define __MY_TCM8230_HEADER__

#include <stdint.h>

// Functions
void camera_tcm8230_i2c_init(void);
void camera_tcm8230_read(void);
void camera_tcm8230_config(void);

typedef struct {
  uint8_t framesize;
  uint8_t framerate;
  uint8_t sw_codes;
} tcm8230_settings;


#endif
