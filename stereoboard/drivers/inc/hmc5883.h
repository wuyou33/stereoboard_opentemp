/**
 *  \file hmc5883.h HMC5883L Driver in I2C
 *
 *  Copyright (c) See main file for copyright and liability
 */

#ifndef __MY_HMC5883L_HEADER__
#define __MY_HMC5883L_HEADER__

#include <stdint.h>


// Functions
// Call after tmc8230_i2c_init !!!!!!!!!!!!!!!
int hmc5883_config(void); // return success/fail<0
void hmc5883_read(void);

// Testing
void draw_mag_as_line(int m);

extern int16_t magneticfield[3];


#endif
