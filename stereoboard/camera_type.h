/*
 * camera_type.h
 *
 *  Created on: Jun 20, 2015
 *      Author: mavlab
 */

#ifndef CAMERA_TYPE_H_
#define CAMERA_TYPE_H_



#ifdef USE_OV7670
#include "ov7670.h"
#pragma message "Using OV7670"

#define DCMI_CLOCK_POLARITY DCMI_PCKPolarity_Rising

static inline void camera_control_bus_init()
{
  camera_ov7670_i2c_init();
}

static inline void camera_chip_config()
{
  camera_ov7670_config();
}

#else
#define USE_TCM8230
#pragma message "Using TCM8230"
#include "tcm8230.h"

#define DCMI_CLOCK_POLARITY DCMI_PCKPolarity_Falling

static inline void camera_control_bus_init()
{
  camera_tcm8230_i2c_init();
}

static inline void camera_chip_config()
{
  camera_tcm8230_config();
}

#endif

#endif /* CAMERA_TYPE_H_ */
