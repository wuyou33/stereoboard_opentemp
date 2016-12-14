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
#define CAMERA_CHIP_UNRESET_TIMING 0
static inline void camera_control_bus_init(void) { SCCB_Init(); }
static inline void camera_chip_config(void) { camera_ov7670_config(); }



#elif defined( USE_OV2640 )
#include "ov2640.h"
#pragma message "Using OV2640"
#define DCMI_CLOCK_POLARITY DCMI_PCKPolarity_Rising
#define CAMERA_CHIP_UNRESET_TIMING 0
static inline void camera_control_bus_init(void) { SCCB_Init(); }
static inline void camera_chip_config(void) { camera_ov2640_config(); }



#elif defined( USE_OV9712 )
#include "ov9712.h"
#pragma message "Using OV9712"
#define DCMI_CLOCK_POLARITY DCMI_PCKPolarity_Rising
#define CAMERA_CHIP_UNRESET_TIMING 0x1FFFF
static inline void camera_control_bus_init(void) { SCCB_Init(); }
static inline void camera_chip_config(void) { camera_ov9712_config(); }



#else
#define USE_TCM8230
//#pragma message "Using TCM8230"
#include "tcm8230.h"
#define DCMI_CLOCK_POLARITY DCMI_PCKPolarity_Falling
#define CAMERA_CHIP_UNRESET_TIMING 0x01FFFF
static inline void camera_control_bus_init(void) { camera_tcm8230_i2c_init(); }
static inline void camera_chip_config(void) { camera_tcm8230_config(); }

#endif

#endif /* CAMERA_TYPE_H_ */
