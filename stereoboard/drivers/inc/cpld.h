/*
 * cpld.h
 *
 *  Created on: Jul 31, 2013
 *      Author: mavlab
 */

#ifndef CPLD_H_
#define CPLD_H_

// Setup IO Pins for image merging configuration
void camera_cpld_stereo_init(void);

// Set Stereo Mode
void camera_cpld_stereo_left(void);
void camera_cpld_stereo_right(void);
void camera_cpld_stereo_pixmux(void);
void camera_cpld_stereo_linemux(void);


#endif /* CPLD_H_ */
