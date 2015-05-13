/*
 * main_parameters.h
 *
 *  Created on: Sep 18, 2013
 *      Author: mavlab
 */

#ifndef MAIN_PARAMETERS_H_
#define MAIN_PARAMETERS_H_

/*****************
 * MAIN PARAMETERS
 *****************/

// uncomment for communication with the microcrontroller, this is for sending images:
#define USART_3000000 // commented for competition - necessary for sending images / disparity maps.
#define USE_COLOR 0 // 0
#define SEND_COMMANDS 0 // 1
#define SEND_IMAGE 1 //0
#define SEND_DISPARITY_MAP 0 // 0
#define SEND_ILLUMINANCE 0 // 0
#define SEND_FILTER 0 // 0
#define COLOR_RATIO 0 // 0
#define MAX_RATIO 10 // 10
#define SEND_MATRIX 0
#define BRIGHT_WINDOW 0 // 0
#define STEREO_CAM_NUMBER 0 //  0 = DelFly Explorer cam   1 = spare camera
#define SMALL_IMAGE
//#define LARGE_IMAGE


#endif /* MAIN_PARAMETERS_H_ */
