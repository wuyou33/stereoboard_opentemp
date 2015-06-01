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

#include BOARD_FILE

#define USE_COLOR 0 // 0
#define SEND_COMMANDS 0 // 1
#define SEND_IMAGE 0 //0
#define SEND_DISPARITY_MAP 0 // 0
#define SEND_MATRIX 1
#define SMOOTH_DISPARITY_MAP 0 // 0
#define SEND_ILLUMINANCE 0 // 0
#define SEND_FILTER 0 // 0
#define COLOR_RATIO 0 // 0
#define MAX_RATIO 10 // 10
#define BRIGHT_WINDOW 0 // 0
#define STEREO_CAM_NUMBER 1 //  0 = DelFly Explorer cam   1 = spare camera
#define SMALL_IMAGE
//#define LARGE_IMAGE

// uncomment for communication with the microcrontroller, this is for sending images:
#ifndef UsartTx
	#define USE_USART4
	#define UsartTx Usart4Tx
	#define USART4_BAUD 1000000 // high baudrate necessary for sending images / disparity maps.
#endif

#ifndef CAMERA_CPLD_STEREO
#define CAMERA_CPLD_STEREO camera_cpld_stereo_pixmux
#endif


#endif /* MAIN_PARAMETERS_H_ */
