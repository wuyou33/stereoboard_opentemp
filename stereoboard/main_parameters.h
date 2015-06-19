/*
 * main_parameters.h
 *
 *  This file contains the board configuration defaults
 *
 *  If you would like to change any of these parameters, then make a board file and override the setting there
 */

#ifndef MAIN_PARAMETERS_H_
#define MAIN_PARAMETERS_H_

/*****************
 * MAIN PARAMETERS
 *****************/

// Include board specific overrides
#include BOARD_FILE

#define USE_COLOR 0 // 0

//////////////////////////////////////////////////////
// Define which code should be run:
#ifndef SEND_COMMANDS
	#define SEND_COMMANDS 0 // 1
#endif

#ifndef SEND_IMAGE
	#define SEND_IMAGE 0 // 1
#endif

#ifndef SEND_DISPARITY_MAP
	#define SEND_DISPARITY_MAP 0 // 0
#endif

#ifndef SEND_MATRIX
	#define SEND_MATRIX 0
#endif

//////////////////////////////////////////////////////
// Settings
#ifndef SMOOTH_DISPARITY_MAP
#define SMOOTH_DISPARITY_MAP 1 // 0
#endif
#ifndef SEND_ILLUMINANCE
#define SEND_ILLUMINANCE 0 // 0
#endif
#ifndef SEND_FILTER
#define SEND_FILTER 0 // 0
#endif
#ifndef COLOR_RATIO
#define COLOR_RATIO 0 // 0
#endif
#ifndef MAX_RATIO
#define MAX_RATIO 10 // 10
#endif
#ifndef BRIGHT_WINDOW
#define BRIGHT_WINDOW 0 // 0
#endif
#ifndef STEREO_CAM_NUMBER
#define STEREO_CAM_NUMBER 1 //  0 = DelFly Explorer cam   1 = spare camera
#endif

//////////////////////////////////////////////////////
// Define at least 1 image format: default: SMALL
#if !(defined(SMALL_IMAGE) || defined(MEDIUM_IMAGE) || defined(LARGE_IMAGE))
#define SMALL_IMAGE
#endif

//////////////////////////////////////////////////////
// The default communication via UsartTx must be connected to a Usart
// Stereoboard bottom = Usart1
// Stereoboard top(cpld) = Usart4
#ifndef UsartTx
	#define USE_USART4
	#define UsartTx Usart4Tx
	#define USART4_BAUD 1000000 // high baudrate necessary for sending images / disparity maps.
#endif

//////////////////////////////////////////////////////
// Stereoboard: camera merging type

#ifndef CAMERA_CPLD_STEREO
#define CAMERA_CPLD_STEREO camera_cpld_stereo_pixmux
#endif

//////////////////////////////////////////////////////
// Calibration parameters

#ifndef DISPARITY_OFFSET_RIGHT
#define DISPARITY_OFFSET_RIGHT  0
#endif

#ifndef DISPARITY_OFFSET_LEFT
#define DISPARITY_OFFSET_LEFT  0
#endif

#ifndef DISPARITY_BORDER
#define DISPARITY_BORDER  0
#endif


#endif /* MAIN_PARAMETERS_H_ */
