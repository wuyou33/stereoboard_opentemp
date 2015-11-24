/*
 * main_parameters.h
 *
 *  This file contains the board configuration defaults
 *
 *  If you would like to change any of these parameters, then make a board file and override the setting there
 */

#ifndef MAIN_PARAMETERS_H_
#define MAIN_PARAMETERS_H_

/* Defines possible values in the project file */
#define HISTOGRAM_OBSTACLE_AVOIDANCE_DRONE 1
#define HISTOGRAM_FOLLOW_ME_DRONE 2

/*****************
 * MAIN PARAMETERS
 *****************/
#ifndef PROJECT_FILE
#define PROJECT_FILE "projects/example.h"
#endif
//#pragma message("PROJECT_FILE=" PROJECT_FILE)
#include PROJECT_FILE

#ifndef BOARD_FILE
#define BOARD_FILE "boards/board_default.h"
#endif
//#pragma message("BOARD_FILE=" BOARD_FILE)
#include BOARD_FILE

#ifndef DEFAULT_BOARD_FUNCTION
#define DEFAULT_BOARD_FUNCTION SEND_IMAGE
#endif

//////////////////////////////////////////////////////
// Settings
#ifndef STEREO_BUF_SIZE
#define STEREO_BUF_SIZE 400
#endif

#ifndef USE_COLOR
#define USE_COLOR 0
#endif

#ifndef SMOOTH_DISPARITY_MAP
#define SMOOTH_DISPARITY_MAP 0
#endif

#ifndef SEND_ILLUMINANCE
#define SEND_ILLUMINANCE 0
#endif

#ifndef SEND_FILTER
#define SEND_FILTER 0
#endif

#ifndef COLOR_RATIO
#define COLOR_RATIO 0
#endif

#ifndef BRIGHT_WINDOW
#define BRIGHT_WINDOW 0
#endif

#ifndef STEREO_CAM_NUMBER
#define STEREO_CAM_NUMBER 0 //  0 = DelFly Explorer cam   1 = spare camera
#endif

#define HISTOGRAM_OBSTACLE_AVOIDANCE_DRONE 1
#define HISTOGRAM_FOLLOW_ME_DRONE 2
#ifndef HISTOGRAM_FUNCTION
#define HISTOGRAM_FUNCTION HISTOGRAM_OBSTACLE_AVOIDANCE_DRONE
#endif

#ifndef STEREO_ALGORITHM
#define STEREO_ALGORITHM 1 // 1 = Dense   0 = Sparse
#endif

#if !(defined(SMALL_IMAGE) || defined(LARGE_IMAGE))
#define SMALL_IMAGE //#define LARGE_IMAGE
#endif

#if !(defined(IMAGE_WIDTH) || defined(IMAGE_HEIGHT))
#ifdef SMALL_IMAGE
#define IMAGE_WIDTH 128
#define IMAGE_HEIGHT 96
#else
#define IMAGE_WIDTH 128
#define IMAGE_HEIGHT 96
#endif
#endif

#ifndef CAPTURE_MODE_SNAPSHOT
#define CAPTURE_MODE_SNAPSHOT 0
#endif

#ifndef AVG_FREQ
#define AVG_FREQ 0
#endif
//////////////////////////////////////////////////////
// Define image format
//#define CAMERA_CPLD_STEREO camera_cpld_stereo_left
//#define CAMERA_CPLD_STEREO camera_cpld_stereo_right
//#define CAMERA_CPLD_STEREO camera_cpld_stereo_pixmux
//#define CAMERA_CPLD_STEREO camera_cpld_stereo_linemux

//////////////////////////////////////////////////////
// Stereoboard: camera merging type

#ifndef CAMERA_CPLD_STEREO
#define CAMERA_CPLD_STEREO camera_cpld_stereo_pixmux
#endif

#if (DCMI_TEN_BITS == 1)
#define BYTES_PER_PIXEL 4
#else
#define BYTES_PER_PIXEL 2
#endif

#define FULL_IMAGE_SIZE  (IMAGE_WIDTH*IMAGE_HEIGHT*BYTES_PER_PIXEL)
#if (FULL_IMAGE_SIZE >= (120*1024))
#error "Config error: Image does not fit im RAM"
#endif

//////////////////////////////////////////////////////
// The default communication via UsartTx must be connected to a Usart
// Stereoboard bottom = Usart1
// Stereoboard top(cpld) = Usart4
#ifndef UsartTx
#define USE_USART4
#define UsartTx Usart4Tx
#define UsartRx Usart4Rx
#define UsartCh Usart4Ch

#ifndef USART4_BAUD
#define USART4_BAUD 1000000
#endif
#endif

//////////////////////////////////////////////////////
// Image Encoding

#if ! (defined(USE_RGB565) || defined(USE_YUV422))
#define USE_YUV422
#endif

#ifndef TCM8230_EXTRA_SATURATION
#define TCM8230_EXTRA_SATURATION 0
#endif

#define MIN(X, Y) (((X) < (Y)) ? (X) : (Y))
#define MAX(X, Y) (((X) > (Y)) ? (X) : (Y))

#endif /* MAIN_PARAMETERS_H_ */
