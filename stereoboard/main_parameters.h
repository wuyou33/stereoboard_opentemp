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
#include BOARD_FILE

#define USART4_BAUD 38400 //230400 //921600 //38400

//#define DEFAULT_BOARD_FUNCTION SEND_COMMANDS
//#define DEFAULT_BOARD_FUNCTION SEND_TURN_COMMANDS
//#define DEFAULT_BOARD_FUNCTION SEND_IMAGE
//#define DEFAULT_BOARD_FUNCTION SEND_DISPARITY_MAP
//#define DEFAULT_BOARD_FUNCTION SEND_FRAMERATE_STEREO
//#define DEFAULT_BOARD_FUNCTION SEND_MATRIX
//#define DEFAULT_BOARD_FUNCTION SEND_DIVERGENCE
#define DEFAULT_BOARD_FUNCTION SEND_WINDOW

//////////////////////////////////////////////////////
// Settings
#define USE_COLOR 0
#ifndef SMOOTH_DISPARITY_MAP
#define SMOOTH_DISPARITY_MAP 0 // 0
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
#define STEREO_CAM_NUMBER 0 //  0 = DelFly Explorer cam   1 = spare camera
#endif

#define STEREO_ALGORITHM 1 // 1 = Dense   0 = Sparse
#define SMALL_IMAGE
//#define LARGE_IMAGE

#ifndef CAPTURE_MODE_SNAPSHOT
#define CAPTURE_MODE_SNAPSHOT 1
#endif

//////////////////////////////////////////////////////
// Define image format
#if !(defined(IMAGE_WIDTH) || defined(IMAGE_HEIGHT))
#define IMAGE_WIDTH 128
#define IMAGE_HEIGHT 96
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

//////////////////////////////////////////////////////
// Stereoboard: camera merging type

#ifndef CAMERA_CPLD_STEREO
#define CAMERA_CPLD_STEREO camera_cpld_stereo_pixmux
#endif


#endif /* MAIN_PARAMETERS_H_ */
