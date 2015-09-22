/*
 * project parameters
 *
 *  This file contains the board configuration defaults for this project 
 *
 */

#ifndef PROJECT_HEADER_H_
#define PROJECT_HEADER_H_

/*****************
 * Project parameters
 *****************/
#define DEFAULT_BOARD_FUNCTION SEND_DISPARITY_MAP
#define CAMERA_CPLD_STEREO camera_cpld_stereo_pixmux
#define CAPTURE_MODE_SNAPSHOT 1   // snapshot! Mostly for debugging

//////////////////////////////////////////////////////
// Settings
#define USE_COLOR 0
#define SMOOTH_DISPARITY_MAP 0 // 0
#define SEND_ILLUMINANCE 0 // 0

#define SEND_FILTER 0 // 0
#define COLOR_RATIO 0 // 0
#define MAX_RATIO 10 // 10
#define BRIGHT_WINDOW 0 // 0
#define STEREO_ALGORITHM 1 // 1 = Dense   0 = Sparse
#define SMALL_IMAGE
#define CAPTURE_MODE_SNAPSHOT 1

//////////////////////////////////////////////////////
// Define image format
#define IMAGE_WIDTH 128
#define IMAGE_HEIGHT 96

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
#define USE_USART4
#define UsartTx Usart4Tx
#define UsartRx Usart4Rx
#define UsartCh Usart4Ch
#define USART4_BAUD 1000000

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


#endif /* PROJECT_HEADER_H_ */
