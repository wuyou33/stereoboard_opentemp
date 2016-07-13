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
#define DEFAULT_BOARD_FUNCTION SEND_EDGEFLOW
#define CAMERA_CPLD_STEREO camera_cpld_stereo_pixmux

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
#define LED_TOGGLE

// #define CAPTURE_MODE_SNAPSHOT    // will transfer entire image in one burst
// #deinfe DCMI_DOUBLE_BUFFER       // two image buffers used, while one is used the other is filled.

//////////////////////////////////////////////////////
// Define image format
#if (DCMI_TEN_BITS == 1)
#define BYTES_PER_PIXEL 4
#else
#define BYTES_PER_PIXEL 2
#endif

//////////////////////////////////////////////////////
// The default communication via UsartTx must be connected to a Usart
// Stereoboard bottom = Usart1
// Stereoboard top(cpld) = Usart4
#define USE_USART4
#define UsartTx Usart4Tx
#define UsartRx Usart4Rx
#define UsartCh Usart4Ch
#define USART4_BAUD 1000000;//115200

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
