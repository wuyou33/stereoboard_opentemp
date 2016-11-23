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
#include PROJECT_FILE

#ifndef BOARD_FILE
#define BOARD_FILE "boards/board_default.h"
#endif
#include BOARD_FILE

#ifndef DEFAULT_BOARD_FUNCTION
#define DEFAULT_BOARD_FUNCTION SEND_NONE
#endif

#ifndef RESOLUTION_FACTOR
#define RESOLUTION_FACTOR 6
#endif

// compute full image size
#define FULL_IMAGE_SIZE  (IMAGE_WIDTH*IMAGE_HEIGHT*BYTES_PER_PIXEL)
#if (FULL_IMAGE_SIZE >= (120*1024))
#error "Config error: Image does not fit im RAM"
#endif

//////////////////////////////////////////////////////
// Settings
#ifndef USE_MONOCAM
#define USE_MONOCAM 0
#endif

#ifndef ODOMETRY
#define ODOMETRY 0
#endif

#ifndef STEREO_BUF_SIZE
#define STEREO_BUF_SIZE 4*64
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
#define SMALL_IMAGE
#endif

#if defined(DCMI_DOUBLE_BUFFER) && defined(LARGE_IMAGE)
#error "Cannot define LARGE_IMAGE and DCMI_DOUBLE_BUFFER"
#endif

#if !(defined(IMAGE_WIDTH) || defined(IMAGE_HEIGHT))
#if defined(SMALL_IMAGE) && defined(LARGE_IMAGE)
#error "Cannot define SMALL_IMAGE and LARGE_IMAGE"
#endif
#ifdef SMALL_IMAGE
#define IMAGE_WIDTH 128
#define IMAGE_HEIGHT 96
#endif
#ifdef LARGE_IMAGE
#define IMAGE_WIDTH 640
#define IMAGE_HEIGHT 60
#endif
#endif

#if defined CAPTURE_MODE_SNAPSHOT && defined(DCMI_DOUBLE_BUFFER)
#error "Cannot define DCMI_DOUBLE_BUFFER with CAPTURE_MODE_SNAPSHOT"
#endif

#if !(defined(DCMI_DOUBLE_BUFFER)) && !(defined(CAPTURE_MODE_SNAPSHOT))
//#warning "Both DCMI_DOUBLE_BUFFER and CAPTURE_MODE_SNAPSHOT not defined.This may result in changes to the current_image_buffer while using it."
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

#ifdef DCMI_TEN_BITS
#define BYTES_PER_PIXEL 4
#else
#define BYTES_PER_PIXEL 2
#endif

#define FULL_IMAGE_SIZE  (IMAGE_WIDTH*IMAGE_HEIGHT*BYTES_PER_PIXEL)
#if (FULL_IMAGE_SIZE >= (120*1024))
#error "Config error: Image does not fit im RAM"
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
// Set defaults for stereo
#ifndef DISPARITY_OFFSET_LEFT
#define DISPARITY_OFFSET_LEFT 0
#endif

#ifndef DISPARITY_OFFSET_RIGHT
#define DISPARITY_OFFSET_RIGHT 0
#endif

#ifndef DISPARITY_OFFSET_HORIZONTAL
#define DISPARITY_OFFSET_HORIZONTAL 0
#endif

#ifndef DISPARITY_BORDER
#define DISPARITY_BORDER 0
#endif

#ifndef RESOLUTION_FACTOR
#define RESOLUTION_FACTOR 6 // default for stereocam
#endif


#endif /* MAIN_PARAMETERS_H_ */
