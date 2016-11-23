/**
 * Project parameters
 *
 * @details
 *
 * - Set up CPLD to read in color images from left camera
 * - Selecting color will additionally disable autowhite balance and apply constant RB gains
 * - A higher saturation is also set to improve color appearance indoors
 * - Baud rate of 1000000 for quick transfer of images over uart4
 * - Toggle led with each frame sent
 * - Can optionally filter image before sending by defining FILTER
 *
 */

#ifndef PROJECT_HEADER_H_
#define PROJECT_HEADER_H_

/*****************
 * Project parameters
 *****************/
#define NEW_MAIN

void init_project(void);
void run_project(void);

#define DEFAULT_BOARD_FUNCTION SEND_NONE

// uncomment line below to filter image before sending it to ground station
//#define FILTER

#define CAMERA_CPLD_STEREO camera_cpld_stereo_left

//////////////////////////////////////////////////////
// Settings
#define USE_COLOR 1
#define TCM8230_EXTRA_SATURATION 1
#define LED_TOGGLE

//////////////////////////////////////////////////////
// The default communication via UsartTx must be connected to a Usart
// Stereoboard bottom = Usart1
// Stereoboard top(cpld) = Usart4
#define USE_USART4
#define UsartTx Usart4Tx
#define UsartRx Usart4Rx
#define UsartCh Usart4Ch
#define USART4_BAUD 1000000

#endif /* PROJECT_HEADER_H_ */
