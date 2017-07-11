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

//////////////////////////////////////////////////////
// Settings
#define STEREO_ALGORITHM 1 // 1 = Dense   0 = Sparse
#define DEFAULT_BOARD_FUNCTION SEND_NONE

//#define USE_COLOR 1
#define CAMERA_CPLD_STEREO camera_cpld_stereo_pixmux

//#define TCM8230_DISABLE_AL
//#define TCM_ALC_ESRSPD 0x0C0
#define TCM8230_EXTRA_SATURATION 1
#define DISPARITY_RANGE 20

//////////////////////////////////////////////////////
// The default communication via UsartTx must be connected to a Usart
// Stereoboard bottom = Usart1
// Stereoboard top(cpld) = Usart4
#define USE_USART4
#define UsartTx Usart4Tx
#define UsartRx Usart4Rx
#define UsartCh Usart4Ch
#define USART4_BAUD 921600

#define NEW_MAIN
extern void init_project(void);
extern void run_project(void);

#endif /* PROJECT_HEADER_H_ */
