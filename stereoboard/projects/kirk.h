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
#define WINDOW
#define SMALL_IMAGE
#define STEREO_ALGORITHM 1 // 1 = Dense   0 = Sparse
#define DEFAULT_BOARD_FUNCTION SEND_WINDOW

#define DCMI_DOUBLE_BUFFER
//#define CAPTURE_MODE_SNAPSHOT

//////////////////////////////////////////////////////
// The default communication via UsartTx must be connected to a Usart
// Stereoboard bottom = Usart1
// Stereoboard top(cpld) = Usart4
#define USE_USART4
#define UsartTx Usart4Tx
#define UsartRx Usart4Rx
#define UsartCh Usart4Ch
#define USART4_BAUD 115200

#endif /* PROJECT_HEADER_H_ */
