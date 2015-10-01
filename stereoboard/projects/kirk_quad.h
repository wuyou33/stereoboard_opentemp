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
#define SMALL_IMAGE
#define STEREO_ALGORITHM 0 // 1 = Dense   0 = Sparse

#define DEFAULT_BOARD_FUNCTION SEND_WINDOW

//////////////////////////////////////////////////////
// The default communication via UsartTx must be connected to a Usart
// Stereoboard bottom = Usart1
// Stereoboard top(cpld) = Usart4
#define USE_USART4
#define UsartTx Usart4Tx
#define UsartRx Usart4Rx
#define UsartCh Usart4Ch
#define USART4_BAUD 38400

#endif /* PROJECT_HEADER_H_ */
