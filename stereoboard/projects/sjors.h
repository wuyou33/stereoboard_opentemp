/*
 * project parameters
 *
 *  Description here
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

// Settings
#define LED_TOGGLE

//////////////////////////////////////////////////////
// The default communication via UsartTx must be connected to a Usart
// Stereoboard bottom = Usart1
// Stereoboard top(cpld) = Usart4
#define USE_USART4
#define UsartTx Usart4Tx
#define UsartRx Usart4Rx
#define UsartCh Usart4Ch
#define USART4_BAUD 912600

#endif /* PROJECT_HEADER_H_ */

