/**
 * Project parameters
 *
 * @details
 *
 * - Send pixmuxed stereo-image pair
 * - use pythontools stereocam_calibrate.py to view and adjust
 * - save the calibration values into a BOARD file
 *
 */

#ifndef PROJECT_HEADER_H_
#define PROJECT_HEADER_H_

/*****************
 * Project parameters
 *****************/
#define DEFAULT_BOARD_FUNCTION SEND_IMAGE

//////////////////////////////////////////////////////
// Settings

#define DCMI_MODE DCMI_MODE_4

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
#define USART4_BAUD 921600

#endif /* PROJECT_HEADER_H_ */
