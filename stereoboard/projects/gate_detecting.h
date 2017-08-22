#ifndef PROJECT_HEADER_H_
#define PROJECT_HEADER_H_

/*****************
 * Project parameters
 *****************/

//////////////////////////////////////////////////////
// Settings
#define STEREO_ALGORITHM 0 // 1 = Dense   0 = Sparse
#define DEFAULT_BOARD_FUNCTION SEND_NONE

#define CAMERA_CPLD_STEREO camera_cpld_stereo_pixmux
#define LED_TOGGLE

#define TCM8230_DISABLE_AL
#define TCM_ALC_ESRSPD 0x0C0
#define TCM8230_EXTRA_SATURATION 1

// Gate settings
#define GATE_NSAMPLES 1500
#define GATE_SHAPE DOOR
#define GATE_ROTATE

// uncomment (only) one group of the following to activate the method you want to run

// For color
// #define GATE_METHOD 0
// #define USE_COLOR 1

// For grayscale
// #define GATE_METHOD 1

// For disparity
// #define GATE_METHOD 2
// #define DCMI_MODE DCMI_MODE_4
// #USE_INTEGRAL_IMAGE

// For edge
#define GATE_METHOD 3

// To draw the result of the gate in the image and send the image over uart, uncomment the following
// #define GATE_DETECTION_GRAPHICS 1

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
