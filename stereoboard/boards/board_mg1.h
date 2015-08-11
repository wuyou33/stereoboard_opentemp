#include "multigaze_commmon.h"

#define BOARD_NUMBER 1
#define DISPARITY_OFFSET_LEFT  2
#define DISPARITY_OFFSET_RIGHT  2
#define DISPARITY_BORDER  97
#define USE_USART1
#define UsartTx Usart1Tx
#define USART1_BAUD 1000000 // high baudrate necessary for sending images / disparity maps.

#define CAPTURE_MODE_SNAPSHOT 1   // snapshot! Mostly for debugging
