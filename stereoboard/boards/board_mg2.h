#include "multigaze_common.h"

#define BOARD_NUMBER 2
#define DISPARITY_OFFSET_LEFT  2
#define DISPARITY_OFFSET_RIGHT  2
#define DISPARITY_BORDER  84
#define DISPARITY_OFFSET_HORIZONTAL 10
#define RESOLUTION_FACTOR 6

// Following should be removed
#define USE_USART1
#define UsartTx Usart1Tx
#define USART1_BAUD 1000000 // high baudrate necessary for sending images / disparity maps.
#define CAPTURE_MODE_SNAPSHOT 1   // snapshot! Mostly for debugging
