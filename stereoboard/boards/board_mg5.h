#include "multigaze_common.h"

#define BOARD_NUMBER 5
#define DISPARITY_OFFSET_LEFT  1
#define DISPARITY_OFFSET_RIGHT  1
#define DISPARITY_BORDER  80
#define DISPARITY_OFFSET_HORIZONTAL -6
#define RESOLUTION_FACTOR 6

// Following should be removed
#define USE_USART1
#define UsartTx Usart1Tx
#define USART1_BAUD 1000000 // high baudrate necessary for sending images / disparity maps.
#define CAPTURE_MODE_SNAPSHOT 1   // snapshot! Mostly for debugging
