#include "multigaze_commmon.h"

#define BOARD_NUMBER 1
#define DISPARITY_OFFSET_LEFT  2
#define DISPARITY_OFFSET_RIGHT  0
#define DISPARITY_BORDER  39
#define USE_USART1
#define UsartTx Usart1Tx
#define USART1_BAUD 1000000 // high baudrate necessary for sending images / disparity maps.

