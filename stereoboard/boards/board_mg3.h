#define BOARD_NUMBER 3
#define DISPARITY_OFFSET_LEFT  1
#define DISPARITY_OFFSET_RIGHT  1
#define DISPARITY_BORDER  64
#define USE_USART1
	#define UsartTx Usart1Tx
	#define USART1_BAUD 1000000 // high baudrate necessary for sending images / disparity maps.
