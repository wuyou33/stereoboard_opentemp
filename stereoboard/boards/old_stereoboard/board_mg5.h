#define BOARD_NUMBER 5

#define DISPARITY_OFFSET_LEFT  -3
#define DISPARITY_OFFSET_RIGHT  -3
#define DISPARITY_BORDER  80
#define USE_USART1
	#define UsartTx Usart1Tx
	#define USART1_BAUD 1000000 // high baudrate necessary for sending images / disparity maps.