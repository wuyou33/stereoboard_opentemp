/**
 ******************************************************************************
 * @file    main.c
 * @author  C. De Wagter
 * @author  R. Meertens
 * @version V1.0.0
 * @date    2013
 * @brief   Main program body
 ******************************************************************************
 */

/* Includes ------------------------------------------------------------------*/
#include "led.h"
#include "tunnel.h"
#include "usart.h"
#include "stereo_utils.h"
#include "stm32f4xx_conf.h"
#include "stm32f4xx.h"
#include "stereoboard_parameters.h"
#include "stereo_protocol.h"
//#define SIZE_OF_ONE_IMAGE 80
//#define DOUBLE_IMAGE SIZE_OF_ONE_IMAGE*2

#define STEREO_CAMERAS_COUNT 6

MsgProperties msgProperties[STEREO_CAMERAS_COUNT];
#define STEREO_BUF_SIZE 1024                     // size of circular buffer

typedef struct {
	uint8_t len;
	uint8_t height;
	uint8_t *data;
	uint8_t fresh;
} uint8matrix;

uint8_t ser_read_buf[STEREO_CAMERAS_COUNT][STEREO_BUF_SIZE]; // circular buffer for incoming data
uint16_t insert_loc[STEREO_CAMERAS_COUNT];
uint16_t extract_loc[STEREO_CAMERAS_COUNT];
uint16_t msg_start[STEREO_CAMERAS_COUNT];
uint8_t msg_buf[STEREO_CAMERAS_COUNT][STEREO_BUF_SIZE];     // define local data
uint8matrix stereocam_datas[STEREO_CAMERAS_COUNT];

/* Private functions ---------------------------------------------------------*/

void init_timer2() {
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);
	TIM_TimeBaseInitTypeDef TIM_InitStruct;
	TIM_InitStruct.TIM_Prescaler = 42000 - 1; // This will configure the clock to 2 kHz
	TIM_InitStruct.TIM_CounterMode = TIM_CounterMode_Up;  // Count-up timer mode
	TIM_InitStruct.TIM_Period = 20000 - 1;                   // 10 seconds
	TIM_InitStruct.TIM_ClockDivision = TIM_CKD_DIV1;        // Divide clock by 1
	TIM_InitStruct.TIM_RepetitionCounter = 0;              // Set to 0, not used
	TIM_TimeBaseInit(TIM2, &TIM_InitStruct);
	TIM_Cmd(TIM2, ENABLE);
}

/**
 * Function searches in an array for the 255-0-0-171 bits and returns the location of the 255 bit.
 * When it does not find this it returns -1
 */
int search_start_position(int startPosition, int size_of_one_image,
		uint8_t* raw) {
	// Search for the startposition which is indicated by the 255-0-0-171 bytes

	int i;
	for (i = startPosition; i < size_of_one_image - 1; i++) {
		if ((raw[i] == 255) && (raw[i + 1] == 0) && (raw[i + 2] == 0)) {
			if (raw[i + 3] == 171) {
				return i;
			}
		}
	}
	return -1;
}
void send_matrix_part(uint8_t *response, uint8_t length) {
//	int indexInBuffer=0;
//	for (indexInBuffer = startOfBuf;
//			indexInBuffer < endOfBuf; indexInBuffer++) {
//		if (response[indexInBuffer] > CLOSE_BOUNDARY) {
//			//led_set();
//		}
//
//		Usart1Tx(&response[indexInBuffer], 1);
//	}
/*
	int startPos = 0;
	if (startPos >= 0) {
		int arrayIndex = 0;

		int i;
		int lineReading = 0;
		for (i = startPos; i < SIZE_OF_ONE_IMAGE + startPos; i++) {
			if ((response[i] == 255) && (response[i + 1] == 0)
					&& (response[i + 2] == 0)) {
				if (response[i + 3] == 128) { // Start Of Line
					if (lineReading == matrixLine) {

						int startOfBuf = i + 4;
						int endOfBuf = (i + 4 + MATRIX_WIDTH_BINS);
						int indexInBuffer;
						for (indexInBuffer = startOfBuf;
								indexInBuffer < endOfBuf; indexInBuffer++) {
							if (response[indexInBuffer] > CLOSE_BOUNDARY) {
								//led_set();
							}

							Usart1Tx(&response[indexInBuffer], 1);
						}

					}
					lineReading++;
				}
			}
		}
	}*/
}

int isEndOfImage(uint8_t *stack) {
	if (stack[0] == 255 && (stack[1] == 0) && (stack[2] == 0)
			&& stack[3] == 171) {
		return 1;
	}
	return 0;
}

/**
 * @brief  Main program
 * @param  None
 * @retval None
 */
int main(void) {
	/*
	 At this stage the microcontroller clock setting is already configured,
	 this is done through SystemInit() function which is called from startup
	 file (startup_stm32f4xx.s) before to branch to application main.
	 To reconfigure the default setting of SystemInit() function, refer to
	 system_stm32f4xx.c file
	 */

	// Initialize the LED
	led_init();
	//led_set();
	int indexStereocams = 0;
	for (indexStereocams = 0; indexStereocams < STEREO_CAMERAS_COUNT;
			indexStereocams++) {
		// initialize local variables
		msgProperties[indexStereocams] = (MsgProperties ) { 0, 0, 0 };
		insert_loc[indexStereocams] = 0;
		extract_loc[indexStereocams] = 0;
		msg_start[indexStereocams] = 0;
		stereocam_datas[indexStereocams].fresh = 0;
		stereocam_datas[indexStereocams].data = msg_buf[indexStereocams];

	}
	int currentCamera=0;

#ifdef TUNNEL_NONE
	// Initialize the serial communication (before the camera so we can print status)
	usart_init();

#else
	// Camera interface init
	tunnel_init();
#endif

	// Keep an index for each cameras buffer so we know where to append all pixels
	uint8_t locationsBufferedMatrixes[STEREO_CAMERAS_COUNT];
	memset(locationsBufferedMatrixes, 0, sizeof(locationsBufferedMatrixes));

	int SIZE_OF_ONE_IMAGE = (MATRIX_WIDTH_BINS + 8) * MATRIX_HEIGHT_BINS + 8;
	int DOUBLE_IMAGE = 2 * SIZE_OF_ONE_IMAGE;

	// For each camera we need to remember what pixels we received.
	uint8_t receivedMatrixBuffer[STEREO_CAMERAS_COUNT][DOUBLE_IMAGE];
	int slidingPointer[STEREO_CAMERAS_COUNT];

	int offset_buffer_safety = 4; // We do not want to overflow the buffer

	uint8_t code[4];
	code[0] = 0xff;
	code[1] = 0x00;
	code[2] = 0x00;

	uint8_t lastReceivedStack[STEREO_CAMERAS_COUNT][4];
	uint8_t camerasReady[STEREO_CAMERAS_COUNT];

	int locationInStack;
	for (locationInStack = 0; locationInStack < STEREO_CAMERAS_COUNT;
			locationInStack++) {
		camerasReady[locationInStack] = 0;
		stereocam_datas[locationInStack].fresh=0;
	}

	int currentValueSending = 0;
	while (1) {
#ifdef TUNNEL_NONE
		led_toggle();

		uint8_t c = ' ';
		// TODO can we create an even more generic system that says what inputs
		// have characters and read from those inputs?

		if (Cam1Ch() && stereocam_datas[0].fresh==0)
		{
			currentCamera=0;
			c = Cam1Rx();

			if (handleStereoPackage(c, STEREO_BUF_SIZE, &insert_loc[currentCamera], &extract_loc[currentCamera], &msg_start[currentCamera], msg_buf[currentCamera], ser_read_buf[currentCamera],
							&stereocam_datas[currentCamera].fresh, &stereocam_datas[currentCamera].len,&stereocam_datas[currentCamera].height)) {
				camerasReady[currentCamera]=1;
			}

		}

		if (Cam2Ch() && stereocam_datas[1].fresh==0)
		{

			currentCamera=1;
			c = Cam2Rx();
			if (handleStereoPackage(c, STEREO_BUF_SIZE, &insert_loc[currentCamera], &extract_loc[currentCamera], &msg_start[currentCamera], msg_buf[currentCamera], ser_read_buf[currentCamera],
										&stereocam_datas[currentCamera].fresh, &stereocam_datas[currentCamera].len,&stereocam_datas[currentCamera].height)) {
							camerasReady[currentCamera]=1;
						}
		}
		if (Cam3Ch()&& stereocam_datas[2].fresh==0)
		{

			currentCamera=2;
			c = Cam3Rx();
			if (handleStereoPackage(c, STEREO_BUF_SIZE, &insert_loc[currentCamera], &extract_loc[currentCamera], &msg_start[currentCamera], msg_buf[currentCamera], ser_read_buf[currentCamera],
												&stereocam_datas[currentCamera].fresh, &stereocam_datas[currentCamera].len,&stereocam_datas[currentCamera].height)) {
									camerasReady[currentCamera]=1;
								}
		}
		if (Cam4Ch()&& stereocam_datas[3].fresh==0)
		{

			currentCamera=3;
			c = Cam4Rx();

			if (handleStereoPackage(c, STEREO_BUF_SIZE, &insert_loc[currentCamera], &extract_loc[currentCamera], &msg_start[currentCamera], msg_buf[currentCamera], ser_read_buf[currentCamera],
												&stereocam_datas[currentCamera].fresh, &stereocam_datas[currentCamera].len,&stereocam_datas[currentCamera].height)) {
									camerasReady[currentCamera]=1;
								}
		}
		if (Cam5Ch() && stereocam_datas[4].fresh==0)
		{

			currentCamera=4;
			c = Cam5Rx();
			if (handleStereoPackage(c, STEREO_BUF_SIZE, &insert_loc[currentCamera], &extract_loc[currentCamera], &msg_start[currentCamera], msg_buf[currentCamera], ser_read_buf[currentCamera],
												&stereocam_datas[currentCamera].fresh, &stereocam_datas[currentCamera].len,&stereocam_datas[currentCamera].height)) {
									camerasReady[currentCamera]=1;
								}
		}
		if (Cam6Ch() && stereocam_datas[5].fresh==0)
		{

			currentCamera=5;
			c = Cam6Rx();
			if (handleStereoPackage(c, STEREO_BUF_SIZE, &insert_loc[currentCamera], &extract_loc[currentCamera], &msg_start[currentCamera], msg_buf[currentCamera], ser_read_buf[currentCamera],
												&stereocam_datas[currentCamera].fresh, &stereocam_datas[currentCamera].len,&stereocam_datas[currentCamera].height)) {
									camerasReady[currentCamera]=1;
								}

		}

		// Check how many of the buffers are not full yet

		int camerasToComplete=0;
		int locationInVector;

		for(locationInVector=0; locationInVector < STEREO_CAMERAS_COUNT; locationInVector++)
		{
			c = locationInVector;

			if(stereocam_datas[locationInVector].fresh==0)
			{
				camerasToComplete++;
			}
		}

		// If there are no cameras yet who need to receive data...
		// ... send this data through the serial port
		if(camerasToComplete<3)
		{
			led_toggle();
			//led_clear();

			for(currentCamera=0; currentCamera < STEREO_CAMERAS_COUNT; currentCamera++)
			{
				if (stereocam_datas[currentCamera].fresh)
				{
					stereocam_datas[currentCamera].fresh=0;
				}
			}

			code[3] = 175;
			while (Usart1Tx(code, 4) == 0) {

			}

			int cameraboard;
			int matrixLine;

			for(matrixLine=0; matrixLine < stereocam_datas[0].height; matrixLine++) {
				code[3] = 128;
				while (Usart1Tx(code, 4) == 0) {

				}

				for (cameraboard=0;cameraboard <STEREO_CAMERAS_COUNT; cameraboard++)
				{

					/*while (Usart1Tx(stereocam_datas[cameraboard].data, 	stereocam_datas[cameraboard].len/6) == 0) {

					}*/
					uint8_t toSend[10];
					toSend[0]=1;
					toSend[1]=2;
					toSend[2]=3;
					toSend[3]=4;
					toSend[4]=5;
					uint8_t lengthLine = stereocam_datas[cameraboard].len/stereocam_datas[0].height;
					toSend[0]=lengthLine;
					Usart1Tx(stereocam_datas[cameraboard].data+(matrixLine*lengthLine),lengthLine);

					//send_matrix_part(stereocam_datas[cameraboard].data,cameraboard,matrixLine,MATRIX_WIDTH_BINS,SIZE_OF_ONE_IMAGE);
				}
				code[3] = 218;
				while (Usart1Tx(code, 4) == 0) {

				}
			}
			code[3] = 0xAB; // 171
			while (Usart1Tx(code, 4) == 0) {

			}
		}
#else
		tunnel_run();
#endif

	}
}

#ifdef  USE_FULL_ASSERT

/**
 * @brief  Reports the name of the source file and the source line number
 *         where the assert_param error has occurred.
 * @param  file: pointer to the source file name
 * @param  line: assert_param error line source number
 * @retval None
 */
void assert_failed(uint8_t *file, uint32_t line)
{
	/* User can add his own implementation to report the file name and line number,
	 ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */

	/* Infinite loop */
	while (1) {
	}
}
#endif

/**
 * @}
 */

