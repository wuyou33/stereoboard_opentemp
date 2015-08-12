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
#include "../common/led.h"
#include "tunnel.h"
#include "usart.h"
#include "../common/utils.h"
#include "stm32f4xx_conf.h"
#include "stereoboard_parameters.h"

//#define SIZE_OF_ONE_IMAGE 80
//#define DOUBLE_IMAGE SIZE_OF_ONE_IMAGE*2

#define STEREO_CAMERAS_COUNT 6

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
void send_matrix_part(uint8_t *response, uint8_t boardnumber, int matrixLine,
		int pixelsPerLine, int SIZE_OF_ONE_IMAGE) {

	int startPos = 0;//currentReadLocation-SIZE_OF_ONE_IMAGE;//search_start_position(0, SIZE_OF_ONE_IMAGE, response);
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
							if(response[indexInBuffer]>CLOSE_BOUNDARY){
								//led_set();
							}

							Usart1Tx(&response[indexInBuffer], 1);
						}

					}
					lineReading++;
				}
			}
		}
	}
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
	}
	int currentCamera = 0;
	int currentValueSending=0;
	while (1) {
#ifdef TUNNEL_NONE
/*
		uint8_t c = ' ';

		while(Usart1Ch()){
			c=Usart1Rx();
			code[0] = c;
			while (Usart1Tx(code, 1) == 0) {

			}
		}*/

		/*
		if(c>0){
			led_set();
		}
		else{
			led_clear();
		}*/

		led_toggle();
		/*
		code[0] = currentValueSending;
		currentValueSending=currentValueSending+1;
		currentValueSending=currentValueSending%100;
		while (Usart1Tx(code, 1) == 0) {

		}*/

		uint8_t c = ' ';
		// TODO can we create an even more generic system that says what inputs
		// have characters and read from those inputs?

		if (Cam1Ch() && camerasReady[0]==0)
		{
			currentCamera=0;
			c = Cam1Rx();

			receivedMatrixBuffer[currentCamera][locationsBufferedMatrixes[currentCamera]++]=c;

			for (locationInStack=0; locationInStack<3; locationInStack++) {
				lastReceivedStack[currentCamera][locationInStack]=lastReceivedStack[currentCamera][locationInStack+1];
			}
			lastReceivedStack[currentCamera][3]=c;
			if (isEndOfImage(lastReceivedStack[currentCamera])>0)
			{
				camerasReady[currentCamera]=1;
			}

		}
		if (Cam2Ch() && camerasReady[1]==0)
		{

			currentCamera=1;
			c = Cam2Rx();
			receivedMatrixBuffer[currentCamera][locationsBufferedMatrixes[currentCamera]++]=c;

			for (locationInStack=0; locationInStack<3; locationInStack++) {
				lastReceivedStack[currentCamera][locationInStack]=lastReceivedStack[currentCamera][locationInStack+1];
			}
			lastReceivedStack[currentCamera][3]=c;
			if (isEndOfImage(lastReceivedStack[currentCamera])>0)
			{
				camerasReady[currentCamera]=1;
			}
		}
		if (Cam3Ch()&& camerasReady[2]==0)
		{

			currentCamera=2;
			c = Cam3Rx();
			receivedMatrixBuffer[currentCamera][locationsBufferedMatrixes[currentCamera]++]=c;

			for (locationInStack=0; locationInStack<3; locationInStack++) {
				lastReceivedStack[currentCamera][locationInStack]=lastReceivedStack[currentCamera][locationInStack+1];
			}
			lastReceivedStack[currentCamera][3]=c;
			if (isEndOfImage(lastReceivedStack[currentCamera])>0)
			{
				camerasReady[currentCamera]=1;
			}
		}
		if (Cam4Ch()&& camerasReady[3]==0)
		{

			currentCamera=3;
			c = Cam4Rx();
			receivedMatrixBuffer[currentCamera][locationsBufferedMatrixes[currentCamera]++]=c;

			for (locationInStack=0; locationInStack<3; locationInStack++) {
				lastReceivedStack[currentCamera][locationInStack]=lastReceivedStack[currentCamera][locationInStack+1];
			}
			lastReceivedStack[currentCamera][3]=c;
			if (isEndOfImage(lastReceivedStack[currentCamera])>0)
			{
				camerasReady[currentCamera]=1;
			}
		}
		if (Cam5Ch() && camerasReady[4]==0)
		{

			currentCamera=4;
			c = Cam5Rx();
			receivedMatrixBuffer[currentCamera][locationsBufferedMatrixes[currentCamera]++]=c;

			for (locationInStack=0; locationInStack<3; locationInStack++) {
				lastReceivedStack[currentCamera][locationInStack]=lastReceivedStack[currentCamera][locationInStack+1];
			}
			lastReceivedStack[currentCamera][3]=c;
			if (isEndOfImage(lastReceivedStack[currentCamera])>0)
			{
				camerasReady[currentCamera]=1;
			}
		}
		if (Cam6Ch() && camerasReady[5]==0)
		{

			currentCamera=5;
			c = Cam6Rx();
			receivedMatrixBuffer[currentCamera][locationsBufferedMatrixes[currentCamera]++]=c;

			for (locationInStack=0; locationInStack<3; locationInStack++) {
				lastReceivedStack[currentCamera][locationInStack]=lastReceivedStack[currentCamera][locationInStack+1];
			}
			lastReceivedStack[currentCamera][3]=c;
			if (isEndOfImage(lastReceivedStack[currentCamera])>0)
			{
				camerasReady[currentCamera]=1;
			}
		}



		// Check how many of the buffers are not full yet

		int camerasToComplete=0;
		int locationInVector;

		for(locationInVector=0; locationInVector < STEREO_CAMERAS_COUNT; locationInVector++)
		{
			c = locationInVector;

			if(camerasReady[locationInVector]==0)
			{
				camerasToComplete++;
			}
		}

		// If there are no cameras yet who need to receive data...
		// ... send this data through the serial port
		if(camerasToComplete<3)
		{
			//led_toggle();
			//led_clear();

			for(currentCamera=0; currentCamera < STEREO_CAMERAS_COUNT; currentCamera++)
			{
				if (isEndOfImage(lastReceivedStack[currentCamera])>0)
				{
					camerasReady[currentCamera]=0;
					locationsBufferedMatrixes[currentCamera]=0;
				}
			}

		//	memset(camerasReady,0,STEREO_CAMERAS_COUNT);
		//	memset(locationsBufferedMatrixes,0,STEREO_CAMERAS_COUNT);
			code[3] = 175;
			while (Usart1Tx(code, 4) == 0) {

			}

			int cameraboard;
			int matrixLine;

			for(matrixLine=0; matrixLine < MATRIX_HEIGHT_BINS; matrixLine++) {
				code[3] = 128;
				while (Usart1Tx(code, 4) == 0) {

				}

				for (cameraboard=0;cameraboard <STEREO_CAMERAS_COUNT; cameraboard++)
				{
					send_matrix_part(receivedMatrixBuffer[cameraboard],cameraboard,matrixLine,MATRIX_WIDTH_BINS,SIZE_OF_ONE_IMAGE);
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

