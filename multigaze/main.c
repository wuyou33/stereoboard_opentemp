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


#define SIZE_OF_ONE_IMAGE 80
#define DOUBLE_IMAGE SIZE_OF_ONE_IMAGE*2
//#define MATRIX_WIDTH_BINS 5
//#define MATRIX_HEIGHT_BINS 5
#define STEREO_CAMERAS_COUNT 6

/* Private functions ---------------------------------------------------------*/


void init_timer2()
{
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);
  TIM_TimeBaseInitTypeDef TIM_InitStruct;
  TIM_InitStruct.TIM_Prescaler = 42000 - 1;                // This will configure the clock to 2 kHz
  TIM_InitStruct.TIM_CounterMode = TIM_CounterMode_Up;     // Count-up timer mode
  TIM_InitStruct.TIM_Period = 20000 - 1;                   // 10 seconds
  TIM_InitStruct.TIM_ClockDivision = TIM_CKD_DIV1;         // Divide clock by 1
  TIM_InitStruct.TIM_RepetitionCounter = 0;                // Set to 0, not used
  TIM_TimeBaseInit(TIM2, &TIM_InitStruct);
  TIM_Cmd(TIM2, ENABLE);
}

/**
 * Function searches in an array for the 255-0-0-171 bits and returns the location of the 255 bit.
 * When it does not find this it returns -1
 */
int search_start_position(int startPosition, int size_of_one_image, uint8_t* raw){
	// Search for the startposition which is indicated by the 255-0-0-171 bytes

    int i;
    for (i=startPosition; i < size_of_one_image-1; i++){
        if ((raw[i] == 255) && (raw[i + 1] == 0) && (raw[i + 2] == 0)){
            if (raw[i + 3] == 171){
                return i;
            }
        }
    }
    return -1;
}
void send_matrix_part(uint8_t *response, uint8_t boardnumber, int matrixLine, int pixelsPerLine)
{

	int startPos = search_start_position(0,SIZE_OF_ONE_IMAGE,response);
	if (startPos >=0){
		int arrayIndex = 0;

		int i;
		int lineReading =0;
		for (i = startPos; i < SIZE_OF_ONE_IMAGE + startPos;i++){
			if ((response[i] == 255) && (response[i + 1] == 0) && (response[i + 2] == 0)){
				if (response[i + 3] == 128){ // Start Of Line
					if(lineReading==matrixLine){

						int startOfBuf = i+4;
						int endOfBuf = (i +4+ MATRIX_WIDTH_BINS);
						int indexInBuffer;
						for(indexInBuffer = startOfBuf; indexInBuffer < endOfBuf; indexInBuffer++){
							Usart1Tx(&response[indexInBuffer],1);
						}

					}
					lineReading++;
				}
			}
		}
	}
}


/**
  * @brief  Main program
  * @param  None
  * @retval None
  */
int main(void)
{
  /*
    At this stage the microcontroller clock setting is already configured,
    this is done through SystemInit() function which is called from startup
    file (startup_stm32f4xx.s) before to branch to application main.
    To reconfigure the default setting of SystemInit() function, refer to
    system_stm32f4xx.c file
  */

  // Initialize the LED
  led_init();
  led_set();

#ifdef TUNNEL_NONE
  // Initialize the serial communication (before the camera so we can print status)
  usart_init();

#else
  // Camera interface init
  tunnel_init();
#endif


  // Keep an index for each cameras buffer so we know where to append all pixels
  uint8_t locationsBufferedMatrixes[STEREO_CAMERAS_COUNT];
  memset(locationsBufferedMatrixes,0,sizeof(locationsBufferedMatrixes));

  // For each camera we need to remember what pixels we received.
  uint8_t receivedMatrixBuffer[STEREO_CAMERAS_COUNT][DOUBLE_IMAGE];

  int offset_buffer_safety=4; // We do not want to overflow the buffer

	uint8_t code[4];
	code[0] = 0xff;
	code[1] = 0x00;
	code[2] = 0x00;


  while (1) {
#ifdef TUNNEL_NONE
	  uint8_t c =  ' ';
    // TODO can we create an even more generic system that says what inputs
    // have characters and read from those inputs?
    if (Cam1Ch() && locationsBufferedMatrixes[0] < DOUBLE_IMAGE)
    {
      c = Cam1Rx();
      receivedMatrixBuffer[0][locationsBufferedMatrixes[0]++]=c;

    }
    if (Cam2Ch() && locationsBufferedMatrixes[1] <DOUBLE_IMAGE)
    {
      c = Cam2Rx();
      receivedMatrixBuffer[1][locationsBufferedMatrixes[1]++]=c;
    }
    if (Cam3Ch() && locationsBufferedMatrixes[2] < DOUBLE_IMAGE)
    {
      c = Cam3Rx();
      receivedMatrixBuffer[2][locationsBufferedMatrixes[2]++]=c;
    }
    if (Cam4Ch() && locationsBufferedMatrixes[3] < DOUBLE_IMAGE)
    {
      c = Cam4Rx();
      receivedMatrixBuffer[3][locationsBufferedMatrixes[3]++]=c;
    }
    if (Cam5Ch() && locationsBufferedMatrixes[4] < DOUBLE_IMAGE)
    {
      c = Cam5Rx();
      receivedMatrixBuffer[4][locationsBufferedMatrixes[4]++]=c;
    }
    if (Cam6Ch() && locationsBufferedMatrixes[5] < DOUBLE_IMAGE)
    {
      c = Cam6Rx();
      receivedMatrixBuffer[5][locationsBufferedMatrixes[5]++]=c;
    }

    // Check how many of the buffers are not full yet

    int camerasToComplete=0;
    int locationInVector;

    for(locationInVector=0; locationInVector < STEREO_CAMERAS_COUNT; locationInVector++)
    {
    	c = locationInVector;

    	if(locationsBufferedMatrixes[locationInVector] < (DOUBLE_IMAGE-offset_buffer_safety))
    	{
    		camerasToComplete++;
    	}
    }

    // If there are no cameras yet who need to receive data...
    // ... send this data through the serial port
    if(camerasToComplete==0)
    {
    	led_toggle();
    	// Reset the index in each of the cameras buffer by setting it to zero, now they can all start again
		memset(locationsBufferedMatrixes,0,sizeof locationsBufferedMatrixes);


		code[3] = 0xAB; // 171
		while (Usart1Tx(code, 4) == 0){

		}

		int cameraboard;
		int matrixLine;

		for(matrixLine=0; matrixLine < MATRIX_HEIGHT_BINS; matrixLine++){
			code[3] = 128;
			while (Usart1Tx(code, 4) == 0){

			}

			for (cameraboard=0;cameraboard <STEREO_CAMERAS_COUNT; cameraboard++)
			{
				send_matrix_part(receivedMatrixBuffer[cameraboard],cameraboard,matrixLine,MATRIX_WIDTH_BINS);
			}
			code[3] = 218; // 171
			while (Usart1Tx(code, 4) == 0){

			}
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

