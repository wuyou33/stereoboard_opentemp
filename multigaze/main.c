/**
  ******************************************************************************
  * @file    main.c
  * @author  C. De Wagter
  * @version V1.0.0
  * @date    2013
  * @brief   Main program body
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "led.h"
#include "tunnel.h"
#include "usart.h"
#include "utils.h"
#include "stm32f4xx_conf.h"

#define SIZE_OF_ONE_IMAGE 50
#define DOUBLE_IMAGE SIZE_OF_ONE_IMAGE*2
#define MATRIX_WIDTH 4
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

int search_start_position(int startPosition, int size_of_one_image, uint8_t* raw){
    int sync=0;
    // Search for the startposition
    int i;
    for (i=startPosition; i < size_of_one_image-1; i++){
//    	Usart1Tx(&raw[i],1);
        if ((raw[i] == 255) && (raw[i + 1] == 0) && (raw[i + 2] == 0)){
            if (raw[i + 3] == 171){
                sync = i;
//                Usart1Tx(&raw[i+1],1);
//                Usart1Tx(&raw[i+2],1);
//                Usart1Tx(&raw[i+3],1);
                break;
            }
        }
    }
    return sync;
}
void send_matrix_part(uint8_t *response, uint8_t boardnumber, int matrixLine, int pixelsPerLine)
{

	int startPos = search_start_position(0,SIZE_OF_ONE_IMAGE,response);
	int arrayIndex = 0;

	int i;
	int lineReading =0;
	for (i = startPos; i < SIZE_OF_ONE_IMAGE + startPos;i++){
		if ((response[i] == 255) && (response[i + 1] == 0) && (response[i + 2] == 0)){
			if (response[i + 3] == 128){ // Start Of Line
				if(lineReading==matrixLine){

					int startOfBuf = i+4;
					int endOfBuf = (i +4+ MATRIX_WIDTH);
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

  // Print welcome message
  //char comm_buff[128] = " --- Stereo Camera --- \n\r";
  //usart_tx_ringbuffer_push((uint8_t *)&comm_buff, strlen(comm_buff));
  int verticalLines = 4;
  int pixelsPerLinePerMatrix = 4;
  uint8_t locationsBufferedMatrixes[STEREO_CAMERAS_COUNT];
  uint8_t locationToSet;
  for(locationToSet=0; locationToSet < STEREO_CAMERAS_COUNT; locationToSet++)
  {
	  locationsBufferedMatrixes[locationToSet]=0;
  }
#define OLD
#ifdef OLD
  int spot1=0;
  int spot2=0;
  int spot3=0;
  int spot4=0;
  int spot5=0;
  int spot6=0;

  uint8_t response1[DOUBLE_IMAGE];
  uint8_t response2[DOUBLE_IMAGE];
  uint8_t response3[DOUBLE_IMAGE];
  uint8_t response4[DOUBLE_IMAGE];
  uint8_t response5[DOUBLE_IMAGE];
  uint8_t response6[DOUBLE_IMAGE];
#endif
  uint8_t receivedMatrixBuffer[STEREO_CAMERAS_COUNT][DOUBLE_IMAGE];



  while (1) {
#ifdef TUNNEL_NONE
    uint8_t c = ' ';
    if (Cam1Ch() && spot1 < DOUBLE_IMAGE)
    {
      c = Cam1Rx();
      //receivedMatrixBuffer[1][spot1++]=c;
	  response1[spot1++]=c;
    }
    if (Cam2Ch() && spot2 <DOUBLE_IMAGE)
    {
      c = Cam2Rx();
	  response2[spot2++]=c;
    }
    if (Cam3Ch() && spot3 < DOUBLE_IMAGE)
    {
      c = Cam3Rx();
      response3[spot3++]=c;
    }
    if (Cam4Ch() && spot4 < DOUBLE_IMAGE)
    {
      c = Cam4Rx();
      response4[spot4++]=c;
    }
    if (Cam5Ch() && spot5 < DOUBLE_IMAGE)
    {
      c = Cam5Rx();
      response5[spot5++]=c;
    }
    if (Cam6Ch() && spot6 < DOUBLE_IMAGE)
    {
      c = Cam6Rx();
      response6[spot6++]=c;
    }

    int offset_buffer_safety=4;
    if(spot1 >= DOUBLE_IMAGE-offset_buffer_safety && spot2 >= DOUBLE_IMAGE-offset_buffer_safety &&  spot3 >= DOUBLE_IMAGE-offset_buffer_safety &&  spot4 >= DOUBLE_IMAGE-offset_buffer_safety &&  spot5 >= DOUBLE_IMAGE-offset_buffer_safety &&  spot6 >= DOUBLE_IMAGE-offset_buffer_safety )
    {
	   uint8_t c = 255;
		Usart1Tx(&c,1);
		c=0;
		Usart1Tx(&c,1);
		c=0;
		Usart1Tx(&c,1);
		c=171;
		Usart1Tx(&c,1);

		led_toggle();
		spot1=0;
		spot2=0;
		spot3=0;
		spot4=0;
		spot5=0;
		spot6=0;


		int cameraboard;
		int matrixLine;

		for(matrixLine=0; matrixLine < verticalLines; matrixLine++){
			uint8_t c= 255;
			Usart1Tx(&c,1);
			c= 0;
			Usart1Tx(&c,1);
			c= 0;
			Usart1Tx(&c,1);
			c= 128;
			Usart1Tx(&c,1);

			for (cameraboard=1;cameraboard <=6; cameraboard++)
			{
				if (cameraboard==1){
					   send_matrix_part(response1,cameraboard,matrixLine,pixelsPerLinePerMatrix);
				}
				else if(cameraboard==2){
					send_matrix_part(response2,cameraboard,matrixLine,pixelsPerLinePerMatrix);
				}
				else if(cameraboard==3){
					send_matrix_part(response3,cameraboard,matrixLine,pixelsPerLinePerMatrix);
				}
				else if(cameraboard==4){
					send_matrix_part(response4,cameraboard,matrixLine,pixelsPerLinePerMatrix);
				}
				else if(cameraboard==5){
					send_matrix_part(response5,cameraboard,matrixLine,pixelsPerLinePerMatrix);
				}
				else if(cameraboard==6){
					send_matrix_part(response6,cameraboard,matrixLine,pixelsPerLinePerMatrix);
				}

			}
			c= 255;
			Usart1Tx(&c,1);
			c= 0;
			Usart1Tx(&c,1);
			c= 0;
			Usart1Tx(&c,1);
			c= 218;
			Usart1Tx(&c,1);
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

