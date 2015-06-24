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
#include "../common/led.h"
#include "dcmi.h"
#include "cpld.h"
#include "usart.h"
#include "camera_type.h"
#include "hmc5883.h"
#include "stm32f4xx_conf.h"
#include "jpeg.h"
#include "arm_math.h"
#include "stereo_vision.h"
#include "window_detection.h"
#include "filter_color.h"
#include "../common/utils.h"
#include "usb.h"
#include "sys_time.h"
#include "raw_digital_video_stream.h"
#include "../multigaze/stereoboard_parameters.h"
#include BOARD_FILE
#include "main_parameters.h"
#include "divergence.h"

#include "commands.h"
#define TOTAL_IMAGE_LENGTH IMAGE_WIDTH*IMAGE_HEIGHT;
// integral_image has size 128 * 96 * 4 = 49152 bytes = C000 in hex
//uint32_t *integral_image = ((uint32_t *) 0x10000000); // 0x10000000 - 0x1000 FFFF = CCM data RAM  (64kB)
//uint8_t* jpeg_image_buffer_8bit = ((uint8_t*) 0x1000D000); // 0x10000000 - 0x1000 FFFF = CCM data RAM
uint8_t* disparity_image_buffer_8bit = ((uint8_t*) 0x10000000);

uint16_t offset_crop = 0;

/** @addtogroup StereoCam
  * @{
  */

/* Private functions ---------------------------------------------------------*/

void calculateDistanceMatrix(uint8_t* disparity_image,
		int* matrixBuffer,
		uint8_t blackBorderSize, uint8_t pixelsPerLine, uint8_t widthPerBin,
		uint8_t heightPerBin,uint8_t *toSendBuffer) {

	int indexBuffer;

	uint8_t y;
	uint8_t x;
	for (x = 0; x < MATRIX_WIDTH_BINS; x++) {
		for (y = 0; y < MATRIX_HEIGHT_BINS; y++) {
			int line;
			for (line = 0; line < heightPerBin; line++) {
				int bufferIndex = 0;
				for (bufferIndex = 0; bufferIndex < widthPerBin;
						bufferIndex++) {
					matrixBuffer[y * MATRIX_WIDTH_BINS + x] +=
							disparity_image[pixelsPerLine
									* (y * heightPerBin) + line * pixelsPerLine
									+ widthPerBin * x + blackBorderSize
									+ bufferIndex];
				}
			}
		}
	}


	// Average by dividing by the amount of pixels per bin
	int bufferIndex;
	int anyOn=0;

	for (bufferIndex = 0; bufferIndex < MATRIX_WIDTH_BINS * MATRIX_HEIGHT_BINS;
			bufferIndex++) {
		toSendBuffer[bufferIndex] = matrixBuffer[bufferIndex] / (widthPerBin*heightPerBin);
		if(toSendBuffer[bufferIndex]>CLOSE_BOUNDARY && bufferIndex <MATRIX_WIDTH_BINS*4)
		{
			anyOn=1;
		}
	}
	if(anyOn==1){
		led_set();
	}
	else
	{
		led_clear();
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


  /****************
   * INITIALIZATION
   ****************/

  // Initialize the LED
  led_init();
  led_set();
  // Initialize the serial communication (before the camera so we can print status)
  usart_init();
  // Initialize the CPLD
  camera_cpld_stereo_init();
  // Reset the camera's
  camera_reset_init();
  camera_reset();
  // Make a 21MHz clock signal to the camera's
  camera_clock_init();
  // Stop resetting the camera (pin high)


  // Initialize all camera GPIO and I2C pins
  camera_dcmi_bus_init();
  camera_control_bus_init();
  // Start listening to DCMI frames
  camera_dcmi_init();
  // Start DCMI interrupts (interrupts on frame ready)
  camera_dcmi_it_init();
  camera_dcmi_dma_enable();

  // Start DMA image transfer interrupts (interrupts on buffer full)
  camera_dma_it_init();
  Delay(0x07FFFF);

  camera_unreset();
  // Wait for at least 2000 clock cycles after reset
  Delay(CAMERA_CHIP_UNRESET_TIMING);
  // Communicate with camera, setup image type and start streaming
  camera_chip_config();

  #if USE_COLOR
  // slight waste of memory, if color is not used:
  uint8_t filtered_image[FULL_IMAGE_SIZE / 2];
  for (ind = 0; ind < FULL_IMAGE_SIZE / 2; ind++) {
    filtered_image[ind] = 0;
  }
#endif
  uint8_t min_y, max_y;
  uint32_t image_width = IMAGE_WIDTH;
  uint32_t image_height = IMAGE_HEIGHT;
  uint32_t start, stop;


#if SEND_DIVERGENCE
	//Define arrays and pointers for edge histogram and displacements
	struct displacement_t displacement;
	displacement.horizontal[IMAGE_WIDTH];
	displacement.vertical[IMAGE_HEIGHT];

	//Initializing the dynamic parameters and the edge histogram structure
	int rear=1;
			int front=0;

			/*//Intializing edge histogram structure
			struct edge_hist_t* edge_hist;
			edge_hist=(struct edge_hist_t*)calloc(MAX_HORIZON,sizeof(struct edge_hist_t));*/

	//Intializing edge histogram structure
	struct edge_hist_t edge_hist[MAX_HORIZON];
	int i;
	for(i=0;i<MAX_HORIZON;i++)
	memset(&edge_hist[i],0,sizeof(struct edge_hist_t));

	//Initializing for divergence and flow parameters
	struct edge_flow_t edge_flow;
	edge_flow.horizontal[0]=0.0;
	edge_flow.horizontal[1]=0.0;
	edge_flow.vertical[0]=0.0;
	edge_flow.vertical[1]=0.0;

	/*
	int32_t displacement[IMAGE_WIDTH];
	int32_t* displacement_p=displacement;
	uint8_t prev_image_buffer[FULL_IMAGE_SIZE];
    uint8_t* prev_image_buffer_p=prev_image_buffer;
	uint8_t divergence_image_buffer[FULL_IMAGE_SIZE];
	uint8_t* divergence_image_buffer_p;//;=divergence_image_buffer;
	uint32_t edge_histogram_prev[image_width];
	uint32_t* edge_histogram_prev_p=edge_histogram_prev;
	uint32_t edge_histogram[image_width];
	uint32_t* edge_histogram_p=edge_histogram;
	float slope=0.0;
	float yint=0.0;*/
	#endif


  /***********
   * MAIN LOOP
   ***********/

  volatile int processed = 0;


  	// Disparity image buffer, initialised with zeros
  	//uint8_t disparity_image_buffer_8bit[FULL_IMAGE_SIZE / 2];
    memset(disparity_image_buffer_8bit,0,FULL_IMAGE_SIZE / 2);



	// Stereo parameters:
	uint32_t disparity_range = 20; // at a distance of 1m, disparity is 7-8
	uint32_t disparity_min = 0;
	uint32_t disparity_step = 1;
	uint8_t thr1 = 7;
	uint8_t thr2 = 4;
	uint8_t diff_threshold = 4; // for filtering

	// Settings for the depth matrix algorithm, calculated based on other settings
	// Settings of the camera... used by the distance matrix algorithm
	uint8_t blackBorderSize = 22;
	uint8_t pixelsPerLine = 128;
	uint8_t pixelsPerColumn = 96;

	uint8_t widthPerBin = (pixelsPerLine - 2 * blackBorderSize)
			/ MATRIX_WIDTH_BINS;
	uint8_t heightPerBin = pixelsPerColumn / MATRIX_HEIGHT_BINS;

	// Initialise matrixbuffer
	int matrixBuffer[MATRIX_HEIGHT_BINS * MATRIX_WIDTH_BINS];
	uint8_t toSendBuffer[MATRIX_HEIGHT_BINS * MATRIX_WIDTH_BINS];
  while (1) {
#if (CAPTURE_MODE_SNAPSHOT == 1)
    camera_snapshot();
#endif

#ifdef CROPPING
    offset_crop += 80;
    if (offset_crop == 480) {
      offset_crop = 0;
    }
    camera_crop(offset_crop);
#endif
    // wait for new frame
    while (frame_counter == processed)
      ;
    processed = frame_counter;
    //led_toggle();


// Calculate the disparity map, only when we need it
#if SEND_DISPARITY_MAP || SEND_MATRIX
	// Determine disparities:
	min_y = 0;
	max_y = 96;
	stereo_vision_Kirk(current_image_buffer,
			disparity_image_buffer_8bit, image_width, image_height,
			disparity_min, disparity_range, disparity_step, thr1, thr2,
			min_y, max_y);
#endif


#if SEND_MATRIX
	// Initialise matrixbuffer and sendbuffer by setting all values back to zero.
	memset(matrixBuffer,0,sizeof matrixBuffer);
	memset(toSendBuffer,0,sizeof toSendBuffer);
	//led_clear();
	// Create the distance matrix by summing pixels per bin
	calculateDistanceMatrix(disparity_image_buffer_8bit, matrixBuffer, blackBorderSize,
			pixelsPerLine, widthPerBin, heightPerBin, toSendBuffer);
#endif


#if SEND_DIVERGENCE


	calculate_edge_flow(current_image_buffer, &displacement,&edge_flow, &edge_hist, front,rear,10,10,10, image_width, image_height);



	 //calculate_edge_flow_simple(current_image_buffer,edge_histogram_p,edge_histogram_prev_p,displacement_p,&slope,&yint,IMAGE_WIDTH,IMAGE_HEIGHT);
	  //visualize_divergence(current_image_buffer,displacement_p,slope, yint,IMAGE_WIDTH,IMAGE_HEIGHT);
     //memcpy(edge_histogram_prev_p,edge_histogram_p,sizeof edge_histogram);


     uint8_t divergencearray[2];
		divergencearray[0]=(uint8_t)(edge_flow.horizontal[0]*10+100);


		divergencearray[1]=(uint8_t)(edge_flow.horizontal[1]*1000+100);

	 SendArray( divergencearray,2,1);

#endif
// Now send the data that we want to send
#if SEND_IMAGE
	  led_toggle();
    SendImage(current_image_buffer, IMAGE_WIDTH, IMAGE_HEIGHT);
#endif
#if SEND_DISPARITY_MAP
	SendArray(disparity_image_buffer_8bit,IMAGE_WIDTH,IMAGE_HEIGHT);
#endif
#if SEND_MATRIX
	SendArray(toSendBuffer, MATRIX_WIDTH_BINS, MATRIX_HEIGHT_BINS);
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

