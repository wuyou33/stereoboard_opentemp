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
#include "tcm8230.h"
#include "hmc5883.h"
#include "stm32f4xx_conf.h"
#include "jpeg.h"
#include "arm_math.h"
#include "stereo_vision.h"
#include "window_detection.h"
#include "filter_color.h"
#include "utils.h"
#include "usb.h"
#include "sys_time.h"
#include "raw_digital_video_stream.h"
#include "../multigaze/stereoboard_parameters.h"
#include BOARD_FILE
#include "main_parameters.h"

#include "commands.h"
#define TOTAL_IMAGE_LENGTH IMAGE_WIDTH*IMAGE_HEIGHT;
// integral_image has size 128 * 96 * 4 = 49152 bytes = C000 in hex
uint32_t *integral_image = ((uint32_t *) 0x10000000); // 0x10000000 - 0x1000 FFFF = CCM data RAM  (64kB)
//uint8_t* jpeg_image_buffer_8bit = ((uint8_t*) 0x1000D000); // 0x10000000 - 0x1000 FFFF = CCM data RAM
//uint8_t* disparity_image_buffer_8bit = ((uint8_t*) 0x10000000);

uint16_t offset_crop = 0;

/** @addtogroup StereoCam
  * @{
  */

/* Private functions ---------------------------------------------------------*/

void calculateDistanceMatrix(uint8_t* disparity_image_buffer_8bit,
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
							disparity_image_buffer_8bit[pixelsPerLine
									* (y * heightPerBin) + line * pixelsPerLine
									+ widthPerBin * x + blackBorderSize
									+ bufferIndex];
				}
			}
		}
	}


	// Average by dividing by the amount of pixels per bin
	int bufferIndex;
	for (bufferIndex = 0; bufferIndex < MATRIX_WIDTH_BINS * MATRIX_HEIGHT_BINS;
			bufferIndex++) {
		toSendBuffer[bufferIndex] = matrixBuffer[bufferIndex] / 128;
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

  camera_unreset();
  // Initialize all camera GPIO and I2C pins
  camera_dcmi_bus_init();
  camera_tcm8230_i2c_init();
  // Start listening to DCMI frames
  camera_dcmi_init();
  // Start DCMI interrupts (interrupts on frame ready)
  camera_dcmi_it_init();
  camera_dcmi_dma_enable();


  // Wait for at least 2000 clock cycles after reset
  Delay(0x07FFFF);
  // Start DMA image transfer interrupts (interrupts on buffer full)
  camera_dma_it_init();
  // Communicate with camera, setup image type and start streaming
  camera_tcm8230_config();

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

  /***********
   * MAIN LOOP
   ***********/
  //camera_snapshot();

  volatile int processed = 0;


  	// Disparity image buffer, initialised with zeros
  	uint8_t disparity_image_buffer_8bit[FULL_IMAGE_SIZE / 2];
  	memset(disparity_image_buffer_8bit,0,sizeof disparity_image_buffer_8bit);



	// Stereo parameters:
	uint32_t disparity_range = 20; // at a distance of 1m, disparity is 7-8
	uint32_t disparity_min = 3;
	uint32_t disparity_step = 1;
	uint8_t thr1 = 4;
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
    camera_snapshot();
#ifdef LARGE_IMAGE
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
    led_toggle();


    current_image_buffer[0] = 0;
    current_image_buffer[1] = 0;


// Calculate the disparity map, only when we need it
#if SEND_DISPARITY_MAP || SEND_MATRIX
	// Determine disparities:
	min_y = 0;
	max_y = 95;
	stereo_vision_Kirk(current_image_buffer,
			disparity_image_buffer_8bit, image_width, image_height,
			disparity_min, disparity_range, disparity_step, thr1, thr2,
			min_y, max_y);
#endif


#if SEND_MATRIX
	// Initialise matrixbuffer and sendbuffer by setting all values back to zero.
	memset(matrixBuffer,0,sizeof matrixBuffer);
	memset(toSendBuffer,0,sizeof toSendBuffer);

	// Create the distance matrix by summing pixels per bin
	calculateDistanceMatrix(disparity_image_buffer_8bit, matrixBuffer, blackBorderSize,
			pixelsPerLine, widthPerBin, heightPerBin, toSendBuffer);
#endif


// Now send the data that we want to send
#if SEND_IMAGE
    Send(current_image_buffer, IMAGE_WIDTH, IMAGE_HEIGHT);
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

