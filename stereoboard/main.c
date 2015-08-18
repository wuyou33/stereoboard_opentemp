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
typedef enum {SEND_COMMANDS, SEND_IMAGE, SEND_DISPARITY_MAP, SEND_MATRIX, SEND_DIVERGENCE,SEND_PROXIMITY} stereoboard_algorithm_type;

//////////////////////////////////////////////////////
// Define which code should be run:
 stereoboard_algorithm_type getBoardFunction(void){
	#if ! (defined(SEND_COMMANDS) || defined(SEND_IMAGE) || defined(SEND_DISPARITY_MAP) || defined(SEND_MATRIX) || defined(SEND_DIVERGENCE))
		return DEFAULT_BOARD_FUNCTION;
	#elif defined(SEND_COMMANDS)
		return SEND_COMMANDS;
	#elif defined(SEND_IMAGE)
		return SEND_IMAGE;
	#elif defined(SEND_DISPARITY_MAP)
		return SEND_DISPARITY_MAP;
	#elif defined(SEND_MATRIX)
		return SEND_MATRIX;
	#elif defined(SEND_DIVERGENCE)
		return SEND_DIVERGENCE;
	#endif
}
 int divergence_front;
 int divergence_rear;
 //Element for the kalman filter divergence

    struct coveriance_t coveriance;

struct edge_flow_t prev_edge_flow;

float Q=0.01;//motion model
float R=1.0;//measurement model
struct edge_hist_t edge_hist[MAX_HORIZON];
struct edge_flow_t edge_flow;
struct displacement_t displacement;
uint8_t initialisedDivergence=0;
float height=0;

void initialiseDivergence(){
	    //Define arrays and pointers for edge histogram and displacements
	 	displacement.horizontal[IMAGE_WIDTH];
	 	displacement.vertical[IMAGE_HEIGHT];

	 	//Initializing the dynamic parameters and the edge histogram structure
	 	divergence_rear=1;
	 	divergence_front=0;

	 	//Intializing edge histogram structure
	 	memset(&edge_hist,0,MAX_HORIZON*sizeof(struct edge_hist_t));

	 	//Initializing for divergence and flow parameters
        edge_flow.horizontal_slope=0.0;
	 	edge_flow.horizontal_trans=0.0;
	 	edge_flow.vertical_slope=0.0;
	 	edge_flow.vertical_trans=0.0;

	 	initialisedDivergence=1;
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

  /***********
   * MAIN LOOP
   ***********/
  //camera_snapshot();

#if SEND_DIVERGENCE
	//Define arrays and pointers for edge histogram and displacements
	struct displacement_t displacement;
	displacement.horizontal[IMAGE_WIDTH];
	displacement.vertical[IMAGE_HEIGHT];

	//Initializing the dynamic parameters and the edge histogram structure
	int rear=1;
	int front=0;

	//Intializing edge histogram structure
	struct edge_hist_t edge_hist[MAX_HORIZON];
	memset(&edge_hist,0,MAX_HORIZON*sizeof(struct edge_hist_t));

	//Initializing for divergence and flow parameters
	struct edge_flow_t edge_flow;

	edge_flow.horizontal_slope=0.0;
	edge_flow.horizontal_trans=0.0;
	edge_flow.vertical_slope=0.0;
	edge_flow.vertical_trans=0.0;

	//Element for the kalman filter

	float coveriance_trans_x=0.;
	float coveriance_trans_y=0.;
	float coveriance_slope_x=0.;
	float coveriance_slope_y=0.;

	struct edge_flow_t prev_edge_flow;

	float Q=0.01;//motion model
	float R=1.0;//measurement model
	float new_est_x_trans,new_est_y_trans;
	float new_est_x_slope,new_est_y_slope;

#endif


	/***********
	 * MAIN LOOP
	 ***********/

	stereoboard_algorithm_type current_stereoboard_algorithm=getBoardFunction();
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
	uint8_t toSendCommand;
  while (1) {

  if(current_stereoboard_algorithm==SEND_PROXIMITY){
	  /*
	  uint8_t response[6];
	  response[0]=10;
	  proximity_sensor_WriteReg(REGISTER_ENABLE,COMMAND_POWER_ON | COMMAND_PROXIMITY_ENABLE | COMMAND_ALS_ENABLE);
	  proximity_sensor_WriteReg(REGISTER_CONTROL,COMMAND_AGAIN_64);
	  proximity_sensor_WriteReg(REGISTER_PPULSE,COMMAND_32us_63p);
	  proximity_sensor_WriteReg(REGISTER_CONFIG2,COMMAND_BOOST_150);



	  readRegisterProximitySensor(REGISTER_PDATA,&response[0]);
	  readRegisterProximitySensor( REGISTER_CDATAL ,&response[2]);
	  readRegisterProximitySensor( REGISTER_CDATAH ,&response[3]);
	  SendArray(response,4,1);*/
	}
	else{
	camera_snapshot();

#ifdef LARGE_IMAGE
    offset_crop += 80;
    if (offset_crop == 480) {
      offset_crop = 0;
    }
    camera_crop(offset_crop);
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

    current_image_buffer[0] = 0;
    current_image_buffer[1] = 0;

/*
		uint8_t readChar = ' ';

		while(UsartCh()){
			readChar=UsartRx();
			if(readChar==1)
			{
				current_stereoboard_algorithm=SEND_DISPARITY_MAP;
			}
			else if (readChar==2)
			{
				current_stereoboard_algorithm=SEND_MATRIX;
			}
			else if (readChar==3)
			{
				disparity_min-=1;
			}
			else if (readChar==4)
			{
				disparity_min+=1;
			}
			else if (readChar==5)
			{
				current_stereoboard_algorithm=SEND_DIVERGENCE;
			}
		}*/

		// New frame code: Vertical blanking = ON


	// Calculate the disparity map, only when we need it
	if(current_stereoboard_algorithm==SEND_DISPARITY_MAP || current_stereoboard_algorithm==SEND_MATRIX || current_stereoboard_algorithm==SEND_COMMANDS){
		// Determine disparities:
		min_y = 0;
		max_y = 95;
		memset(disparity_image_buffer_8bit,0,FULL_IMAGE_SIZE / 2);

		if ( STEREO_ALGORITHM )
		{
			stereo_vision_Kirk(current_image_buffer,
					disparity_image_buffer_8bit, image_width, image_height,
					disparity_min, disparity_range, disparity_step, thr1, thr2,
					min_y, max_y);
		}
		else {
			stereo_vision_sparse_block_two_sided(current_image_buffer,
					disparity_image_buffer_8bit, image_width, image_height,
					disparity_min, disparity_range, disparity_step, thr1, thr2,
					min_y, max_y);
		}
	}

	if(current_stereoboard_algorithm==SEND_COMMANDS || current_stereoboard_algorithm==SEND_DISPARITY_MAP){

		int disparities_high = 0;
		disparities_high =  evaluate_disparities_droplet(disparity_image_buffer_8bit, image_width, image_height);
		if ( disparities_high >8 )
		{
			toSendCommand = 1;
			led_set();
		}
		else
		{
			toSendCommand = 0;
			led_clear();
		}
		//led_toggle();

	}


	if(current_stereoboard_algorithm==SEND_MATRIX){

		// Initialise matrixbuffer and sendbuffer by setting all values back to zero.
		memset(matrixBuffer,0,sizeof matrixBuffer);
		memset(toSendBuffer,0,sizeof toSendBuffer);
		//led_clear();
		// Create the distance matrix by summing pixels per bin
		calculateDistanceMatrix(disparity_image_buffer_8bit, matrixBuffer, blackBorderSize,
				pixelsPerLine, widthPerBin, heightPerBin, toSendBuffer, disparity_range);
	}


	if(current_stereoboard_algorithm==SEND_DIVERGENCE){
			if(initialisedDivergence==0){
				initialiseDivergence();
			}
			//calculate the edge flow
			int previous_frame=calculate_edge_flow(current_image_buffer, &displacement,&edge_flow, edge_hist, &height,divergence_front,divergence_rear,10,10,10, IMAGE_WIDTH, IMAGE_HEIGHT);

			//move the indices for the edge hist structure
			divergence_front++;
			divergence_rear++;

			if(divergence_front>MAX_HORIZON-1)
				divergence_front=0;
			if(divergence_rear>MAX_HORIZON-1)
				divergence_rear=0;

			//Kalman filtering
			totalKalmanFilter(&coveriance,&prev_edge_flow, &edge_flow,Q, R);

			//send array with flow parameters

			uint8_t divergencearray[5];
			divergencearray[0]=(uint8_t)(edge_flow.horizontal_slope*1000+100);
			divergencearray[1]=(uint8_t)(edge_flow.horizontal_trans*100+100);
			divergencearray[2]=(uint8_t)(edge_flow.vertical_slope*1000+100);
			divergencearray[3]=(uint8_t)(edge_flow.vertical_trans*100+100);
			divergencearray[4]=(uint8_t)height;

			SendArray( divergencearray,5,1);
			led_toggle();
			memcpy(&prev_edge_flow,&edge_flow,4*sizeof(float));


	}
	// Now send the data that we want to send
	if(current_stereoboard_algorithm==SEND_IMAGE){
		SendImage(current_image_buffer, IMAGE_WIDTH, IMAGE_HEIGHT);
	}
	if(current_stereoboard_algorithm==SEND_DISPARITY_MAP){
		SendArray(disparity_image_buffer_8bit,IMAGE_WIDTH,IMAGE_HEIGHT);
	}
	if(current_stereoboard_algorithm== SEND_MATRIX){
		SendArray(toSendBuffer, MATRIX_WIDTH_BINS, MATRIX_HEIGHT_BINS);
	}
	if(current_stereoboard_algorithm== SEND_COMMANDS){
		SendCommand(toSendCommand);
	}
	}
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

