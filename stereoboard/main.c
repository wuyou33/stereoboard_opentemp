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

// include system files
#include "arm_math.h"
#include "stm32f4xx_conf.h"
#include "sys_time.h"

// include utility headers
#include "../common/led.h"
#include "../common/utils.h"

// include camera headers
#include "camera_type.h"
#include "cpld.h"
#include "dcmi.h"

// Other sensors
//#include "hmc5883.h"

// include setttings
#include "main_parameters.h"
#include "../multigaze/stereoboard_parameters.h"

// include coms
#include "usart.h"
#include "usb.h"

#include "commands.h"
#include "jpeg.h"
#include "raw_digital_video_stream.h"

// include functions headers
#include "distance_matrix.h"
#include "divergence.h"
#include "droplet_algorithm.h"
#include "filter_color.h"
#include "stereo_vision.h"
#include "window_detection.h"

/********************************************************************/

#define TOTAL_IMAGE_LENGTH IMAGE_WIDTH*IMAGE_HEIGHT;
// integral_image has size 128 * 96 * 4 = 49152 bytes = C000 in hex
//uint32_t *integral_image = ((uint32_t *) 0x10000000); // 0x10000000 - 0x1000 FFFF = CCM data RAM  (64kB)
//uint8_t* jpeg_image_buffer_8bit = ((uint8_t*) 0x1000D000); // 0x10000000 - 0x1000 FFFF = CCM data RAM
uint8_t *disparity_image_buffer_8bit = ((uint8_t *) 0x10000000);

uint16_t offset_crop = 0;

/** @addtogroup StereoCam
  * @{
  */

/* Private functions ---------------------------------------------------------*/
typedef enum {SEND_TURN_COMMANDS, SEND_COMMANDS, SEND_IMAGE, SEND_DISPARITY_MAP, SEND_FRAMERATE_STEREO, SEND_MATRIX, SEND_DIVERGENCE, SEND_PROXIMITY, SEND_WINDOW} stereoboard_algorithm_type;

//////////////////////////////////////////////////////
// Define which code should be run:
stereoboard_algorithm_type getBoardFunction(void)
{
#if ! (defined(SEND_COMMANDS) || defined(SEND_IMAGE) || defined(SEND_DISPARITY_MAP) || defined(SEND_MATRIX) || defined(SEND_DIVERGENCE) || defined(SEND_WINDOW))
  return DEFAULT_BOARD_FUNCTION;
#elif defined(SEND_COMMANDS)
  return SEND_COMMANDS;
#elif defined(SEND_IMAGE)
  return SEND_IMAGE;
#elif defined(SEND_DISPARITY_MAP)
  return SEND_DISPARITY_MAP;
#elif defined(SEND_FRAMERATE_STEREO)
  return SEND_FRAMERATE_STEREO;
#elif defined(SEND_MATRIX)
  return SEND_MATRIX;
#elif defined(SEND_DIVERGENCE)
  return SEND_DIVERGENCE;
#elif defined(SEND_WINDOW)
  return SEND_WINDOW;
  //Initializing the dynamic parameters and the edge histogram structure
  int rear = 1;
  int front = 0;
#endif
}

//Element for the kalman filter divergence
const uint32_t RES = 100;   // resolution scaling for integer math

struct coveriance_t coveriance;
const uint32_t Q = 10;    // motion model; 0.25*RES
const uint32_t R = 100;   // measurement model  1*RES

uint8_t current_frame_nr = 0;

struct edge_hist_t edge_hist[MAX_HORIZON];
struct edge_flow_t edge_flow;
struct edge_flow_t prev_edge_flow;

struct displacement_t displacement;
uint8_t initialisedDivergence = 0;
int16_t height = 0;

//send array with flow parameters
uint8_t divergencearray[5];

void divergence_init()
{
  //Define arrays and pointers for edge histogram and displacements
  memset(displacement.horizontal, 0, IMAGE_WIDTH);
  memset(displacement.vertical, 0, IMAGE_WIDTH);

  //Initializing the dynamic parameters and the edge histogram structure
  current_frame_nr = 0;

  //Intializing edge histogram structure
  memset(edge_hist, 0, MAX_HORIZON * sizeof(struct edge_hist_t));

  //Initializing for divergence and flow parameters
  edge_flow.horizontal_slope = prev_edge_flow.horizontal_slope = 0;
  edge_flow.horizontal_trans = prev_edge_flow.horizontal_trans = 0;
  edge_flow.vertical_slope = prev_edge_flow.vertical_trans = 0;
  edge_flow.vertical_trans = prev_edge_flow.vertical_trans = 0;

  coveriance.slope_x = 20;
  coveriance.slope_y = 20;
  coveriance.trans_x = 20;
  coveriance.trans_y = 20;

  height = 0;

  initialisedDivergence = 1;
}

#define WINDOWBUFSIZE 13  // 8 for window and 5 for divergence

uint8_t windowMsgBuf[WINDOWBUFSIZE];
uint8_t coordinate[2];
uint8_t window_size;
uint32_t integral_image[IMAGE_WIDTH *IMAGE_HEIGHT];

void window_init()
{
  coordinate[0] = IMAGE_WIDTH / 2;
  coordinate[1] = IMAGE_HEIGHT / 2;
  memset(integral_image, 0, IMAGE_WIDTH * IMAGE_HEIGHT);
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
  sys_time_init();

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

  /***********
   * MAIN LOOP
   ***********/

  stereoboard_algorithm_type current_stereoboard_algorithm = getBoardFunction();
  volatile int processed = 0;

  // Disparity image buffer, initialised with zeros
  //uint8_t disparity_image_buffer_8bit[FULL_IMAGE_SIZE / 2];
  memset(disparity_image_buffer_8bit, 0, FULL_IMAGE_SIZE / 2);

  /*// Stereo parameters:
  uint32_t disparity_range = 16; // at a distance of 1m, disparity is 7-8
  uint32_t disparity_min = 0;
  uint32_t disparity_step = 2;
  uint8_t thr1 = 7;
  uint8_t thr2 = 4;*/
  // uint8_t diff_threshold = 4; // for filtering

  // init droplet parameters
  volatile uint16_t current_phase = 1;

  // Settings for the depth matrix algorithm, calculated based on other settings
  // Settings of the camera... used by the distance matrix algorithm
  uint8_t blackBorderSize = 22;
  uint8_t pixelsPerLine = 128;
  uint8_t pixelsPerColumn = 96;

  uint8_t widthPerBin = (pixelsPerLine - 2 * blackBorderSize)
                        / MATRIX_WIDTH_BINS;
  uint8_t heightPerBin = pixelsPerColumn / MATRIX_HEIGHT_BINS;

  // Initialize matrixbuffer
  int matrixBuffer[MATRIX_HEIGHT_BINS * MATRIX_WIDTH_BINS];
  uint8_t toSendBuffer[MATRIX_HEIGHT_BINS * MATRIX_WIDTH_BINS];
  uint8_t toSendCommand = 0;
  volatile uint64_t sys_time_prev = sys_time_get();
  uint32_t freq_counter = 0;

  // Initialize window
  window_init();

  // Settings for SEND_TURN_COMMANDS
  uint8_t n_disp_bins = 6;
  uint32_t disparities[n_disp_bins];
  uint8_t RESOLUTION = 100;

  // initialize divergence
  divergence_init();

  while (1) {
    if (current_stereoboard_algorithm == SEND_PROXIMITY) {
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
    } else {
      camera_snapshot();

#if defined(LARGE_IMAGE) || defined(CROPPING)
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
      if (current_stereoboard_algorithm == SEND_DISPARITY_MAP || current_stereoboard_algorithm == SEND_MATRIX
          || current_stereoboard_algorithm == SEND_COMMANDS || current_stereoboard_algorithm == SEND_TURN_COMMANDS ||
          current_stereoboard_algorithm == SEND_FRAMERATE_STEREO || current_stereoboard_algorithm == SEND_WINDOW) {
        // Determine disparities:
        min_y = 0;
        max_y = 96;
        memset(disparity_image_buffer_8bit, 0, FULL_IMAGE_SIZE / 2);

        if (STEREO_ALGORITHM) {
          stereo_vision_Kirk(current_image_buffer,
                             disparity_image_buffer_8bit, image_width, image_height,
                             disparity_min, disparity_range, disparity_step, thr1, thr2,
                             min_y, max_y);
        } else {
          stereo_vision_sparse_block_two_sided(current_image_buffer,
                                               disparity_image_buffer_8bit, image_width, image_height,
                                               disparity_min, disparity_range, disparity_step, thr1, thr2,
                                               min_y, max_y);
        }
      }

      // determine phase of flight
      if (current_stereoboard_algorithm == SEND_COMMANDS || current_stereoboard_algorithm == SEND_DISPARITY_MAP ||
          current_stereoboard_algorithm == SEND_FRAMERATE_STEREO) {

        int disparities_high = 0;
        disparities_high =  evaluate_disparities_droplet(disparity_image_buffer_8bit, image_width, image_height);
        current_phase = run_droplet_algorithm(disparities_high, sys_time_get());

        if (current_phase == 1) {
          toSendCommand = 0;
          //led_set();
        }
        if (current_phase == 2) {
          toSendCommand = 1;
          //led_set();
        }
        if (current_phase == 3) {
          toSendCommand = 2;
          //led_set();
        }
        if (current_phase == 4) {
          toSendCommand = 3;
          //led_set();
        }
      }

      if (current_stereoboard_algorithm == SEND_TURN_COMMANDS) {
        uint8_t border = 0;
        uint8_t disp_threshold = 5 * RESOLUTION_FACTOR;
        evaluate_central_disparities2(disparity_image_buffer_8bit, image_width, image_height, disparities, n_disp_bins, min_y,
                                      max_y, disp_threshold, border);
        disparities[0] = (disparities[0] * RESOLUTION) / ((max_y - min_y) * (image_width));
        disparities[1] = (disparities[1] * RESOLUTION) / image_width;

        // Send commands
        // send 0xff
        SendStartComm();
        // percentage of close pixels
        SendCommandNumber((uint8_t) disparities[0]);
        // percentage of x-location
        SendCommandNumber((uint8_t) disparities[1]);
      }

      // compute run frequency
      if (current_stereoboard_algorithm == SEND_FRAMERATE_STEREO || current_stereoboard_algorithm == SEND_WINDOW) {
        //led_toggle();
        freq_counter++;
        if ((sys_time_get() - sys_time_prev) >= 2000) { // clock at 2kHz
          toSendCommand = (uint8_t)((freq_counter * (sys_time_get() - sys_time_prev)) / 2000);
          freq_counter = 0;
          sys_time_prev = sys_time_get();
        }
        //toSendCommand = (2000/(sys_time_get()-sys_time_prev))-15;
      }

      // send matrix buffer
      if (current_stereoboard_algorithm == SEND_MATRIX) {

        // Initialise matrixbuffer and sendbuffer by setting all values back to zero.
        memset(matrixBuffer, 0, sizeof matrixBuffer);
        memset(toSendBuffer, 0, sizeof toSendBuffer);
        //led_clear();
        // Create the distance matrix by summing pixels per bin
        calculateDistanceMatrix(disparity_image_buffer_8bit, matrixBuffer, blackBorderSize,
                                pixelsPerLine, widthPerBin, heightPerBin, toSendBuffer, disparity_range);
      }

      // compute and send divergence
      if (current_stereoboard_algorithm == SEND_DIVERGENCE){// || current_stereoboard_algorithm == SEND_WINDOW) {
        //if (initialisedDivergence == 0) {
        //  initialiseDivergence();
        //}

        //calculate the edge flow
        calculate_edge_flow(current_image_buffer, &displacement, &edge_flow, edge_hist, &height, current_frame_nr , 10, 10, 10,
                            IMAGE_WIDTH, IMAGE_HEIGHT, RES);

        //move the indices for the edge hist structure
        current_frame_nr = (current_frame_nr + 1) % MAX_HORIZON;

        //Kalman filtering
        totalKalmanFilter(&coveriance, &prev_edge_flow, &edge_flow, Q, R, RES);

        // TODO: find a better way to scale the data
        divergencearray[0] = (uint8_t)(edge_flow.horizontal_slope + 127);
        divergencearray[1] = (uint8_t)(edge_flow.horizontal_trans / 4 + 127);
        divergencearray[2] = (uint8_t)(edge_flow.vertical_slope + 127);
        divergencearray[3] = (uint8_t)(edge_flow.vertical_trans / 4 + 127);
        divergencearray[4] = (uint8_t) height;

        memcpy(&prev_edge_flow, &edge_flow, sizeof(struct edge_flow_t));
      }

      // compute and send window detection parameters
      if (current_stereoboard_algorithm == SEND_WINDOW) {
        // XPOS, YPOS, RESPONSE, DISP_SUM, DISP_HOR, DISP_VERT

        windowMsgBuf[2] = (uint8_t)detect_window_sizes(disparity_image_buffer_8bit, image_width, image_height, coordinate,
                          &window_size, integral_image, MODE_DISPARITY, (uint8_t)(disparity_range - disparity_min));
        windowMsgBuf[0] = coordinate[0];
        windowMsgBuf[1] = coordinate[1];

        windowMsgBuf[3] = (uint8_t)(get_sum_disparities(0, 0, 127, 95, integral_image, 128, 96) / 512);
        windowMsgBuf[4] = (uint8_t)((get_sum_disparities(0, 0, 63, 95, integral_image, 128, 96) - get_sum_disparities(64, 0,
                                     127, 95, integral_image, 128, 96)) / windowMsgBuf[3]) + 128;
        windowMsgBuf[5] = (uint8_t)((get_sum_disparities(0, 0, 127, 47, integral_image, 128, 96) - get_sum_disparities(0, 48,
                                     127, 95, integral_image, 128, 96)) / windowMsgBuf[3]) + 128;
        windowMsgBuf[6] = window_size;
        windowMsgBuf[7] = toSendCommand; // frequency

        memcpy(windowMsgBuf + 8, divergencearray, 5);
      }
      // Now send the data that we want to send
      if (current_stereoboard_algorithm == SEND_IMAGE) {
        SendImage(current_image_buffer, IMAGE_WIDTH, IMAGE_HEIGHT);
      }
      if (current_stereoboard_algorithm == SEND_DISPARITY_MAP) {
        SendArray(disparity_image_buffer_8bit, IMAGE_WIDTH, IMAGE_HEIGHT);
      }
      if (current_stereoboard_algorithm == SEND_MATRIX) {
        SendArray(toSendBuffer, MATRIX_WIDTH_BINS, MATRIX_HEIGHT_BINS);
      }
      if (current_stereoboard_algorithm == SEND_WINDOW) {
        SendArray(windowMsgBuf, WINDOWBUFSIZE, 1);
      }
      if (current_stereoboard_algorithm == SEND_DIVERGENCE) {
        SendArray(divergencearray, 5, 1);
      }
      if (current_stereoboard_algorithm == SEND_COMMANDS || current_stereoboard_algorithm == SEND_FRAMERATE_STEREO) {
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

