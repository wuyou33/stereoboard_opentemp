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

// include std libs
#include <math.h>

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
typedef enum {SEND_TURN_COMMANDS, SEND_COMMANDS, SEND_IMAGE, SEND_DISPARITY_MAP, SEND_FRAMERATE_STEREO, SEND_MATRIX, SEND_DIVERGENCE, SEND_PROXIMITY, SEND_WINDOW, SEND_HISTOGRAM, SEND_DELFLY_CORRIDOR} stereoboard_algorithm_type;

//////////////////////////////////////////////////////
// Define which code should be run:
stereoboard_algorithm_type getBoardFunction(void)
{
#if ! (defined(SEND_COMMANDS) || defined(SEND_IMAGE) || defined(SEND_DISPARITY_MAP) || defined(SEND_MATRIX) || defined(SEND_DIVERGENCE) || defined(SEND_WINDOW) || defined(SEND_HISTOGRAM) || defined(SEND_DELFLY_CORRIDOR))
  return DEFAULT_BOARD_FUNCTION;

#elif defined(SEND_DELFLY_CORRIDOR)
  return SEND_DELFLY_CORRIDOR
#elif defined(SEND_HISTOGRAM)
  return SEND_HISTOGRAM;
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

//Initializing all structures and elements for the optical flow algorithm (divergence.c)
const int32_t RES = 100;   // resolution scaling for integer math

struct covariance_t covariance;
const uint32_t Q = 10;    // motion model; 0.25*RES
const uint32_t R = 100;   // measurement model  1*RES

uint8_t current_frame_nr = 0;

struct edge_hist_t edge_hist[MAX_HORIZON];
struct edge_flow_t edge_flow;
struct edge_flow_t prev_edge_flow;

struct displacement_t displacement;
uint8_t initialisedDivergence = 0;
int32_t avg_disp = 0;
int32_t avg_dist = 0;
uint8_t previous_frame_offset[2] = {1, 1};
uint8_t quality_measures_edgeflow[10] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0};

const int8_t FOVX = 104;   // 60deg = 1.04 rad
const int8_t FOVY = 79;    // 45deg = 0.785 rad

//send array with flow parameters
uint8_t divergenceArray[18];

void divergence_init()
{
  //Define arrays and pointers for edge histogram and displacements
  // memset(displacement.horizontal, 0, IMAGE_WIDTH);
  // memset(displacement.vertical, 0, IMAGE_WIDTH);

  //Initializing the dynamic parameters and the edge histogram structure
  current_frame_nr = 0;

  // Intializing edge histogram structure
  // memset(edge_hist, 0, MAX_HORIZON * sizeof(struct edge_hist_t));

  //Initializing for divergence and flow parameters
  edge_flow.horizontal_flow = prev_edge_flow.horizontal_flow = 0;
  edge_flow.horizontal_div = prev_edge_flow.horizontal_div = 0;
  edge_flow.vertical_flow = prev_edge_flow.vertical_flow = 0;
  edge_flow.vertical_div = prev_edge_flow.vertical_div = 0;

  covariance.flow_x = 20;
  covariance.flow_y = 20;
  covariance.div_x = 20;
  covariance.div_y = 20;

  avg_dist = 0;
  avg_disp = 0;

  initialisedDivergence = 1;
}

#define WINDOWBUFSIZE 8  // 8 for window and 8 for divergence

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

  // Stereo parameters:
  uint32_t disparity_range = 20; // at a distance of 1m, disparity is 7-8. disp = Npix*cam_separation /(2*dist*tan(FOV/2))
  uint32_t disparity_min = 0;
  uint32_t disparity_step = 1;
  uint8_t thr1 = 7;
  uint8_t thr2 = 4;
  uint8_t diff_threshold = 4; // for filtering

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
  uint32_t sys_time_prev = sys_time_get();
  uint32_t freq_counter = 0;
  int32_t frameRate = 0;

  uint8_t histogramBuffer[pixelsPerLine];

  // Initialize window
  window_init();

  // Settings for SEND_TURN_COMMANDS
  uint8_t n_disp_bins = 6;
  uint32_t disparities[n_disp_bins];
  uint8_t RESOLUTION = 100;

  // Stereo communication input protocol
  uint8_t ser_read_buf[STEREO_BUF_SIZE];           // circular buffer for incoming data
  uint8_t msg_buf[STEREO_BUF_SIZE];         // define local data
  typedef struct {
    uint8_t len;
    uint8_t *data;
    uint8_t data_new;
  } uint8array;
  uint8array stereocam_data = {.len = 0, .data = msg_buf, .data_new = 0};  // buffer used to contain image without line endings
  uint16_t insert_loc, extract_loc, msg_start;   // place holders for buffer read and write
  insert_loc = 0;
  extract_loc = 0;
  msg_start = 0;

  // initialize divergence
  divergence_init();
  led_clear();
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


      current_image_buffer[0] = 0;
      current_image_buffer[1] = 0;

      // compute run frequency
#ifdef AVG_FREQ
      freq_counter++;
      if ((sys_time_get() - sys_time_prev) >= 2000) { // clock at 2kHz
        frameRate = freq_counter * (sys_time_get() - sys_time_prev) / 2000; // in Hz
        freq_counter = 0;
        sys_time_prev = sys_time_get();
      }
#else
      frameRate = 2000 / (sys_time_get() - sys_time_prev); // in Hz
      sys_time_prev = sys_time_get();
#endif
      // Read from other device with the stereo communication protocol.
      uint8_t readChar = ' ';
      while (UsartCh()) {
        readChar = UsartRx();
        uint16_t length = STEREO_BUF_SIZE;
        if (handleStereoPackage(readChar, length, &insert_loc, &extract_loc, &msg_start, msg_buf, ser_read_buf,
                                &stereocam_data.data_new, &stereocam_data.len)) {
          if (stereocam_data.len > 50) {

            int32_t *pointer = (int32_t *)msg_buf;
            int someHeightOnDrone = pointer[9];
            if (someHeightOnDrone > 100) {
              led_set();
            } else {
              led_clear();
            }
          }
        }
      }

      // New frame code: Vertical blanking = ON


      // Calculate the disparity map, only when we need it
      if (current_stereoboard_algorithm == SEND_DISPARITY_MAP || current_stereoboard_algorithm == SEND_MATRIX
          || current_stereoboard_algorithm == SEND_COMMANDS || current_stereoboard_algorithm == SEND_TURN_COMMANDS ||
          current_stereoboard_algorithm == SEND_FRAMERATE_STEREO || current_stereoboard_algorithm == SEND_WINDOW ||
          current_stereoboard_algorithm == SEND_HISTOGRAM || current_stereoboard_algorithm == SEND_DELFLY_CORRIDOR) {
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

      if (current_stereoboard_algorithm == SEND_HISTOGRAM || current_stereoboard_algorithm == SEND_DELFLY_CORRIDOR) {
        calculateHistogram(disparity_image_buffer_8bit, histogramBuffer, blackBorderSize, pixelsPerLine, image_height);

        if (current_stereoboard_algorithm == SEND_DELFLY_CORRIDOR) {
          toSendCommand = calculateHeadingFromHistogram(histogramBuffer);
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
        }
        if (current_phase == 2) {
          toSendCommand = 1;
        }
        if (current_phase == 3) {
          toSendCommand = 2;
        }
        if (current_phase == 4) {
          toSendCommand = 3;
        }
      }

      // compute run frequency
      if (current_stereoboard_algorithm == SEND_FRAMERATE_STEREO) {
        toSendCommand = (uint8_t) frameRate;
        // toSendCommand = (2000/(sys_time_get()-sys_time_prev))-15;
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

      // send matrix buffer
      if (current_stereoboard_algorithm == SEND_MATRIX) {

        // Initialise matrixbuffer and sendbuffer by setting all values back to zero.
        memset(matrixBuffer, 0, sizeof matrixBuffer);
        memset(toSendBuffer, 0, sizeof toSendBuffer);
        // Create the distance matrix by summing pixels per bin
        calculateDistanceMatrix(disparity_image_buffer_8bit, matrixBuffer, blackBorderSize,
                                pixelsPerLine, widthPerBin, heightPerBin, toSendBuffer, disparity_range);
      }

      // compute and send divergence
      if (current_stereoboard_algorithm == SEND_DIVERGENCE) { // || current_stereoboard_algorithm == SEND_WINDOW) {
        //if (initialisedDivergence == 0) {
        //  initialiseDivergence();
        //}
        led_toggle();
        // calculate the edge flow
        calculate_edge_flow(current_image_buffer, &displacement, &edge_flow, edge_hist, &avg_disp,
                            previous_frame_offset, current_frame_nr, &quality_measures_edgeflow, 10, 20, 0,
                            IMAGE_WIDTH, IMAGE_HEIGHT, RES);

        // Filter flow
        // totalKalmanFilter(&covariance, &prev_edge_flow, &edge_flow, Q, R, RES);

        divergenceArray[0] = (uint8_t)(edge_flow.horizontal_div / previous_frame_offset[0] +
                                       127);          // should be in 0.01px/frame
        divergenceArray[1] = (uint8_t)(edge_flow.horizontal_flow / (10 * previous_frame_offset[0]) +
                                       127);   // should be in 0.1px/frame

        divergenceArray[2] = (uint8_t)(edge_flow.vertical_div / previous_frame_offset[1] +
                                       127);           // should be in 0.01px/frame
        divergenceArray[3] = (uint8_t)(edge_flow.vertical_flow / (10 * previous_frame_offset[1]) +
                                       127);     // should be in 0.1px/frame

        // disparity to distance in dm given 6cm dist between cams and Field of View (FOV) of 60deg
        // d =  Npix*cam_separation /(2*disp*tan(FOV/2))
        // d = 0.06*128 / (2*tan(disp*1.042/2))
        // d = 0.06*128 / (2*disp*1.042/2)
        // d = RES*0.06*128 / (disp*RES*1.042)
        // d = RES*0.06*PIX / (disp*FOVX)

        divergenceArray[4] = (uint8_t)avg_disp;

        if (avg_disp > 0) {
          avg_dist = RES * 6 * IMAGE_WIDTH / (avg_disp * FOVX);
        } else {
          avg_dist = 1477; // 2 * RES * 6 * IMAGE_WIDTH / 104;
        }

        divergenceArray[4] = (uint8_t)avg_dist / 10;
        memcpy(divergenceArray + 5, previous_frame_offset, 2); // copy frame offset to output array
        divergenceArray[7] = frameRate;

        //store the time of the frame
        edge_hist[current_frame_nr].frame_time = sys_time_get();

        // Calculate velocity
        int32_t hz_x = 2000 / ((int32_t)sys_time_get() - edge_hist[(current_frame_nr - previous_frame_offset[0] + MAX_HORIZON) %
                               MAX_HORIZON].frame_time); // in s
        int32_t hz_y = 2000 / ((int32_t)sys_time_get() - edge_hist[(current_frame_nr - previous_frame_offset[1] + MAX_HORIZON) %
                               MAX_HORIZON].frame_time); // in s
        int32_t vel_hor = edge_flow.horizontal_flow * avg_dist * hz_x *  FOVX / (RES * RES * IMAGE_WIDTH);
        int32_t vel_ver = edge_flow.vertical_flow  * avg_dist * hz_y  *  FOVY / (RES * RES * IMAGE_HEIGHT);

        divergenceArray[7] = hz_x;
        //TODO: Find where the multi. of 10 comes from, the optitrack gives a lower value in speed.
        divergenceArray[8] = (uint8_t)(vel_hor / 10 + 127); // in cm/s
        divergenceArray[9] = (uint8_t)(vel_ver / 10 + 127); // in cm/s

        memcpy(divergenceArray + 10, &quality_measures_edgeflow, 10 * sizeof(uint8_t)); // copy quality measures to output array

        memcpy(&prev_edge_flow, &edge_flow, sizeof(struct edge_flow_t));

        // move the indices for the edge hist structure
        current_frame_nr = (current_frame_nr + 1) % MAX_HORIZON;
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
                                     127, 95, integral_image, 128, 96)) / windowMsgBuf[3]) + 127;
        windowMsgBuf[5] = (uint8_t)((get_sum_disparities(0, 0, 127, 47, integral_image, 128, 96) - get_sum_disparities(0, 48,
                                     127, 95, integral_image, 128, 96)) / windowMsgBuf[3]) + 127;
        windowMsgBuf[6] = window_size;
        windowMsgBuf[7] = (uint8_t)frameRate;

        //memcpy(windowMsgBuf + 8, divergenceArray, 8);
      }
      // Now send the data that we want to send
      if (current_stereoboard_algorithm == SEND_IMAGE) {
        SendImage(current_image_buffer, IMAGE_WIDTH, IMAGE_HEIGHT);
      }
      if (current_stereoboard_algorithm == SEND_DISPARITY_MAP) {
        SendArray(disparity_image_buffer_8bit, IMAGE_WIDTH, IMAGE_HEIGHT);
      }
      if (current_stereoboard_algorithm == SEND_HISTOGRAM) {
        SendArray(histogramBuffer, pixelsPerLine, 1);
      }
      if (current_stereoboard_algorithm == SEND_MATRIX) {
        SendArray(toSendBuffer, MATRIX_WIDTH_BINS, MATRIX_HEIGHT_BINS);
      }
      if (current_stereoboard_algorithm == SEND_WINDOW) {
        SendArray(windowMsgBuf, WINDOWBUFSIZE, 1);
      }
      if (current_stereoboard_algorithm == SEND_DIVERGENCE) {
        SendArray(divergenceArray, 19, 1);
      }
      if (current_stereoboard_algorithm == SEND_COMMANDS || current_stereoboard_algorithm == SEND_FRAMERATE_STEREO) {
        SendCommand(toSendCommand);
      }
      if (current_stereoboard_algorithm == SEND_DELFLY_CORRIDOR) {
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

