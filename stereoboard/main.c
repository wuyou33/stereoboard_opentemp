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
#include <stdint.h>
#include <inttypes.h>

#include "xprintf.h"

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
#include "tmg3993.h" // IR proximity sensor

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
#include "../common/stereoprotocol.h"
#include "forward_velocity_estimator.h"
#include "disparity_map_functions.h"
#include "odometry.h"
#include "learning.h"
#include "VL6180.h"
/********************************************************************/

#define TOTAL_IMAGE_LENGTH IMAGE_WIDTH*IMAGE_HEIGHT;
#define DIVERGENCE_QUALITY_MEASURES_LENGTH 10
// we have a total of 64 KB of space starting at 0x10000000

//uint32_t *integral_image = ((uint32_t *) 0x10000000); // 0x10000000 - 0x1000 FFFF = CCM data RAM  (64kB)
//uint8_t* jpeg_image_buffer_8bit = ((uint8_t*) 0x1000D000); // 0x10000000 - 0x1000 FFFF = CCM data RAM
uint8_t *disparity_image_buffer_8bit = ((uint8_t *) 0x10000000);

// integral_image has size 128 * 96 * 4 = 49152 bytes = C000 in hex
#if defined(WINDOW)
#if 5*(IMAGE_WIDTH * IMAGE_HEIGHT) > 65536
#error "intergral_image can't fit in CCM data RAM"
#endif
uint32_t integral_image = ((uint32_t *)(0x10000000 + (IMAGE_WIDTH *IMAGE_HEIGHT)));
#endif

uint16_t offset_crop = 0;

/** @addtogroup StereoCam
 * @{
 */

/* Private functions ---------------------------------------------------------*/
typedef enum {SEND_TURN_COMMANDS, SEND_COMMANDS, SEND_IMAGE, SEND_DISPARITY_MAP, SEND_FRAMERATE_STEREO, SEND_MATRIX, SEND_DIVERGENCE, SEND_IMAGE_AND_PROXIMITY, SEND_PROXIMITY_AND_ANGLE, SEND_WINDOW, SEND_HISTOGRAM, SEND_DELFLY_CORRIDOR, SEND_FOLLOW_YOU, SEND_SINGLE_DISTANCE, DISPARITY_BASED_VELOCITY, STEREO_VELOCITY, SEND_ROTATIONS, SEND_LEARNING_COLLISIONS, SEND_VL6180} stereoboard_algorithm_type;

//////////////////////////////////////////////////////
// Define which code should be run:
stereoboard_algorithm_type getBoardFunction(void)
{
#if ! (defined(SEND_COMMANDS) || defined(SEND_IMAGE) || defined(SEND_DISPARITY_MAP) || defined(SEND_MATRIX) || defined(SEND_DIVERGENCE) || defined(SEND_WINDOW) || defined(SEND_HISTOGRAM) || defined(SEND_DELFLY_CORRIDOR) || defined(SEND_FOLLOW_YOU) || defined(SEND_SINGLE_DISTANCE) || defined(DISPARITY_BASED_VELOCITY) || defined( STEREO_VELOCITY) || defined( SEND_ROTATIONS) || defined(SEND_IMAGE_AND_PROXIMITY) || defined(SEND_PROXIMITY_AND_ANGLE) || defined(SEND_LEARNING_COLLISIONS) || defined(SEND_VL6180))
  return DEFAULT_BOARD_FUNCTION;
#elif defined(SEND_VL6180)
  return SEND_VL6180;
#elif defined(SEND_ROTATIONS)
  return SEND_ROTATIONS;
#elif defined(DISPARITY_BASED_VELOCITY)
  return DISPARITY_BASED_VELOCITY;
#elif defined( STEREO_VELOCITY)
  return  STEREO_VELOCITY;
#elif defined(SEND_FOLLOW_YOU)
  return SEND_FOLLOW_YOU;
#elif defined(SEND_SINGLE_DISTANCE)
  return SEND_SINGLE_DISTANCE;
#elif defined(SEND_DELFLY_CORRIDOR)
  return SEND_DELFLY_CORRIDOR;
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
#elif defined( SEND_IMAGE_AND_PROXIMITY)
  return  SEND_IMAGE_AND_PROXIMITY;
#elif defined( SEND_PROXIMITY_AND_ANGLE)
  return  SEND_PROXIMITY_AND_ANGLE;
  //Initializing the dynamic parameters and the edge histogram structure
  int rear = 1;
  int front = 0;
#elif defined(SEND_LEARNING_COLLISIONS)
  return SEND_LEARNING_COLLISIONS;
#endif
}

//Initializing all structures and elements for the optical flow algorithm (divergence.c)
struct edgeflow_parameters_t edgeflow_parameters;
struct edgeflow_results_t edgeflow_results;
const int8_t FOVX = 104;   // 60deg = 1.04 rad
const int8_t FOVY = 79;    // 45deg = 0.785 rad

//send array with flow parameters
uint8_t divergenceArray[24];

void getPartOfImage(uint8_t *originalImage, uint8_t *newImage, uint8_t imagePartX, uint8_t imagePartY,
                    uint8_t imagePartWidth, uint8_t imagePartHeight, uint8_t image_width_bytes)
{
//  int indexX;
//  int indexY;
//  int indexNewImage=0;
//  for(indexY=0; indexY < 30; indexY++){
//    for(indexX=0; indexX < imagePartWidth; indexX++){
//      //newImage[indexNewImage++]=originalImage[(imagePartY+indexY)*originalImageWidth*2+(imagePartX*2+indexX)];
//      originalImage[((indexX+imagePartX)*2) + 1 + ((indexY+30) * image_width_bytes)] = 255;
//    }
//
//  }
  originalImage[((imagePartX + 1) * 2) + 1 + ((imagePartY + 1) * image_width_bytes)] = 255;
  originalImage[((imagePartX + 2) * 2) + 1 + ((imagePartY + 1) * image_width_bytes)] = 255;
  originalImage[((imagePartX + 3) * 2) + 1 + ((imagePartY + 1) * image_width_bytes)] = 255;

}

#ifdef WINDOW

#define WINDOWBUFSIZE 8  // 8 for window and 8 for divergence

uint8_t windowMsgBuf[WINDOWBUFSIZE];
uint8_t coordinate[2];
uint8_t window_size;

void window_init()
{
  coordinate[0] = IMAGE_WIDTH / 2;
  coordinate[1] = IMAGE_HEIGHT / 2;
  memset(integral_image, 0, IMAGE_WIDTH * IMAGE_HEIGHT);
}

#endif

void array_pop(float *array, int lengthArray)
{
  int index;
  for (index = 1; index < lengthArray; index++) {
    array[index - 1] = array[index];
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
  int ind;
  // slight waste of memory, if color is not used:
  uint8_t filtered_image[FULL_IMAGE_SIZE / 2];
  for (ind = 0; ind < FULL_IMAGE_SIZE / 2; ind++) {
    filtered_image[ind] = 0;
  }
#endif


#ifndef SUB_SAMPLING
#define SUB_SAMPLING 1
#endif

  uint8_t min_y, max_y;
  uint32_t image_width = IMAGE_WIDTH;
  uint32_t image_height = IMAGE_HEIGHT;

  // for STEREO_VELOCITY:
  uint8_t inv_freq_stereo = 5;
  uint8_t avg_disp_left, avg_disp_right;

  /***********
   * MAIN LOOP
   ***********/

  stereoboard_algorithm_type current_stereoboard_algorithm = getBoardFunction();
  volatile int processed = 0;

#if current_stereoboard_algorithm == SEND_PROXIMITY_AND_ANGLE || current_stereoboard_algorithm == SEND_IMAGE_AND_PROX   // initialize proximity sensor
  TMG3993_Init();
  int16_t prx;
  int16_t ang;
  int16_t prx_east;
  int16_t prx_west;
  uint8_t ang_read = 1;
#endif

#if current_stereoboard_algorithm == SEND_VL6180
  VL6180xInit();
  VL6180xDefautSettings();
#endif

  // Disparity image buffer, initialised with zeros
  //uint8_t disparity_image_buffer_8bit[FULL_IMAGE_SIZE / 2];
  memset(disparity_image_buffer_8bit, 0, FULL_IMAGE_SIZE / 2);

  // Stereo parameters:
  uint32_t disparity_range = 20; // at a distance of 1m, disparity is 7-8. disp = Npix*cam_separation /(2*dist*tan(FOV/2))
  uint32_t disparity_min = 0;
  uint32_t disparity_step = 1;
  uint8_t thr1 = 7;
  uint8_t thr2 = 4;
  uint16_t processed_pixels =
    0; // how many pixels have been considered a local maximum and hence processed by the sparse algorithm
  //uint8_t diff_threshold = 4; // for filtering

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
  uint8_t toSendBuffer[MATRIX_HEIGHT_BINS * MATRIX_WIDTH_BINS];
  uint8_t toSendCommand = 0;

  uint32_t sys_time_prev = sys_time_get();

#ifdef AVG_FREQ
  uint32_t freq_counter = 0;
#endif

  volatile int32_t frameRate = 0;

  uint8_t histogramBuffer[pixelsPerLine];
  uint8_t histogramBufferX[pixelsPerLine];

#ifdef WINDOW

  // Initialize window
  window_init();

#endif

  // Settings for SEND_TURN_COMMANDS
  uint8_t n_disp_bins = 6;
  uint32_t disparities[n_disp_bins];
  uint8_t RESOLUTION = 100;

  // Settings and initialisation for FOLLOW_YOU
  uint16_t feature_count_limit = 10;
  int no_prev_measurment = 0;
  uint8_t feature_image_locations [3 * feature_count_limit];
  //float feature_XYZ_locations[3*feature_count_limit];
  volatile uint16_t nr_of_features = 0;
  uint8_t target_location [3];

#if ODOMETRY
  // Settings for SEND_ROTATIONS
  uint16_t features_max_number = 300;
  uint16_t features_TOTAL_number = 50;
  uint8_t feature_window_size_2 = 2; // one sided window size (total size is *2 + 1)
  float rotation_step_size = 0.03;
  int rotation_step_number = 7;

  uint8_t feature_image_coordinates [3 * features_max_number];
  uint16_t features_ROT_number;
  uint8_t feature_window_size = (2 * feature_window_size_2) + 1;
  q15_t feature_window_data [feature_window_size * feature_window_size * features_TOTAL_number *
                             2]; // values of pixels in feature window
  int number_of_rotations = rotation_step_number * rotation_step_number * rotation_step_number;
  float32_t rotation_coefficients [number_of_rotations * 9]; // 9 coefficients per rotation
  rotation_coefficients[0] = 0;
#endif

  if (current_stereoboard_algorithm == SEND_LEARNING_COLLISIONS) {
    learning_collisions_init();
  }


  uint16_t features_max_number = 300;
  uint8_t feature_image_coordinates [3 * features_max_number];



  int pos_y = 0;

  // Stereo communication input protocol
  uint8_t ser_read_buf[STEREO_BUF_SIZE];           // circular buffer for incoming data
  uint8_t msg_buf[STEREO_BUF_SIZE];         // define local data
  typedef struct {
    uint8_t len;
    uint8_t height;
    uint8_t *data;
    uint8_t data_new;
  } uint8array;
  uint8array stereocam_data = {.len = 0, .data = msg_buf, .data_new = 0, .height = 0}; // buffer used to contain image without line endings
  uint16_t insert_loc, extract_loc, msg_start;   // place holders for buffer read and write
  insert_loc = 0;
  extract_loc = 0;
  msg_start = 0;

  // Set the histogram type to what was defined in the
  horizontal_histogram_type histogram_type;
  if (HISTOGRAM_FUNCTION == HISTOGRAM_FOLLOW_ME_DRONE) {
    histogram_type = FOLLOW_ME_HISTOGRAM;
  } else {
    histogram_type = AVOID_ME_HISTOGRAM;
  }

  // Disparity based velocity estimation variables
  uint8_t maxDispFound = 0;
  int disparity_velocity_step = 0;

  // initialize divergence
  divergence_init(&edgeflow_parameters, &edgeflow_results, FOVX, FOVY, IMAGE_WIDTH, IMAGE_HEIGHT, USE_MONOCAM);
  led_clear();
  uint8_t quality_measures_index;
  for (quality_measures_index = 0; quality_measures_index < DIVERGENCE_QUALITY_MEASURES_LENGTH;
       quality_measures_index++) {
    edgeflow_results.quality_measures_edgeflow[quality_measures_index++] = 0;
  }

  while (1) {
    if (current_stereoboard_algorithm == SEND_VL6180) {
      uint8_t dist = getDistance();
      SendCommandNumber((uint8_t) dist);

/*    if (current_stereoboard_algorithm == SEND_PROXIMITY_AND_ANGLE) {
      // Read proximity sensor
      prx = TMG3993_Read_Proximity();
      prx_east = TMG3993_Read_Proximity_East();
      prx_west = TMG3993_Read_Proximity_West();
      ang = TMG3993_Read_Angle();

      // Check if obstacle is near
      if ((prx > 25) && (ang_read == 1)) {
        led_set();
        //ang = TMG3993_Read_Angle();
        ang_read = 0;
      }
      if (prx <= 25) {
        led_clear();
        ang_read = 1;
      }

      // Print proximity and angle measurements to UART
      char buffer[20];
      xsprintf(buffer, "Proximity: %d, Angle: %d, East: %d, West: %d\r\n", prx, ang, prx_east, prx_west);
      while (UsartTx(buffer, 55) == 0)
        ;
*/
    } else {
      camera_snapshot();

#if defined(LARGE_IMAGE) || defined(CROPPING)
      offset_crop += 60;
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

      // compute run frequency
#ifdef AVG_FREQ
      freq_counter++;
      if (get_timer_interval(sys_time_prev) >= TIMER_TICKS_PER_SEC) { // clock at 2kHz
        frameRate = freq_counter * get_timer_interval(sys_time_prev) / TIMER_TICKS_PER_SEC; // in Hz
        freq_counter = 0;
        sys_time_prev = sys_time_get();
      }
#else
      frameRate = TIMER_TICKS_PER_SEC / get_timer_interval(sys_time_prev); // in Hz
      sys_time_prev = sys_time_get();
#endif
      // Read from other device with the stereo communication protocol.
      while (UsartCh() && stereoprot_add(insert_loc, 1, STEREO_BUF_SIZE) != extract_loc) {

        uint16_t length = STEREO_BUF_SIZE;
        if (handleStereoPackage(UsartRx(), length, &insert_loc, &extract_loc, &msg_start, msg_buf, ser_read_buf,
                                &stereocam_data.data_new, &stereocam_data.len, &stereocam_data.height)) {
          break;
        }
      }

      // New frame code: Vertical blanking = ON

      // Calculate the disparity map, only when we need it
      if (current_stereoboard_algorithm == SEND_DISPARITY_MAP || current_stereoboard_algorithm == SEND_MATRIX
          || current_stereoboard_algorithm == SEND_COMMANDS || current_stereoboard_algorithm == SEND_TURN_COMMANDS ||
          current_stereoboard_algorithm == SEND_FRAMERATE_STEREO || current_stereoboard_algorithm == SEND_WINDOW ||
          current_stereoboard_algorithm == SEND_HISTOGRAM || current_stereoboard_algorithm == SEND_DELFLY_CORRIDOR
          || current_stereoboard_algorithm == SEND_SINGLE_DISTANCE || current_stereoboard_algorithm == DISPARITY_BASED_VELOCITY
          || (current_stereoboard_algorithm == STEREO_VELOCITY && !SUB_SAMPLING)) {

        if (current_stereoboard_algorithm != STEREO_VELOCITY || frame_counter % inv_freq_stereo == 0) {
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
            processed_pixels = stereo_vision_sparse_block_two_sided(current_image_buffer,
                               disparity_image_buffer_8bit, image_width, image_height,
                               disparity_min, disparity_range, disparity_step, thr1, thr2,
                               min_y, max_y);
          }
        }
      }

      if (current_stereoboard_algorithm == STEREO_VELOCITY && SUB_SAMPLING) {
        min_y = 0;
        max_y = 96;
        nr_of_features = stereo_vision_sparse_block_features(current_image_buffer,
                         disparity_image_buffer_8bit, feature_image_coordinates, features_max_number, image_width, image_height,
                         disparity_min, disparity_range, disparity_step, thr1, thr2,
                         min_y, max_y);
      }

      if (current_stereoboard_algorithm == SEND_HISTOGRAM || current_stereoboard_algorithm == SEND_DELFLY_CORRIDOR) {

        histogram_x_direction(disparity_image_buffer_8bit, histogramBuffer, histogram_type, blackBorderSize, pixelsPerLine,
                              image_height);

        if (current_stereoboard_algorithm == SEND_DELFLY_CORRIDOR) {
          toSendCommand = calculateHeadingFromHistogram(histogramBuffer);
        }
      }

      // determine phase of flight
      if (current_stereoboard_algorithm == SEND_COMMANDS || current_stereoboard_algorithm == SEND_FRAMERATE_STEREO) {

        int disparities_high = 0;
        disparities_high =  evaluate_disparities_droplet(disparity_image_buffer_8bit, image_width, image_height, 30);
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
      if (current_stereoboard_algorithm == SEND_MATRIX) {
        // Initialise matrixbuffer and sendbuffer by setting all values back to zero.
        memset(toSendBuffer, 0, sizeof toSendBuffer);
        // Create the distance matrix by summing pixels per bin
        calculateDistanceMatrix(disparity_image_buffer_8bit, blackBorderSize,
                                pixelsPerLine, widthPerBin, heightPerBin, toSendBuffer, disparity_range);
      }
      if (current_stereoboard_algorithm == SEND_SINGLE_DISTANCE || current_stereoboard_algorithm == STEREO_VELOCITY
          || current_stereoboard_algorithm == DISPARITY_BASED_VELOCITY) {
        // Determine the maximum disparity using the disparity map
        if (!SUB_SAMPLING) {
          // Determine the maximum disparity using the disparity map
          histogram_z_direction(disparity_image_buffer_8bit, histogramBuffer, pixelsPerLine, image_height);
          histogram_x_direction(disparity_image_buffer_8bit, histogramBufferX, histogram_type, blackBorderSize, pixelsPerLine,
                                image_height);
          get_average_left_right(histogramBufferX, &avg_disp_left, &avg_disp_right, pixelsPerLine);

        } else {
          histogram_z_direction_features(feature_image_coordinates, histogramBuffer, nr_of_features, pixelsPerLine);
          get_average_left_right_features(feature_image_coordinates, nr_of_features, &avg_disp_left, &avg_disp_right,
                                          pixelsPerLine);
        }
        int amountDisparitiesRejected = processed_pixels / 8;
        int histogramIndex = pixelsPerLine;
        int amountDisparitiesCount = 0;
        maxDispFound = 0;
        for (histogramIndex = pixelsPerLine - 2; histogramIndex > 0; histogramIndex--) {
          amountDisparitiesCount += histogramBuffer[histogramIndex];
          if (amountDisparitiesCount > amountDisparitiesRejected) {
            maxDispFound = histogramIndex;
            break;
          }
        }
      }

      if (current_stereoboard_algorithm == DISPARITY_BASED_VELOCITY) {
        float  BASELINE_STEREO_MM = 60.0;
        float BRANDSPUNTSAFSTAND_STEREO = 118.0 * 6.0 * 2.0;
        disparity_velocity_step += 1;
        // for now maximum disparity, later the average:

        float dist = 5.0;
        if (maxDispFound > 0) {
          dist = ((BASELINE_STEREO_MM * BRANDSPUNTSAFSTAND_STEREO / (float)maxDispFound)) / 1000;
        }
        calculateForwardVelocity(dist, 0.65, 5, 5);
      }

      // compute and send divergence
      if (current_stereoboard_algorithm == SEND_DIVERGENCE || current_stereoboard_algorithm == STEREO_VELOCITY) {
        if (current_stereoboard_algorithm == SEND_DIVERGENCE) {
          //led_toggle();
        }

        int16_t pitch = 0;
        int16_t roll = 0;

        // calculate the edge flow
        divergence_total(divergenceArray, current_image_buffer, &edgeflow_parameters, &edgeflow_results, sys_time_get(), roll ,
                         pitch);

        stereocam_data.data_new = 0;
      }

      // compute and send window detection parameters
      if (current_stereoboard_algorithm == SEND_WINDOW) {
        // XPOS, YPOS, RESPONSE, DISP_SUM, DISP_HOR, DISP_VERT

#ifdef WINDOW
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

#endif

        //memcpy(windowMsgBuf + 8, divergenceArray, 8);
      }

      if (current_stereoboard_algorithm == SEND_FOLLOW_YOU) {

        uint8_t search_window = 10;
        nr_of_features = 0;

        memset(disparity_image_buffer_8bit, 0, FULL_IMAGE_SIZE / 2);

        // if no previous measurement, search in the whole image for nearby person
        if (no_prev_measurment == 0) {
          min_y = 0;
          max_y = image_height - 1;

        } else { // only search in range of previous measurement
          min_y = MAX(0, pos_y - search_window);
          max_y = MIN(image_height, pos_y - search_window);
        }

        // run stereo vision algorithm
        stereo_vision_sparse_block_fast_version(current_image_buffer,
                                                disparity_image_buffer_8bit, image_width, image_height,
                                                disparity_min, disparity_range, disparity_step, thr1, thr2,
                                                min_y, max_y);

        // subtract feature locations from disparitymap in specified range
        //nr_of_features = getFeatureImageLocations(disparity_image_buffer_8bit, feature_image_locations, image_width, image_height, min_y, max_y, feature_count_limit);
        nr_of_features = getFeatureImageLocations(current_image_buffer, disparity_image_buffer_8bit, feature_image_locations,
                         target_location, image_width, image_height, min_y, max_y, feature_count_limit);

        // visualize features
        if (nr_of_features == feature_count_limit) {
          //nr_of_features = visualizeBlobImageLocation(current_image_buffer, feature_image_locations, target_location, nr_of_features, image_width, feature_count_limit);
          //visualizeFeatureImageLocations(current_image_buffer, feature_image_locations, nr_of_features, image_width, feature_count_limit);

        }
        // rotate and convert image points to real world points
        //getFeatureXYZLocations(feature_image_locations, feature_XYZ_locations, nr_of_features, image_width, image_height);


      }

#if ODOMETRY

      if (current_stereoboard_algorithm == SEND_ROTATIONS) {

        // precompute grid-rotation-coefficients
        if (rotation_coefficients[0] == 0) {
          precompute_rotation_coefficients(rotation_coefficients, rotation_step_size, rotation_step_number, number_of_rotations);
        }

        min_y = 0;
        max_y = image_height - 1;

        sys_time_prev = sys_time_get();

        if (frame_counter > (30 * 5)) {
          if (nr_of_features == 0)  // after 5 seconds obtain disparity map once

          {

            memset(disparity_image_buffer_8bit, 0, FULL_IMAGE_SIZE / 2);

            // run stereo vision algorithm
            nr_of_features = stereo_vision_sparse_block_features(current_image_buffer,
                             disparity_image_buffer_8bit, feature_image_coordinates, features_max_number, image_width, image_height,
                             disparity_min, disparity_range, disparity_step, thr1, thr2,
                             min_y, max_y);

            if (nr_of_features > 50) {

              features_ROT_number = 5;
              //features_TOTAL_number = 50;
              //feature_window = 2; // one sided window size (total size is *2 + 1)

              odometry_select_features(current_image_buffer, feature_image_coordinates,
                                       nr_of_features, features_TOTAL_number, image_width, image_height);

              odometry_extract_features(current_image_buffer, feature_window_data, feature_image_coordinates,
                                        features_TOTAL_number, features_ROT_number, image_width, image_height, feature_window_size_2);
            }
            //led_toggle();

          } else { // after obtaining disparity map and good features, run odometry
            odometry_translate_and_match_features(current_image_buffer, feature_window_data, feature_image_coordinates,
                                                  features_ROT_number, feature_window_size_2, rotation_coefficients, number_of_rotations, image_width);
            //led_toggle();
          }
        }
      }
#endif

      if (current_stereoboard_algorithm == SEND_LEARNING_COLLISIONS) {

        sys_time_prev = sys_time_get();
        learning_collisions_run(current_image_buffer);
        frameRate = TIMER_TICKS_PER_SEC / get_timer_interval(sys_time_prev); // in Hz
      }


      // Now send the data that we want to send
      if (current_stereoboard_algorithm == SEND_IMAGE) {
#if SET_LINE_NUMBERS
        uint8_t horizontalLine;
        uint8_t copyOfThing[IMAGE_WIDTH * IMAGE_HEIGHT * 2];
        int someIndexImage = 0;
        for (; someIndexImage < IMAGE_WIDTH * IMAGE_HEIGHT * 2; someIndexImage++) {
          copyOfThing[someIndexImage] = current_image_buffer[someIndexImage];
        }
        setLineNumbersImage(&copyOfThing, IMAGE_WIDTH, IMAGE_HEIGHT);

        SendImage(copyOfThing, IMAGE_WIDTH, IMAGE_HEIGHT);
#else
        SendImage(current_image_buffer, IMAGE_WIDTH, IMAGE_HEIGHT);
#endif
      }
      if (current_stereoboard_algorithm == SEND_DISPARITY_MAP) {
#if SET_LINE_NUMBERS
        setLineNumbers(&disparity_image_buffer_8bit, IMAGE_WIDTH, IMAGE_HEIGHT);
#endif
        SendArray(disparity_image_buffer_8bit, IMAGE_WIDTH, IMAGE_HEIGHT);
      }
      if (current_stereoboard_algorithm == SEND_HISTOGRAM) {
        SendArray(histogramBuffer, pixelsPerLine, 1);
      }
      if (current_stereoboard_algorithm == SEND_MATRIX) {
        SendArray(toSendBuffer, MATRIX_WIDTH_BINS, MATRIX_HEIGHT_BINS);
      }
      if (current_stereoboard_algorithm == SEND_SINGLE_DISTANCE) {
        uint8_t toSendNow[1];
        led_clear();
        if (maxDispFound > 30) {
          led_set();
        }
        toSendNow[0] = maxDispFound;
        SendArray(toSendNow, 1, 1);
      }

#ifdef WINDOW

      if (current_stereoboard_algorithm == SEND_WINDOW) {
        SendArray(windowMsgBuf, WINDOWBUFSIZE, 1);
      }
#endif

      if (current_stereoboard_algorithm == STEREO_VELOCITY) {
        divergenceArray[4] = maxDispFound;
        int disparities_high =  evaluate_disparities_droplet(disparity_image_buffer_8bit, image_width, image_height, 80);
        if (disparities_high > 200) {
          divergenceArray[5] = 200;
        } else {
          divergenceArray[5] = (uint8_t)disparities_high;
        }
        divergenceArray[6] = (uint8_t)(processed_pixels / 100);
        divergenceArray[10] = avg_disp_left;
        divergenceArray[11] = avg_disp_right;
        led_clear();
        if (maxDispFound > 22) {
          led_set();
        }
        SendArray(divergenceArray, 23, 1);
      }

      if (current_stereoboard_algorithm == SEND_DIVERGENCE) {
        SendArray(divergenceArray, 25, 1);
      }
      if (current_stereoboard_algorithm == SEND_COMMANDS || current_stereoboard_algorithm == SEND_FRAMERATE_STEREO) {
        SendCommand(toSendCommand);
      }
      if (current_stereoboard_algorithm == SEND_DELFLY_CORRIDOR) {
        SendCommand(toSendCommand);
      }
      if (current_stereoboard_algorithm == SEND_FOLLOW_YOU) {
//        SendImage(current_image_buffer, IMAGE_WIDTH, IMAGE_HEIGHT); // show image with target-cross
        //SendArray(disparity_image_buffer_8bit, IMAGE_WIDTH, IMAGE_HEIGHT); // show disparity map
        SendArray(target_location, 3, 1); // send 3D location of target
      }
      if (current_stereoboard_algorithm == SEND_ROTATIONS) {
        //SendArray(disparity_image_buffer_8bit, IMAGE_WIDTH, IMAGE_HEIGHT); // show disparity map
      }
      if (current_stereoboard_algorithm == SEND_LEARNING_COLLISIONS) {
        //SendImage(current_image_buffer, IMAGE_WIDTH, IMAGE_HEIGHT);
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

