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
#include "main.h"

#include "std.h"
#include <math.h>
#include <stdint.h>
#include <inttypes.h>

#include "meanshift.h"
#include "xprintf.h"
#include "data_types.h"
// include system files
#include "arm_math.h"
#include "stm32f4xx_conf.h"
#include "sys_time.h"
#include "stereo_math.h"
#include "image.h"

// include utility headers
#include "led.h"
#include "commands.h"

// include camera headers
#include "camera_type.h"
#include "cpld.h"
#include "dcmi.h"

// Other sensors
//#include "hmc5883.h"
#include "tmg3993.h" // IR proximity sensor
#include "vl6180.h"

// include setttings
#include "main_parameters.h"
#include "../multigaze/stereoboard_parameters.h"

// include coms
#include "usart.h"
#include "usb.h"

#include "stereo_protocol.h"
#include "utils.h"
#include "encoding/jpeg.h"
#include "raw_digital_video_stream.h"

// include functions headers
#include "distance_matrix.h"
#include "edgeflow.h"
#include "droplet_algorithm.h"
#include "filter_color.h"
#include "stereo_vision.h"
#include "window_detection.h"
#include "forward_velocity_estimator.h"
#include "disparity_map_functions.h"
#include "odometry.h"
#include "learning.h"
#include "dronerace_gate_detector.h"
#include "gate_detection.h"
#include "gate_detection_fp.h"
/********************************************************************/

//uint8_t __ccmram jpeg_image_buffer_8bit[FULL_IMAGE_SIZE];  // todo how big should this actually be?
uint8_t __ccmram disparity_image_buffer[FULL_IMAGE_SIZE / 2];

// integral_image has size 128 * 96 * 4 = 49152 bytes = C000 in hex
#if defined(WINDOW)
uint32_t __ccmram integral_image[FULL_IMAGE_SIZE / 2];
#endif

uint16_t offset_crop = 0;
/** @addtogroup StereoCam
 * @{
 */

/* Private functions ---------------------------------------------------------*/
typedef enum {SEND_TURN_COMMANDS, SEND_COMMANDS, SEND_IMAGE, SEND_DISPARITY_MAP, SEND_FRAMERATE_STEREO, SEND_MATRIX, SEND_EDGEFLOW, SEND_IMAGE_AND_PROXIMITY, SEND_PROXIMITY_AND_ANGLE, SEND_WINDOW, SEND_HISTOGRAM, SEND_DELFLY_CORRIDOR, SEND_FOLLOW_YOU, SEND_SINGLE_DISTANCE, DISPARITY_BASED_VELOCITY, STEREO_VELOCITY, SEND_ROTATIONS, SEND_LEARNING_COLLISIONS,
              SEND_MEANSHIFT, SEND_VL6180, DRONERACE, SEND_NONE
             } stereoboard_algorithm_type;

//////////////////////////////////////////////////////
// Define which code should be run:
stereoboard_algorithm_type getBoardFunction(void)
{
#if defined(SEND_COMMANDS)
  return SEND_COMMANDS;
#elif defined(DRONERACE)
  return DRONERACE;
#elif defined(SEND_IMAGE)
  return SEND_IMAGE;
#elif defined(SEND_DISPARITY_MAP)
  return SEND_DISPARITY_MAP;
#elif defined(SEND_MATRIX)
  return SEND_MATRIX;
#elif defined(SEND_EDGEFLOW)
  return SEND_EDGEFLOW;
#elif defined(SEND_WINDOW)
  return SEND_WINDOW;
#elif defined(SEND_HISTOGRAM)
  return SEND_HISTOGRAM;
#elif defined(SEND_DELFLY_CORRIDOR)
  return SEND_DELFLY_CORRIDOR;
#elif defined(SEND_FOLLOW_YOU)
  return SEND_FOLLOW_YOU;
#elif defined(SEND_SINGLE_DISTANCE)
  return SEND_SINGLE_DISTANCE;
#elif defined(DISPARITY_BASED_VELOCITY)
  return DISPARITY_BASED_VELOCITY;
#elif defined( STEREO_VELOCITY)
  return  STEREO_VELOCITY;
#elif defined(SEND_ROTATIONS)
  return SEND_ROTATIONS;
#elif defined( SEND_IMAGE_AND_PROXIMITY)
  return  SEND_IMAGE_AND_PROXIMITY;
#elif defined( SEND_PROXIMITY_AND_ANGLE)
  return  SEND_PROXIMITY_AND_ANGLE;
  //Initializing the dynamic parameters and the edge histogram structure
  int rear = 1;
  int front = 0;
#elif defined(SEND_LEARNING_COLLISIONS)
  return SEND_LEARNING_COLLISIONS;
#elif defined(SEND_VL6180)
  return SEND_VL6180;
#elif defined(SEND_MEANSHIFT)
  return SEND_MEANSHIFT;
#elif defined(SEND_FRAMERATE_STEREO)
  return SEND_FRAMERATE_STEREO;
#else
  return DEFAULT_BOARD_FUNCTION;
#endif
}

//Initializing all structures and elements for the optical flow algorithm (divergence.c)
struct edgeflow_parameters_t edgeflow_parameters;
struct edgeflow_results_t edgeflow_results;

//send array with flow parameters
#ifdef EDGEFLOW_DEBUG
uint8_t edgeflowArray[128 * 5];
#else
uint8_t edgeflowArray[25];
#endif

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


struct image_t current_image_pair = {
  .w = IMAGE_WIDTH,
  .h = IMAGE_HEIGHT,
  .buf_size = FULL_IMAGE_SIZE,
  .type = IMAGE_GRAYSCALE
};  // todo this should probably be dependent on the cpld config

struct image_t disparity_image = {
  .w = IMAGE_WIDTH,
  .h = IMAGE_HEIGHT,
  .buf_size = FULL_IMAGE_SIZE / 2,
  .type = IMAGE_GRAYSCALE
};

// Timing counters
uint32_t freq_counter = 0;
uint32_t frame_dt = 0;
uint32_t frame_rate = 0;

/**
 * @brief  Main program
 * @param  None
 * @retval None
 */
int main(void)
{
  frame_processed = frame_counter;

  disparity_image.buf = disparity_image_buffer;

#ifdef NEW_MAIN
  init_project();
#endif

  // init buffers
  memset((uint8_t *)disparity_image.buf, 0, disparity_image.buf_size);

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
  // Initialize the camera
  camera_init();
  sys_time_init();

#ifndef SUB_SAMPLING
#define SUB_SAMPLING 1
#endif

  uint8_t min_y = 0, max_y = IMAGE_HEIGHT;    // initialise with default full image processing
  uint32_t image_width = IMAGE_WIDTH;
  uint32_t image_height = IMAGE_HEIGHT;

  // for STEREO_VELOCITY:
  uint8_t inv_freq_stereo = 5;
  uint8_t avg_disp_left, avg_disp_right;

  /***********
   * MAIN LOOP
   ***********/

  stereoboard_algorithm_type current_stereoboard_algorithm = getBoardFunction();

#if current_stereoboard_algorithm == SEND_PROXIMITY_AND_ANGLE || current_stereoboard_algorithm == SEND_IMAGE_AND_PROX   // initialize proximity sensor
  TMG3993_Init();
#endif

#if current_stereoboard_algorithm == SEND_VL6180
  VL6180xInit();
  VL6180xDefautSettings();
#endif

  // Disparity image buffer, initialised with zeros
  memset(disparity_image_buffer, 0, FULL_IMAGE_SIZE / 2);

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
  uint8_t pixelsPerLine = 128;  // usse disparity_image.w and .h everywhere instad of pixelsPerLine/Column?
  uint8_t pixelsPerColumn = 96;

  uint8_t widthPerBin = (pixelsPerLine - 2 * blackBorderSize)
                        / MATRIX_WIDTH_BINS;
  uint8_t heightPerBin = pixelsPerColumn / MATRIX_HEIGHT_BINS;

  uint8_t toSendCommand = 0;

  uint32_t sys_time_prev = sys_time_get();
  uint32_t sys_time_prev_long = sys_time_get();

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

  uint16_t features_max_number = 500;
  uint8_t feature_image_coordinates [3 * features_max_number];

  int pos_y = 0;

  // Stereo communication input protocol
  uint8_t ser_read_buf[STEREO_BUF_SIZE];           // circular buffer for incoming data
  uint8_t msg_buf[STEREO_BUF_SIZE];         // define local data
  struct uint8array {
    uint8_t len;
    uint8_t height;
    uint8_t *data;
    uint8_t data_new;
  };
  struct uint8array stereocam_data = {.len = 0, .data = msg_buf, .data_new = 0, .height = 0}; // buffer used to contain image without line endings
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

  // DRONERACE:
  float x_center = 64;
  float y_center = 48;
  float radius = 50;
  float x0 = x_center;
  float y0 = y_center;
  float size0 = radius;
  float fitness = 0.4f;
  q15_t x_center_fp = 64;
  q15_t y_center_fp = 48;
  q15_t radius_fp = 50;
  q15_t fitness_fp = 4;
  uint8_t dronerace_message[8]; // the above + frame rate
  float feature_list_f [4 * features_max_number];

  // variable for making a sub-pixel disparity histogram:
  q15_t sub_disp_histogram[disparity_range * RESOLUTION_FACTOR];

  // Disparity based velocity estimation variables
  uint8_t maxDispFound = 0;
  int disparity_velocity_step = 0;

  // initialize edgeflow
  edgeflow_init(&edgeflow_parameters, &edgeflow_results, IMAGE_WIDTH, IMAGE_HEIGHT, USE_MONOCAM);
// led_clear();
  uint8_t quality_measures_index;
  for (quality_measures_index = 0; quality_measures_index < DIV_QUALITY_LENGTH;
       quality_measures_index++) {
    edgeflow_results.quality_meas[quality_measures_index++] = 0;
  }

  // main loop
  while (1) {
    if (current_stereoboard_algorithm == SEND_VL6180) {
      uint8_t dist = getDistance();
      SendCommandNumber((uint8_t) dist);
      led_toggle();
    } else {

#if defined(LARGE_IMAGE) || defined(CROPPING)
      offset_crop += 60;
      if (offset_crop == 480) {
        offset_crop = 0;
      }
      camera_crop(offset_crop);
#endif

      // wait for new frame
      current_image_pair.buf = camera_wait_for_frame();
      edgeflow_results.edge_hist[edgeflow_results.current_frame_nr].frame_time = sys_time_get();

#ifdef NEW_MAIN
      run_project();
#endif

      // compute run frequency
      freq_counter++;
      if (get_timer_interval(sys_time_prev_long) >= TIMER_TICKS_PER_SEC) { // clock at 2kHz
        frame_rate = freq_counter * get_timer_interval(sys_time_prev_long) / TIMER_TICKS_PER_SEC; // in Hz
        freq_counter = 0;
        sys_time_prev_long = sys_time_get();
      }
      frame_dt = 1000 * get_timer_interval(sys_time_prev) / TIMER_TICKS_PER_SEC;
      sys_time_prev = sys_time_get();

      // Read from other device with the stereo communication protocol.
      while (UsartCh() && stereoprot_add(insert_loc, 1, STEREO_BUF_SIZE) != extract_loc) {
        if (handleStereoPackage(UsartRx(), STEREO_BUF_SIZE, &insert_loc, &extract_loc, &msg_start, msg_buf, ser_read_buf,
                                &stereocam_data.data_new, &stereocam_data.len, &stereocam_data.height)) {
        }
      }

      // New frame code: Vertical blanking = ON

      // Calculate the disparity map, only when we need it
      if (current_stereoboard_algorithm == SEND_DISPARITY_MAP || current_stereoboard_algorithm == SEND_MEANSHIFT
          || current_stereoboard_algorithm == SEND_MATRIX
          || current_stereoboard_algorithm == SEND_COMMANDS || current_stereoboard_algorithm == SEND_TURN_COMMANDS ||
          current_stereoboard_algorithm == SEND_FRAMERATE_STEREO || current_stereoboard_algorithm == SEND_WINDOW ||
          current_stereoboard_algorithm == SEND_HISTOGRAM || current_stereoboard_algorithm == SEND_DELFLY_CORRIDOR
          || current_stereoboard_algorithm == SEND_SINGLE_DISTANCE || current_stereoboard_algorithm == DISPARITY_BASED_VELOCITY
          || (current_stereoboard_algorithm == STEREO_VELOCITY && !SUB_SAMPLING)) {

        // If we do STEREO_VELOCITY we only determine disparities from time to time:
        if (current_stereoboard_algorithm != STEREO_VELOCITY || frame_counter % inv_freq_stereo == 0) {
          // Determine disparities:
          min_y = 0;
          max_y = 96;
          memset((uint8_t *)disparity_image.buf, 0, FULL_IMAGE_SIZE / 2);

          if (STEREO_ALGORITHM) {
            stereo_vision_Kirk((uint8_t *)current_image_pair.buf,
                               (uint8_t *)disparity_image.buf, image_width, image_height,
                               disparity_min, disparity_range, disparity_step, thr1, thr2,
                               min_y, max_y);
          } else {
            processed_pixels = stereo_vision_sparse_block_two_sided((uint8_t *)current_image_pair.buf,
                               (uint8_t *)disparity_image.buf, image_width, image_height,
                               disparity_min, disparity_range, disparity_step, thr1, thr2,
                               min_y, max_y, sub_disp_histogram);
          }
        }
      }

      if (current_stereoboard_algorithm == STEREO_VELOCITY && SUB_SAMPLING) {
        min_y = 0;
        max_y = 96;
        nr_of_features = stereo_vision_sparse_block_features((uint8_t *)current_image_pair.buf,
                         (uint8_t *)disparity_image.buf, feature_image_coordinates, features_max_number, image_width, image_height,
                         disparity_min, disparity_range, disparity_step, thr1, thr2,
                         min_y, max_y);
      }
      if (current_stereoboard_algorithm == SEND_MEANSHIFT) {
        run_meanshift(&disparity_image);
      }
      if (current_stereoboard_algorithm == SEND_HISTOGRAM || current_stereoboard_algorithm == SEND_DELFLY_CORRIDOR) {

        histogram_x_direction((uint8_t *)disparity_image.buf, histogramBuffer, histogram_type, blackBorderSize, pixelsPerLine,
                              image_height);

        if (current_stereoboard_algorithm == SEND_DELFLY_CORRIDOR) {
          toSendCommand = calculateHeadingFromHistogram(histogramBuffer);
        }
      }

      if (current_stereoboard_algorithm == DRONERACE) {
        //First run specific stereo algorithm that filters away background disparity values
        min_y = 0;
        max_y = 96;
        memset((uint8_t *)disparity_image.buf, 0, IMAGE_WIDTH * IMAGE_HEIGHT);
        processed_pixels = stereo_vision_sparse_block_two_sided_features((uint8_t *)current_image_pair.buf,
                           (uint8_t *)disparity_image.buf, feature_list_f, features_max_number,
                           image_width, image_height, disparity_min, disparity_range, disparity_step,
                           thr1, thr2, min_y, max_y, sub_disp_histogram);
        //SendArray((uint8_t*)disparity_image.buf, IMAGE_WIDTH, IMAGE_HEIGHT);

        int initialize_fit_with_pars = 0;
        int FP = 0;
        int min_sub_disparity = disparity_range * RESOLUTION_FACTOR - 1;
        int sum_points = 0;
        uint32_t mean_disparity = 0;
        while ((min_sub_disparity > 0) && (sum_points < MAX_POINTS)) {
          min_sub_disparity--; // TODO: don't we skip the first value like this?
          sum_points += sub_disp_histogram[min_sub_disparity];
          mean_disparity += sub_disp_histogram[min_sub_disparity] * min_sub_disparity;
        }
        mean_disparity /= sum_points;
        if (sum_points > MAX_POINTS && sub_disp_histogram[min_sub_disparity] < MAX_POINTS / 2) {
          // we should take one sub-disparity higher, as else we supersede the maximum number of points:
          // however, if the number of points in the last bin is substantial, we rely on the cut-off in the
          // convert disparity map function.
          min_sub_disparity++;
        }
        // potentially enforce a minimum require disparity:
        int enforced_min = 12;
        min_sub_disparity = min_sub_disparity > enforced_min ? min_sub_disparity : enforced_min;

        if (!FP) {
          gate_detection(&disparity_image, &x_center, &y_center, &radius, &fitness, &x0, &y0, &size0, min_sub_disparity);
        } else {
          // fixed point implementation:
          //int initialize_fit_with_pars = 0;
          gate_detection_fp(&disparity_image, &x_center_fp, &y_center_fp, &radius_fp, &fitness_fp, initialize_fit_with_pars, min_sub_disparity);
          x_center = (float) x_center_fp;
          y_center = (float) y_center_fp;
          radius = (float) radius_fp;
          fitness = ((float) fitness_fp) / FITNESS_RESOLUTION;
        }

        // fit parameters, where the gate is likely to be:
        dronerace_message[0] = (uint8_t) x_center; // TODO: what if these are outside of the image?
        dronerace_message[1] = (uint8_t) y_center; // what if these are outside of the image?
        dronerace_message[2] = (uint8_t) radius;
        // fitness of the fit - lower is better:
        dronerace_message[3] = (uint8_t)(100 * fitness);
        // what is our update rate?
        dronerace_message[4] = (uint8_t) 1000 / frame_dt;
        // these last three message elements indicate where the closest obstacle roughly is:
        dronerace_message[5] = (uint8_t) x0;
        dronerace_message[6] = (uint8_t) y0;
        dronerace_message[7] = (uint8_t)(mean_disparity / RESOLUTION_FACTOR);

        // send disparity image:
        //SendArray((uint8_t*)disparity_image.buf, IMAGE_WIDTH, IMAGE_HEIGHT);

        // send message:
        // Note:
        // In the project file, the baud rate should be changed
        // For max speed, the drawing functions in gate_detection should be switched off (GRAPHICS, GRAPHICS_FP)
        SendArray(dronerace_message, 5, 1);

      }
      // determine phase of flight
      if (current_stereoboard_algorithm == SEND_COMMANDS || current_stereoboard_algorithm == SEND_FRAMERATE_STEREO) {

        uint32_t disparities_high = 0;
        uint32_t count_disps_left = 0;
        uint32_t count_disps_right = 0;
        uint32_t hist_obs_sum = 0;

        //disparities_high =  evaluate_disparities_droplet(disparity_image.image, image_width, image_height, 30);
        disparities_high = evaluate_disparities_droplet_low_texture(&disparity_image, &count_disps_left, &count_disps_right, &hist_obs_sum);
        //current_phase = run_droplet_algorithm(disparities_high, processed_pixels);
        current_phase = run_droplet_algorithm_low_texture(disparities_high, processed_pixels, hist_obs_sum, count_disps_left, count_disps_right);

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
        toSendCommand = (uint8_t) 1000 / frame_dt;
      }

      if (current_stereoboard_algorithm == SEND_TURN_COMMANDS) {
        uint8_t border = 0;
        uint8_t disp_threshold = 5 * RESOLUTION_FACTOR;
        evaluate_central_disparities2((uint8_t *)disparity_image.buf, image_width, image_height, disparities, n_disp_bins, min_y,
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
        memset(matrix_buffer, 0, sizeof matrix_buffer);
        // Create the distance matrix by summing pixels per bin
        calculateDistanceMatrix((uint8_t *)disparity_image.buf, blackBorderSize,
                                pixelsPerLine, widthPerBin, heightPerBin, disparity_range);
      }
      if (current_stereoboard_algorithm == SEND_SINGLE_DISTANCE || current_stereoboard_algorithm == STEREO_VELOCITY
          || current_stereoboard_algorithm == DISPARITY_BASED_VELOCITY) {
        // Determine the maximum disparity using the disparity map
        if (!SUB_SAMPLING) {
          // Determine the maximum disparity using the disparity map
          histogram_z_direction((uint8_t *)disparity_image.buf, histogramBuffer, pixelsPerLine, image_height);
          histogram_x_direction((uint8_t *)disparity_image.buf, histogramBufferX, histogram_type, blackBorderSize, pixelsPerLine,
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
#ifdef LED_TOGGLE
      led_toggle();
#endif
      if (current_stereoboard_algorithm == SEND_EDGEFLOW || current_stereoboard_algorithm == STEREO_VELOCITY) {

        // calculate the edge flow
        edgeflow_total(edgeflowArray, (int16_t *)stereocam_data.data, stereocam_data.len, (uint8_t *)current_image_pair.buf,
                       &edgeflow_parameters, &edgeflow_results);

        stereocam_data.data_new = 0;
      }

      // compute and send window detection parameters
      if (current_stereoboard_algorithm == SEND_WINDOW) {
        // XPOS, YPOS, RESPONSE, DISP_SUM, DISP_HOR, DISP_VERT

#ifdef WINDOW
        windowMsgBuf[2] = (uint8_t)detect_window_sizes((uint8_t *)disparity_image.buf, image_width, image_height, coordinate,
                          &window_size, integral_image, MODE_DISPARITY, (uint8_t)(disparity_range - disparity_min));
        windowMsgBuf[0] = coordinate[0];
        windowMsgBuf[1] = coordinate[1];

        windowMsgBuf[3] = (uint8_t)(get_sum_disparities(0, 0, 127, 95, integral_image, 128, 96) / 512);
        windowMsgBuf[4] = (uint8_t)((get_sum_disparities(0, 0, 63, 95, integral_image, 128, 96) - get_sum_disparities(64, 0,
                                     127, 95, integral_image, 128, 96)) / windowMsgBuf[3]) + 127;
        windowMsgBuf[5] = (uint8_t)((get_sum_disparities(0, 0, 127, 47, integral_image, 128, 96) - get_sum_disparities(0, 48,
                                     127, 95, integral_image, 128, 96)) / windowMsgBuf[3]) + 127;
        windowMsgBuf[6] = window_size;
        windowMsgBuf[7] = (uint8_t) 1000 / frame_dt;

#endif

        //memcpy(windowMsgBuf + 8, divergenceArray, 8);
      }

      if (current_stereoboard_algorithm == SEND_FOLLOW_YOU) {

        uint8_t search_window = 10;
        nr_of_features = 0;

        memset((uint8_t *)disparity_image.buf, 0, FULL_IMAGE_SIZE / 2);

        // if no previous measurement, search in the whole image for nearby person
        if (no_prev_measurment == 0) {
          min_y = 0;
          max_y = image_height - 1;

        } else { // only search in range of previous measurement
          min_y = MAX(0, pos_y - search_window);
          max_y = MIN(image_height, pos_y - search_window);
        }

        stereo_vision_sparse_block_fast_version((uint8_t *)current_image_pair.buf,
                                                (uint8_t *)disparity_image.buf, image_width, image_height,
                                                disparity_min, disparity_range, disparity_step, thr1, thr2,
                                                min_y, max_y);

        // subtract feature locations from disparitymap in specified range
        //nr_of_features = getFeatureImageLocations((uint8_t*)disparity_image.buf, feature_image_locations, image_width, image_height, min_y, max_y, feature_count_limit);
        nr_of_features = getFeatureImageLocations((uint8_t *)current_image_pair.buf, (uint8_t *)disparity_image.buf, feature_image_locations,
                         target_location, image_width, image_height, min_y, max_y, feature_count_limit);

        // visualize features
        if (nr_of_features == feature_count_limit) {
          //nr_of_features = visualizeBlobImageLocation((uint8_t*)current_image_pair.buf, feature_image_locations, target_location, nr_of_features, image_width, feature_count_limit);
          //visualizeFeatureImageLocations((uint8_t*)current_image_pair.buf, feature_image_locations, nr_of_features, image_width, feature_count_limit);

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
            memset((uint8_t *)disparity_image.buf, 0, FULL_IMAGE_SIZE / 2);

            // run stereo vision algorithm
            nr_of_features = stereo_vision_sparse_block_features((uint8_t *)current_image_pair.buf,
                             (uint8_t *)disparity_image.buf, feature_image_coordinates, features_max_number, image_width, image_height,
                             disparity_min, disparity_range, disparity_step, thr1, thr2,
                             min_y, max_y);

            if (nr_of_features > 50) {

              features_ROT_number = 5;
              //features_TOTAL_number = 50;
              //feature_window = 2; // one sided window size (total size is *2 + 1)

              odometry_select_features((uint8_t *)current_image_pair.buf, feature_image_coordinates,
                                       nr_of_features, features_TOTAL_number, image_width, image_height);

              odometry_extract_features((uint8_t *)current_image_pair.buf, feature_window_data, feature_image_coordinates,
                                        features_TOTAL_number, features_ROT_number, image_width, image_height, feature_window_size_2);
            }
            //led_toggle();

          } else { // after obtaining disparity map and good features, run odometry
            odometry_translate_and_match_features((uint8_t *)current_image_pair.buf, feature_window_data, feature_image_coordinates,
                                                  features_ROT_number, feature_window_size_2, rotation_coefficients, number_of_rotations, image_width);
            //led_toggle();
          }
        }
      }
#endif

      if (current_stereoboard_algorithm == SEND_LEARNING_COLLISIONS) {
        sys_time_prev = sys_time_get();
        learning_collisions_run((uint8_t *)current_image_pair.buf);
        frame_dt = 1000 * get_timer_interval(sys_time_prev) / TIMER_TICKS_PER_SEC;
      }

      // Now send the data that we want to send
      if (current_stereoboard_algorithm == SEND_IMAGE) {
#ifdef SET_LINE_NUMBERS
        uint8_t horizontalLine;
        uint8_t copyOfThing[IMAGE_WIDTH * IMAGE_HEIGHT * 2];
        int someIndexImage = 0;
        for (; someIndexImage < IMAGE_WIDTH * IMAGE_HEIGHT * 2; someIndexImage++) {
          copyOfThing[someIndexImage] = (uint8_t *)current_image_pair.buf[someIndexImage];
        }
        setLineNumbersImage(&copyOfThing, IMAGE_WIDTH, IMAGE_HEIGHT);

        SendImage(copyOfThing, IMAGE_WIDTH, IMAGE_HEIGHT);
#else
        SendImage((uint8_t *)current_image_pair.buf, current_image_pair.w, current_image_pair.h);
#endif
      }

      if (current_stereoboard_algorithm == SEND_DISPARITY_MAP) {
#ifdef SET_LINE_NUMBERS
        setLineNumbers(&(uint8_t *)disparity_image.buf, IMAGE_WIDTH, IMAGE_HEIGHT);
#endif
        SendArray((uint8_t *)disparity_image.buf, IMAGE_WIDTH, IMAGE_HEIGHT);
      }
      if (current_stereoboard_algorithm == SEND_HISTOGRAM) {
        SendArray(histogramBuffer, pixelsPerLine, 1);
      }
      if (current_stereoboard_algorithm == SEND_MATRIX) {
        SendArray(matrix_buffer, MATRIX_WIDTH_BINS, MATRIX_HEIGHT_BINS);
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
        edgeflowArray[4] = maxDispFound;
        int disparities_high =  evaluate_disparities_droplet((uint8_t *)disparity_image.buf, image_width, image_height, 80);
        if (disparities_high > 200) {
          edgeflowArray[5] = 200;
        } else {
          edgeflowArray[5] = (uint8_t)disparities_high;
        }
        edgeflowArray[6] = (uint8_t)(processed_pixels / 100);
        edgeflowArray[10] = avg_disp_left;
        edgeflowArray[11] = avg_disp_right;
        led_clear();
        if (maxDispFound > 22) {
          led_set();
        }
        SendArray(edgeflowArray, 23, 1);
      }

      if (current_stereoboard_algorithm == SEND_EDGEFLOW) {
#ifdef EDGEFLOW_DEBUG
        SendArray(edgeflowArray, 128, 5);
#else
        SendArray(edgeflowArray, 25, 1);
#endif
      }
      if (current_stereoboard_algorithm == SEND_COMMANDS || current_stereoboard_algorithm == SEND_FRAMERATE_STEREO) {
        SendCommand(toSendCommand);
      }
      if (current_stereoboard_algorithm == SEND_DELFLY_CORRIDOR) {
        SendCommand(toSendCommand);
      }
      if (current_stereoboard_algorithm == SEND_FOLLOW_YOU) {
//        SendImage((uint8_t*)current_image_pair.buf, IMAGE_WIDTH, IMAGE_HEIGHT); // show image with target-cross
        //SendArray((uint8_t*)disparity_image.buf, IMAGE_WIDTH, IMAGE_HEIGHT); // show disparity map
        SendArray(target_location, 3, 1); // send 3D location of target
      }
      if (current_stereoboard_algorithm == SEND_ROTATIONS) {
        //SendArray((uint8_t*)disparity_image.buf, IMAGE_WIDTH, IMAGE_HEIGHT); // show disparity map
      }
      if (current_stereoboard_algorithm == SEND_LEARNING_COLLISIONS) {
        //SendImage((uint8_t*)current_image_pair.buf, IMAGE_WIDTH, IMAGE_HEIGHT);
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

