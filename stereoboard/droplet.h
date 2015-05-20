/*
 * droplet.h
 *
 *  Created on: May 18, 2015
 *      Author: mavlab
 */

#ifndef DROPLET_H_
#define DROPLET_H_

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

#include BOARD_FILE
#include "main_parameters.h"

#include "commands.h"

// integral_image has size 128 * 96 * 4 = 49152 bytes = C000 in hex
uint32_t *integral_image = ((uint32_t *) 0x10000000); // 0x10000000 - 0x1000 FFFF = CCM data RAM  (64kB)
//uint8_t* jpeg_image_buffer_8bit = ((uint8_t*) 0x1000D000); // 0x10000000 - 0x1000 FFFF = CCM data RAM
//uint8_t* disparity_image_buffer_8bit = ((uint8_t*) 0x10000000);

uint16_t offset_crop = 0;

/** @addtogroup StereoCam
  * @{
  */

/* Private functions ---------------------------------------------------------*/


uint16_t map_value_to_range(uint16_t value, uint16_t range, uint16_t min_val, uint16_t max_val)
{
  value = (value < min_val) ? min_val : value;
  value = (value > max_val) ? max_val : value;
  value -= min_val;
  value = (range * value) / (max_val - min_val);
  value = (value >= range) ? range - 1 : value;
  return value;
}






/**************
 * MAIN DEFINES
 **************/
#define STEREO_PIXMUX 0
#define YUV_COLOR 1


/**
  * @brief  Main program
  * @param  None
  * @retval None
  */
int droplet(void)
{
  // Disparity image buffer:
  uint8_t disparity_image_buffer_8bit[FULL_IMAGE_SIZE / 2];
  uint16_t ind;
  for (ind = 0; ind < FULL_IMAGE_SIZE / 2; ind++) {
    disparity_image_buffer_8bit[ind] = 0;
  }
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
  init_timer2();

  /*******************
   * MINOR PARAMETERS:
   *******************/

  // Avoidance parameters;
#ifdef DELFLY
#warning COMPILING_FOR_DELFLY
  uint16_t obst_thr1 = 2000; // number of pixels with high disparity [1700] [3000]
  uint8_t obst_thr2 = 5; // number of obstacle detections in row
  uint16_t obst_wait = 2779; // time to wait before avoidance manoeuver [ms]
  uint16_t obst_thr3 = 2000; // number of pixels with low disparity (phase 3) [1500]
  uint8_t obst_thr4 = 2; // number of NO obstacle detections in row (phase 3)
  uint16_t obst_entr = 70; // entropy threshold
  uint8_t obst_thr5 = 3; // number of obstacle detections in row (phase 4)
  uint16_t obst_wait2 = 500; // time to wait before going from phase 4 to 1 [ms]
#else
  uint16_t obst_thr1 = 1700; // number of pixels with high disparity
  uint8_t obst_thr2 = 5; // number of obstacle detections in row
  uint16_t obst_wait = 800; // time to wait before avoidance manoeuver [ms]
  uint16_t obst_thr3 = 1500; // number of pixels with low disparity (phase 3)
  uint8_t obst_thr4 = 2; // number of NO obstacle detections in row (phase 3)
  uint16_t obst_entr = 70; // entropy threshold
  uint8_t obst_thr5 = 3; // number of obstacle detections in row (phase 4)
  uint16_t obst_wait2 = 500; // time to wait before going from phase 4 to 1 [ms]
#endif

  uint8_t phase = 1;
  uint8_t obst_dect = 0;
  uint32_t obst_time = 0;
  uint8_t obst_free = 0;
  uint8_t obst_dect2 = 0;
  uint32_t obst_time2 = 0;

#ifdef DELFLY
  uint8_t disparity_threshold = 4; // [5] [4]
  uint32_t disparities_high = 0;
  uint32_t entropy;

  // Stereo parameters:
  uint32_t disparity_range = 15; // at a distance of 1m, disparity is 7-8
  uint32_t disparity_min = 0;
  uint32_t disparity_step = 1;
  uint8_t thr1 = 4;
  uint8_t thr2 = 4;
  uint8_t diff_threshold = 4; // for filtering
#else
  uint8_t disparity_threshold = 5;
  uint32_t disparities_high = 0;
  uint32_t entropy;

  // Stereo parameters:
  uint32_t disparity_range = 30; // at a distance of 1m, disparity is 7-8
  uint32_t disparity_min = 0;
  uint32_t disparity_step = 3;
  uint8_t thr1 = 4;
  uint8_t thr2 = 4;
  uint8_t diff_threshold = 4; // for filtering
#endif

  // Color filtering:
  uint8_t min_U = 0;
  uint8_t min_V = 128;
  uint8_t max_U = 128;
  uint8_t max_V = 255;
  uint16_t n_red_pixels = 0;

  // Avoidance parameters;
  uint8_t disp_threshold = 5;
  if (STEREO_CAM_NUMBER == 1) {
    disp_threshold = 3;
  }
  uint8_t n_disp_bins = 6;
  uint32_t disparities[n_disp_bins];
  uint8_t RESOLUTION = 100;

  uint8_t bin;
  for (bin = 0; bin < n_disp_bins; bin++) {
    disparities[bin] = (uint8_t)48 + bin;
  }
  /*uint32_t avg_disparities[n_disp_bins];
  uint8_t bin;
  for(bin = 0; bin < n_disp_bins; bin++)
  {
    avg_disparities[bin] = 0;
  }*/

  // window coordinate:
  /*uint16_t coordinate[2]; // instantaneous
  uint16_t window_coordinate[2]; // low-pass filtered
  window_coordinate[0] = image_width / 2;
  window_coordinate[1] = image_height / 2;
  uint16_t avg_response = 100;
  uint16_t window_threshold = 90;
  uint16_t escape_coordinate[2];
  uint16_t min_disp = 0;
  uint8_t n_cells = 5;
  uint16_t min_response = 100;
  uint8_t n_bits = 2;
  uint32_t weight_new = 1;
  uint32_t weight_old = 3;
  uint32_t weight_total = weight_old + weight_new;*/

  // counter for toggling image type: ONLY necessary if we USE_COLOR
  uint16_t counter = 0;
  uint8_t toggled = 0;
  uint8_t toggle_image_type = STEREO_PIXMUX;



  /***********
   * MAIN LOOP
   ***********/
  //DCMI_CaptureCmd(ENABLE);

  volatile int processed = 0;
  while (1) {

#ifdef LARGE_IMAGE
    offset_crop += 80;
    if (offset_crop == 480) {
      offset_crop = 0;
    }
    camera_crop(offset_crop);
#endif
    DCMI_CaptureCmd(ENABLE); // while no new frame, ask DCMI to capture a new frame
    // wait for new frame
    while (frame_counter == processed)
      ;
    processed = frame_counter;

    //led_toggle();




#ifdef DELFLY
      if (SEND_COMMANDS) {
        // Control logic
        if (phase == 1) { // unobstructed flight
          if (disparities_high > obst_thr1) { //|| entropy < obst_entr) // if true, obstacle in sight
            obst_dect++;
          } else {
            obst_dect = 0;
          }

          if (obst_dect > obst_thr2) { // if true, obstacle is consistent
            phase = 2;
            obst_dect = 0; // set zero for later
          }
        } else if (phase == 2) { // obstacle detected, wait for action
          if (obst_time == 0) { // when entering phase, set start time
            obst_time = TIM_GetCounter(TIM2);
          }

          if ((TIM_GetCounter(TIM2) - obst_time) > obst_wait * 2) {  // wait (2 clocks per ms)
            phase = 3;
            obst_time = 0; // set zero for later
          }
        } else if (phase == 3) { // avoid
          // Turn command signal for AutoPilot ???
          if (disparities_high < obst_thr3) { // if true, flight direction is safe
            obst_free++;
          } else {
            obst_free = 0;
          }

          if (obst_free > obst_thr4) { // if true, consistently no obstacles
            if (entropy > obst_entr) { // do the entropy check
              phase = 4;
              obst_free = 0; // set zero for later
            }
          }
        } else if (phase == 4) { // fly straight, but be aware of undetected obstacles
          if (obst_time2 == 0) { // when entering phase, set start time
            obst_time2 =  TIM_GetCounter(TIM2);
          }

          if (disparities_high > obst_thr1) { // if true, obstacle in sight
            obst_dect2++;
          } else {
            obst_dect2 = 0;
          }

          if (obst_dect2 > obst_thr5) { // if true, obstacle is consistent
            phase = 3; // go back to phase 3
            obst_time2 = 0; // set zero for later
            obst_dect2 = 0; // set zero for later

          } else if ((TIM_GetCounter(TIM2) - obst_time2) > obst_wait2 * 2) {  // wait (2 clocks per ms)
            phase = 1;
            obst_time2 = 0; // set zero for later
            obst_dect2 = 0;
          }
        }

        // turn command:
        if (phase == 3) {
          SendCommand(1);
        } else {
          SendCommand(0);
        }

        if (phase == 2 || phase == 3) {
          led_set();
        } else {
          led_clear();
        }

      }
#else
    if (SEND_COMMANDS) {
      // Determine disparities:
      min_y = 0;
      max_y = 95;
      stereo_vision_Kirk(current_image_buffer, disparity_image_buffer_8bit, image_width, image_height, disparity_min,
                         disparity_range, disparity_step, thr1, thr2, min_y, max_y);


      uint8_t border = 0; // 10 was the standard value
      // GUIDO
      evaluate_central_disparities2(disparity_image_buffer_8bit, image_width, image_height, disparities, n_disp_bins, min_y,
                                    max_y, disp_threshold, border);
      //    express the outputs as percentages:
      //    number of pixels relative to the evaluated part of the image with high disparities:
      disparities[0] = (disparities[0] * RESOLUTION) / ((max_y - min_y) * (image_width - 2 * border));
      //    x-coordinate as coordinate of the entire image:
      disparities[1] = (disparities[1] * RESOLUTION) / image_width;

      // Send commands
      // send 0xff
      SendStartComm();
      // percentage of close pixels
      SendCommandNumber((uint8_t) disparities[0]);
      // percentage of x-location
      SendCommandNumber((uint8_t) disparities[1]);
    }
#endif

    if (SEND_DISPARITY_MAP) {
      // Determine disparities:
      min_y = 0;
      max_y = 95;
      stereo_vision_Kirk(current_image_buffer, disparity_image_buffer_8bit, image_width, image_height, disparity_min,
                         disparity_range, disparity_step, thr1, thr2, min_y, max_y);

      SendDisparityMap(disparity_image_buffer_8bit);


    }

  }
}


#endif /* DROPLET_H_ */
