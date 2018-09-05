#include "droplet.h"

#include "main_parameters.h"
#include "main.h"
#include "dcmi.h"
#include "image.h"
#include "raw_digital_video_stream.h"
#include "stereo_math.h"
#include "stereo_vision.h"
#include "utils.h"
#include "led.h"

uint32_t message[5];

#ifndef FULL_CALIBRATION
uint16_t cal_width = IMAGE_WIDTH;
uint16_t cal_height = IMAGE_HEIGHT;
#endif

void init_project(void)
{
}

//#define SEND_IMAGE

void run_project(void)
{
#ifdef SEND_IMAGE
  //SendImage((uint8_t*)current_image_pair.buf, current_image_pair.w, current_image_pair.h);
#else

//  SendArray((uint8_t*)disparity_image.buf, disparity_image.w, disparity_image.h);

  // DROPLET
  static q15_t sub_disp_histogram[DISPARITY_RANGE * RESOLUTION_FACTOR] = {0};
  static uint16_t y_min = (IMAGE_HEIGHT - cal_height) / 2, y_max = IMAGE_HEIGHT - (IMAGE_HEIGHT - cal_height) / 2;
  static uint32_t processed_pixels;
  memset(disparity_image.buf, 0, disparity_image.buf_size);
  processed_pixels = stereo_vision_sparse_block_two_sided((uint8_t *)current_image_pair.buf, (uint8_t *)disparity_image.buf,
                     disparity_image.w, disparity_image.h, 0, DISPARITY_RANGE, 1, 7, 4, y_min, y_max, sub_disp_histogram);

  static uint32_t count_disps_left, count_disps_right, hist_obs_sum;
  static uint32_t disparities_high;
  //disparities_high =  evaluate_disparities_droplet((uint8_t*)disparity_image.buf, disparity_image.w, disparity_image.h, 30);
  disparities_high = evaluate_disparities_droplet_low_texture(&disparity_image,
                     &count_disps_left, &count_disps_right, &hist_obs_sum);

  message[0] = disparities_high;
  message[1] = processed_pixels;
  message[2] = hist_obs_sum;
  message[3] = count_disps_left;
  message[4] = count_disps_right;

  SendArray((uint8_t *)message, 5 * sizeof(uint32_t), 1);

  if (disparities_high > 25 || processed_pixels < 10) {
    led_set();
  } else {
    led_clear();
  }
#endif
}
