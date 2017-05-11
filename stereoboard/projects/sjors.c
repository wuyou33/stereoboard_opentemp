/**
 * sjors.c
 * @author: Sjors
 * @details
 *
 * - description!
 *
 */

#include "sjors.h"
#include "main_parameters.h"
#include "main.h"
#include "image.h"
#include "raw_digital_video_stream.h"
#include "stereo_vision.h"

#define DISPARITY_RANGE 20

// settings
uint16_t disparity_min = 0;
uint16_t disparity_step = 1;
uint16_t thr1 = 7;
uint16_t thr2 = 4;
uint16_t min_y = 0;
uint16_t max_y = IMAGE_HEIGHT;

// variable for making a sub-pixel disparity histogram:
int16_t sub_disp_histogram[DISPARITY_RANGE * RESOLUTION_FACTOR];

void init_project(void){
}

void run_project(void){
  memset((uint8_t*)disparity_image.buf, 0, FULL_IMAGE_SIZE / 2);
  int32_t processed_pixels = stereo_vision_sparse_block_two_sided((uint8_t *)current_image_pair.buf,
                                     (uint8_t*)disparity_image.buf, IMAGE_WIDTH, IMAGE_HEIGHT,
                                     disparity_min, DISPARITY_RANGE, disparity_step, thr1, thr2,
                                     min_y, max_y, sub_disp_histogram);

  SendArray((uint8_t*)disparity_image.buf, disparity_image.w, disparity_image.h);
  // visulaise with disparity_viz.py
}
