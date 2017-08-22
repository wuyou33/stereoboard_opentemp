/*
 * gate_detecting.c
 *
 *  Created on: 17 Aug 2017
 *      Author: kirk
 */

#include "gate_detecting.h"

#include "main_parameters.h"
#include "main.h"
#include "dcmi.h"
#include "image.h"
#include "raw_digital_video_stream.h"
#include "gate_detection.h"
#include "stereo_math.h"
#include "stereo_vision.h"
#include "utils.h"

void color_gate_detector(void);
void grayscale_gate_detector(void);
void disparity_gate_detector(void);
void edge_gate_detector(void);

#ifndef FULL_CALIBRATION
static const uint16_t cal_width = IMAGE_WIDTH;
static const uint16_t cal_height = IMAGE_HEIGHT;
#endif

struct image_t color_img = {
  .w = IMAGE_WIDTH,
  .h = IMAGE_HEIGHT,
  .buf_size = FULL_IMAGE_SIZE,
  .type = IMAGE_YUV422
};

uint8_t left[IMAGE_WIDTH * IMAGE_HEIGHT];
struct image_t left_img = {
  .w = IMAGE_WIDTH,
  .h = IMAGE_HEIGHT,
  .buf = left,
  .buf_size = IMAGE_WIDTH * IMAGE_HEIGHT,
  .type = IMAGE_GRAYSCALE
};

uint8_t grad[IMAGE_WIDTH * IMAGE_HEIGHT];
struct image_t gradient = {
  .w = IMAGE_WIDTH,
  .h = IMAGE_HEIGHT,
  .buf = grad,
  .buf_size = IMAGE_WIDTH * IMAGE_HEIGHT,
  .type = IMAGE_GRAYSCALE
};

static struct point_t roi[2];
static struct gate_t gate;

void init_project(void)
{
  // initialize paramters
  memset(current_image_pair.buf, 0, current_image_pair.buf_size);

  color_img.buf = current_image_pair.buf;

  // set pixel search bounds
  gate_set_intensity(0, 255);
  gate_set_color(0, 255, 0, 255, 0, 255);

  // set region of intrest for gate detector
  roi[0].x = (IMAGE_WIDTH - cal_width) / 2;
  roi[0].y = (IMAGE_HEIGHT - cal_height) / 2;
  roi[1].x = (IMAGE_WIDTH + cal_width) / 2;
  roi[1].y = (IMAGE_HEIGHT+cal_height)/2;
}

void run_project(void)
{
  switch(GATE_METHOD){
    case 1:
      color_gate_detector();
      break;
    case 2:
      grayscale_gate_detector();
      break;
    case 3:
      disparity_gate_detector();
      break;
    case 4:
      edge_gate_detector();
      break;
    default:
      ;
  }
}

void color_gate_detector(void){
  snake_gate_detection(&current_image_pair, &gate, false, NULL, NULL, NULL);
#ifdef GATE_DETECTION_GRAPHICS
  SendImage((uint8_t*)current_image_pair.buf, current_image_pair.w, current_image_pair.h);
#endif
}

void grayscale_gate_detector(void){
  getLeftFromStereo(&left_img, &current_image_pair);
  snake_gate_detection(&left_img, &gate, false, NULL, NULL, NULL);
#ifdef GATE_DETECTION_GRAPHICS
  SendArray((uint8_t*)left_img.buf, left_img.w, left_img.h);
#endif
}

void disparity_gate_detector(void){
  // generate disparity map
  static q15_t sub_disp_histogram[20 * RESOLUTION_FACTOR] = {0};
  static uint16_t y_min = (IMAGE_HEIGHT - cal_height) / 2, y_max = IMAGE_HEIGHT - (IMAGE_HEIGHT - cal_height) / 2;
  uint32_t processed_pixels;
  memset(disparity_image.buf, 0, disparity_image.buf_size);
  processed_pixels = stereo_vision_sparse_block_two_sided((uint8_t *)current_image_pair.buf, (uint8_t *)disparity_image.buf, disparity_image.w, disparity_image.h,
0, DISPARITY_RANGE, 1, 7, 4, y_min, y_max, sub_disp_histogram);

#ifdef USE_INTEGRAL_IMAGE
  gen_gate_detection(&disparity_image, NULL, &gate, integral_image);
#endif
#ifdef GATE_DETECTION_GRAPHICS
  SendArray((uint8_t*)disparity_image.buf, disparity_image.w, disparity_image.h);
#endif
}

void edge_gate_detector(void){
  getLeftFromStereo(&left_img, &current_image_pair);
  image_2d_gradients(&left_img, &gradient);
  snake_gate_detection(&gradient, &gate, false, NULL, NULL, NULL);
#ifdef GATE_DETECTION_GRAPHICS
  SendArray((uint8_t*)gradient.buf, gradient.w, gradient.h);
#endif
}
