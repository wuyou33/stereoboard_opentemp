/**
 * Color.c
 * @author: Kirk Scheper
 * @details
 *
 * - Sends color images over serial
 * - Can optionally filter image before sending
 *
 */

#include "color.h"
#include "main_parameters.h"
#include "dcmi.h"
#include "image.h"
#include "raw_digital_video_stream.h"

uint8_t color_buf[BYTES_PER_PIXEL * IMAGE_WIDTH * IMAGE_HEIGHT];
struct image_t incoming_img;
struct image_t color_img;

void init_project(void){
  color_img.w = IMAGE_WIDTH;
  color_img.h = IMAGE_HEIGHT;
  color_img.buf_size = BYTES_PER_PIXEL * IMAGE_WIDTH * IMAGE_HEIGHT;
  color_img.buf = color_buf;
  color_img.type = IMAGE_YUV422;

  incoming_img.w = IMAGE_WIDTH;
  incoming_img.h = IMAGE_HEIGHT;
  incoming_img.buf_size = BYTES_PER_PIXEL * IMAGE_WIDTH * IMAGE_HEIGHT;
  incoming_img.buf = current_image_buffer;
  incoming_img.type = IMAGE_YUV422;
}

void run_project(void){
#ifdef FITLER
  image_yuv422_colorfilt(&incoming_img, &color_img, 90, 150, 100, 140, 160, 255);
#else
  image_copy(&incoming_img, &color_img);
#endif

  SendArray((uint8_t*)color_img.buf, color_img.w*2, color_img.h);
}
