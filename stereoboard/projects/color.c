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
#include "main.h"
#include "dcmi.h"
#include "image.h"
#include "raw_digital_video_stream.h"

struct image_t incoming_img;

void init_project(void)
{
  incoming_img.w = IMAGE_WIDTH;
  incoming_img.h = IMAGE_HEIGHT;
  incoming_img.buf_size = BYTES_PER_PIXEL * IMAGE_WIDTH * IMAGE_HEIGHT;
  incoming_img.buf = current_image_buffer;
  incoming_img.type = IMAGE_YUV422;
}

void run_project(void)
{
#ifdef FITLER
  image_yuv422_colorfilt(&incoming_img, &incoming_img, 90, 150, 100, 140, 160, 255);
#endif

  SendImage((uint8_t *)incoming_img.buf, incoming_img.w, incoming_img.h);
}
