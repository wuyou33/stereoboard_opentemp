
#include "stereo_image.h"

#include "stereo_math.h"
#include <stdlib.h>  /* abs */
#include <string.h>
#include <stdbool.h>
#include "main_parameters.h"

void setLineNumbers(uint8_t **givenImage, uint16_t array_width, uint16_t array_height)
{
  uint8_t *b = *givenImage;
  uint8_t horizontalLine = 0;
  for (horizontalLine = 0; horizontalLine < array_height; horizontalLine++) {
    // Line data
    b[array_width * horizontalLine] = horizontalLine;
    b[array_width * horizontalLine + 1] = horizontalLine;
    b[array_width * horizontalLine + 2] = horizontalLine;
    b[array_width * horizontalLine + 3] = horizontalLine;
    b[array_width * horizontalLine + 4] = horizontalLine;
    b[array_width * horizontalLine + 5] = horizontalLine;
    b[array_width * horizontalLine + 6] = horizontalLine;
  }
}

void setLineNumbersImage(uint8_t *b, uint16_t width, uint16_t height)
{
  uint8_t j = 0;
  for (j = 0; j < height; j++) {
    b[j * width * BYTES_PER_PIXEL] = j;
    b[j * width * BYTES_PER_PIXEL + 1] = j;
    b[j * width * BYTES_PER_PIXEL + 2] = j;
    b[j * width * BYTES_PER_PIXEL + 3] = j;
    b[j * width * BYTES_PER_PIXEL + 4] = j;
    b[j * width * BYTES_PER_PIXEL + 5] = j;

  }
}

void getLeftFromStereo(struct image_t *left_img, struct image_t *stereo)
{
  left_img->w = stereo->w / 2; left_img->h = stereo->h; left_img->type = IMAGE_GRAYSCALE;
  uint8_t *buf = (uint8_t *)left_img->buf;
  uint8_t *stereo_buf = (uint8_t *)stereo->buf;
  for (uint16_t i = 0; i < left_img->w / 2; i++) {
    for (uint16_t j = 0; j < left_img->h; j++) {
      buf[i + (left_img->w * j)] = stereo_buf[2 * i + (stereo->w * j)];
    }
  }
}

void getRightFromStereo(struct image_t *right_img, struct image_t *stereo)
{
  right_img->w = stereo->w / 2; right_img->h = stereo->h; right_img->type = IMAGE_GRAYSCALE;
  uint8_t *buf = (uint8_t *)right_img->buf;
  uint8_t *stereo_buf = (uint8_t *)stereo->buf;
  for (uint16_t i = 0; i < right_img->w / 2; i++) {
    for (uint16_t j = 0; j < right_img->h; j++) {
      buf[i + (right_img->w * j)] = stereo_buf[2 * i + (stereo->w * j) + 1];
    }
  }
}

void split_color_from_pixmux_stereo(struct image_t *img, struct image_t *color, struct image_t *stereo)
{
  color->w = img->w; color->h = img->h / 2; color->type = IMAGE_YUV422;
  stereo->w = img->w; stereo->h = img->h / 2; stereo->type = IMAGE_GRAYSCALE;

  uint8_t *img_buf = (uint8_t *)img->buf;
  uint8_t *color_buf = (uint8_t *)color->buf;
  uint8_t *stereo_buf = (uint8_t *)stereo->buf;
  for (uint16_t i = 0; i < img->w; i++) {
    for (uint16_t j = 0; j < img->h; j+=2) {
      color_buf[i + (color->w * j / 2)] = img_buf[i + (img->w * j)];
      stereo_buf[i + (stereo->w * j / 2)] = img_buf[i + (img->w * (j + 1))];
    }
  }
}

#if (CAMERA_CPLD_STEREO == camera_cpld_stereo_pixmux)
void separate_image_line_offset(uint8_t *in, q15_t *left, q15_t *right, uint32_t image_width_bytes)
{
  uint32_t i, j;
  int8_t offset = DISPARITY_OFFSET_LEFT;
  for (i = 0; i < image_width_bytes; i += 2) {
    j = i >> 1;
    if (i >= DISPARITY_BORDER) {
      offset = DISPARITY_OFFSET_RIGHT;
    }
    if (offset >= 0) {
      left[j] = (q15_t) in[i];
      // We add one because images are interlaced
      right[j] = (q15_t) in[i + 1 + (image_width_bytes * offset)];
    } else if (offset < 0) {
      left[j] = (q15_t) in[i - (image_width_bytes * offset)];
      // We add one because images are interlaced
      right[j] = (q15_t) in[i + 1];
    }
  }
}

void separate_image_line_offset_block(uint8_t *in, q15_t *block_left, q15_t *block_right, uint32_t image_width_bytes,
                                      uint8_t idx, uint32_t image_width)
{
  uint32_t i, j;
  int8_t offset = DISPARITY_OFFSET_LEFT;
  for (i = 0; i < image_width_bytes; i += 2) {
    j = i >> 1;
    if (i >= DISPARITY_BORDER) {
      offset = DISPARITY_OFFSET_RIGHT;
    }
    if (offset >= 0) {
      block_left[j + (image_width * idx)] = (q15_t) in[i];
      // We add one because images are interlaced
      block_right[j + (image_width * idx)] = (q15_t) in[i + 1 + (image_width_bytes * offset)];
    } else if (offset < 0) {
      block_left[j + (image_width * idx)] = (q15_t) in[i - (image_width_bytes * offset)];
      // We add one because images are interlaced
      block_right[j + (image_width * idx)] = (q15_t) in[i + 1];
    }
  }
}

#elif (CAMERA_CPLD_STEREO == camera_cpld_stereo_linemux)
void separate_image_line_offset(uint8_t *in, q15_t *line1, q15_t *line2, uint32_t image_width_bytes)
{
  uint32_t i;
  int8_t offset = DISPARITY_OFFSET_LEFT + 1;  // We add one because images are interlaced
  for (i = 0; i < image_width_bytes; i++) {
    if (i >= DISPARITY_BORDER) {
      offset = DISPARITY_OFFSET_RIGHT;
    }
    if (offset >= 0) {
      line1[i] = (q15_t) in[i];
      line2[i] = (q15_t) in[i + (image_width_bytes * offset)];
    } else if (offset < 0) {
      line1[i] = (q15_t) in[i - (image_width_bytes * offset)];
      line2[i] = (q15_t) in[i];
    }
  }
}

void separate_image_line_offset_block(uint8_t *in, q15_t *block_left, q15_t *block_right, uint32_t image_width_bytes,
                                      uint8_t idx, uint32_t image_width)
{
  uint32_t i;
  int8_t offset = DISPARITY_OFFSET_LEFT + 1;  // We add one because images are interlaced
  for (i = 0; i < image_width_bytes; i++) {
    if (i >= DISPARITY_BORDER) {
      offset = DISPARITY_OFFSET_RIGHT;
    }
    if (offset >= 0) {
      block_left[i + (image_width * idx)] = (q15_t) in[i];
      block_right[i + (image_width * idx)] = (q15_t) in[i + (image_width_bytes * offset)];
    } else if (offset < 0) {
      block_left[i + (image_width * idx)] = (q15_t) in[i - (image_width_bytes * offset)];
      block_right[i + (image_width * idx)] = (q15_t) in[i];
    }
  }
}

#endif