
#include "stereo_image.h"

#include "stereo_math.h"
#include <stdlib.h>  /* abs */
#include <string.h>
#include <stdbool.h>
#include "main_parameters.h"
#include "image.h"

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

// TODO: rotate left image one pixel due to pixmux
void getLeftFromStereo(struct image_t *left_img, struct image_t *stereo)
{
  left_img->w = stereo->w; left_img->h = stereo->h; left_img->type = IMAGE_GRAYSCALE;
  uint8_t *buf = (uint8_t *)left_img->buf;
  uint8_t *stereo_buf = (uint8_t *)stereo->buf;
  for (uint16_t i = 0, j = 0; i < stereo->buf_size; i+=2, j++) {
    buf[j] = stereo_buf[i];
  }
}

void getRightFromStereo(struct image_t *right_img, struct image_t *stereo)
{
  right_img->w = stereo->w; right_img->h = stereo->h; right_img->type = IMAGE_GRAYSCALE;
  uint8_t *buf = (uint8_t *)right_img->buf;
  uint8_t *stereo_buf = (uint8_t *)stereo->buf;
  for (uint16_t i = 1, j = 0; i < stereo->buf_size; i+=2, j++) {
    buf[j] = stereo_buf[i];
  }
}

void split_color_from_pixmux_stereo(struct image_t *img, struct image_t *color, struct image_t *stereo)
{
  color->w = img->w; color->h = img->h / 2; color->type = IMAGE_YUV422;
  stereo->w = img->w; stereo->h = img->h / 2; stereo->type = IMAGE_GRAYSCALE;

  uint8_t *img_buf = (uint8_t *)img->buf;
  uint8_t *color_buf = (uint8_t *)color->buf;
  uint8_t *stereo_buf = (uint8_t *)stereo->buf;
  uint32_t idx = 0, size = img->w * img->h;
  while (idx < size) {
    memcpy(color_buf, img_buf+idx, img->w);
    idx += img->w;
    color_buf += img->w;
    memcpy(stereo_buf, img_buf+idx, img->w);
    idx += img->w;
    stereo_buf += idx;
  }
}

#if (CAMERA_CPLD_STEREO == camera_cpld_stereo_pixmux)
// TODO: rotate left image one pixel due to pixmux
// TODO: Apply DISPARITY_OFFSET_HORIZONTAL correction here
void separate_image_line_offset(uint8_t *in, q15_t *left, q15_t *right, uint32_t image_width_bytes)
{
  uint32_t i, j;
#ifdef FULL_CALIBRATION
  for (i = 0, j = 0; i < image_width_bytes; i += 2, j++) {
    left[j] = (q15_t) in[i + 2]; // skip first pixel
    right[j] = (q15_t) in[i + 1];
  }
#else
  int8_t offset = DISPARITY_OFFSET_LEFT;
  for (i = 0; i < image_width_bytes; i += 2) {
    j = i >> 1;
    if (i >= DISPARITY_BORDER) {
      offset = DISPARITY_OFFSET_RIGHT;
    }
    if (offset >= 0) {
      left[j] = (q15_t) in[i + 2]; // skip first pixel
      // We add one because images are interlaced
      right[j] = (q15_t) in[i + 1 + (image_width_bytes * offset)];
    } else if (offset < 0) {
      left[j] = (q15_t) in[i - (image_width_bytes * offset) + 2];
      // We add one because images are interlaced
      right[j] = (q15_t) in[i + 1];
    }
  }
#endif
}

// TODO: rotate left image one pixel due to pixmux
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

void calibrate_image(uint8_t *out, uint8_t *in)
{
// TODO we can speed up the use of the image if we convert the pixmux image to linemux or framemux
// during the calibration

#ifdef FULL_CALIBRATION
  int32_t i, j;

  static int32_t padding_x = BYTES_PER_PIXEL*(IMAGE_WIDTH-cal_width);
  static int32_t img_start = BYTES_PER_PIXEL*((((IMAGE_HEIGHT-cal_height)/2) * IMAGE_WIDTH)
      - (IMAGE_WIDTH-cal_width)/2);
#if CAMERA_CPLD_STEREO == camera_cpld_stereo_pixmux
  for (j = 0, i = img_start; j < cal_size; j++, i+=2) {
    if (!(j % cal_width)){
      i += padding_x; // pad image to the left
    }
    out[i] = in[calL[j]];
    out[i+1] = in[calR[j]];
  }
#else
  memcpy(out, in, FULL_IMAGE_SIZE);
#endif
#else
  memcpy(out, in, FULL_IMAGE_SIZE);
#endif
}
