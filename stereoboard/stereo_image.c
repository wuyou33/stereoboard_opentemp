
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
  for (uint16_t i = 0, j = 0; i < stereo->buf_size; i += 2, j++) {
    buf[j] = stereo_buf[i];
  }
}

void getRightFromStereo(struct image_t *right_img, struct image_t *stereo)
{
  right_img->w = stereo->w; right_img->h = stereo->h; right_img->type = IMAGE_GRAYSCALE;
  uint8_t *buf = (uint8_t *)right_img->buf;
  uint8_t *stereo_buf = (uint8_t *)stereo->buf;
  for (uint16_t i = 1, j = 0; i < stereo->buf_size; i += 2, j++) {
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
    memcpy(color_buf, img_buf + idx, img->w);
    idx += img->w;
    color_buf += img->w;
    memcpy(stereo_buf, img_buf + idx, img->w);
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

  static int32_t padding_x = BYTES_PER_PIXEL * (IMAGE_WIDTH - cal_width);
  static int32_t img_start = BYTES_PER_PIXEL * ((((IMAGE_HEIGHT - cal_height) / 2) * IMAGE_WIDTH)
                             - (IMAGE_WIDTH - cal_width) / 2);
#if CAMERA_CPLD_STEREO == camera_cpld_stereo_pixmux
  for (j = 0, i = img_start; j < cal_size; j++, i += 2) {
    if (!(j % cal_width)) {
      i += padding_x; // pad image to the left
    }
    out[i] = in[calL[j]];
    out[i + 1] = in[calR[j]];
  }
#else
  memcpy(out, in, FULL_IMAGE_SIZE);
#endif
#else
  memcpy(out, in, FULL_IMAGE_SIZE);
#endif
}

void get_integral_image(uint8_t *in, uint32_t image_width, uint32_t image_height, uint32_t *integral_image)
{
  if (integral_image == NULL) {
    return;
  }
  uint16_t x, y;
  for (x = 0; x < image_width; x++) {
    for (y = 0; y < image_height; y++) {
      if (x >= 1 && y >= 1) {
        integral_image[x + y * image_width] = (uint32_t) in[x + y * image_width] + integral_image[x - 1 + y * image_width] +
                                              integral_image[x + (y - 1) * image_width] - integral_image[x - 1 + (y - 1) * image_width];
      } else if (x >= 1) {
        integral_image[x + y * image_width] = (uint32_t) in[x + y * image_width] + integral_image[x - 1 + y * image_width];
      } else if (y >= 1) {
        integral_image[x + y * image_width] = (uint32_t) in[x + y * image_width] + integral_image[x + (y - 1) * image_width];
      } else {
        integral_image[x + y * image_width] = (uint32_t) in[x + y * image_width];
      }
    }
  }
}

uint32_t get_sum_disparities(uint16_t min_x, uint16_t min_y, uint16_t max_x, uint16_t max_y, uint32_t *integral_image,
                             uint32_t image_width, uint32_t image_height)
{
  return integral_image[min_x + min_y * image_width] + integral_image[max_x + max_y * image_width] -
         integral_image[max_x + min_y * image_width] - integral_image[min_x + max_y * image_width];
}

uint32_t get_avg_disparity(uint16_t min_x, uint16_t min_y, uint16_t max_x, uint16_t max_y, uint32_t *integral_image,
                           uint32_t image_width, uint32_t image_height)
{
  // width and height of the window
  uint32_t w = max_x - min_x + 1;
  uint32_t h = max_y - min_y + 1;

  return get_sum_disparities(min_x, min_y, max_x, max_y, integral_image, image_width, image_height) / (w * h);
}


uint16_t image_yuv422_colorfilt_mod(struct image_t *input, struct image_t *output, uint8_t y_m, uint8_t y_M,
                                    uint8_t u_m, uint8_t u_M, uint8_t v_m, uint8_t v_M, uint16_t *cnt_bin)
{
  int8_t N = 3;

  uint16_t cnt = 0;
  uint8_t *source = input->buf;
  uint8_t *dest = output->buf;

  // Copy the creation timestamp (stays the same)
  output->ts = input->ts;

  // Go through all the pixels
  for (uint16_t y = 0; y < input->h; y++) {
    for (uint16_t x = 0; x < input->w; x += 2) {
      // Check if the color is inside the specified values
      if ((source[0] >= u_m)
          && (source[0] <= u_M)
          && (source[2] >= v_m)
          && (source[2] <= v_M)
         ) {
        // UYVY
        if (source[1] >= y_m && source[1] <= y_M) {
          dest[0] = source[0];  // U
          cnt++;

          for (int j = 0; j < N; j++) {
            if (x < input->w * (j + 1) / N) {
              cnt_bin[j]++;
              break;
            }
          }

        } else {
          dest[0] = 127;        // U
        }
        if (source[3] >= y_m && source[3] <= y_M) {
          dest[2] = source[2];  // V
          cnt++;

          for (int j = 0; j < N; j++) {
            if (x < input->w * (j + 1) / N) {
              cnt_bin[j]++;
              break;
            }
          }

        } else {
          dest[2] = 127;        // V
        }
      } else {
        // UYVY
        dest[0] = 127;        // U
        dest[2] = 127;        // V
      }

      dest[1] = source[1];  // Y1
      dest[3] = source[3];  // Y2

      // Go to the next 2 pixels
      dest += 4;
      source += 4;
    }
  }
  return cnt;
}
