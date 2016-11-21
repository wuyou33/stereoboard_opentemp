#ifndef _MY_IMAGE_HEADER_
#define _MY_IMAGE_HEADER_

#include <inttypes.h>
#include <stdbool.h>
#include "main_parameters.h"
#include "arm_math.h"


void setLineNumbers(uint8_t **givenImage, uint16_t width, uint16_t height);
void setLineNumbersImage(uint8_t *givenImage, uint16_t width, uint16_t height);

/* The different type of images we currently support */
enum image_type {
  IMAGE_YUV422,     ///< UYVY format (uint16 per pixel)
  IMAGE_GRAYSCALE,  ///< Grayscale image with only the Y part (uint8 per pixel)
  IMAGE_JPEG,       ///< An JPEG encoded image (not per pixel encoded)
  IMAGE_GRADIENT    ///< An image gradient (int16 per pixel)
};


/* Main image structure */
struct image_t {
  enum image_type type;   ///< The image type
  uint16_t w;             ///< Image width
  uint16_t h;             ///< Image height
  uint32_t ts;            ///< The timestamp of creation

  uint32_t buf_size;      ///< The buffer size
  void *buf;              ///< Image buffer (depending on the image_type)
};

/* Image point structure */
struct point_t {
  uint32_t x;             ///< The x coordinate of the point
  uint32_t y;             ///< The y coordinate of the point
};

/* Vector structure for point differences */
struct flow_t {
  struct point_t pos;         ///< The original position the flow comes from in subpixels
  int16_t flow_x;             ///< The x direction flow in subpixels
  int16_t flow_y;             ///< The y direction flow in subpixels
};

/* Image size structure */
struct img_size_t {
  uint16_t w;     ///< The width
  uint16_t h;     ///< The height
};

/* Image crop structure */
struct crop_t {
  uint16_t x;    ///< Start position x (horizontal)
  uint16_t y;    ///< Start position y (vertical)
  uint16_t w;    ///< Width of the cropped area
  uint16_t h;    ///< height of the cropped area
};

void getLeftFromStereo(struct image_t *img, uint8_t *stereo);
void getRightFromStereo(struct image_t *img, uint8_t *stereo);

/* separate multiplexed stereoimage */
void separate_image_line_offset(uint8_t *in, q15_t *line1, q15_t *line2, uint32_t image_width_bytes);
void separate_image_line_offset_block(uint8_t *in, q15_t *block_left, q15_t *block_right, uint32_t image_width_bytes,
                                      uint8_t idx, uint32_t image_width);

/* the following is copied from pprz */

/* Useful image functions */
void image_add_border(struct image_t *input, struct image_t *output, uint8_t border_size);
void image_create(struct image_t *img, uint16_t width, uint16_t height, enum image_type type);
void image_free(struct image_t *img);
void image_copy(struct image_t *input, struct image_t *output);
void image_swap(struct image_t *a, struct image_t *b);
void image_to_grayscale(struct image_t *input, struct image_t *output);
uint16_t image_yuv422_colorfilt(struct image_t *input, struct image_t *output, uint8_t y_m, uint8_t y_M, uint8_t u_m,
                                uint8_t u_M, uint8_t v_m, uint8_t v_M);
void image_yuv422_downsample(struct image_t *input, struct image_t *output, uint16_t downsample);
void image_subpixel_window(struct image_t *input, struct image_t *output, struct point_t *center,
                           uint32_t subpixel_factor, uint8_t border_size);
void image_gradients(struct image_t *input, struct image_t *dx, struct image_t *dy);
void image_calculate_g(struct image_t *dx, struct image_t *dy, int32_t *g);
uint32_t image_difference(struct image_t *img_a, struct image_t *img_b, struct image_t *diff);
int32_t image_multiply(struct image_t *img_a, struct image_t *img_b, struct image_t *mult);
void image_show_points(struct image_t *img, struct point_t *points, uint16_t points_cnt);
void image_show_flow(struct image_t *img, struct flow_t *vectors, uint16_t points_cnt, uint8_t subpixel_factor);
void image_draw_line(struct image_t *img, struct point_t *from, struct point_t *to);
void pyramid_next_level(struct image_t *input, struct image_t *output, uint8_t border_size);
void pyramid_build(struct image_t *input, struct image_t *output_array, uint8_t pyr_level, uint8_t border_size);

#endif
