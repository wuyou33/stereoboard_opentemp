#ifndef _STEREO_IMAGE_HEADER_
#define _STEREO_IMAGE_HEADER_

#include <inttypes.h>
#include "image.h"
#ifdef COMPILE_ON_LINUX
#define q15_t int16_t
#else
#include "arm_math.h"
#endif

void setLineNumbers(uint8_t **givenImage, uint16_t width, uint16_t height);
void setLineNumbersImage(uint8_t *givenImage, uint16_t width, uint16_t height);

/* separate multiplexed stereo image */
void getLeftFromStereo(struct image_t *left_img, struct image_t *stereo);
void getRightFromStereo(struct image_t *right_img, struct image_t *stereo);
void split_color_from_pixmux_stereo(struct image_t *img, struct image_t *color, struct image_t *stereo);

void separate_image_line_offset(uint8_t *in, q15_t *line1, q15_t *line2, uint32_t image_width_bytes);
void separate_image_line_offset_block(uint8_t *in, q15_t *block_left, q15_t *block_right, uint32_t image_width_bytes,
                                      uint8_t idx, uint32_t image_width);

void get_integral_image(uint8_t *in, uint32_t image_width, uint32_t image_height, uint32_t *integral_image);
uint32_t get_avg_disparity(uint16_t min_x, uint16_t min_y, uint16_t max_x, uint16_t max_y, uint32_t *integral_image,
                           uint32_t image_width, uint32_t image_height);
uint32_t get_sum_disparities(uint16_t min_x, uint16_t min_y, uint16_t max_x, uint16_t max_y, uint32_t *integral_image,
                             uint32_t image_width, uint32_t image_height);

void calibrate_image(uint8_t *out, uint8_t *in);

uint16_t image_yuv422_colorfilt_mod(struct image_t *input, struct image_t *output, uint8_t y_m, uint8_t y_M,
                                    uint8_t u_m,
                                    uint8_t u_M, uint8_t v_m, uint8_t v_M, uint16_t *cnt_bin);

#endif //_STEREO_IMAGE_HEADER_
