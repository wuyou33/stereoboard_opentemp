#ifndef _STEREO_IMAGE_HEADER_
#define _STEREO_IMAGE_HEADER_

#include <inttypes.h>
#include "image.h"
#include "arm_math.h"

void setLineNumbers(uint8_t **givenImage, uint16_t width, uint16_t height);
void setLineNumbersImage(uint8_t *givenImage, uint16_t width, uint16_t height);

/* separate multiplexed stereo image */
void getLeftFromStereo(struct image_t *left_img, struct image_t *stereo);
void getRightFromStereo(struct image_t *right_img, struct image_t *stereo);
void split_color_from_pixmux_stereo(struct image_t *img, struct image_t *color, struct image_t *stereo);

void separate_image_line_offset(uint8_t *in, q15_t *line1, q15_t *line2, uint32_t image_width_bytes);
void separate_image_line_offset_block(uint8_t *in, q15_t *block_left, q15_t *block_right, uint32_t image_width_bytes,
                                      uint8_t idx, uint32_t image_width);

#endif //_STEREO_IMAGE_HEADER_
