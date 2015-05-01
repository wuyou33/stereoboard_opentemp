
#ifndef __STEREO_VISION_HEADER__
#define __STEREO_VISION_HEADER__

#include <arm_math.h>

// Stereo vision header file
void stereo_vision(uint8_t *in, q7_t *out, uint32_t image_width, uint32_t image_height, uint32_t disparity_range,
                   uint8_t thr1, uint8_t thr2);
void separate_image_line(uint8_t *in, q15_t *line1, q15_t *line2, uint32_t image_width);
uint32_t evaluate_disparities(uint8_t *in, uint32_t image_width, uint32_t image_height, uint8_t disparity_threshold,
                              uint32_t disparities_high);
void evaluate_disparities_altitude(uint8_t *in, uint32_t image_width, uint32_t image_height,
                                   uint8_t disparity_threshold, uint32_t *disparities, uint8_t altitude_levels, uint16_t x_min, uint16_t x_max,
                                   uint32_t *bad_pixels);

#endif
