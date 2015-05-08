
#ifndef __OPTIC_FLOW_HEADER__
#define __OPTIC_FLOW_HEADER__

#include <arm_math.h>

// Stereo vision header file
void optic_flow_horizontal(uint8_t *prev_im, uint8_t *curr_im, uint32_t image_width, uint32_t image_height,
                           uint32_t max_flow, uint8_t min_y, uint8_t max_y, int *divergence, int *displacement);
void fitLinearModel(uint32_t *X, int *FLOW, uint32_t n_entries, int *divergence, int *displacement);
uint8_t createHistogramSobel(uint8_t *histogram, uint8_t *im, uint32_t image_width, uint32_t image_height,
                             uint8_t min_y, uint8_t max_y);
uint32_t getMinimum(uint32_t *flow_error, uint32_t max_ind);

#endif
