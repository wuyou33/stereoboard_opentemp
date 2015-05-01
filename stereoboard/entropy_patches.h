
#ifndef __ENTROPY_PATCHES_HEADER__
#define __ENTROPY_PATCHES_HEADER__

#include <arm_math.h>

// Entropy patches header file
// uint32_t get_entropy_patches(uint8_t left, uint8_t* in, uint32_t image_width, uint32_t image_height, uint8_t min_x, uint8_t max_x, uint8_t min_y, uint8_t max_y, uint8_t patch_size, uint8_t step_size);
uint32_t get_entropy_patches(uint8_t right, uint8_t *in, uint32_t image_width, uint32_t image_height,
                             uint8_t min_x, uint8_t max_x, uint8_t min_y, uint8_t max_y,
                             uint8_t patch_size, uint8_t step_size, uint8_t n_bins);

uint32_t calculate_entropy(q15_t *P, uint8_t n_bins, uint32_t n_elements);

#endif
