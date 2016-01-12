
#ifndef __STEREO_VISION_HEADER__
#define __STEREO_VISION_HEADER__

#include <arm_math.h>
#include "main_parameters.h"
#include "../multigaze/stereoboard_parameters.h"
#include "stm32f4xx_conf.h"
#include "sys_time.h"

// Stereo vision header file
void stereo_vision_sparse_block_fast_version(uint8_t *in, q7_t *out, uint32_t image_width, uint32_t image_height,
    uint32_t disparity_min, uint32_t disparity_range, uint32_t disparity_step, uint8_t thr1, uint8_t thr2, uint8_t min_y, uint8_t max_y);
uint16_t stereo_vision_sparse_block_features(uint8_t *in, q7_t *out, uint8_t *features, uint16_t features_max_number, uint32_t image_width, uint32_t image_height,
    uint32_t disparity_min, uint32_t disparity_range, uint32_t disparity_step, uint8_t thr1, uint8_t thr2, uint8_t min_y, uint8_t max_y);
void stereo_vision_sparse_block_two_sided(uint8_t *in, q7_t *out, uint32_t image_width, uint32_t image_height,
    uint32_t disparity_min,
    uint32_t disparity_range, uint32_t disparity_step, uint8_t thr1, uint8_t thr2, uint8_t min_y, uint8_t max_y);
void stereo_vision_sparse_block(uint8_t *in, q7_t *out, uint32_t image_width, uint32_t image_height,
                                uint32_t disparity_min,
                                uint32_t disparity_range, uint32_t disparity_step, uint8_t thr1, uint8_t thr2, uint8_t min_y, uint8_t max_y);
void stereo_vision_Kirk(uint8_t *in, q7_t *out, uint32_t image_width, uint32_t image_height, uint32_t disparity_min,
                        uint32_t disparity_range, uint32_t disparity_step, uint8_t thr1, uint8_t thr2, uint8_t min_y, uint8_t max_y);
void stereo_vision(uint8_t *in, q7_t *out, uint32_t image_width, uint32_t image_height, uint32_t disparity_range,
                   uint8_t thr1, uint8_t thr2, uint8_t min_y, uint8_t max_y);
void stereo_vision_cigla(uint8_t *in, q7_t *out, uint32_t image_width, uint32_t image_height, uint32_t disparity_range,
                         uint8_t sadWS, uint8_t sigma, uint32_t diff_threshold, uint8_t min_y, uint8_t max_y);

void evaluate_disparities_control(uint8_t *in, uint32_t image_width, uint32_t image_height, uint32_t control_output,
                                  uint32_t disparity_range, uint8_t sadWS, uint8_t crop, uint8_t thr3);
uint32_t evaluate_disparities_control2(uint8_t *in, uint32_t image_width, uint32_t image_height,
                                       uint32_t disparity_range, uint8_t sadWS, uint8_t crop, uint32_t thr3);

void separate_image_line_offset(uint8_t *in, q15_t *line1, q15_t *line2, uint32_t image_width_bytes);

void separate_image_line_offset_block(uint8_t *in, q15_t *line1, q15_t *line2, uint32_t image_width_bytes, uint8_t idx,
                                      uint32_t image_width);

uint32_t evaluate_disparities(uint8_t *in, uint32_t image_width, uint32_t image_height, uint8_t disparity_threshold,
                              uint32_t disparities_high);
void evaluate_disparities_altitude(uint8_t *in, uint32_t image_width, uint32_t image_height,
                                   uint8_t disparity_threshold, uint32_t *disparities, uint8_t altitude_levels, uint16_t x_min, uint16_t x_max,
                                   uint32_t *bad_pixels);
void evaluate_central_disparities(uint8_t *in, uint32_t image_width, uint32_t image_height, uint32_t *disparities,
                                  uint8_t n_disp_bins, uint8_t min_y, uint8_t max_y, uint8_t border);
void evaluate_central_disparities2(uint8_t *in, uint32_t image_width, uint32_t image_height, uint32_t *disparities,
                                   uint8_t n_disp_bins, uint8_t min_y, uint8_t max_y, uint8_t disp_threshold, uint8_t border);
void evaluate_central_disparities_bins(uint8_t *in, uint32_t image_width, uint32_t image_height,
                                       uint32_t disparity_range, uint32_t *disparities, uint8_t n_disp_bins, uint8_t min_y, uint8_t max_y, uint8_t border);
uint32_t evaluate_disparities_droplet(uint8_t *in, uint32_t image_width, uint32_t image_height);
void filter_disparity_map(uint8_t *in, uint8_t diff_threshold, uint32_t image_width, uint32_t image_height,
                          uint8_t min_y, uint8_t max_y);
uint16_t getFeatureImageLocations( uint8_t *current_image_buffer, uint8_t *in, uint8_t *out, uint8_t *target_location, uint32_t image_width, uint32_t image_height, uint8_t min_y, uint8_t max_y, uint16_t feature_count_limit);
uint16_t getFeatureImageLocations_old(uint8_t *in, uint8_t *out, uint32_t image_width, uint32_t image_height, uint8_t min_y, uint8_t max_y, uint16_t feature_count_limit);
void visualizeFeatureImageLocations( uint8_t *current_image_buffer, uint8_t *feature_image_locations, uint16_t nr_of_features, uint32_t image_width, uint16_t feature_count_limit);
void getFeatureXYZLocations(uint8_t *feature_image_locations, float *feature_XYZ_locations, uint16_t nr_of_features, uint32_t image_width, uint32_t image_height);
uint16_t visualizeBlobImageLocation(uint8_t *inI, uint8_t *inF, uint8_t *target_location, volatile uint16_t nr_of_features, uint32_t image_width, uint16_t feature_count_limit);


#endif
