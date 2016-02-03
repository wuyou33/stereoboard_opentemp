/*
 * odometry.h
 *
 *  Created on: Dec 21, 2015
 *      Author: Sjoerd
 */

#ifndef ODOMETRY_H_
#define ODOMETRY_H_

#include <arm_math.h>
#include "main_parameters.h"

void odometry_select_features(uint8_t *in, uint8_t *features, uint16_t feature_count, uint16_t feature_number_select,
                              uint32_t image_width, uint32_t image_height);
void odometry_extract_features(uint8_t *in, q15_t *out, uint8_t *features, uint16_t feature_count,
                               uint16_t feature_number_select, uint32_t image_width, uint32_t image_height, uint8_t feature_window_size_half);
void precompute_rotation_coefficients(float32_t *rotation_coefficients, float rotation_step_size,
                                      int rotation_step_number, int number_of_rotations);
void odometry_translate_and_match_features(uint8_t *images, q15_t *feature_window_data,
    uint8_t *feature_image_coordinates, uint16_t features_ROT_number, uint8_t feature_window_size_half,
    float32_t *rotation_coefficients, int number_of_rotations, uint32_t image_width);

#endif /* ODOMETRY_H_ */
