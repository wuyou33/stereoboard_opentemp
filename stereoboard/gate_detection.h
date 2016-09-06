/*
 * gate_detection.h
 *
 *  Created on: Sep 5, 2016
 *      Author: Guido de Croon
 */

#ifndef STEREOBOARD_GATE_DETECTION_H_
#define STEREOBOARD_GATE_DETECTION_H_

#include "inttypes.h"
#include "data_types.h"

#define GOOD_FIT 0.04
#define BAD_FIT 0.12

  // main gate detection function:
  void gate_detection(struct image_i* disparity_image, float* x_center, float* y_center, float* radius, float* fitness, int initialize_fit_with_pars);

	// "private" functions:
	void convert_disparitymap_to_points(struct image_i* disparity_image);
	void fit_window_to_points(float* x0, float* y0, float* size0, float* fitness);
	float mean_distance_to_circle(float* genome);

  // square not implemented for now, but could easily be done:
	// float mean_distance_to_square();

	float distance_to_line(struct point_f Q1, struct point_f Q2, struct point_f P);
	float distance_to_segment(struct point_f Q1, struct point_f Q2, struct point_f P);

  // utility functions: should probably be placed in some other file:
	float get_random_number(void);
	float get_minimum(float* nums, int n_elements, int *index);
	float get_sum(float* nums, int n_elements);
  // drawing function:
  void draw_circle(struct image_i* Im, float x_center, float y_center, float radius, uint8_t* color);

  // calculating the color fit cannot be done with the current stereo output:
	// float check_color_fit();


#endif /* STEREOBOARD_GATE_DETECTION_H_ */
