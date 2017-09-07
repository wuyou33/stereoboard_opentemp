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
#include "image.h"

#define GOOD_FIT 4
#define BAD_FIT 127
#define MAX_POINTS 1000

/* Gate structure */
struct gate_t {
  int16_t x;            ///< The image x coordinate of the gate centre
  int16_t y;            ///< The image y coordinate of the gate centre
  int16_t sz;           ///< Half the image size of the gate
  uint8_t q;            ///< Gate quality
  uint8_t n_sides;      ///< How many sides are orange (to prevent detecting a small gate in the corner of a big one partially out of view).
  int16_t sz_left;      ///< Half the image size of the left side
  int16_t sz_right;     ///< Half the image size of the right side
  float rot;          ///< Rotation angle of gate [-pi/2, pi/2]
};

void pprz_send_gate(struct gate_t *gate, float depth);

void gate_set_intensity(uint8_t Y_m, uint8_t Y_M);
void gate_set_color(uint8_t Y_m, uint8_t Y_M, uint8_t U_m, uint8_t U_M, uint8_t V_m, uint8_t V_M);

// main gate detection function:
float gen_gate_detection(struct image_t *image, struct point_t *roi, struct gate_t *gate, uint32_t *integral_image);
//, float *angle_1, float *angle_2, float *psi, uint16_t *s_left, uint16_t *s_right);

// "private" functions:
uint32_t convert_image_to_points(struct image_t *image, struct point_t roi_min, struct point_t roi_max,
                                 struct point_t points[], uint8_t weights[]);
float gen_run(struct gate_t *gate0, struct gate_t *gen_gate, uint32_t *integral_image);
float get_outlier_ratio(int16_t *genome, float total_sum_weights);
float mean_distance_to_circle(int16_t *genome, struct point_t *points, uint32_t n_points);
float mean_distance_to_square(int16_t *genome, struct point_t *points, uint32_t n_points);
float mean_distance_to_rectangle(int16_t *genome, struct point_t *points, uint32_t n_points);
float mean_distance_to_polygon(int16_t *genome, struct point_t *points, uint32_t n_points);

float distance_to_line(struct point_f Q1, struct point_f Q2, struct point_f P);
float distance_to_segment(struct point_f Q1, struct point_f Q2, struct point_f P);
float distance_to_vertical_segment(struct point_f Q1, struct point_f Q2, struct point_f P);
float distance_to_horizontal_segment(struct point_f Q1, struct point_f Q2, struct point_f P);

// drawing functions:
void draw_stick(struct image_i *Im, float x_center, float y_center, float radius, uint8_t *color);

bool snake_gate_detection(struct image_t *img, struct gate_t *best_gate, bool run_gen_alg, uint16_t *bins,
                          struct point_t *roi, uint32_t *integral_image);

// calculating the color fit cannot be done with the current stereo output:
// float check_color_fit();

float bounding_box_score(int16_t *genome, struct point_t *points, uint32_t n_points, uint32_t *integral_image);

#endif /* STEREOBOARD_GATE_DETECTION_H_ */
