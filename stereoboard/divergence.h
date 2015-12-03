/*
 * divergence.h
 *
 *  Created on: Apr 9, 2015
 *      Author: knmcguire
 */

#ifndef DIVERGENCE_H_
#define DIVERGENCE_H_

#include "arm_math.h"
#include "main_parameters.h"

#ifndef MAX_HORIZON
#define MAX_HORIZON 10
#endif
#ifndef DISP_RANGE_MAX 20
#define DISP_RANGE_MAX 20
#endif

// #define MAX_FLOW 1.0
// #define RANSAC 1
// #define ADAPTIVE_THRES_EDGE 0

struct edge_hist_t {
  int32_t horizontal[IMAGE_WIDTH];
  int32_t vertical[IMAGE_HEIGHT];
  int32_t frame_time;
};

//Edge Flow calculated from previous frame (adaptive frame selection)
struct edge_flow_t {
  int32_t horizontal_flow;
  int32_t horizontal_div;
  int32_t vertical_flow;
  int32_t vertical_div;
};

struct displacement_t {
  int32_t horizontal[IMAGE_WIDTH];
  int32_t vertical[IMAGE_HEIGHT];
};

struct covariance_t {
  int32_t flow_x;
  int32_t flow_y;
  int32_t div_x;
  int32_t div_y;
  int32_t height;
};

void divergence_init();
void calculate_edge_flow_simple(uint8_t *in, int32_t *edge_histogram, int32_t *edge_histogram_prev,
                                int32_t *displacement, float *slope, float *yint, uint32_t image_width, uint32_t image_height);
void calculate_edge_flow(uint8_t *in, struct displacement_t *displacement, struct edge_flow_t *edge_flow,
                         struct edge_hist_t edge_hist[], int32_t *avg_disp, uint8_t *previous_frame_offset,
                         uint8_t current_frame_nr, uint8_t *quality_measures, uint8_t window_size, uint8_t disp_range, uint16_t edge_threshold,
                         uint16_t image_width, uint16_t image_height, uint16_t RES);
void image_difference(uint8_t *in, uint8_t *in_prev, uint8_t *out, uint16_t image_width, uint16_t image_height);
void calculate_edge_histogram(uint8_t *in, int32_t *edge_histogram, uint16_t image_width, uint16_t image_height,
                              char direction, char side, uint16_t edge_threshold);
uint32_t calculate_displacement(int32_t *edge_histogram, int32_t *edge_histogram_prev, int32_t *displacement,
                                uint16_t size, uint8_t window, uint8_t disp_range);
int32_t calculate_displacement_fullimage(int32_t *edge_histogram, int32_t *edge_histogram_2, uint16_t size,
    uint8_t disp_range);

uint32_t line_fit(int32_t *displacement, int32_t *Slope, int32_t *Yint, uint32_t image_width, uint32_t border,
                  uint16_t RES);
void line_fit_RANSAC(int32_t *displacement, int32_t *slope, int32_t *yInt, uint16_t size, uint32_t border,
                     uint32_t RES);

void totalKalmanFilter(struct covariance_t *coveriance, struct edge_flow_t *prev_edge_flow,
                       struct edge_flow_t *edge_flow, uint32_t Q, uint32_t R, uint32_t RES);
int32_t simpleKalmanFilter(int32_t *cov, int32_t previous_est, int32_t current_meas, int32_t Q, int32_t R, int32_t RES);

void visualize_divergence(uint8_t *in, int32_t *displacement, int32_t slope, int32_t yInt, uint32_t image_width,
                          uint32_t image_height);

//Helpfull functions
uint32_t getMinimum2(uint32_t *a, uint32_t n, uint32_t *min_error);
uint32_t getMaximum(uint32_t *a, uint32_t n);
uint32_t getMedian(int32_t *daArray, int32_t iSize);
uint32_t getMean(int32_t *daArray, int32_t iSize);
uint32_t getTotalIntensityImage(uint8_t *in, uint32_t image_height, uint32_t image_width);
uint32_t getAmountPeaks(int32_t *edgehist, uint32_t median, int32_t size);

#endif /* DIVERGENCE_H_ */
