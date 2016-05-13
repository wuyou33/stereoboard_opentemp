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
#ifndef DISP_RANGE_MAX
#define DISP_RANGE_MAX 20
#endif
#define DIVERGENCE_QUALITY_MEASURES_LENGTH 10

struct edge_hist_t {
  int32_t x[IMAGE_WIDTH];
  int32_t y[IMAGE_HEIGHT];
  int32_t frame_time;
  int16_t roll;
  int16_t pitch;
};

//Edge Flow calculated from previous frame (adaptive frame selection)
struct edge_flow_t {
  int32_t flow_x;
  int32_t div_x;
  int32_t flow_y;
  int32_t div_y;
};

struct displacement_t {
  int32_t x[IMAGE_WIDTH];
  int32_t y[IMAGE_HEIGHT];
};

struct covariance_t {
  int32_t C_flow_x;
  int32_t C_flow_y;
  int32_t C_div_x;
  int32_t C_div_y;
  int32_t C_height;
};

struct edgeflow_parameters_t {
  int8_t FOVX;
  int8_t FOVY;
  int8_t image_width;
  int8_t image_height;
  int8_t max_disparity_range;
  int8_t max_horizon;
  int8_t adapt_horizon;
  int8_t snapshot;
  int16_t snapshot_lenght;
  int8_t autopilot_mode;
  int8_t disparity_range;
  int8_t window_size;
  int8_t derotation;
  int8_t edge_flow_kalman;
  int32_t Q;
  int32_t R;
  uint8_t initialisedDivergence;
  int16_t alt_state_lisa;
  int16_t dphi;
  int16_t dtheta;
  int32_t RES;
  int32_t use_monocam;
};

struct edgeflow_results_t {
  struct edge_hist_t edge_hist[MAX_HORIZON];
  struct edge_hist_t edge_hist_snapshot;
  struct edge_flow_t edge_flow;
  struct edge_flow_t prev_edge_flow;
  struct displacement_t displacement;
  struct covariance_t covariance;
  uint8_t snapshot_is_taken;
  uint16_t snapshot_counter;
  uint8_t quality_measures_edgeflow[DIVERGENCE_QUALITY_MEASURES_LENGTH];
  uint8_t current_frame_nr;
  uint32_t R_height;
  uint32_t R_x;
  uint32_t R_y;
  int32_t avg_disp;
  int32_t avg_dist;
  int32_t prev_avg_dist;
  int32_t vel_x;
  int32_t vel_y;
  int32_t prev_vel_x;
  int32_t prev_vel_y;
  uint8_t previous_frame_offset[2];
  int32_t hz_x;
  int32_t hz_y;
};



// Global Functions divergence
void divergence_init(struct edgeflow_parameters_t *edgeflow_parameters, struct edgeflow_results_t *edgeflow_results,
                     const int8_t FOVX, const int8_t FOVY, int8_t image_width, int8_t image_height, int8_t use_monocam);
void divergence_total(uint8_t divergenceArray[], int16_t *stereocam_data_int16t, uint8_t stereocam_len,
                      uint8_t current_image_buffer[],
                      struct edgeflow_parameters_t *edgeflow_parameters, struct edgeflow_results_t *edgeflow_results);
int32_t divergence_calc_vel(struct edgeflow_parameters_t *edgeflow_parameters,
                            struct edgeflow_results_t *edgeflow_results);
void divergence_to_sendarray(uint8_t divergenceArray[24],
                             const struct edge_flow_t *edge_flow, uint8_t previous_frame_offset[2],
                             int32_t avg_dist, int32_t hz_x, int32_t vel_hor,
                             int32_t vel_ver,
                             uint8_t quality_measures_edgeflow[]);

// Calculation functions
void calculate_edge_flow(uint8_t in[], struct edgeflow_parameters_t *edgeflow_parameters,
                         struct edgeflow_results_t *edgeflow_results);
void image_difference(uint8_t *in, uint8_t *in_prev, uint8_t *out, uint16_t image_width, uint16_t image_height);
void calculate_edge_histogram(uint8_t *in, int32_t *edge_histogram, uint16_t image_width, uint16_t image_height,
                              char direction, char side, uint16_t edge_threshold);
uint32_t calculate_displacement(int32_t *edge_histogram, int32_t *edge_histogram_prev, int32_t *displacement,
                                uint16_t size, uint8_t window, uint8_t disp_range, int32_t der_shift);
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
uint8_t boundint8(int32_t value);
#endif /* DIVERGENCE_H_ */
