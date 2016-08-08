/*
 * divergence.h
 *
 *  Created on: Apr 9, 2015
 *      Author: knmcguire
 */

#ifndef DIVERGENCE_H_
#define DIVERGENCE_H_

#if COMPILE_ON_LINUX
#include <inttypes.h>
#include <stdio.h>
#else
#include "arm_math.h"
#endif
#include "main_parameters.h"


#ifndef MAX_HORIZON
#define MAX_HORIZON 10
#endif
#ifndef DISP_RANGE_MAX
#define DISP_RANGE_MAX 20
#endif
#define DIVERGENCE_QUALITY_MEASURES_LENGTH 10



struct edge_hist_t {
  int32_t x[IMAGE_WIDTH];     // Edge_hist: edgehistogram in x-direction (image coordinates)
  int32_t y[IMAGE_HEIGHT];      // Edge_hist: edgehistogram in y-direction (image coordinates)
  int32_t stereo[IMAGE_WIDTH];      // Edge_hist: edgehistogram in y-direction (image coordinates)
  int32_t frame_time;       // Edge_hist: Frame time corresponding to the image from which the edge histogram is made
  int16_t roll;           // Edge_hist: roll position at the time
  int16_t pitch;          // Edge_hist: pitch position at the time
  int16_t yaw;            // Edge_hist: yaw position at the time
};

struct edge_flow_t {
  int32_t flow_x;         // Edge_flow: translational flow in x-direction (image coordinates)
  int32_t div_x;          // Edge_flow: divergence in x-direction (image coordinates)
  int32_t flow_y;         // Edge_flow: translational flow in y-direction (image coordinates)
  int32_t div_y;          // Edge_flow: divergence in y-direction (image coordinates)
};

struct displacement_t {
  int32_t x[IMAGE_WIDTH];     // Displacement: pixel displacement of edgehistograms (in time) in x-direction (image coordinates)
  int32_t y[IMAGE_HEIGHT];      // Displacement: pixel displacement of edgehistograms (in time) in Y-direction (image coordinates)
  int32_t stereo[IMAGE_WIDTH];    // Displacement: pixel displacement of edgehistograms (in stereo) in x-direction (image coordinates)
};

struct covariance_t {
  int32_t C_flow_x;         // Kalman: covariance matrix of translational flow in x-direction (image coordinates)
  int32_t C_flow_y;         // Kalman: covariance matrix of translational flow in y-direction (image coordinates)
  int32_t C_div_x;          // Kalman: covariance matrix of divergence in x-direction (image coordinates)
  int32_t C_div_y;          // Kalman: covariance matrix of divergence in y-direction (image coordinates)
  int32_t C_height;         // Kalman: covariance matrix of global
};

struct edgeflow_parameters_t {
  int8_t FOVX;                // Camera Parameters: Field of view of x axis
  int8_t FOVY;                // Camera Parameters: Field of view of y axis
  int16_t image_width;            // Camera Parameters: Image width in pixels
  int16_t image_height;           // Camera Parameters: Image height in pixels
  int16_t stereo_camera_seperation;     // Camera Parameters: How far the cameras are apart on a stereocamera
  int8_t max_disparity_range;       // Block matching: Maximum disparity range
  int8_t max_horizon;           // EdgeFlow: Maximum horizon for edgeflow to look back in time
  int8_t adapt_horizon;           // Edgeflow: 0 = off 1 = on, To use adaptive horizon comparison or only compare with the next immediate frame
  int8_t snapshot;              // Modes: 0 = off 1 = on, Take snapshot to compare next frames to (in this case edgehistograms)
  int16_t snapshot_lenght;          // Snapshot: How many loops you have to wait before taking another snapshot
  int8_t disparity_range;         // Block matching: Actual search disparity range for block matching
  int8_t window_size;           // Block matching: Window size of neighbouring values
  int8_t derotation;            // Modes: 0 = off 1 = on,  To derotate the flow based on IMU data  (received from the autopilot)
  int8_t kalman_on;             // Kalman Filter: Use it or not
  int32_t Q;                // Kalman Filter: Process noise parameter
  int32_t R;                // Kalman Filter: Measurement noise parameter
  int32_t RES;                // EdgeFlow: Resolution factor
  int32_t use_monocam;            // Modes: 0 = using stereocamera, 1 = using monocam
  int16_t stereo_shift;           // Edgeflow: necessary stereo_shift (from calibration) to obtain the right
  int16_t alt_state_lisa;         // Paparazzi: Receive altitude from state from autopilot
  int8_t autopilot_mode;          // Paparazzi: Receive autopilot mode from paparazzi

};

struct edgeflow_results_t {
  struct edge_hist_t edge_hist[MAX_HORIZON];      // Edgeflow: Stores an array of edgehistograms for a maximum horizon
  struct edge_hist_t edge_hist_snapshot;          // Edgeflow: Stores a snapshot edgehistogram
  struct edge_flow_t edge_flow;             // Edgeflow: Translational flow and divergence measured in current time step
  struct edge_flow_t
      prev_edge_flow;          // Edgeflow: Translational flow and divergence measured in previous time step
  struct displacement_t
      displacement;         // Edgeflow: Saves displacement of edgeflow in x and y direction (image coordinates) and stereo
  int32_t stereo_distance_per_column[IMAGE_WIDTH];    // Edgeflow: Stereo distance (from stereo displacement and camera parameters)
  int32_t prev_stereo_distance_per_column[IMAGE_WIDTH]; // Edgeflow: Previous stereo distance
  int32_t velocity_per_column[IMAGE_WIDTH];       // Edgeflow: Displacement x stereo distance x frequency per column
  int32_t velocity_stereo_mean;             // Edgeflow: Velocity as measured from the difference between stereo distances in time
  struct covariance_t covariance;           // Kalman: Covariance value for kalman filter
  uint8_t snapshot_is_taken;              // Snapshot: Measure if snapshot is taken
  uint16_t snapshot_counter;              // Snapshot: Counter to know how long ago the snapshot is taken
  uint8_t quality_measures_edgeflow[DIVERGENCE_QUALITY_MEASURES_LENGTH]; //Quality measures
  uint8_t current_frame_nr;               // Edgeflow: current frame number to indicate where the edge histogram was of the current time
  uint8_t previous_frame_offset[2];           // Edgeflow: previous frame offset for the y direction (image coordinates)
  int32_t avg_disp;                   // Edgeflow: displacement from full image matching
  int32_t avg_dist;                   // Edgeflow: distance from full image matching
  int32_t prev_avg_dist;                // Edgeflow: previous distance from full image matching
  int32_t vel_x_global;                 // Edgeflow: Velocity measured from flow on x axis (x direction, image coordinates, sideways)
  int32_t vel_y_global;                 // Edgeflow: Velocity measured from flow on y axis (y direction, image coordinates, upwards)
  int32_t vel_z_global;                 // Edgeflow: Velocity measured from divergence on x axis (z direction, image coordinates) (out of image)
  int32_t vel_x_pixelwise;                  // Edgeflow: Velocity measured of intercept of the line fit of velocity_per_column ( xdirection, image coordinates, sideways)
  int32_t vel_z_pixelwise;                  // Edgeflow: Velocity measured of slope of the line fit of velocity_per_column (zdirection, image coordinates, out of image)
  int32_t vel_x_stereo_avoid_pixelwise;         // Edgeflow: Avoid velocity based on stereo (xdirection, image coordinates, sideways)
  int32_t vel_z_stereo_avoid_pixelwise;         // Edgeflow: Avoid velocity based on stereo (zdirection, image coordinates, out of image)
  int32_t prev_vel_x_global;              // Edgeflow: Previous value of vel_x_global
  int32_t prev_vel_y_global;              // Edgeflow: Previous value of vel_y_global
  int32_t prev_vel_z_global;              // Edgeflow: Previous value of vel_z_global
  int32_t prev_vel_x_pixelwise;             // Edgeflow: Previous value of vel_x_pixelwise
  int32_t prev_vel_z_pixelwise;             // Edgeflow: Previous value of vel_z_pixelwise
  int32_t hz_x;                     // Edgeflow: Frames per second, taking the compare horizon into account (x-direction image coordinates)
  int32_t hz_y;                     // Edgeflow: Frames per second, taking the compare horizon into account (z-direction image coordinates)
  int16_t dphi;                     // Edgeflow: Difference in angles roll from one frame to another
  int16_t dtheta;                   // Edgeflow: Difference in angles pitch from one frame to another
  int16_t dpsi;                     // Edgeflow: Difference in angles yaw from one frame to another
};



// Global Functions divergence
void edgeflow_init(struct edgeflow_parameters_t *edgeflow_parameters, struct edgeflow_results_t *edgeflow_results,
                   const int8_t FOVX, const int8_t FOVY, int16_t image_width, int16_t image_height, int8_t use_monocam);
void edgeflow_total(uint8_t divergenceArray[], int16_t *stereocam_data_int16t, uint8_t stereocam_len,
                    uint8_t current_image_buffer[],
                    struct edgeflow_parameters_t *edgeflow_parameters, struct edgeflow_results_t *edgeflow_results);
int32_t edgeflow_calc_vel(struct edgeflow_parameters_t *edgeflow_parameters,
                          struct edgeflow_results_t *edgeflow_results);
void edgeflow_to_sendarray(uint8_t edgeflow_array[24], struct edgeflow_parameters_t *edgeflow_parameters,
                           struct edgeflow_results_t *edgeflow_results);

// Calculation functions
void calculate_edge_flow(uint8_t in[], struct edgeflow_parameters_t *edgeflow_parameters,
                         struct edgeflow_results_t *edgeflow_results);
void image_difference(uint8_t *in, uint8_t *in_prev, uint8_t *out, uint16_t image_width, uint16_t image_height);
void calculate_edge_histogram(uint8_t *in, int32_t *edge_histogram, uint16_t image_width, uint16_t image_height,
                              char direction, char side, uint16_t edge_threshold);
uint32_t calculate_displacement(int32_t *edge_histogram,
                                int32_t *edge_histogram_prev, int32_t *displacement, uint16_t size,
                                uint8_t window, uint8_t disp_range, int32_t der_shift);
uint32_t calculate_displacement_stereo(int32_t *edge_histogram,
                                       int32_t *edge_histogram_right, int32_t *displacement, uint16_t size,
                                       uint8_t window, uint8_t disp_range, int16_t stereo_shift);
int32_t calculate_displacement_fullimage(int32_t *edge_histogram,
    int32_t *edge_histogram_2, uint16_t size, uint8_t disp_range, int16_t stereo_shift);

void avoid_velocity_from_stereo(int32_t *stereo_distance_per_column, int32_t *vel_x_stereo_avoid_pixelwise,
                                int32_t *vel_z_stereo_avoid_pixelwise, int32_t max_velocity, int32_t size, int32_t border, int16_t stereo_shift);
uint32_t line_fit(int32_t *displacement, int32_t *Slope, int32_t *Yint, uint32_t image_width, uint32_t border,
                  uint16_t RES);
uint32_t weighted_line_fit(int32_t *displacement, uint8_t *faulty_distance, int32_t *divergence, int32_t *flow,
                           uint32_t size, uint32_t border,
                           uint16_t RES);
void line_fit_RANSAC(int32_t *displacement, int32_t *divergence, int32_t *flow,
                     uint8_t *faulty_distance, uint16_t size, uint32_t border, int32_t RES);

int32_t simpleKalmanFilter(int32_t *cov, int32_t previous_est, int32_t current_meas, int32_t Q, int32_t R, int32_t RES);
int32_t moving_fading_average(int32_t previous_est, int32_t current_meas, int32_t alpha, int32_t RES);

void visualize_divergence(uint8_t *in, int32_t *displacement, int32_t slope, int32_t yInt, uint32_t image_width,
                          uint32_t image_height);

//Help functions
uint32_t getMinimum2(uint32_t *a, uint32_t n, uint32_t *min_error);
uint32_t getMaximum(uint32_t *a, uint32_t n);
uint32_t getMedian(int32_t *daArray, int32_t iSize);
uint32_t getMean(int32_t *daArray, int32_t iSize);
uint32_t getTotalIntensityImage(uint8_t *in, uint32_t image_height, uint32_t image_width);
uint32_t getAmountPeaks(int32_t *edgehist, uint32_t median, int32_t size);
uint8_t boundint8(int32_t value);
#endif /* DIVERGENCE_H_ */
