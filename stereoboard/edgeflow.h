/*
 * divergence.h
 *
 *  Created on: Apr 9, 2015
 *      Author: knmcguire
 */

#ifndef DIVERGENCE_H_
#define DIVERGENCE_H_

#ifdef COMPILE_ON_LINUX
#include <inttypes.h>
#include <stdio.h>
#else
#include "arm_math.h"
#endif
#include "main_parameters.h"
#include "image.h"

#ifndef MAX_HORIZON
#define MAX_HORIZON 10
#endif
#define DIV_QUALITY_LENGTH 10

/*    Coordinate system stereocam (looking into the lens)
*    x      z
* <-----(( (*) ))              ((     )) = camera lens
*           |                     (*)    = arrow pointed outwards
*           | y                              (towards your direction)
*           V
*/

struct edge_hist_t {
  int32_t x[IMAGE_WIDTH];   // Edge_hist: edgehistogram in x-direction (image coordinates)
  int32_t y[IMAGE_HEIGHT];  // Edge_hist: edgehistogram in y-direction (image coordinates)
  int32_t frame_time;       // Edge_hist: Frame time corresponding to the image from which the edge histogram is made
  int16_t roll;             // Edge_hist: roll: rotation around the x-axis of the camera (along image width) [rad]
  int16_t pitch;            // Edge_hist: pitch: rotation around the y-axis of the camera (along image height)[rad]
  int16_t yaw;              // Edge_hist: yaw: rotation around the z-axis of the camera (perpendicular on frame plane) [rad]
};

struct edge_flow_t {
  int32_t flow_x;         // Edge_flow: translational flow in x-direction (image coordinates)
  int32_t div_x;          // Edge_flow: divergence in x-direction (image coordinates)
  int32_t flow_y;         // Edge_flow: translational flow in y-direction (image coordinates)
  int32_t div_y;          // Edge_flow: divergence in y-direction (image coordinates)
  int32_t scaled_flow_x;  // Edge_flow: divergence in y-direction (image coordinates)
  int32_t scaled_div;     // Edge_flow: divergence in y-direction (image coordinates)
};

struct displacement_t {
  int32_t x[IMAGE_WIDTH];           // Displacement: pixel displacement of edgehistograms (in time) in x-direction (image coordinates) [in pix/s]
  int32_t y[IMAGE_HEIGHT];          // Displacement: pixel displacement of edgehistograms (in time) in Y-direction (image coordinates) [in pix/s]
  uint32_t stereo[IMAGE_WIDTH];     // Displacement: pixel displacement of edgehistograms (in stereo) in x-direction (image coordinates) [in pix]
  uint32_t confidence[IMAGE_WIDTH]; // confidence of fit measured by slope of disparity error
};

struct covariance_t {
  int32_t C_flow_x;         // Kalman: covariance matrix of translational flow in x-direction (image coordinates)
  int32_t C_flow_y;         // Kalman: covariance matrix of translational flow in y-direction (image coordinates)
  int32_t C_div_x;          // Kalman: covariance matrix of divergence in x-direction (image coordinates)
  int32_t C_div_y;          // Kalman: covariance matrix of divergence in y-direction (image coordinates)
  int32_t C_height;         // Kalman: covariance matrix of global
};

//TODO: reduce structures edgeflow_parameters and edgeflow_results

struct edgeflow_parameters_t {
  int16_t fovx;                 // Camera Parameters: Field of view of x axis scaled by RES
  int16_t fovy;                 // Camera Parameters: Field of view of y axis scaled by RES
  int16_t img_width;            // Camera Parameters: Image width in pixels
  int16_t img_height;           // Camera Parameters: Image height in pixels
  int16_t camera_seperation;    // Camera Parameters: How far the cameras are apart on a stereocamera
  int8_t max_disparity_range;   // Block matching: Maximum disparity range
  int8_t max_horizon;           // EdgeFlow: Maximum horizon for edgeflow to look back in time
  int8_t adapt_horizon;         //  0 = off 1 = on, To use adaptive horizon comparison or only compare with the next immediate frame
  int8_t snapshot;              // Modes: 0 = off 1 = on, Take snapshot to compare next frames to (in this case edgehistograms)
  int16_t snapshot_length;      // Snapshot: How many loops you have to wait before taking another snapshot
  int8_t disparity_range;       // Block matching: Actual search disparity range for block matching
  int8_t window_size;           // Block matching: Window size of neighbouring values
  int8_t derotation;            // Modes: 0 = off 1 = on,  To derotate the flow based on IMU data  (received from the autopilot)
  int8_t filter_type;           // Filter method: NONE, KALMAN, MOVING_AVGERAGE
  int32_t Q;                    // Kalman Filter: Process noise parameter
  int32_t R;                    // Kalman Filter: Measurement noise parameter
  int32_t alpha;                // Moving Average: Low pass filter constant
  int32_t RES;                  // EdgeFlow: Resolution factor
  int32_t use_monocam;          // Modes: 0 = using stereocamera, 1 = using monocam
  int16_t stereo_shift;         //  necessary stereo_shift (from calibration) to obtain the right
  int16_t alt_state_lisa;       // Paparazzi: Receive altitude from state from autopilot
  int8_t autopilot_mode;        // Paparazzi: Receive autopilot mode from paparazzi
};

//TODO: reduce structure size

struct edgeflow_results_t {
  struct edge_hist_t edge_hist[MAX_HORIZON];  // Stores an array of edgehistograms for a maximum horizon
  int32_t edge_hist_right[IMAGE_WIDTH];       // Edgehistogram in x-direction for right image (image coordinates)
  struct edge_hist_t edge_hist_snapshot;      // Stores a snapshot edgehistogram
  struct edge_flow_t edge_flow;               // Translational flow and divergence measured in current time step
  struct edge_flow_t prev_edge_flow;          // Translational flow and divergence measured in previous time step
  struct displacement_t
    displacement;         // Saves displacement of edgeflow in x and y direction (image coordinates) and stereo
  int32_t dist_per_column[IMAGE_WIDTH];       // Stereo distance (from stereo displacement and camera parameters)
  int32_t vel_per_column[IMAGE_WIDTH];        // Displacement x stereo distance x frequency per column
  int32_t vel_stereo_mean;                    // Velocity as measured from the difference between stereo distances in time
  struct covariance_t covariance;             // Covariance value for kalman filter
  uint8_t snapshot_is_taken;                  // Measure if snapshot is taken
  uint16_t snapshot_counter;                  // Counter to know how long ago the snapshot is taken
  uint8_t quality_meas[DIV_QUALITY_LENGTH];   //Quality measures
  uint8_t current_frame_nr;                   // Current frame number to indicate where the edge histogram was of the current time
  uint8_t prev_frame_offset_x;                // Previous frame offset for the y direction (image coordinates)
  uint8_t prev_frame_offset_y;
  uint8_t prev_frame_x;
  uint8_t prev_frame_y;
  int32_t avg_disp;                       // Displacement from full image matching
  int32_t avg_dist;                       // Distance from full image matching in m * RES
  int32_t prev_avg_dist;                  //  previous distance from full image matching
  int32_t vel_x_global;                   //  Velocity measured from flow on x axis (x direction, image coordinates, sideways)
  int32_t vel_y_global;                   //  Velocity measured from flow on y axis (y direction, image coordinates, upwards)
  int32_t vel_z_global;                   //  Velocity measured from divergence on x axis (z direction, image coordinates) (out of image)
  int32_t vel_x_pixelwise;                //  Velocity measured of intercept of the line fit of velocity_per_column ( xdirection, image coordinates, sideways)
  int32_t vel_z_pixelwise;                //  Velocity measured of slope of the line fit of velocity_per_column (zdirection, image coordinates, out of image)
  int32_t vel_x_stereo_avoid_pixelwise;   //  Avoid velocity based on stereo (xdirection, image coordinates, sideways)
  int32_t vel_z_stereo_avoid_pixelwise;   //  Avoid velocity based on stereo (zdirection, image coordinates, out of image)
  int32_t prev_vel_x_global;              //  Previous value of vel_x_global
  int32_t prev_vel_y_global;              //  Previous value of vel_y_global
  int32_t prev_vel_z_global;              //  Previous value of vel_z_global
  int32_t prev_vel_x_pixelwise;           //  Previous value of vel_x_pixelwise
  int32_t prev_vel_z_pixelwise;           //  Previous value of vel_z_pixelwise
  int32_t hz_x;                     //  Frames per second scaled by RES, taking the compare horizon into account (x-direction image coordinates)
  int32_t hz_y;                     //  Frames per second scaled by RES, taking the compare horizon into account (z-direction image coordinates)
  int16_t dphi;                     //  Difference in angles roll from one frame to another
  int16_t dtheta;                   //  Difference in angles pitch from one frame to another
  int16_t dpsi;                     //  Difference in angles yaw from one frame to another
  uint8_t obst_mode;                //  Behavior based on EdgeStereo
  int32_t distance_closest_obstacle;   // Avoidance: Distance of closes obstacle
  uint8_t obstacle_detect[IMAGE_WIDTH];   // Avoidance: Isolated obstacles with distances
};

// Global Functions divergence
void edgeflow_init(struct edgeflow_parameters_t *edgeflow_parameters, struct edgeflow_results_t *edgeflow_results,
                   int16_t img_w, int16_t img_h, int8_t use_monocam);
void edgeflow_total(uint8_t *divergenceArray, int16_t *stereocam_data_int16t, uint8_t data_len,
                    uint8_t *current_image_buffer, struct edgeflow_parameters_t *edgeflow_parameters,
                    struct edgeflow_results_t *edgeflow_results);
void edgeflow_calc_vel(struct edgeflow_parameters_t *edgeflow_parameters, struct edgeflow_results_t *edgeflow_results);
void edgeflow_to_sendarray(uint8_t *edgeflow_array, struct edgeflow_parameters_t *edgeflow_parameters,
                           struct edgeflow_results_t *edgeflow_results);

// Calculation functions
void calculate_edge_flow(uint8_t in[], struct edgeflow_parameters_t *edgeflow_parameters,
                         struct edgeflow_results_t *edgeflow_results);
void calculate_edge_hist(uint8_t *in, int32_t *edge_hist, uint16_t img_w, uint16_t img_h, char direction, char side);
uint32_t calculate_displacement(int32_t *edge_hist, int32_t *edge_hist_prev, int32_t *displacement, uint16_t size,
                                uint8_t window, uint8_t disp_range, int32_t der_shift, int32_t scale);
uint32_t calculate_disparity(int32_t *edge_hist_l, int32_t *edge_hist_r, uint32_t *disparity, uint16_t size,
                             uint8_t window, uint8_t disp_range, int16_t stereo_shift, uint32_t *confidence);
int32_t calculate_disparity_fullimage(int32_t *edge_hist_l, int32_t *edge_hist_r, uint16_t size, uint8_t disp_range,
                                      int16_t stereo_shift);
void avoid_velocity_from_stereo(int32_t *stereo_distance_per_column, int32_t *vel_x_stereo_avoid_pixelwise,
                                int32_t *vel_z_stereo_avoid_pixelwise, int32_t max_velocity, int32_t size, int32_t border, int16_t stereo_shift);

// line fit funcitons
uint32_t line_fit(int32_t *displacement, int32_t *slope, int32_t *intercept, uint32_t size, uint32_t border,
                  int32_t RES, int32_t x_offset);
uint32_t weighted_line_fit(int32_t *points, int32_t *weight, int32_t *slope, int32_t *intercept, uint32_t size,
                           uint32_t border, uint16_t RES, int32_t x_offset);
uint32_t constrained_line_fit(int32_t *points, int32_t *weight, int32_t *slope, int32_t *intercept, uint32_t size,
                              uint32_t border, uint16_t RES, struct point_t p0);
void line_fit_RANSAC(int8_t *points, int32_t *divergence, int32_t *flow, uint8_t *faulty_distance, uint16_t size,
                     uint32_t border, int32_t RES);

// filter functions
int32_t simpleKalmanFilter(int32_t *cov, int32_t previous_est, int32_t current_meas, int32_t Q, int32_t R, int32_t RES);
int32_t moving_fading_average(int32_t previous_est, int32_t current_meas, int32_t alpha, int32_t RES);

// misc funcitons
void visualize_divergence(uint8_t *in, int8_t *displacement, int32_t slope, int32_t yInt, uint32_t img_w,
                          uint32_t img_h);
uint8_t evaluate_edgeflow_stereo(int32_t *dist_per_column, int32_t size, int32_t border);
int32_t edgestereo_obstacle_detection(int32_t *dist_per_column, uint8_t *obstacle_detect, int32_t size, int32_t border);

//Help functions
uint32_t getMinimum2(uint32_t *a, uint32_t n, uint32_t *min_error);
void find_minimum(int32_t *pSrc, uint8_t blockSize, int32_t *pResult, uint32_t *pIndex, uint8_t min_index);
uint32_t getMaximum(uint32_t *a, uint32_t n);
uint32_t getMedian(int32_t *daArray, int32_t iSize);
uint32_t getMean(int32_t *daArray, int32_t iSize);
uint32_t getTotalIntensityImage(uint8_t *in, uint32_t image_height, uint32_t image_width);
uint32_t getAmountPeaks(int32_t *edgehist, uint32_t median, int32_t size);
uint32_t getMeanDisp(int32_t *a, int32_t n, int32_t border);

#endif /* DIVERGENCE_H_ */
