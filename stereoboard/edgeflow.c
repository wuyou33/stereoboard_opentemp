/*
 * divergence.c
 *
 *  Created on: Apr 9, 2015
 *      Author: knmcguire
 */

#include "edgeflow.h"

#include <stdlib.h>

#ifdef COMPILE_ON_LINUX
#include <string.h>
#include <iostream>
#include <stdio.h>
#define FOVX 1.001819   // 57.4deg = 1.001819 rad
#define FOVY 0.776672   // 44.5deg = 0.776672 rad
using namespace std;

/* define arm functions used */
void arm_fill_q31(int32_t val, int32_t *array, uint32_t size)
{
  uint32_t i;
  for (i = 0; i < size; i++) {
    array[i] = val;
  }
}
#else
#include "camera_type.h"
#include "sys_time.h"
#endif

#include "stereo_math.h"

enum FilterType {
  NONE,
  KALMAN,
  MOVING_AVGERAGE
};

#ifndef DISPARITY_RANGE
#define DISPARITY_RANGE 15
#endif

#ifndef WINDOW_SIZE
#define WINDOW_SIZE 8
#endif

// used as a temporary storage for Sum of Absolute difference computations
static int32_t __ccmram SAD[IMAGE_WIDTH] = {0};
static int32_t __ccmram SAD_right[IMAGE_WIDTH] = {0};
static int32_t __ccmram run_sum1[IMAGE_WIDTH] = {0};
static int32_t __ccmram run_sum2[IMAGE_WIDTH] = {0};

/* edgeflow_total: The total function for the edgeflow algorithm
 * \param divergenceArray is Array containing information to send to lisa-s
 * \param stereocam_data_int16  is the data the stereocam receives from the lisa s
 * \param stereocam_len is the length of the received data array, to make sure that it receives data.
 * \param current_image_buffer is the image of the present image step
 * \param edgeflow_params is a struct containing al the parameters for edgeflow
 * \param edgeflow_results is a struct containing the resulting values of edgeflow
 * */
void edgeflow_total(uint8_t edgeflowArray[], int16_t *stereocam_data_int16,
                    uint8_t data_len, uint8_t *current_image_buffer,
                    struct edgeflow_parameters_t *edgeflow_params,
                    struct edgeflow_results_t *edgeflow_results)
{
  if (data_len > 0) {
    edgeflow_results->edge_hist[edgeflow_results->current_frame_nr].roll = stereocam_data_int16[0]; //in RES * [rad]
    edgeflow_results->edge_hist[edgeflow_results->current_frame_nr].pitch = stereocam_data_int16[1]; //in RES * [rad]
    edgeflow_results->edge_hist[edgeflow_results->current_frame_nr].yaw = stereocam_data_int16[2];  //in RES * [rad]

    edgeflow_params->derotation = (int8_t) stereocam_data_int16[3];
    /*
    edgeflow_params->derotation = stereocam_data_int16[2];
    edgeflow_params->adapt_horizon = stereocam_data_int16[3];
    edgeflow_params->window_size = stereocam_data_int16[4];
    edgeflow_params->disparity_range = stereocam_data_int16[5];
    edgeflow_params->snapshot = stereocam_data_int16[6];
    edgeflow_params->stereo_shift = stereocam_data_int16[7];

    edgeflow_params->autopilot_mode = stereocam_data_int16[8];
    */

  } else {
    edgeflow_results->edge_hist[edgeflow_results->current_frame_nr].roll = 0;
    edgeflow_results->edge_hist[edgeflow_results->current_frame_nr].pitch = 0;
    edgeflow_results->edge_hist[edgeflow_results->current_frame_nr].yaw =  0;
    edgeflow_params->derotation = 0;
  }

  calculate_edge_flow(current_image_buffer, edgeflow_params, edgeflow_results);

  edgeflow_calc_vel(edgeflow_params, edgeflow_results);

#ifndef COMPILE_ON_LINUX
  edgeflow_to_sendarray(edgeflowArray, edgeflow_params, edgeflow_results);
#endif
}

/*  edgeflow_init: Initialize structures edgeflow_params and results
 * \param edgeflow_params is a struct containing al the parameters for edgeflow
 * \param edgeflow_results is a struct containing the resulting values of edgeflow
 * \param FOVX and FOVY are the field of view of the camera
 * \param img_w and img_h are the pixel dimensions of the image
 * \param use_monocam is a boolean that indicates if a monocam or stereocam is used
 * */
void edgeflow_init(struct edgeflow_parameters_t *edgeflow_params, struct edgeflow_results_t *edgeflow_results,
                   int16_t img_w, int16_t img_h, int8_t use_monocam)
{
  memset(SAD, 0, IMAGE_WIDTH);
  memset(SAD_right, 0, IMAGE_WIDTH);
  memset(run_sum1, 0, IMAGE_WIDTH);
  memset(run_sum2, 0, IMAGE_WIDTH);

  edgeflow_params->RES = 100;
  edgeflow_params->fovx = (int32_t)(FOVX * edgeflow_params->RES);
  edgeflow_params->fovy = (int32_t)(FOVY * edgeflow_params->RES);
  edgeflow_params->img_height = img_h;
  edgeflow_params->img_width = img_w;
  edgeflow_params->max_horizon = MAX_HORIZON;

#ifdef COMPILE_ON_LINUX
  edgeflow_params->stereo_shift =  0;
#else
  edgeflow_params->stereo_shift =  DISPARITY_OFFSET_HORIZONTAL;
#endif
  edgeflow_params->camera_seperation =  6;       // in cm

  edgeflow_params->max_disparity_range = DISPARITY_RANGE;
  edgeflow_params->Q = (int32_t)(0.2 * edgeflow_params->RES);
  edgeflow_params->R = (int32_t)(1. * edgeflow_params->RES);
  edgeflow_params->alpha = (int32_t)(0.6 * edgeflow_params->RES);

  edgeflow_params->use_monocam = use_monocam;

  edgeflow_params->disparity_range = DISPARITY_RANGE; // this is not used as range but as max, maybe rename?
  edgeflow_params->window_size = WINDOW_SIZE;
  edgeflow_params->derotation = 0;
  edgeflow_params->adapt_horizon = 1;
  edgeflow_params->snapshot = 0;
  edgeflow_params->filter_type = NONE;

  // Initialize variables
  edgeflow_results->current_frame_nr = 0;
  edgeflow_results->prev_frame_x = 0;
  edgeflow_results->prev_frame_y = 0;
  edgeflow_results->prev_frame_offset_x = 1;
  edgeflow_results->prev_frame_offset_y = 1;
  // edgeflow_results->edge_hist[(MAX_HORIZON - 1) % MAX_HORIZON].frame_time = 0;  // used for sensing first run

  memset(edgeflow_results->displacement.x, 0, sizeof(edgeflow_results->displacement.x));
  memset(edgeflow_results->displacement.y, 0, sizeof(edgeflow_results->displacement.y));
  memset(edgeflow_results->displacement.stereo, 0, sizeof(edgeflow_results->displacement.stereo));
  memset(edgeflow_results->dist_per_column, 0, sizeof(edgeflow_results->dist_per_column));
  memset(edgeflow_results->vel_per_column, 0, sizeof(edgeflow_results->vel_per_column));

  edgeflow_results->hz_x = 25 * edgeflow_params->RES;
  edgeflow_results->hz_y = 25 * edgeflow_params->RES;

  // Edgeflow: initialize previous translational flow in x-direction (image coordinates)
  edgeflow_results->edge_flow.flow_x = edgeflow_results->prev_edge_flow.flow_x = 0;
  // Edgeflow: initialize previous divergence in x-direction (image coordinates)
  edgeflow_results->edge_flow.div_x = edgeflow_results->prev_edge_flow.div_x = 0;
  // Edgeflow: initialize previous translational flow in y-direction (image coordinates)
  edgeflow_results->edge_flow.flow_y = edgeflow_results->prev_edge_flow.flow_y = 0;
  // Edgeflow: initialize previous translational flow in y-direction (image coordinates)
  edgeflow_results->edge_flow.div_y = edgeflow_results->prev_edge_flow.div_y = 0;

  // Initialize kalman
  edgeflow_results->covariance.C_flow_x = (int32_t)(0.2 * edgeflow_params->RES);
  edgeflow_results->covariance.C_flow_y = (int32_t)(0.2 * edgeflow_params->RES);
  edgeflow_results->covariance.C_div_x = (int32_t)(0.2 * edgeflow_params->RES);
  edgeflow_results->covariance.C_div_y = (int32_t)(0.2 * edgeflow_params->RES);
  edgeflow_results->covariance.C_height = (int32_t)(0.2 * edgeflow_params->RES);

  edgeflow_results->avg_dist = 0;
  edgeflow_results->avg_disp = 0;
  edgeflow_results->prev_avg_dist = 0;
  edgeflow_results->vel_x_global = 0;
  edgeflow_results->prev_vel_x_global = 0;
  edgeflow_results->vel_y_global = 0;
  edgeflow_results->prev_vel_y_global = 0;
  edgeflow_results->vel_z_global = 0;
  edgeflow_results->prev_vel_z_global = 0;
  edgeflow_results->vel_x_pixelwise = 0;
  edgeflow_results->prev_vel_x_pixelwise = 0;
  edgeflow_results->vel_z_pixelwise = 0;
  edgeflow_results->prev_vel_z_pixelwise = 0;

  edgeflow_results->snapshot_is_taken = 0;
  edgeflow_params->snapshot_length = 300;
  edgeflow_results->snapshot_counter = edgeflow_params->snapshot_length + 1;
}

/*  edgeflow_to_sendarray: This function fills up the array that is send to the lisa -s
 * \param edgeflow_array is the array to be send back to the autopilot
 * \param edgeflow_params is a struct containing al the parameters for edgeflow
 * \param edgeflow_results is a struct containing the resulting values of edgeflow
 * */
void edgeflow_to_sendarray(uint8_t edgeflow_array[],
                           struct edgeflow_parameters_t *edgeflow_params,
                           struct edgeflow_results_t *edgeflow_results)
{

  /*EDGEFLOW_DEBUG defines which type of information is send through the edgelflowArray.
   For debugging, intermediate results are necessary to simplify the programming
   When EDGEFLOW_DEBUG is defined, it will send through the current histogram, the previous and the calculated displacement
   when it is not defined, it will send through flow, divergence and velocity*/

#ifdef EDGEFLOW_DEBUG
  uint8_t edgeflow_debug_array[128 * 5];
  uint8_t *current_frame_nr = &edgeflow_results->current_frame_nr;
  uint8_t *previous_frame_offset = &edgeflow_results->previous_frame_offset;

  uint8_t previous_frame_x = (*current_frame_nr - previous_frame_offset[0] + MAX_HORIZON) %
                             MAX_HORIZON; // wrap index

  uint8_t x = 0;
  uint8_t edge_hist_int8[128];
  uint8_t edge_hist_prev_int8[128];
  uint8_t displacement_int8[128];
  uint8_t plot2[128];
  uint8_t plot3[128];
  for (x = 0; x < 128; x++) {

    plot3[x] = boundint8((edgeflow_results->displacement.stereo[x] * 10 + 127));

    plot2[x] = boundint8((edgeflow_results->displacement.x[x] * 20 + 127));
    edge_hist_int8[x] = boundint8((edgeflow_results->vel_per_column[x] / 100 + 127));
    displacement_int8[x] = boundint8((edgeflow_results->stereo_distance_per_column[x] / 10  + 127));

    edge_hist_prev_int8[x] = boundint8((edgeflow_results->vel_x_pixelwise * (128 * 100 / 104)
                                        + edgeflow_results->vel_z_pixelwise * (x - 64)) / 100 + 127);

  }

  memcpy(edgeflow_debug_array, &edge_hist_int8, 128 * sizeof(uint8_t)); // copy quality measures to output array
  memcpy(edgeflow_debug_array + 128, &edge_hist_prev_int8,
         128 * sizeof(uint8_t));// copy quality measures to output array
  memcpy(edgeflow_debug_array + 128 * 2, &displacement_int8,
         128 * sizeof(uint8_t));// copy quality measures to output array
  memcpy(edgeflow_debug_array + 128 * 3, &plot2,
         128 * sizeof(uint8_t));// copy quality measures to output array
  memcpy(edgeflow_debug_array + 128 * 4, &plot3,
         128 * sizeof(uint8_t));// copy quality measures to output array

  memcpy(edgeflow_array, edgeflow_debug_array, 128 * 5 * sizeof(uint8_t));

#else
  edgeflow_array[0] = (edgeflow_results->edge_flow.div_x >> 8) & 0xff;
  edgeflow_array[1] = (edgeflow_results->edge_flow.div_x) & 0xff;
  edgeflow_array[2] = (edgeflow_results->edge_flow.flow_x >> 8) & 0xff;
  edgeflow_array[3] = (edgeflow_results->edge_flow.flow_x) & 0xff;
  edgeflow_array[4] = (edgeflow_results->edge_flow.div_y >> 8) & 0xff;
  edgeflow_array[5] = (edgeflow_results->edge_flow.div_y) & 0xff;
  edgeflow_array[6] = (edgeflow_results->edge_flow.flow_y >> 8) & 0xff;
  edgeflow_array[7] = (edgeflow_results->edge_flow.flow_y) & 0xff;

  edgeflow_array[8] = boundint8(edgeflow_results->distance_closest_obstacle / 10);
  edgeflow_array[9] = boundint8(edgeflow_results->hz_x / edgeflow_params->RES);

  edgeflow_array[10] = (edgeflow_results->vel_x_global >> 8) & 0xff;
  edgeflow_array[11] = (edgeflow_results->vel_x_global) & 0xff;
  edgeflow_array[12] = (edgeflow_results->vel_y_global >> 8) & 0xff;
  edgeflow_array[13] = (edgeflow_results->vel_y_global) & 0xff;
  edgeflow_array[14] = (edgeflow_results->vel_z_global >> 8) & 0xff;
  edgeflow_array[15] = (edgeflow_results->vel_z_global) & 0xff;

  edgeflow_array[16] = (edgeflow_results->vel_x_pixelwise >> 8) & 0xff;
  edgeflow_array[17] = (edgeflow_results->vel_x_pixelwise) & 0xff;
  edgeflow_array[18] = (edgeflow_results->vel_z_pixelwise >> 8) & 0xff;
  edgeflow_array[19] = (edgeflow_results->vel_z_pixelwise) & 0xff;

  edgeflow_array[20] = (edgeflow_results->vel_x_stereo_avoid_pixelwise >> 8) & 0xff;
  edgeflow_array[21] = (edgeflow_results->vel_x_stereo_avoid_pixelwise) & 0xff;
  edgeflow_array[22] = (edgeflow_results->vel_z_stereo_avoid_pixelwise >> 8) & 0xff;
  edgeflow_array[23] = (edgeflow_results->vel_z_stereo_avoid_pixelwise) & 0xff;

  /* edgeflow_array[20] = ((uint8_t)(edgeflow_results->vel_x_global / 10) + 127);
   edgeflow_array[21] = ((uint8_t)(edgeflow_results->vel_y_global / 10) + 127);

   edgeflow_array[22] = ((uint8_t)(edgeflow_results->vel_x_pixelwise / 10) + 127);
   edgeflow_array[23] = ((uint8_t)(edgeflow_results->vel_z_pixelwise / 10) + 127);*/

#endif
}

/*  edgeflow_calc_vel: calculate height and by edgeflow
 *\param edgeflow_params is a struct containing al the parameters for edgeflow
 *\param edgeflow_results is a struct containing the resulting values of edgeflow
 * */
void edgeflow_calc_vel(struct edgeflow_parameters_t *edgeflow_params, struct edgeflow_results_t *edgeflow_results)
{
  // Assign pointers to variable values in results (for smaller calculations)
  int32_t *vel_x_global = &edgeflow_results->vel_x_global;
  int32_t *vel_y_global = &edgeflow_results->vel_y_global;
  int32_t *vel_z_global = &edgeflow_results->vel_z_global;

  int32_t *vel_x_pixelwise = &edgeflow_results->vel_x_pixelwise;
  int32_t *vel_z_pixelwise = &edgeflow_results->vel_z_pixelwise;
  int32_t *prev_vel_x_global = &edgeflow_results->prev_vel_x_global;
  int32_t *prev_vel_y_global = &edgeflow_results->prev_vel_y_global;
  int32_t *prev_vel_z_global = &edgeflow_results->prev_vel_z_global;
  int32_t *prev_vel_x_pixelwise = &edgeflow_results->prev_vel_x_pixelwise;
  int32_t *prev_vel_z_pixelwise = &edgeflow_results->prev_vel_z_pixelwise;

  //int32_t *prev_vel_y = &edgeflow_results->prev_vel_y;
  int32_t *avg_disp = &edgeflow_results->avg_disp;
  int32_t *avg_dist = &edgeflow_results->avg_dist;
  int32_t *prev_avg_dist = &edgeflow_results->prev_avg_dist;
  struct edge_flow_t *edge_flow = &edgeflow_results->edge_flow;
  struct covariance_t *covariance = &edgeflow_results->covariance;

  // Assign pointers to parameters
  int32_t monocam = edgeflow_params->use_monocam;
  int32_t fov_x = edgeflow_params->fovx;
  int32_t fov_y = edgeflow_params->fovy;
  int32_t RES = edgeflow_params->RES;
  uint32_t R = edgeflow_params->R;
  uint32_t Q = edgeflow_params->Q;
  int16_t img_w = edgeflow_params->img_width;
  int16_t img_h = edgeflow_params->img_height;

  // disparity to distance in dm given 6cm dist between cams and Field of View (FOV) of 60deg
  // d =  Npix*cam_separation /(2*disp*tan(FOV/2))
  // d = 0.06*128 / (2*tan(disp*1.042/2))
  // d = 0.06*128 / (2*disp*1.042/2)
  // d = RES*0.06*128 / (disp*RES*1.042)
  // d = RES*0.06*PIX / (disp*FOVX)
  if (*avg_disp > 0) {
    *avg_dist = (RES * RES * edgeflow_params->camera_seperation * img_w) / (*avg_disp * fov_x * 100);
  } else {
    // use avg_disp = 1
    *avg_dist = (RES * RES * edgeflow_params->camera_seperation * img_w) / (fov_x * 100);
  }
  if (monocam) {
    *avg_dist = 1;
  }

  //filter height: TODO: use height directly from lisa s edgeflow_params->snapshot = 0;
  switch (edgeflow_params->filter_type) {
    case KALMAN:
      // todo use a variable covariance value maybe based on fit quality
      *avg_dist = simpleKalmanFilter(&(covariance->C_height), *prev_avg_dist, *avg_dist, Q, R, RES);
      break;
    case MOVING_AVGERAGE:
      *avg_dist =  moving_fading_average(*prev_avg_dist, *avg_dist, edgeflow_params->alpha, RES);
      break;
    default:
      break;
  }

  // Calculate the stereo distance and flow x distance per column
  int32_t x;
  int32_t border = edgeflow_params->disparity_range + edgeflow_params->window_size;

  int32_t weight[IMAGE_WIDTH];
  for (x = border; x < img_w - border; x++) {
    if (edgeflow_results->displacement.stereo[x] != 0) {
      edgeflow_results->dist_per_column[x] = (RES * RES * (int32_t)edgeflow_params->camera_seperation
                                              * img_w) / (edgeflow_results->displacement.stereo[x] * fov_x * 100); // RES * RES / RES [m]

      // Calculate stereo distance per column
      edgeflow_results->vel_per_column[x] = (edgeflow_results->dist_per_column[x] * edgeflow_results->displacement.x[x]) / RES; // RES * RES / RES

      weight[x] = 1;    // todo test effect of using real weighting
    } else {
      edgeflow_results->vel_per_column[x] = 0;
      edgeflow_results->dist_per_column[x] = 0;
      weight[x] = 0;
    }
  }

  // Detect the closest obstacle in EdgeStereo
  edgeflow_results->distance_closest_obstacle = edgestereo_obstacle_detection(
        edgeflow_results->dist_per_column, edgeflow_results->obstacle_detect, img_w, border);

  // calculate avoidance manouvers
  avoid_velocity_from_stereo(edgeflow_results->dist_per_column,
                             &edgeflow_results->vel_x_stereo_avoid_pixelwise, &edgeflow_results->vel_z_stereo_avoid_pixelwise, 50, img_w,
                             border, edgeflow_params->stereo_shift);

  edgeflow_results->vel_stereo_mean = getMean(edgeflow_results->vel_per_column, img_w);

  weighted_line_fit(edgeflow_results->vel_per_column, weight, &edge_flow->scaled_div, &edge_flow->scaled_flow_x,
                    img_w, border, RES, 0);

  // change axis system from left-top to centre-centre
  edge_flow->scaled_flow_x += edge_flow->scaled_div * img_w / 2;

  *vel_x_pixelwise = edge_flow->scaled_flow_x * fov_x / (RES * RES * img_w);
  *vel_z_pixelwise = -edge_flow->scaled_div / RES;

#if EDGEFLOW_USE_HEIGHT_AUTOPILOT
  *avg_dist = edgeflow_params->alt_state_lisa;
#else
  // we do a bit of resolution gymnastics here...
  *vel_x_global = (edge_flow->flow_x * (*avg_dist) / RES) * fov_x / (RES * RES * img_w);
  *vel_y_global = (edge_flow->flow_y * (*avg_dist) / RES) * fov_y / (RES * RES * img_h);
  *vel_z_global = -edge_flow->div_x * (*avg_dist) / (RES * RES);
#endif

  // filter output
  switch (edgeflow_params->filter_type) {
    case KALMAN:
      // todo use a variable covariance value maybe based on fit quality
      *vel_x_global = simpleKalmanFilter(&(covariance->C_flow_x), *prev_vel_x_global, *vel_x_global, Q, R, RES);
      *vel_y_global = simpleKalmanFilter(&(covariance->C_flow_y), *prev_vel_y_global, *vel_y_global, Q, R, RES);
      *vel_z_global = simpleKalmanFilter(&(covariance->C_div_x), *prev_vel_z_global, *vel_z_global, Q, R, RES);
      *vel_x_pixelwise = simpleKalmanFilter(&(covariance->C_flow_x), *prev_vel_x_pixelwise, *vel_x_pixelwise, Q, R, RES);
      *vel_z_pixelwise = simpleKalmanFilter(&(covariance->C_flow_y), *prev_vel_z_pixelwise, *vel_z_pixelwise, Q, R, RES);
      break;
    case MOVING_AVGERAGE:
      *vel_x_global =  moving_fading_average(*prev_vel_x_global, *vel_x_global, edgeflow_params->alpha, RES);
      *vel_y_global =  moving_fading_average(*prev_vel_y_global, *vel_y_global, edgeflow_params->alpha, RES);
      *vel_x_pixelwise =  moving_fading_average(*prev_vel_x_pixelwise, *vel_x_pixelwise, edgeflow_params->alpha, RES);
      *vel_z_pixelwise =  moving_fading_average(*prev_vel_z_pixelwise, *vel_z_pixelwise, edgeflow_params->alpha, RES);
      break;
    default:
      break;
  }

  // Store previous values
  *prev_avg_dist = *avg_dist;
  *prev_vel_x_global = *vel_x_global;
  *prev_vel_y_global = *vel_y_global;
  *prev_vel_z_global = *vel_z_global;
  *prev_vel_x_pixelwise = *vel_x_pixelwise;
  *prev_vel_z_pixelwise = *vel_z_pixelwise;
}

/*  calculate_edge_flow: calculate the global optical flow by edgeflow
 * \param in[] is an array containing the pixel intensities of the (stereo)image
 * \param edgeflow_params is a struct containing al the parameters for edgeflow
 * \param edgeflow_results is a struct containing the resulting values of edgeflow
 *
 *   TODO: reintergrate snapshot (doesn't work very well yet!!)
 *
 * */
void calculate_edge_flow(uint8_t *in, struct edgeflow_parameters_t *edgeflow_params,
                         struct edgeflow_results_t *edgeflow_results)
{
  struct displacement_t *displacement = &edgeflow_results->displacement;
  struct edge_hist_t *edge_hist = edgeflow_results->edge_hist;
  struct edge_flow_t *edge_flow = &edgeflow_results->edge_flow;

  int32_t img_w = edgeflow_params->img_width;
  int32_t img_h = edgeflow_params->img_height;
  int32_t RES = edgeflow_params->RES;
  uint8_t disp_range = edgeflow_params->disparity_range;
  uint8_t window_size = edgeflow_params->window_size;

  // check that inputs within allowable ranges
  if (disp_range > DISPARITY_RANGE) {
    disp_range = DISPARITY_RANGE;
  }



  // Define arrays and pointers for edge histogram and displacements
  int32_t *edge_hist_x = edge_hist[edgeflow_results->current_frame_nr].x;
  int32_t *edge_hist_x_right =  edgeflow_results->edge_hist_right;
  int32_t *edge_hist_y = edge_hist[edgeflow_results->current_frame_nr].y;

  // Calculate Edge Histogram
  calculate_edge_hist(in, edge_hist_x, img_w, img_h, 'x', 'l');
  calculate_edge_hist(in, edge_hist_y, img_w, img_h, 'y', 'l');
  calculate_edge_hist(in, edge_hist_x_right, img_w, img_h, 'x', 'r');

  // compute previous frame number relative to dynamic parameters
  edgeflow_results->prev_frame_x = (edgeflow_results->current_frame_nr - edgeflow_results->prev_frame_offset_x
                                    + MAX_HORIZON) %  MAX_HORIZON; // wrap index
  edgeflow_results->prev_frame_y = (edgeflow_results->current_frame_nr - edgeflow_results->prev_frame_offset_y
                                    + MAX_HORIZON) %  MAX_HORIZON; // wrap index

  //store the time of the frame
  // Calculate velocity TODO:: Find a way to use extract function correctly for this, since the results is different with than without
#ifdef COMPILE_ON_LINUX
  printf("num frames back, x: %d, y: %d\n", edgeflow_results->prev_frame_offset_x, edgeflow_results->prev_frame_offset_y);
  edgeflow_results->hz_x = RES * 7 / edgeflow_results->prev_frame_offset_x;
  edgeflow_results->hz_y = RES * 7 / edgeflow_results->prev_frame_offset_y;
#else
  edgeflow_results->hz_x = (RES * TIMER_TICKS_PER_SEC)
                           / calc_abs_time_interval(edge_hist[edgeflow_results->current_frame_nr].frame_time,
                               edge_hist[edgeflow_results->prev_frame_x].frame_time);
  edgeflow_results->hz_y = (RES * TIMER_TICKS_PER_SEC)
                           / calc_abs_time_interval(edge_hist[edgeflow_results->current_frame_nr].frame_time,
                               edge_hist[edgeflow_results->prev_frame_y].frame_time);
#endif

  int32_t *prev_edge_hist_x, *prev_edge_hist_y;
  //TODO: Snapshot commented out, have to debug!

  // copy previous edge histogram based on previous frame number
  /* if (edgeflow_results->snapshot_is_taken == 1 && edgeflow_params->snapshot == 1
   && (edgeflow_params->autopilot_mode == 10 || edgeflow_params->autopilot_mode == 11
   || edgeflow_params->autopilot_mode == 12)) {
   prev_edge_hist_x = edgeflow_results->edge_hist_snapshot.x;
   prev_edge_hist_y = edgeflow_results->edge_hist_snapshot.y;
   edgeflow_results->snapshot_counter++;
   } else {*/
  prev_edge_hist_x = edge_hist[edgeflow_results->prev_frame_x].x;
  prev_edge_hist_y = edge_hist[edgeflow_results->prev_frame_y].y;
  //}

  /*if (edgeflow_params->snapshot == 1 && (edgeflow_results->snapshot_is_taken == 0
   || edgeflow_results->snapshot_counter > edgeflow_params->snapshot_lenght)) {
   memcpy(&edgeflow_results->edge_hist_snapshot.x, edge_hist_x, sizeof(int32_t) * img_w);
   memcpy(&edgeflow_results->edge_hist_snapshot.y, edge_hist_y, sizeof(int32_t) * img_h);
   edgeflow_results->snapshot_counter = 0;
   edgeflow_results->snapshot_is_taken = 1;
   }*/

  //calculate angle diff [RAD * RES]

  // TODO: implementing derotation like this is very low resolution...
  int16_t der_shift_x = 0;
  int16_t der_shift_y = 0;

  //TODO: test with paparazzi implementation if rotation is done correctly
  if (edgeflow_params->derotation) {
    int16_t roll_prev = edge_hist[edgeflow_results->prev_frame_y].roll;
    int16_t pitch_prev = edge_hist[edgeflow_results->prev_frame_x].pitch;

    der_shift_y = (roll_prev - edge_hist[edgeflow_results->current_frame_nr].roll) * img_h / (edgeflow_params->fovy);
    der_shift_x = (pitch_prev - edge_hist[edgeflow_results->current_frame_nr].pitch) * img_w / (edgeflow_params->fovx);
  }
  // Calculate displacement
  uint32_t error_hor = calculate_displacement(edge_hist_x, prev_edge_hist_x, displacement->x, img_w, window_size,
                       disp_range, der_shift_x, edgeflow_results->hz_x);

  uint32_t error_ver = calculate_displacement(edge_hist_y, prev_edge_hist_y, displacement->y, img_h, window_size,
                       disp_range, der_shift_y, edgeflow_results->hz_y);

  calculate_disparity(edge_hist_x, edge_hist_x_right, displacement->stereo, img_w, window_size, disp_range,
                      edgeflow_params->stereo_shift, displacement->confidence);

  /*edgeflow_results->avg_disp = calculate_disparity_fullimage(edge_hist_x, edge_hist_x_right, img_w, disp_range,
                               edgeflow_params->stereo_shift);*/

  edgeflow_results->avg_disp = getMeanDisp(displacement->stereo, img_w, window_size + disp_range);

  // Fit a linear line
  uint32_t line_error_fit_hor = line_fit(displacement->x, &edge_flow->div_x, &edge_flow->flow_x, img_w,
                                         window_size + disp_range, RES, 0);
  uint32_t line_error_fit_ver = line_fit(displacement->y, &edge_flow->div_y, &edge_flow->flow_y, img_h,
                                         window_size + disp_range, RES, 0);

  // change axis system from left-top to centre-centre
  edge_flow->flow_x += edge_flow->div_x * img_w / 2;
  edge_flow->flow_y += edge_flow->div_y * img_h / 2;

  //Calculate and Store quality values
  uint32_t totalIntensity = getTotalIntensityImage(in, img_w, img_h);
  uint32_t mean_hor = getMean(edge_hist_x, img_w);
  uint32_t mean_ver = getMean(edge_hist_y, img_h);
  uint32_t median_hor = getMedian(edge_hist_x, img_w);
  uint32_t median_ver = getMedian(edge_hist_y, img_h);
  uint32_t amountPeaks_hor = getAmountPeaks(edge_hist_x, 500, img_w);
  uint32_t amountPeaks_ver = getAmountPeaks(edge_hist_y, 500, img_h);

  edgeflow_results->quality_meas[0] = boundint8(totalIntensity / 20000);
  edgeflow_results->quality_meas[1] = boundint8(mean_hor / 20);
  edgeflow_results->quality_meas[2] = boundint8(mean_ver / 20);
  edgeflow_results->quality_meas[3] = boundint8(median_hor / 20);
  edgeflow_results->quality_meas[4] = boundint8(median_ver / 20);
  edgeflow_results->quality_meas[5] = boundint8(amountPeaks_hor);
  edgeflow_results->quality_meas[6] = boundint8(amountPeaks_ver);
  edgeflow_results->quality_meas[7] = boundint8(line_error_fit_hor / 10);
  edgeflow_results->quality_meas[8] = boundint8(line_error_fit_ver / 10);

  int32_t *pointer = (int32_t *) edgeflow_results->quality_meas + 9;
  pointer[0] = error_hor;
  pointer[1] = error_ver;

  /* if (edgeflow_results->snapshot_is_taken == 1 && edgeflow_params->snapshot == 0) {
   edgeflow_results->snapshot_counter = edgeflow_params->snapshot_lenght + 1;
   edgeflow_results->snapshot_is_taken = 0;
   }*/

  // move the indices for the edge hist structure
  edgeflow_results->current_frame_nr = (edgeflow_results->current_frame_nr + 1) % MAX_HORIZON;

  // update previous frame offset for next computation
  if (MAX_HORIZON > 2 && (edgeflow_params->adapt_horizon == 1)) { //&& edgeflow_params->snapshot == 0) {
    uint32_t flow_mag_x, flow_mag_y;
    uint32_t div_mag_x, div_mag_y;
    flow_mag_x = abs(edge_flow->flow_x);
    flow_mag_y = abs(edge_flow->flow_y);
    div_mag_x = abs(edge_flow->div_x);
    div_mag_y = abs(edge_flow->div_y);

    uint32_t min_flow = RES * RES * disp_range / 8;
    uint32_t max_flow = RES * RES * disp_range / 2;

    // Increment or decrement previous frame offset based on previous measured flow.
    if ((flow_mag_x > max_flow || div_mag_x > 2 * max_flow / img_w) &&
        edgeflow_results->prev_frame_offset_x > 1) {
      edgeflow_results->prev_frame_offset_x--;
    } else if ((flow_mag_x < min_flow && div_mag_x < min_flow / (2 * img_w))
               && edgeflow_results->prev_frame_offset_x < MAX_HORIZON - 1) {
      edgeflow_results->prev_frame_offset_x++;
    }
    if ((flow_mag_y > max_flow  || div_mag_y > 2 * max_flow / img_h) &&
        edgeflow_results->prev_frame_offset_y > 1) {
      edgeflow_results->prev_frame_offset_y--;
    } else if ((flow_mag_y < min_flow && div_mag_y < min_flow / (2 * img_h))
               && edgeflow_results->prev_frame_offset_y < MAX_HORIZON - 1) {
      edgeflow_results->prev_frame_offset_y++;
    }
  }
}

/*  calculate_edge_histogram: calculate_edge_histogram calculates the image gradient of the images and makes a edge feature histogram
 * \param uint8_t in[] is an array containing the pixel intensities of the (stereo)image
 * \param edge_histogram is an array containing the summed up gradient intensities
 * \param image width
 * \param image height
 * \param direction indicates if the edge histogam is made for the x or y direction of the image
 * \param side indicates if the left side or right side stereo image is used (l = left, r = right)
 * \param edge_histogram is a threshold which determines if a gradient is a edge or not
 * */
void calculate_edge_hist(uint8_t *in, int32_t *edge_hist, uint16_t img_w, uint16_t img_h, char direction, char side)
{
  int32_t sobel[3] = {0};
  uint32_t y = 0, x = 0, idx;
  uint32_t image_width2 = BYTES_PER_PIXEL * img_w;

  // set pixel offset based on which image needed from interlaced image
  uint32_t px_offset = 0;
  if (side == 'l') {
    px_offset = 0;
  } else if (side == 'r') {
    px_offset = 1;
  } else
    while (1)
      ; // let user know something is wrong

  // compute edge histogram
  if (direction == 'x') {
    // set values that are not visited
    for (x = 0; x < img_w; x++) {
      edge_hist[x] = 0;
      sobel[2] = 0;
      idx = BYTES_PER_PIXEL * x + px_offset;
      for (y = 0; y < img_h; y++) {
        // sum pixels over entire column
        sobel[2] += in[idx];
        idx += image_width2;
      }
      if (x > 1 && sobel[0] > 0 && sobel[2] > 0) {
        // compute Sobel sum
        edge_hist[x - 1] = abs(sobel[2] - sobel[0]);
      }
      // shift sobel sum
      sobel[0] = sobel[1];
      sobel[1] = sobel[2];
    }
  } else if (direction == 'y') {
    // set values that are not visited
    for (y = 0; y < img_h; y++) {
      edge_hist[y] = 0;
      sobel[2] = 0;
      idx = y * image_width2 + px_offset;
      for (x = 0; x < img_w; x++) {
        // sum pixels over entire column
        sobel[2] += in[idx];
        idx += BYTES_PER_PIXEL;
      }
      if (y > 1 && sobel[0] > 0 && sobel[2] > 0) {
        // compute Sobel sum
        edge_hist[y - 1] = abs(sobel[2] - sobel[0]);
      }
      // shift sobel sum
      sobel[0] = sobel[1];
      sobel[1] = sobel[2];
    }
  } else
    while (1)
      ;  // hang to show user something isn't right
}

/* calculate_displacement: Calculate_displacement calculates the displacement between two histograms in time
 * \param edge_histogram is an array containing the summed up gradient intensities
 * \param edge_histogram_prev contains the edge_histogram from the previous timeframe
 * \param displacement is an array that contains the pixel displacements of the compared edgehistograms
 * \param size indicates the size of array (important for x and y direction)
 * \param window indicates the pixel size of the window of neighboring pixels
 * \param disp_range indicates how far the block matching algorithm should search
 * \param der_shift is the pixel shift caused by rotation of the camera
 * */
uint32_t calculate_displacement(int32_t *edge_hist, int32_t *edge_hist_prev, int32_t *displacement, uint16_t size,
                                uint8_t window, uint8_t disp_range, int32_t der_shift, int32_t scale)
{
  int32_t c;
  int32_t x;
  int32_t SAD_temp;

  int32_t W = window;
  int32_t D = disp_range;
  uint32_t sum_error = 0;
  uint8_t SHIFT_TOO_FAR = 0;

  uint16_t border_left = W + D;
  uint16_t border_right = size - W - D;

  if (der_shift > 0) {
    border_left += der_shift;
  } else if (der_shift < 0) {
    border_right += der_shift;
  }

  if (border_left >= border_right || abs(der_shift) >= 10) {
    SHIFT_TOO_FAR = 1;
  }

  if (!SHIFT_TOO_FAR) {
    arm_fill_q31(INT32_MAX, SAD, IMAGE_WIDTH);

    for (c = -D; c <= D; c++) {
      // compute running sum of SAD for all pixels at current disparity
      run_sum1[border_left - W] = abs(edge_hist[D] - edge_hist_prev[D + c + der_shift]);
      for (x = border_left - W + 1; x < border_right + W; x++) {
        run_sum1[x] = run_sum1[x - 1] + abs(edge_hist[x] - edge_hist_prev[x + c + der_shift]);
      }

      // store displacement with minimum SAD error
      // todo: it is possible to combine this for loop with the previous one...
      for (x = border_left; x < border_right; x++) {
        SAD_temp = run_sum1[x + W] - run_sum1[x - W - 1];
        if (SAD_temp < SAD[x] || (SAD_temp == SAD[x] && (abs(scale * c) < abs(displacement[x])))) {
          sum_error += SAD_temp - SAD[x] * (c != -D);
          SAD[x] = SAD_temp;
          displacement[x] = scale * c;
        }
      }
    }
  }

  return sum_error;
}

/* calculate_displacement_stereo: Calculate_displacement calculates the displacement between two histograms in stereo
 * \param edge_histogram is an array containing the summed up gradient intensities
 * \param edge_histogram_right contains the edge_histogram from the previous timeframe
 * \param displacement is an array that contains the pixel displacements of the compared edgehistograms
 * \param size indicates the size of array (important for x and y direction)
 * \param window indicates the pixel size of the window of neighboring pixels
 * \param disp_range indicates how far the block matching algorithm should search
 * \param stereo_shift is the pixel shift between the stereocameras, determined by calibration
 * */
uint32_t calculate_disparity(int32_t *edge_hist_l, int32_t *edge_hist_r, uint32_t *disparity, uint16_t size,
                             uint8_t window, uint8_t disp_range, int16_t stereo_shift, uint32_t *confidence)
{
  int32_t c, xl, xr;
  int32_t SAD_temp, mean_temp;
  int32_t mean[IMAGE_WIDTH] = {0};
  int32_t *var = confidence;

  int8_t disp_right[IMAGE_WIDTH];

  int32_t W = window;
  int32_t D = disp_range;
  uint32_t sum_error = 0;

  int16_t border_left = W + D;
  int16_t border_right = size - W;

  if (stereo_shift > 0) {
    border_left += stereo_shift;
  } else if (stereo_shift < 0) {
    border_right += stereo_shift;
  }

  arm_fill_q31(INT32_MAX, SAD, IMAGE_WIDTH);
  arm_fill_q31(INT32_MAX, SAD_right, IMAGE_WIDTH);
  arm_fill_q31(0, var, IMAGE_WIDTH);

  for (c = 0; c <= D; c++) {
    // compute running sum of SAD for all pixels at current disparity
    run_sum1[border_left - W] = abs(edge_hist_l[border_left - W] - edge_hist_r[border_left - W - c - stereo_shift]);
    run_sum2[border_left - W - D] = abs(edge_hist_l[border_left - W - D + c] -
                                        edge_hist_r[border_left - W - D - stereo_shift]);
    for (xl = border_left - W + 1, xr = xl - D; xl < border_right + W; xl++, xr++) {
      run_sum1[xl] = run_sum1[xl - 1] + abs(edge_hist_l[xl] - edge_hist_r[xl - c - stereo_shift]);
      run_sum2[xr] = run_sum2[xr - 1] + abs(edge_hist_l[xr + c] - edge_hist_r[xr - stereo_shift]);
    }

    // compute SAD over window [x-W,x+W] for each pixel using the running sum
    for (xl = border_left, xr = xl - D; xl < border_right; xl++, xr++) {
      SAD_temp = run_sum1[xl + W] - run_sum1[xl - W - 1];

      // compute variance
      mean_temp = mean[xl];
      mean[xl] += (SAD_temp - mean[xl]) / (c + 1);
      var[xl] += mean_temp * (SAD_temp - mean[xl]);

      // store disparity with minimum SAD error for left image
      if (SAD_temp < SAD[xl]) {
        sum_error += SAD_temp - SAD[xl] * (c != 0);
        SAD[xl] = SAD_temp;
        disparity[xl] = c;
      }
      // Same for right image
      SAD_temp = run_sum2[xr + W] - run_sum2[xr - W - 1];
      if (SAD_temp < SAD_right[xr]) {
        SAD_right[xr] = SAD_temp;
        disp_right[xr] = c;
      }
    }
  }

  for (xl = border_left; xl < border_right; xl++) {
    if (disparity[xl] != disp_right[xl - disparity[xl]]) {
      // occluded image column
      disparity[xl] = 0;//disparity[xl-1];
      confidence[xl] = 0;
    } else if (SAD[xl] != 0) {
      confidence[xl] /= D * SAD[xl]; // scale the variance with the minimum error
    } else {
      confidence[xl] = INT32_MAX;
    }
  }

  return sum_error;
}

/* calculate_displacement_fullimage: Calculate_displacement calculates the displacement between two histograms of the full image
 * \param edge_histogram is an array containing the summed up gradient intensities
 * \param edge_histogram_prev constraints the edge_histogram from the previous timeframe
 * \param displacement is an array that contains the pixel displacements of the compared edgehistograms
 * \param size indicates the size of array (important for x and y direction)
 * \param disp_range indicates how far the block matching algorithm should search
 * \param stereo_shift is the pixel shift between the stereocameras, determined by calibration
 */
int32_t calculate_disparity_fullimage(int32_t *edge_hist_l, int32_t *edge_hist_r, uint16_t size, uint8_t D,
                                      int16_t stereo_shift)
{
  int32_t c, x;
  int32_t min_error, min_index;

  uint16_t border_left = D;
  uint16_t border_right = size;

  if (stereo_shift > 0) {
    border_left += stereo_shift;
  } else if (stereo_shift < 0) {
    border_right += stereo_shift;
  }

  // TODO check if one loop can be replaced by line diff
  for (c = 0; c <= D; c++) {
    SAD[c] = 0;
    for (x = border_left; x < border_right; x++) {
      SAD[c] += abs(edge_hist_l[x] - edge_hist_r[x - c - stereo_shift]);
    }
  }
  // ignore the zero disparity
  find_minimum(&SAD[1], D, &min_error, &min_index, D);
  return min_index + 1;
}

/* avoid_velocity_from_stereo: calculate the wanted velocity to avoid an near object based on the stereo_distance
 * \param stereo_distance_per_column Stereo distance (from stereo displacement and camera parameters)
 * \param  vel_x_stereo_avoid_pixelwise is the wanted avoid velocity based on stereo (xdirection, image coordinates, sideways)
 * \param  vel_z_stereo_avoid_pixelwise is the wanted avoid velocity based on stereo (zdirection, image coordinates, out of image)
 * \param max_velocity is the maximum velocity for avoidance
 * \param size is the  size of stereo_distance_per_column
 * \param border is the search window + search distance used in blockmatching
 * \param stereo_shift is the pixel shift between the stereocameras, determined by calibration
 * */
void avoid_velocity_from_stereo(int32_t *stereo_distance_per_column, int32_t *vel_x_stereo_avoid_pixelwise,
                                int32_t *vel_z_stereo_avoid_pixelwise, int32_t max_velocity, int32_t size, int32_t border, int16_t stereo_shift)
{
  int32_t x;
  int32_t vel_x_temp = 0;
  int32_t vel_z_temp = 0;
  int32_t total_size = size - 2 * border;

  /* Summing up the max velocity divided by the stereo_distance_per_column
   * If the max_velocity is divided by the stereo_distance_per_column, which means the closer the stereo disparity is
   * the smaller the avoid velocity for that column becomes. This is done 3 parts of the image, where the
   * left [0 <-> 1/3 size], the middle [1/3 <-> 2/3 size] and right part [ 2/3<-> 1 size] of the image,
   * The left and right side are used to calculated the sideways avoidance velocity
   * The middle part is used to calculate the backwards avoidance velocity
   *
   * Note: for left and right part, the avoid velocity per column are balanced between each other.
   * */
  for (x = border; x < size - border; x++) {
    if (stereo_distance_per_column[x - stereo_shift / 2] != 0) {
      if (x < (total_size / 3 + border)) {
        // Avoid velocity for left side of image per column (negative)
        vel_x_temp += - max_velocity * 100 / stereo_distance_per_column[x - stereo_shift / 2];

      } else {
        if (x < (total_size * 2 / 3 + border)) {
          // Avoid velocity for middle of image per column
          if (stereo_distance_per_column[x] < 200) {
            vel_z_temp += - max_velocity * 100 / stereo_distance_per_column[x - stereo_shift / 2];
          }
        } else {
          // Avoid velocity for right part of image per column (positive)
          vel_x_temp +=  max_velocity * 100 / stereo_distance_per_column[x - stereo_shift / 2];
        }
      }
    }
  }
  //Average avoid velocity for each part of the image
  *vel_x_stereo_avoid_pixelwise = vel_x_temp / (total_size * 2 / 3); //left and right part
  *vel_z_stereo_avoid_pixelwise = vel_z_temp / (total_size  / 3);  //middle part

}
/* edgestereo_obstacle_detection: Detect the closest isolated obstacle, used for obstacle avoidance
 * \param stereo_distance_per_column Stereo distance (from stereo displacement and camera parameters)
 * \param stereo_distance_filtered Stereo distance filtered by binary filter operation
 * \param size is the  size of stereo_distance_per_column
 * \param border is the search window + search distance used in blockmatching
 * \return distance to the closest isolated obstacle
 *
 * */
int32_t edgestereo_obstacle_detection(int32_t *stereo_distance_per_column, uint8_t *stereo_distance_filtered,
                                      int32_t size, int32_t border)
{

  int32_t x, c;
  int32_t max_distance_obstacle = 500; // in [cm]
  uint8_t obstc_thres = 5;




  // Measure where obstacles are within detection range and turn it into a booleean
  for (x = border; x < size - border; x++) {

    if (stereo_distance_per_column[x] < max_distance_obstacle && stereo_distance_per_column[x] != 0) {
      stereo_distance_filtered[x] = 1;
    } else {
      stereo_distance_filtered[x] = 0;
    }

  }
  // Erosion of binary array
  uint8_t min_value;
  uint8_t morph_value = 5;
  uint8_t stereo_distance_filtered_temp[IMAGE_WIDTH] = {0};
  for (x = border; x < size - border; x++) {

    min_value = 1;
    for (c = -morph_value; c <= morph_value; c++) {
      if (min_value > stereo_distance_filtered[x + c]) {
        min_value = stereo_distance_filtered[x + c];
      }
    }
    stereo_distance_filtered_temp[x] =  min_value;
  }

  memcpy(stereo_distance_filtered, stereo_distance_filtered_temp, sizeof(stereo_distance_filtered_temp));

  //Dilation
  uint8_t max_value;
  for (x = border; x < size - border - 1; x++) {

    max_value = 0;
    for (c = -morph_value; c <= morph_value; c++) {
      if (max_value < stereo_distance_filtered[x + c]) {
        max_value = stereo_distance_filtered[x + c];
      }
    }
    stereo_distance_filtered_temp[x] =  max_value;

  }
  memcpy(stereo_distance_filtered, stereo_distance_filtered_temp, sizeof(stereo_distance_filtered_temp));

  // Seperate obstacles with a large distance difference
  for (x = border; x < size - border; x++) {

    if (abs(stereo_distance_per_column[x] - stereo_distance_per_column[x + 1]) > 55) {
      stereo_distance_filtered[x] = 0;
    }
  }


  //calculate the distance of the closest obstacle
  int32_t counter = 0;
  int32_t distance_sum = 0;
  int32_t average_distance = 0;
  int32_t closest_average_distance = 1500;


  for (x = border; x < size - border; x++) {
//if obstacle is detected, start counting how many you see in a row, sum op the distances
    if (stereo_distance_filtered[x] == 1) {
      counter ++;
      distance_sum += stereo_distance_per_column[x];

    } else {
      if (counter > obstc_thres) {
        average_distance = distance_sum / counter;

        if (closest_average_distance > average_distance) {
          closest_average_distance = average_distance;
        }

      }
      counter = 0;
      distance_sum = 0;
    }
  }

  return closest_average_distance;


}

/* evaluate_edgeflow_stereo: Calculate the droplet mode based on edgeStereo, based on the balance of the obstacles on the screen
 * \param stereo_distance_per_column Stereo distance (from stereo displacement and camera parameters)
 * \param size is the  size of stereo_distance_per_column
 * \param border is the search window + search distance used in blockmatching
 * \return droplet mode
 *
 *TODO: Re-evaluate this function, to remove the droplet states and to make it more generic
 * */
uint8_t evaluate_edgeflow_stereo(int32_t *stereo_distance_per_column, int32_t size, int32_t border)
{

  int32_t x;
  uint8_t nr_distance_2m_left = 0;
  uint8_t nr_distance_2m_right = 0;
  uint8_t nr_distance_1m_left = 0;
  uint8_t nr_distance_1m_right = 0;

  uint8_t obst_thres = 5;

  for (x = border; x < size - border; x++) {

    if (stereo_distance_per_column[x] != 0) {
      if (stereo_distance_per_column[x] <= 200 && stereo_distance_per_column[x] > 100) {
        if (x < size / 2) {
          nr_distance_2m_left++;
        }
        if (x >= size / 2) {
          nr_distance_2m_right++;
        }
      }
      if (stereo_distance_per_column[x] <= 100) {
        if (x < size / 2) {
          nr_distance_1m_left++;
        }
        if (x >= size / 2) {
          nr_distance_1m_right++;
        }
      }
    }
  }

  if ((nr_distance_1m_left + nr_distance_1m_right) > obst_thres) {
    if (nr_distance_1m_left <= nr_distance_1m_right) {
      return 21; // STOP! and turn right until save area
    }
    if (nr_distance_1m_left > nr_distance_1m_right) {
      return 22; // STOP! and turn left until save area
    }
  } else {
    if ((nr_distance_2m_left + nr_distance_2m_right) > obst_thres) {
      if (nr_distance_2m_left <= nr_distance_2m_right) {
        return 11;  // keep flying with same speed but turn slightly right
      }
      if (nr_distance_2m_left > nr_distance_2m_right) {
        return 12;  // keep flying with same speed but turn slightly left
      }
    } else {
      return 4;  // keep flying straight with same speed
    }
  }
  return 0;
}


/* line_fit: fits a line using least squares to the histogram disparity map
 * \param displacement is an array that contains the pixel displacements of the compared edgehistograms
 * \param divergence is slope of the optical flow field
 * \param slope is intercept of the optical flow (calculated from middle from image)
 * \param size is the  size of stereo_distance_per_column
 * \param border is the search window + search distance used in blockmatching
 * \param RES is resolution used to calculate the line fit (int based math)
 * \param x_offset offset is x value for linefit
 * */
// Line_fit fits a line using least squares to the histogram disparity map
uint32_t line_fit(int32_t *points, int32_t *slope, int32_t *intercept,
                  uint32_t size, uint32_t border, int32_t RES, int32_t x_offset)
{
  int32_t x;

  int32_t count = 0;
  int32_t sumY = 0;
  int32_t sumX = 0;
  int32_t sumX2 = 0;
  int32_t sumXY = 0;
  int32_t border_int = (int32_t) border;
  int32_t size_int = (int32_t) size;
  uint32_t total_error = 0;

  *slope = 0;
  *intercept = 0;

  // compute fixed sums
  int32_t xend = size_int - border_int - 1;
  sumX = xend * (xend + 1) / 2 - border_int * (border_int + 1) / 2
         + border_int;
  sumX2 = xend * (xend + 1) * (2 * xend + 1) / 6 - border_int * (border_int + 1) * (2 * border_int + 1) / 6 + border_int * border_int;
  count = size_int - 2 * border_int;

  for (x = border_int; x < size_int - border_int; x++) {
    sumY += points[x];
    sumXY += (x + x_offset) * points[x];
  }

  // We need at least two points to do a line fit
  if (count < 2) {
    return INT32_MAX;
  }

  // *divergence = (sumXY - sumX * yMean) / (sumX2 - sumX * xMean); // compute slope of line ax + b
  if ((sumX2 - sumX * sumX / count) != 0) {
    *slope = RES * (sumXY - sumX * sumY / count) / (sumX2 - sumX * sumX / count);  // compute slope of line ax + b
  } else {
    //divergence_int = 10000;
  }

  // compute b (or y) intercept of line ax + b
  *intercept = (RES * sumY - *slope * sumX) / count;

  for (x = border_int; x < size_int - border_int; x++) {
    total_error += abs(RES * points[x] - *slope * (x + x_offset) - *intercept);
  }

  return total_error / count;
}

/* weighted_line_fit: fits a line using least squares to the histogram disparity map, excluding the areas that have faulty distance measurements
 * \param displacement is an array that contains the pixel displacements of the compared edgehistograms
 * \param faulty_distance is an array with binary values, to indicate where the distance measure was faulty and not (those coordinates will not be included in the line fit)
 * \param divergence is slope of the optical flow field
 * \param slope is intercept of the optical flow (calculated from middle from image)
 * \param size is the  size of stereo_distance_per_column
 * \param border is the search window + search distance used in blockmatching
 * \param RES is resolution used to calculate the line fit (int based math)
 * \param x_offset offset is x value for linefit
 * */
uint32_t weighted_line_fit(int32_t *points, int32_t *weight, int32_t *slope,
                           int32_t *intercept, uint32_t size, uint32_t border, uint16_t RES,
                           int32_t x_offset)
{
  int32_t x;
  int32_t count = 0;
  int32_t Sx = 0, Sy = 0;   // weighted average of x and y
  int32_t sumW = 0;

  int32_t border_int = (int32_t) border;
  int32_t size_int = (int32_t) size;
  uint32_t total_error = 0;

  *slope = 0;
  *intercept = 0;

  for (x = border_int; x < size_int - border_int; x++) {
    if (weight[x] > 0) {
      Sx += weight[x] * (x + x_offset);
      Sy += weight[x] * points[x];
      sumW += weight[x];
      count++;
    }
  }

  // We need at least two points to do a line fit
  if (count < 2 || sumW == 0) {
    return INT32_MAX;
  }

  Sx /= sumW;
  Sy /= sumW;

  // compute fixed sums
  int32_t nominator = 0, denominator = 0;
  for (x = border_int; x < size_int - border_int; x++) {
    nominator += weight[x] * (points[x] - Sy) * (x + x_offset - Sx);
    denominator += weight[x] * (x + x_offset - Sx) * (x + x_offset - Sx);
  }

  if (denominator != 0) {
    *slope = RES * nominator / denominator;
  }

  *intercept = RES * Sy - *slope * Sx; // compute b (or y) intercept of line ax + b.

  for (x = border_int; x < size_int - border_int; x++) {
    total_error += abs(RES * points[x] - *slope * (x + x_offset) - *intercept);
  }

  return total_error / count;
}

/* weighted_line_fit: fits a line using least squares to the histogram disparity map, excluding the areas that have faulty distance measurements
 * \param displacement is an array that contains the pixel displacements of the compared edgehistograms
 * \param faulty_distance is an array with binary values, to indicate where the distance measure was faulty and not (those coordinates will not be included in the line fit)
 * \param divergence is slope of the optical flow field
 * \param slope is intercept of the optical flow (calculated from middle from image)
 * \param size is the  size of stereo_distance_per_column
 * \param border is the search window + search distance used in blockmatching
 * \param RES is resolution used to calculate the line fit (int based math)
 * \param p0 is the constraint condition, line must pass through (x0,y0), set (struct point_t){0,0} if not used
 * */
uint32_t constrained_line_fit(int32_t *points, int32_t *weight, int32_t *slope,
                              int32_t *intercept, uint32_t size, uint32_t border, uint16_t RES,
                              struct point_t p0)
{
  // solve least squares minimization for equation y - y0 = m(x - x0)
  int32_t x;
  uint32_t error;
  for (x = border; x < size - border; x++) {
    points[x] -= p0.y;
  }
  if (weight != NULL) {
    error = weighted_line_fit(points, weight, slope, intercept, size, border, RES, p0.x);
  } else {
    error = line_fit(points, slope, intercept, size, border, RES, p0.x);
  }
  *intercept = p0.y - *slope * p0.x;
  return error;
}

/* weighted_line_fit: fits a line using least squares to the histogram disparity map, excluding the areas that have faulty distance measurements
 * \param displacement is an array that contains the pixel displacements of the compared edgehistograms
 * \param divergence is slope of the optical flow field
 * \param slope is intercept of the optical flow (calculated from middle from image)
 * \param faulty_distance is an array with binary values, to indicate where the distance measure was faulty and not (those coordinates will not be included in the line fit)
 * \param size is the  size of stereo_distance_per_column
 * \param border is the search window + search distance used in blockmatching
 * \param RES is resolution used to calculate the line fit (int based math)
 *
 * TODO: Make the inlier_threshold and inlier ratio adaptable
 * */
void line_fit_RANSAC(int8_t *displacement, int32_t *divergence, int32_t *flow,
                     uint8_t *faulty_distance, uint16_t size, uint32_t border, int32_t RES)
{

  int16_t inlier_threshold = 2000;
  int16_t inlier_ratio = 40;
  int16_t num_inliers_wanted = (int16_t)(inlier_ratio
                                         * (size - (int32_t) border * 2) / 100);
  int16_t num_inliers = 0;
  //Fit a linear line with RANSAC (from Guido's code)
  int32_t ransac_iter = 50;
  int32_t it;
  uint32_t ind1, ind2, tmp, entry;
  int32_t total_error = 0, best_ind = 0;
  int32_t error;
  int32_t dx, dflow, predicted_flow;
  // flow = a * x + b
  int32_t a[ransac_iter];
  int32_t b[ransac_iter];
  uint32_t errors[ransac_iter];

  uint16_t  counter_pass_check = 0;

  uint16_t entries = size - 2 * border;

  for (it = 0; it < ransac_iter; it++) {
    ind1 = rand() % entries + border;
    ind2 = rand() % entries + border;

    while (ind1 == ind2) {
      ind2 = rand() % entries + border;
    }
    // TODO: is this really necessary?
    if (ind1 > ind2) {
      tmp = ind2;
      ind2 = ind1;
      ind1 = tmp;
    }

    dx = ind2 - ind1;   // never zero
    dflow = displacement[ind2] - displacement[ind1];

    // Fit line with two points
    a[it] = RES * dflow / dx;
    b[it] = RES * (int32_t)displacement[ind1] - (a[it] * ind1);
    // evaluate fit:

    total_error = 0;
    for (entry = border; entry < size - border; entry++) {
      predicted_flow = a[it] * entry + b[it];
      error = abs((RES * (int32_t)displacement[entry] - predicted_flow));

      if ((int32_t) error < inlier_threshold * RES && faulty_distance[entry] == 0) {
        num_inliers++;
        total_error += (error / (RES));
      }

      //total_error += ipow(RES*displacement[entry] - predicted_flow,2);
    }


    if ((num_inliers > num_inliers_wanted)) {
      errors[it] = total_error;
      counter_pass_check++;

    } else {

      errors[it] = UINT32_MAX;
    }

    num_inliers = 0;
    total_error = 0;
  }

  // select best fit:
  uint32_t min_error = 0;
  best_ind = getMinimum2(errors, ransac_iter, &min_error);

  if (counter_pass_check > 0) {
    *divergence = a[best_ind];
    *flow = b[best_ind];
  } else {
    *divergence = 0;
    *flow = 0;
  }

}

/* simpleKalmanFilter: simple one dimension kalman filter
 * \param cov is covariance value of the measurement
 * \param previous_est is the previous estimate (from kalmanfilter previous timestep)
 * \param current_meas is current measurement update
 * \param Q is process noise
 * \param R is measurement noise
 * \param RES is resolution used for the int based math
 * \return new variable estimate
 *
 * */
int32_t simpleKalmanFilter(int32_t *cov, int32_t previous_est,
                           int32_t current_meas, int32_t Q, int32_t R, int32_t RES)
{
  int32_t predict_cov = *cov + Q;
  int32_t K = RES * predict_cov / (*cov + R);

  *cov = ((RES - K) * predict_cov) / RES;

  return (previous_est + (K * (current_meas - previous_est)) / RES);
}


/* moving_fading_average: an average filter with a forget function
 * \param previous_est is the previous smoothed variable
 * \param current_meas is current measurement update
 * \param alpha is the forget factor
 * \param RES is resolution used for the int based math
 * \return new variable estimate
 *
 * */
int32_t moving_fading_average(int32_t previous_est, int32_t current_meas,
                              int32_t alpha, int32_t RES)
{
  return (alpha * current_meas + (RES - alpha) * previous_est) / RES;
}

/* visuzlize_divergence This function is a visualization tool which visualizes the Edge filter in the one image and the histogram disparity with line fit in the second.
 * \param in[] is an array containing the pixel intensities of the (stereo)image
 * \param displacement is pixel displacement of edgehistograms
 * \param slope of the linefit
 * \param yint is the intercept of the line fit (From start of image)
 * \param img_w
 * \param img_h
 *
 * */
void visualize_divergence(uint8_t *in, int8_t *displacement, int32_t slope,
                          int32_t yInt, uint32_t img_w, uint32_t img_h)
{
  uint32_t y = 0;
  uint32_t x = 0;
  uint32_t idx = 0;

  for (y = 0; y < img_h; y++) {
    //line_check1=(uint32_t)(Slope*(float)x+(Yint)+(float)img_h/2);
    //line_check2=(uint32_t)(displacement[x]+img_h/2);
    for (x = 0; x < img_w; x++) {
      idx = 2 * (img_w * y + x);

      /*if(y==line_check1)
       out[idx]=255;
       else if(y==line_check2)
       out[idx]=100;
       else
       out[idx]=0;*/

      in[idx] = 100;
      in[idx + 1] = 0; //in[idx+1];
    }
  }
}

/* getMinimum2: finds minimum value in array
 * \param a is an array containing the values
 * \param n is size of the array
 * \param min_error is the minimum value of array
 * \return min_ind is the index of the minimum value located on the array
 *
 * */
uint32_t getMinimum2(uint32_t *a, uint32_t n, uint32_t *min_error)
{
  uint32_t i;
  uint32_t min_ind = 0;
  *min_error = a[min_ind];
  for (i = 1; i < n; i++) {
    if (a[i] < *min_error) {
      min_ind = i;
      *min_error = a[i];
    }
  }
  return min_ind;
}

/** Returns the minimum value in an array with the accompanying index
 * If there are multiple indices containing the same minimum value,
 * the index closest to the min_index will be returned
 *
 */
void find_minimum(int32_t *pSrc, uint8_t blockSize, int32_t *pResult, uint32_t *pIndex, uint8_t min_index)
{
  uint32_t i;
  *pResult = pSrc[0];
  *pIndex = 0;
  for (i = 1; i < blockSize; i++) {
    if (pSrc[i] < *pResult) {
      *pIndex = i;
      *pResult = pSrc[i];
    } else if (pSrc[i] == *pResult && abs((int32_t)i - (int32_t)min_index) < abs((int32_t)*pIndex - (int32_t)min_index)) {
      *pIndex = i;
      *pResult = pSrc[i];
    }
  }
}


/* getMaximum: finds maximum value in array
 * \param a is an array containing the values
 * \param n is size of the array
 * \return max_ind is the index of the maximum value located on the array
 *
 * */
uint32_t getMaximum(uint32_t *a, uint32_t n)
{
  uint32_t c;
  uint32_t max_error = a[0];
  uint32_t max_ind = 0;

  for (c = 1; c < n; c++) {
    if (a[c] > max_error) {
      max_ind = c;
      max_error = a[c];
    }
  }
  return max_ind;
}

/* getMedian: finds maximum value in array
 * \param a is an array containing the values
 * \param n is size of the array
 * \return median of array
 * */
uint32_t getMedian(int32_t *a, int32_t n)
{
  // Allocate an array of the same size and sort it.
  int32_t i, j;

  int dpSorted[n];
  for (i = 0; i < n; ++i) {
    dpSorted[i] = a[i];
  }
  for (i = n - 1; i > 0; --i) {
    for (j = 0; j < i; ++j) {
      if (dpSorted[j] > dpSorted[j + 1]) {
        int32_t dTemp = dpSorted[j];
        dpSorted[j] = dpSorted[j + 1];
        dpSorted[j + 1] = dTemp;
      }
    }
  }

  // Middle or average of middle values in the sorted array.
  int32_t dMedian = 0;
  if ((n % 2) == 0) {
    dMedian = (dpSorted[n / 2] + dpSorted[(n / 2) - 1]) / 2.0;
  } else {
    dMedian = dpSorted[n / 2];
  }
  return dMedian;
}
/* getMean: calculate mean of array
 * \param a is an array containing the values
 * \param n is size of the array
 * \return mean of array
 * */
uint32_t getMean(int32_t *a, int32_t n)
{
  int32_t dSum = a[0];
  int32_t i;
  for (i = 1; i < n; ++i) {
    dSum += a[i];
  }
  return dSum / n;
}

/* getMean: calculate mean of array
 * \param a is an array containing the values
 * \param n is size of the array
 * \return mean of array
 * */
uint32_t getMeanDisp(int32_t *a, int32_t n, int32_t border)
{
  int32_t dSum = 0;
  int32_t i, count = 0;
  for (i = border; i < n - border; ++i) {
    if (a[i] != 0) {
      dSum += a[i];
      count++;
    }
  }
  return dSum / count;
}

/* getTotalIntensityImage: calculate summed intensity of image
 * \param in is the image buffer
 * \param img_h
 * \param img_w
 * \return mean summed up intensities
 * */
uint32_t getTotalIntensityImage(uint8_t *in, uint32_t img_h, uint32_t img_w)
{

  uint32_t y = 0, x = 0;
  uint32_t idx = 0;
  uint32_t px_offset = 0;
  uint32_t totalIntensity = 0;
  for (x = 1; x < img_w - 1; x++) {
    for (y = 0; y < img_h; y++) {
      idx = 2 * (img_w * y + (x)); // 2 for interlace

      totalIntensity += (uint32_t) in[idx + px_offset];
    }
  }
  return totalIntensity;
}

/* getAmountPeaks: calculate amount of peaks of an edgehistogram
 * \param edgehist is an edgehistogram
 * \param median of edgehistogram
 * \param size is size of edgehistogram
 * \return amount of peaks
 * */
uint32_t getAmountPeaks(int32_t *edgehist, uint32_t median, int32_t size)
{
  uint32_t amountPeaks = 0;
  int32_t i = 0;

  for (i = 1; i < size + 1; i++) {
    if (edgehist[i - 1] < edgehist[i] && edgehist[i] > edgehist[i + 1]
        && edgehist[i] > (int16_t) median) {
      amountPeaks++;
    }
  }
  return amountPeaks;
}

/* ipow: calculates the power of the value
 * \param base
 * \param exp (the power of the base)
 * \return result
 *
 * */
int ipow(int base, int exp)
{
  int result = 1;
  while (exp) {
    if (exp & 1) {
      result *= base;
    }
    exp >>= 1;
    base *= base;
  }

  return result;
}

