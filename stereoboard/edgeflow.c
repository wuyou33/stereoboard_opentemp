/*
 * divergence.c
 *
 *  Created on: Apr 9, 2015
 *      Author: knmcguire
 */

#include "edgeflow.h"

#include <stdlib.h>

#if COMPILE_ON_LINUX
#include <string.h>
#include <iostream>
#include <stdio.h>
using namespace std;
#else
#include "sys_time.h"
#endif

#include "camera_type.h"
#include "stereo_math.h"

/* edgeflow_total: The total function for the edgeflow algorithm
 * \param divergenceArray is Array containing information to send to lisa-s
 * \param stereocam_data_int16  is the data the stereocam receives from the lisa s
 * \param stereocam_len is the length of the received data array, to make sure that it receives data.
 * \param current_image_buffer is the image of the present image step
 * \param edgeflow_parameters is a struct containing al the parameters for edgeflow
 * \param edgeflow_results is a struct containing the resulting values of edgeflow
 * */
void edgeflow_total(uint8_t edgeflowArray[], int16_t *stereocam_data_int16,
                    uint8_t stereocam_len, uint8_t current_image_buffer[],
                    struct edgeflow_parameters_t *edgeflow_parameters,
                    struct edgeflow_results_t *edgeflow_results)
{

#if !COMPILE_ON_LINUX
  //TODO: get this timestap exactly on when the frame is captured!
  edgeflow_results->edge_hist[edgeflow_results->current_frame_nr].frame_time =
    sys_time_get();
#endif

  if (stereocam_len > 0) {
    edgeflow_results->edge_hist[edgeflow_results->current_frame_nr].pitch =
      stereocam_data_int16[0];
    edgeflow_results->edge_hist[edgeflow_results->current_frame_nr].roll =
      stereocam_data_int16[1];
    edgeflow_parameters->derotation = stereocam_data_int16[2];
    edgeflow_parameters->adapt_horizon = stereocam_data_int16[3];
    edgeflow_parameters->window_size = stereocam_data_int16[4];
    edgeflow_parameters->disparity_range = stereocam_data_int16[5];
    edgeflow_parameters->snapshot = stereocam_data_int16[6];
    edgeflow_parameters->stereo_shift = stereocam_data_int16[7];

    edgeflow_parameters->autopilot_mode = stereocam_data_int16[8];

  } else {
    edgeflow_results->edge_hist[edgeflow_results->current_frame_nr].pitch =
      0;
    edgeflow_results->edge_hist[edgeflow_results->current_frame_nr].roll =
      0;
    edgeflow_parameters->alt_state_lisa = 0;
    edgeflow_results->dphi = 0;
    edgeflow_results->dtheta = 0;
  }

  calculate_edge_flow(current_image_buffer, edgeflow_parameters,
                      edgeflow_results);

  edgeflow_results->hz_x = edgeflow_calc_vel(edgeflow_parameters,
                           edgeflow_results);

#if !COMPILE_ON_LINUX
  edgeflow_to_sendarray(edgeflowArray, edgeflow_parameters, edgeflow_results);
#endif
}

/*  edgeflow_init: Initialize structures edgeflow_parameters and results
 * \param edgeflow_parameters is a struct containing al the parameters for edgeflow
 * \param edgeflow_results is a struct containing the resulting values of edgeflow
 * \param FOVX and FOVY are the field of view of the camera
 * \param image_width and image_height are the pixel dimensions of the image
 * \param use_monocam is a boolean that indicates if a monocam or stereocam is used
 * */
void edgeflow_init(struct edgeflow_parameters_t *edgeflow_parameters,
                   struct edgeflow_results_t *edgeflow_results,
                   int16_t image_width, int16_t image_height, int8_t use_monocam)
{

  edgeflow_parameters->fovx = (int32_t)(FOVX * edgeflow_parameters->RES);
  edgeflow_parameters->fovy = (int32_t)(FOVY * edgeflow_parameters->RES);
  edgeflow_parameters->image_height = image_height;
  edgeflow_parameters->image_width = image_width;

  edgeflow_parameters->max_horizon = MAX_HORIZON;
  edgeflow_parameters->RES = 100;
#if COMPILE_ON_LINUX
  edgeflow_parameters->stereo_shift =  0;
#else
  edgeflow_parameters->stereo_shift =  DISPARITY_OFFSET_HORIZONTAL;
#endif
  edgeflow_parameters->stereo_camera_seperation =  6;       // in cm

  edgeflow_parameters->max_disparity_range = DISP_RANGE_MAX;
  edgeflow_parameters->disparity_range = 15;
  edgeflow_parameters->window_size = 8;
  edgeflow_parameters->kalman_on = 0;
  edgeflow_parameters->Q = 20;
  edgeflow_parameters->R = 100;

  edgeflow_parameters->use_monocam = use_monocam;
  edgeflow_parameters->snapshot = 0;
  edgeflow_parameters->derotation = 0;
  edgeflow_parameters->adapt_horizon = 1;

  // Initialize variables
  edgeflow_results->current_frame_nr = 0;
  edgeflow_results->previous_frame_offset[0] = 1;
  edgeflow_results->previous_frame_offset[1] = 1;
  edgeflow_results->edge_flow.flow_x =
    edgeflow_results->prev_edge_flow.flow_x =
      0;          // Edgeflow: initialize previous translational flow in x-direction (image coordinates)
  edgeflow_results->edge_flow.div_x =
    edgeflow_results->prev_edge_flow.div_x =
      0;       // Edgeflow: initialize previous divergence in x-direction (image coordinates)
  edgeflow_results->edge_flow.flow_y =
    edgeflow_results->prev_edge_flow.flow_y =
      0;          // Edgeflow: initialize previous translational flow in y-direction (image coordinates)
  edgeflow_results->edge_flow.div_y =
    edgeflow_results->prev_edge_flow.div_y =
      0;       // Edgeflow: initialize previous translational flow in y-direction (image coordinates)

  // Initialize kalman
  edgeflow_results->covariance.C_flow_x = 20;
  edgeflow_results->covariance.C_flow_y = 20;
  edgeflow_results->covariance.C_div_x = 20;
  edgeflow_results->covariance.C_div_y = 20;
  edgeflow_results->covariance.C_height = 20;

  edgeflow_results->avg_dist = 0;
  edgeflow_results->avg_disp = 0;
  edgeflow_results->prev_avg_dist = 0;
  edgeflow_results->vel_x_global = 0;
  edgeflow_results->prev_vel_x_global = 0;
  edgeflow_results->vel_y_global = 0;
  edgeflow_results->prev_vel_y_global = 0;
  edgeflow_results->vel_x_pixelwise = 0;
  edgeflow_results->prev_vel_x_pixelwise = 0;
  edgeflow_results->vel_z_pixelwise = 0;
  edgeflow_results->prev_vel_z_pixelwise = 0;

  edgeflow_results->snapshot_is_taken = 0;
  edgeflow_parameters->snapshot_lenght = 300;
  edgeflow_results->snapshot_counter = edgeflow_parameters->snapshot_lenght
                                       + 1;

}

/*  edgeflow_to_sendarray: This function fills up the array that is send to the lisa -s
 * \param edgeflow_array is the array to be send back to the autopilot
 * \param edgeflow_parameters is a struct containing al the parameters for edgeflow
 * \param edgeflow_results is a struct containing the resulting values of edgeflow
 * */
void edgeflow_to_sendarray(uint8_t edgeflow_array[],
                           struct edgeflow_parameters_t *edgeflow_parameters,
                           struct edgeflow_results_t *edgeflow_results)
{

  /*EDGEFLOW_DEBUG defines which type of information is send through the edgelflowArray.
   For debugging, intermediate results are nessasary to simplify the programming
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
    edge_hist_int8[x] = boundint8((edgeflow_results->velocity_per_column[x] / 100 + 127));
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

  edgeflow_array[8] = boundint8(edgeflow_results->avg_dist / 10);
  edgeflow_array[9] = boundint8(edgeflow_results->hz_x);

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
 *\param edgeflow_parameters is a struct containing al the parameters for edgeflow
 *\param edgeflow_results is a struct containing the resulting values of edgeflow
 * */

int32_t edgeflow_calc_vel(struct edgeflow_parameters_t *edgeflow_parameters,
                          struct edgeflow_results_t *edgeflow_results)
{

  // Assign pointers to changable values in results (for smaller calculations)
  int32_t *vel_x_global = &edgeflow_results->vel_x_global;
  int32_t *vel_y_global = &edgeflow_results->vel_y_global;
  int32_t *vel_z_global = &edgeflow_results->vel_z_global;

  int32_t *vel_x_pixelwise = &edgeflow_results->vel_x_pixelwise;
  int32_t *vel_z_pixelwise = &edgeflow_results->vel_z_pixelwise;
  int32_t *prev_vel_x_global = &edgeflow_results->prev_vel_x_global;
  int32_t *prev_vel_y_global = &edgeflow_results->prev_vel_y_global;
  int32_t *prev_vel_x_pixelwise = &edgeflow_results->prev_vel_x_pixelwise;
  int32_t *prev_vel_z_pixelwise = &edgeflow_results->prev_vel_z_pixelwise;

//int32_t *prev_vel_y = &edgeflow_results->prev_vel_y;
  int32_t *avg_disp = &edgeflow_results->avg_disp;
  int32_t *avg_dist = &edgeflow_results->avg_dist;
  int32_t *prev_avg_dist = &edgeflow_results->prev_avg_dist;
  int32_t *hz_x = &edgeflow_results->hz_x;
  int32_t *hz_y = &edgeflow_results->hz_y;
  uint8_t *previous_frame_offset = edgeflow_results->previous_frame_offset;
  uint8_t *current_frame_nr = &edgeflow_results->current_frame_nr;
  struct edge_flow_t *edge_flow = &edgeflow_results->edge_flow;
  struct covariance_t *covariance = &edgeflow_results->covariance;
  struct edge_hist_t *edge_hist = edgeflow_results->edge_hist;

  // Assign pointers to parameters
  int32_t monocam = edgeflow_parameters->use_monocam;
  int32_t fov_x = edgeflow_parameters->fovx;
  int32_t fov_y = edgeflow_parameters->fovy;
  int32_t RES = edgeflow_parameters->RES;
  uint32_t R = edgeflow_parameters->R;
  uint32_t Q = edgeflow_parameters->Q;
  int16_t image_width = edgeflow_parameters->image_width;
  int16_t image_height = edgeflow_parameters->image_height;

  // disparity to distance in dm given 6cm dist between cams and Field of View (FOV) of 60deg
  // d =  Npix*cam_separation /(2*disp*tan(FOV/2))
  // d = 0.06*128 / (2*tan(disp*1.042/2))
  // d = 0.06*128 / (2*disp*1.042/2)
  // d = RES*0.06*128 / (disp*RES*1.042)
  // d = RES*0.06*PIX / (disp*FOVX)
  if (*avg_disp > 0) {
    *avg_dist = RES * edgeflow_parameters->stereo_camera_seperation * image_width / (*avg_disp * edgeflow_parameters->fovx);
  } else {
    *avg_dist = 100; // 2 * RES * 6 * IMAGE_WIDTH / 104;
  }
  if (monocam) {
    *avg_dist = 1.0;
  }

  //filter height: TODO: use height directly from lisa s  edgeflow_parameters->snapshot = 0;

  if (edgeflow_parameters->kalman_on == 1)
    *avg_dist = simpleKalmanFilter(&(covariance->C_height), *prev_avg_dist,
                                   *avg_dist, Q, R, RES);
  //store the time of the frame
  // Calculate velocity TODO:: Find a way to use extract function correctly for this, since the results is different with than without
#if COMPILE_ON_LINUX
  *hz_x = 16 * previous_frame_offset[0];
  *hz_y = 16 * previous_frame_offset[1];
#else
  *hz_x = TIMER_TICKS_PER_SEC
          / (edge_hist[*current_frame_nr].frame_time
             - edge_hist[(*current_frame_nr - previous_frame_offset[0]
                          + MAX_HORIZON) %
                         MAX_HORIZON].frame_time);
  *hz_y = TIMER_TICKS_PER_SEC
          / (edge_hist[*current_frame_nr].frame_time
             - edge_hist[(*current_frame_nr - previous_frame_offset[1]
                          + MAX_HORIZON) %
                         MAX_HORIZON].frame_time);
#endif

  // Calculate the stereo distance and flow x distance per column
  int32_t x;

  int32_t border = edgeflow_parameters->disparity_range
                   + edgeflow_parameters->window_size;

  uint8_t faulty_distance[IMAGE_WIDTH];
  int32_t velocity_stereo[IMAGE_WIDTH];
  for (x = 0; x < image_width - 1; x++) {

    if (x > border && x < image_width - border
        && edgeflow_results->displacement.stereo[x] > 0
        && edgeflow_results->displacement.stereo[x]
        < (edgeflow_parameters->disparity_range))

    {
      edgeflow_results->stereo_distance_per_column[x] = RES * RES *  edgeflow_parameters->stereo_camera_seperation
          * image_width
          / abs(
            edgeflow_results->displacement.stereo[x]
            * ((int32_t) fov_x * RES)); //RES * RES * RES / (RES * RES * RES)
      faulty_distance[x] = 0;

    } else {
      edgeflow_results->stereo_distance_per_column[x] =
        0;
      faulty_distance[x] = 1;

    }

    // Calculate stereodistance per column
    edgeflow_results->velocity_per_column[x] =
      edgeflow_results->stereo_distance_per_column[x]
      * edgeflow_results->displacement.x[x] * (*hz_x) ; // RES * RES

  }


// calculate avoidance manouvers
  avoid_velocity_from_stereo(edgeflow_results->stereo_distance_per_column,
                             &edgeflow_results->vel_x_stereo_avoid_pixelwise, &edgeflow_results->vel_z_stereo_avoid_pixelwise, 50, image_width,
                             border, edgeflow_parameters->stereo_shift);


  edgeflow_results->velocity_stereo_mean = getMean(velocity_stereo, 128);

// Fit the flow times distance, to get forward and sideways flow.
  int32_t forward_vel;
  int32_t sideways_vel;

  // TODO: Choose which line fit to use.
/*   uint32_t line_error_fit_hor = line_fit(edgeflow_results->velocity_per_column, &forward_vel,
                                          &sideways_vel, 128, border, 1);*/

  weighted_line_fit(edgeflow_results->velocity_per_column, faulty_distance, &forward_vel,
                    &sideways_vel, image_width, border, 1);

/*  line_fit_RANSAC(edgeflow_results->velocity_per_column, &forward_vel,
                  &sideways_vel, faulty_distance, 128, border, RES);*/

  *vel_z_pixelwise = forward_vel / (RES) ;
  *vel_x_pixelwise = (sideways_vel + forward_vel * image_width / 2) * fov_x
                     / (RES * RES * image_width) ; // RES * RES * RES / RES * RES

#if EDGEFLOW_USE_HEIGHT_AUTOPILOT
  int32_t alt_state_lisa = edgeflow_parameters->alt_state_lisa;
  *vel_x_global = edge_flow->flow_x * (alt_state_lisa) * (*hz_x) * fovx
                  / (RES * RES * image_width);
  *vel_y_global = edge_flow->flow_y * (alt_state_lisa) * (*hz_y) * fovy
                  / (RES * RES * image_height);
#else
  *vel_x_global = edge_flow->flow_x * (*avg_dist) * (*hz_x) * fov_x
                  / (RES * RES * image_width);
  *vel_y_global = edge_flow->flow_y * (*avg_dist) * (*hz_y) * fov_y
                  / (RES * RES * image_height);
  *vel_z_global = edge_flow->div_x * (*avg_dist) * (*hz_y) / RES;
#endif

  //TODO: implement smoothing on lisas

  /*int32_t alpha = 60;
   *vel_x_global =  moving_fading_average(*prev_vel_x_global, *vel_x_global,  alpha, RES);
   *vel_y_global =  moving_fading_average(*prev_vel_y_global, *vel_y_global,  alpha, RES);
   *vel_x_pixelwise =  moving_fading_average(*prev_vel_x_pixelwise, *vel_x_pixelwise,  alpha, RES);
   *vel_z_pixelwise =  moving_fading_average(*prev_vel_z_pixelwise, *vel_z_pixelwise,  alpha, RES);*/

  if (edgeflow_parameters->kalman_on == 1) {
    *vel_x_global = simpleKalmanFilter(&(covariance->C_flow_x), *prev_vel_x_global, *vel_x_global,
                                       Q, R, RES);
    *vel_y_global = simpleKalmanFilter(&(covariance->C_flow_y), *prev_vel_y_global, *vel_y_global,
                                       Q, R, RES);
    *vel_x_pixelwise = simpleKalmanFilter(&(covariance->C_flow_x), *prev_vel_x_pixelwise, *vel_x_pixelwise,
                                          Q, R, RES);
    *vel_z_pixelwise = simpleKalmanFilter(&(covariance->C_flow_y), *prev_vel_z_pixelwise, *vel_z_pixelwise,
                                          Q, R, RES);
  }

  // Store previous values
  *prev_avg_dist = *avg_dist;
  *prev_vel_x_global = *vel_x_global;
  *prev_vel_y_global = *vel_y_global;
  *prev_vel_x_pixelwise = *vel_x_pixelwise;
  *prev_vel_z_pixelwise = *vel_z_pixelwise;
  memcpy(&edgeflow_results->prev_stereo_distance_per_column, &edgeflow_results->stereo_distance_per_column,
         128 * sizeof(int32_t)); // copy quality measures to output array


  // move the indices for the edge hist structure
  *current_frame_nr = (*current_frame_nr + 1) % MAX_HORIZON;
  return *hz_x;
}

/*  calculate_edge_flow: calculate the global optical flow by edgeflow
 * \param in[] is an array containing the pixel intensities of the (stereo)image
 * \param edgeflow_parameters is a struct containing al the parameters for edgeflow
 * \param edgeflow_results is a struct containing the resulting values of edgeflow
 *
 *   TODO: reintergrate snapshot (doesn't work very well yet!!)
 *
 * */

void calculate_edge_flow(uint8_t in[],
                         struct edgeflow_parameters_t *edgeflow_parameters,
                         struct edgeflow_results_t *edgeflow_results)
{

  struct displacement_t *displacement = &edgeflow_results->displacement;
  struct edge_hist_t *edge_hist = edgeflow_results->edge_hist;
  struct edge_flow_t *edge_flow = &edgeflow_results->edge_flow;

  int32_t *avg_disp = &edgeflow_results->avg_disp;
  uint8_t *previous_frame_offset = edgeflow_results->previous_frame_offset;
  uint8_t *current_frame_nr = &edgeflow_results->current_frame_nr;
  uint8_t *quality_measures = edgeflow_results->quality_measures_edgeflow;

  int32_t image_width = IMAGE_WIDTH; //edgeflow_parameters->image_width;
  int32_t image_height = IMAGE_HEIGHT; //edgeflow_parameters->image_height;
  int32_t RES = edgeflow_parameters->RES;
  uint8_t disp_range = edgeflow_parameters->disparity_range;
  uint8_t window_size = edgeflow_parameters->window_size;
  uint16_t edge_threshold = 0;

  // check that inputs within allowable ranges
  if (disp_range > DISP_RANGE_MAX) {
    disp_range = DISP_RANGE_MAX;
  }

  // Define arrays and pointers for edge histogram and displacements
  int32_t *edge_histogram_x = edge_hist[*current_frame_nr].x;
  int32_t *prev_edge_histogram_x;
  int32_t *edge_histogram_x_right =  edge_hist[*current_frame_nr].stereo;

  int32_t *edge_histogram_y = edge_hist[*current_frame_nr].y;
  int32_t *prev_edge_histogram_y;

  // Calculate previous frame number

  if (MAX_HORIZON > 2 && (edgeflow_parameters->adapt_horizon == 1)) { //&& edgeflow_parameters->snapshot == 0) {
    uint32_t flow_mag_x, flow_mag_y;
    flow_mag_x = abs(edge_flow->flow_x);
    flow_mag_y = abs(edge_flow->flow_y);

    uint32_t min_flow = 3* RES;
    uint32_t max_flow = (disp_range - 2)*RES;
    uint8_t previous_frame_offset_x = previous_frame_offset[0];
    uint8_t previous_frame_offset_y = previous_frame_offset[1];

    // Increment or deincrement previous frame offset based on previous measured flow.
    if (flow_mag_x > max_flow && previous_frame_offset_x > 1) {
      previous_frame_offset[0] = previous_frame_offset_x - 1;
    }
    if (flow_mag_x < min_flow
        && previous_frame_offset_x < MAX_HORIZON - 1) {
      previous_frame_offset[0] = previous_frame_offset_x + 1;
    }
    if (flow_mag_y > max_flow && previous_frame_offset_y > 1) {
      previous_frame_offset[1] = previous_frame_offset_y - 1;
    }
    if (flow_mag_y < min_flow
        && previous_frame_offset_y < MAX_HORIZON - 1) {
      previous_frame_offset[1] = previous_frame_offset_y + 1;
    }
  }

  // the previous frame number relative to dynamic parameters
  uint8_t previous_frame_x = (*current_frame_nr - previous_frame_offset[0]
                              + MAX_HORIZON) %
                             MAX_HORIZON; // wrap index
  uint8_t previous_frame_y = (*current_frame_nr - previous_frame_offset[1]
                              + MAX_HORIZON) %
                             MAX_HORIZON; // wrap index

  //TODO: Snapshot commented out, have to debug!

  // copy previous edge histogram based on previous frame number
  /* if (edgeflow_results->snapshot_is_taken == 1 && edgeflow_parameters->snapshot == 1
   && (edgeflow_parameters->autopilot_mode == 10 || edgeflow_parameters->autopilot_mode == 11
   || edgeflow_parameters->autopilot_mode == 12)) {
   prev_edge_histogram_x = edgeflow_results->edge_hist_snapshot.x;
   prev_edge_histogram_y = edgeflow_results->edge_hist_snapshot.y;
   edgeflow_results->snapshot_counter++;
   } else {*/
  prev_edge_histogram_x = edge_hist[previous_frame_x].x;
  prev_edge_histogram_y = edge_hist[previous_frame_y].y;
  //}


  // Calculate Edge Histogram
  calculate_edge_histogram(in, edge_histogram_x, image_width, image_height,
                           'x', 'l', edge_threshold);
  calculate_edge_histogram(in, edge_histogram_y, image_width, image_height,
                           'y', 'l', edge_threshold);
  calculate_edge_histogram(in, edge_histogram_x_right, image_width,
                           image_height, 'x', 'r', edge_threshold);

  /*if (edgeflow_parameters->snapshot == 1 && (edgeflow_results->snapshot_is_taken == 0
   || edgeflow_results->snapshot_counter > edgeflow_parameters->snapshot_lenght)) {
   memcpy(&edgeflow_results->edge_hist_snapshot.x, edge_histogram_x, sizeof(int32_t) * image_width);
   memcpy(&edgeflow_results->edge_hist_snapshot.y, edge_histogram_y, sizeof(int32_t) * image_height);
   edgeflow_results->snapshot_counter = 0;
   edgeflow_results->snapshot_is_taken = 1;
   }*/

  //calculate angle diff [RAD * RES]

  int16_t der_shift_x = 0;
  int16_t der_shift_y = 0;
  if (edgeflow_parameters->derotation) {
    int16_t roll_prev = edge_hist[previous_frame_x].roll;
    int16_t pitch_prev = edge_hist[previous_frame_y].pitch;

    der_shift_x = (roll_prev - edge_hist[*current_frame_nr].roll)
                  * image_width / (edgeflow_parameters->fovx);
    der_shift_y = (pitch_prev - edge_hist[*current_frame_nr].pitch)
                  * image_height / (edgeflow_parameters->fovy);
  }
  // Calculate displacement
  uint32_t min_error_hor = calculate_displacement(edge_histogram_x,
                           prev_edge_histogram_x, displacement->x, image_width, window_size,
                           disp_range, der_shift_x);
  uint32_t min_error_ver = calculate_displacement(edge_histogram_y,
                           prev_edge_histogram_y, displacement->y, image_height, window_size,
                           disp_range, der_shift_y);
  calculate_displacement_stereo(edge_histogram_x,
                                edge_histogram_x_right, displacement->stereo, image_width,
                                window_size, disp_range, edgeflow_parameters->stereo_shift);
  *avg_disp = calculate_displacement_fullimage(edge_histogram_x,
              edge_histogram_x_right, image_width, disp_range, edgeflow_parameters->stereo_shift);

  // Fit a linear line
  int32_t div_x_temp, div_y_temp, flow_x_temp, flow_y_temp;
  uint32_t line_error_fit_hor = line_fit(displacement->x, &div_x_temp,
                                         &flow_x_temp, image_width, window_size + disp_range, RES);
  uint32_t line_error_fit_ver = line_fit(displacement->y, &div_y_temp,
                                         &flow_y_temp, image_height, window_size + disp_range, RES);

  edge_flow->div_x = div_x_temp;
  edge_flow->div_y = div_y_temp;
  edge_flow->flow_x = flow_x_temp + (div_x_temp * image_width / 2);
  edge_flow->flow_y = flow_y_temp + (div_y_temp * image_height / 2);

  //Calculate and Store quality values
  uint32_t totalIntensity = getTotalIntensityImage(in, image_width,
                            image_height);
  uint32_t mean_hor = getMean(edge_histogram_x, image_width);
  uint32_t mean_ver = getMean(edge_histogram_y, image_height);
  uint32_t median_hor = getMedian(edge_histogram_x, image_width);
  uint32_t median_ver = getMedian(edge_histogram_y, image_height);
  uint32_t amountPeaks_hor = getAmountPeaks(edge_histogram_x, 500,
                             image_width);
  uint32_t amountPeaks_ver = getAmountPeaks(edge_histogram_y, 500,
                             image_height);

  quality_measures[0] = boundint8(totalIntensity / 20000);
  quality_measures[1] = boundint8(mean_hor / 20);
  quality_measures[2] = boundint8(mean_ver / 20);
  quality_measures[3] = boundint8(median_hor / 20);
  quality_measures[4] = boundint8(median_ver / 20);
  quality_measures[5] = boundint8(amountPeaks_hor);
  quality_measures[6] = boundint8(amountPeaks_ver);
  quality_measures[7] = boundint8(line_error_fit_hor / 10);
  quality_measures[8] = boundint8(line_error_fit_ver / 10);

  int32_t *pointer = (int32_t *) quality_measures + 9;
  pointer[0] = min_error_hor;
  pointer[1] = min_error_ver;

  /* if (edgeflow_results->snapshot_is_taken == 1 && edgeflow_parameters->snapshot == 0) {
   edgeflow_results->snapshot_counter = edgeflow_parameters->snapshot_lenght + 1;
   edgeflow_results->snapshot_is_taken = 0;
   }*/
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
void calculate_edge_histogram(uint8_t *in, int32_t *edge_histogram,
                              uint16_t image_width, uint16_t image_height, char direction, char side,
                              uint16_t edge_threshold)
{
  // TODO use arm_conv_q31()
  int32_t sobel_sum = 0;
  int32_t Sobel[3] = { -1, 0, 1 };

  int32_t y = 0, x = 0;
  int32_t c = 0;

  uint32_t idx = 0;

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
    edge_histogram[0] = edge_histogram[image_width - 1] = 0;
    for (x = 1; x < (int16_t) image_width - 1; x++) {
      edge_histogram[x] = 0;
      for (y = 0; y < image_height; y++) {
        sobel_sum = 0;

        for (c = -1; c <= 1; c++) {
          idx = 2 * (image_width * y + (x + c)); // 2 for interlace

          sobel_sum += Sobel[c + 1] * (int32_t) in[idx + px_offset];
        }
        sobel_sum = abs(sobel_sum);
        if (sobel_sum > edge_threshold) {
          edge_histogram[x] += sobel_sum;
        }
      }
    }
  } else if (direction == 'y') {
    // set values that are not visited
    edge_histogram[0] = edge_histogram[image_height - 1] = 0;
    for (y = 1; y < image_height - 1; y++) {
      edge_histogram[y] = 0;
      for (x = 0; x < image_width; x++) {
        sobel_sum = 0;

        for (c = -1; c <= 1; c++) {
          idx = 2 * (image_width * (y + c) + x); // 2 for interlace

          sobel_sum += Sobel[c + 1] * (int32_t) in[idx + px_offset];
        }
        sobel_sum = abs(sobel_sum);
        if (sobel_sum > edge_threshold) {
          edge_histogram[y] += sobel_sum;
        }
      }
    }
  } else
    while (1)
      ;  // hang to show user something isn't right
}


/* calculate_displacement: Calculate_displacement calculates the displacement between two histograms in time
 * \param edge_histogram is an array containing the summed up gradient intensities
 * \param edge_histogram_prev containts the edge_histogram from the previous timeframe
 * \param displacement is an array that contains the pixel displacements of the compared edgehistograms
 * \param size indicates the size of array (important for x and y direction)
 * \param window indicates the pixel size of the window of neighboring pixels
 * \param disp_range indicates how far the block matcihing algorithm should search
 * \param der_shift is the pixel shift caused by rotation of the camera
 * */
uint32_t calculate_displacement(int32_t *edge_histogram,
                                int32_t *edge_histogram_prev, int32_t *displacement, uint16_t size,
                                uint8_t window, uint8_t disp_range, int32_t der_shift)
{
  int32_t c = 0, r = 0;
  int32_t x = 0;
  uint32_t SAD_temp[2 * DISP_RANGE_MAX + 1]; // size must be at least 2*D + 1

  int32_t W = window;
  int32_t D = disp_range;
  uint32_t min_error = 0;
  uint32_t min_error_tot = 0;
  uint8_t SHIFT_TOO_FAR = 0;
  memset(displacement, 0, size);

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

  // TODO: replace with arm offset subtract
  if (!SHIFT_TOO_FAR) {

    for (x = border_left; x < border_right; x++) {
      displacement[x] = 0;

      for (c = -D; c <= D; c++) {
        SAD_temp[c + D] = 0;
        for (r = -W; r <= W; r++) {
          SAD_temp[c + D] +=
            abs(
              edge_histogram[x + r]
              - edge_histogram_prev[x + r + c
                                    + der_shift]);
        }
      }
      displacement[x] = (int32_t) getMinimum2(SAD_temp, 2 * D + 1,
                                              &min_error) - D;

      min_error_tot += min_error;
    }

  }

  return min_error_tot;
}

/* calculate_displacement_stereo: Calculate_displacement calculates the displacement between two histograms in stereo
 * \param edge_histogram is an array containing the summed up gradient intensities
 * \param edge_histogram_right containts the edge_histogram from the previous timeframe
 * \param displacement is an array that contains the pixel displacements of the compared edgehistograms
 * \param size indicates the size of array (important for x and y direction)
 * \param window indicates the pixel size of the window of neighboring pixels
 * \param disp_range indicates how far the block matcihing algorithm should search
 * \param stereo_shift is the pixel shift between the stereocameras, determined by calibration
 * */
uint32_t calculate_displacement_stereo(int32_t *edge_histogram,
                                       int32_t *edge_histogram_right, int32_t *displacement, uint16_t size,
                                       uint8_t window, uint8_t disp_range, int16_t stereo_shift)
{
  int32_t c = 0, r = 0;
  int32_t x = 0;
  uint32_t SAD_temp[DISP_RANGE_MAX + 1]; // size must be at least 2*D + 1
  memset(&SAD_temp, 10000, sizeof(uint32_t) * DISP_RANGE_MAX);
  int32_t W = window;
  int32_t D = disp_range;
  uint32_t min_error = 0;
  uint32_t min_error_tot = 0;
  memset(displacement, 0, size);

  uint16_t border_left = W + D;
  uint16_t border_right = size - W;

  if (stereo_shift > 0) {
    border_left += stereo_shift;
  } else if (stereo_shift < 0) {
    border_right += stereo_shift;
  }

  // TODO: replace with arm offset subtract

  for (x = border_left; x < border_right; x++) {

    for (c = -D; c <= 0; c++) {
      SAD_temp[c + D] = 0;
      for (r = -W; r <= W; r++) {
        SAD_temp[c + D] +=
          abs(
            edge_histogram[x + r]
            - edge_histogram_right[x + r + c
                                   - stereo_shift]);
      }
    }
    displacement[x] = D
                      - (int32_t) getMinimum2(SAD_temp, D + 1, &min_error);

    min_error_tot += min_error;
  }

  return min_error_tot;
}

/* calculate_displacement_fullimage: Calculate_displacement calculates the displacement between two histograms of the full image
 * \param edge_histogram is an array containing the summed up gradient intensities
 * \param edge_histogram_prev containts the edge_histogram from the previous timeframe
 * \param displacement is an array that contains the pixel displacements of the compared edgehistograms
 * \param size indicates the size of array (important for x and y direction)
 * \param disp_range indicates how far the block matcihing algorithm should search
 * \param stereo_shift is the pixel shift between the stereocameras, determined by calibration
 *
 *
 * */
int32_t calculate_displacement_fullimage(int32_t *edge_histogram,
    int32_t *edge_histogram_right, uint16_t size, uint8_t disp_range, int16_t stereo_shift)
{
  int32_t c = 0;
  int32_t x = 0;
  uint32_t SAD_temp[2 * DISP_RANGE_MAX + 1]; // size must be at least 2*D + 1
  memset(&SAD_temp, UINT32_MAX, sizeof(uint32_t) * 2 * DISP_RANGE_MAX + 1);

  int32_t D = disp_range;
  uint32_t min_error = 0;

  uint16_t border_left =  D;
  uint16_t border_right = size;

  if (stereo_shift > 0) {
    border_left += stereo_shift;
  } else if (stereo_shift < 0) {
    border_right += stereo_shift;
  }

  // TODO check if one loop can be replaced by line diff
  for (c = -D; c <= 0; c++) {
    SAD_temp[c + D] = 0;
    for (x = border_left; x < border_right; x++) {
      SAD_temp[c + D] += abs(edge_histogram[x] - edge_histogram_right[x + c - stereo_shift]);
    }
  }

  return D - (int32_t) getMinimum2(SAD_temp, 2 * D + 1, &min_error);
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

/* line_fit: fits a line using least squares to the histogram disparity map
 * \param displacement is an array that contains the pixel displacements of the compared edgehistograms
 * \param divergence is slope of the optical flow field
 * \param slope is intercept of the optical flow (calculated from middle from image)
 * \param size is the  size of stereo_distance_per_column
 * \param border is the search window + search distance used in blockmatching
 * \param RES is resolution used to calculate the line fit (int based math)
 * */
// Line_fit fits a line using least squares to the histogram disparity map
uint32_t line_fit(int32_t *displacement, int32_t *divergence, int32_t *flow,
                  uint32_t size, uint32_t border, uint16_t RES)
{
  int32_t x;

  int32_t count = 0;
  int32_t sumY = 0;
  int32_t sumX = 0;
  int32_t sumX2 = 0;
  int32_t sumXY = 0;
  int32_t xMean = 0;
  int32_t yMean = 0;
  int32_t divergence_int = 0;
  int32_t border_int = (int32_t) border;
  int32_t size_int = (int32_t) size;
  uint32_t total_error = 0;

  *divergence = 0;
  *flow = 0;

  // compute fixed sums
  int32_t xend = size_int - border_int - 1;
  sumX = xend * (xend + 1) / 2 - border_int * (border_int + 1) / 2
         + border_int;
  sumX2 = xend * (xend + 1) * (2 * xend + 1) / 6;
  xMean = (size_int - 1) / 2;
  count = size_int - 2 * border_int;

  for (x = border_int; x < size_int - border_int; x++) {
    sumY += displacement[x];
    sumXY += x * displacement[x];
  }

  yMean = RES * sumY / count;

  divergence_int = (RES * sumXY - sumX * yMean) / (sumX2 - sumX * xMean); // compute slope of line ax + b
  *divergence = divergence_int;
  *flow = yMean - *divergence * xMean; // compute b (or y) intercept of line ax + b

  for (x = border_int; x < size_int - border_int; x++) {
    total_error += abs(RES * displacement[x] - divergence_int * x + yMean);
  }

  return total_error / size;
}

/* weighted_line_fit: fits a line using least squares to the histogram disparity map, excluding the areas that have faulty distance measurements
 * \param displacement is an array that contains the pixel displacements of the compared edgehistograms
 * \param faulty_distance is an array with binary values, to indicate where the distance measure was faulty and not (those coordinates will not be included in the line fit)
 * \param divergence is slope of the optical flow field
 * \param slope is intercept of the optical flow (calculated from middle from image)
 * \param size is the  size of stereo_distance_per_column
 * \param border is the search window + search distance used in blockmatching
 * \param RES is resolution used to calculate the line fit (int based math)
 *
 * TODO: make the linefit so, that it can fit the line with uncertainty weights.
 * */
uint32_t weighted_line_fit(int32_t *displacement, uint8_t *faulty_distance,
                           int32_t *divergence, int32_t *flow, uint32_t size, uint32_t border,
                           uint16_t RES)
{

  int32_t x;

  int32_t count = 0;
  int32_t sumY = 0;
  int32_t sumX = 0;
  int32_t sumX2 = 0;
  int32_t sumXY = 0;

  int32_t xMean = 0;
  int32_t yMean = 0;
  int32_t divergence_int = 0;
  int32_t border_int = (int32_t) border;
  int32_t size_int = (int32_t) size;
  uint32_t total_error = 0;

  *divergence = 0;
  *flow = 0;

  // compute fixed sums
  for (x = border_int; x < size_int - border_int; x++) {
    if (faulty_distance[x] == 0) {
      sumX += x;
      sumY += RES *  displacement[x];

      sumX2 += x * x;
      sumXY += x * displacement[x] * RES;

      count++;
    }
  }

  *divergence = 0;  //slope;
  *flow = 0;  //intercept;
  if (count == 0) {
    return 0;
  }

  xMean = sumX / count;
  yMean = sumY / count;

  if ((sumX2 - sumX * xMean) != 0) { // preven seg fault
    divergence_int = (sumXY - sumX * yMean) / (sumX2 - sumX * xMean);  // compute slope of line ax + b
    *divergence = divergence_int;
  } else {
    return 0;
  }

  *flow = yMean - divergence_int * xMean; // compute b (or y) intercept of line ax + b


  for (x = border_int; x < size_int - border_int; x++) {
    total_error += abs(RES * displacement[x] - divergence_int * x + yMean);
  }

  return total_error / size;
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
void line_fit_RANSAC(int32_t *displacement, int32_t *divergence, int32_t *flow,
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
    b[it] = RES * displacement[ind1] - (a[it] * ind1);
    // evaluate fit:

    total_error = 0;
    for (entry = border; entry < size - border; entry++) {
      predicted_flow = a[it] * entry + b[it];
      error = abs((RES * displacement[entry] - predicted_flow));

      if ((int32_t) error < inlier_threshold * RES &&  faulty_distance[entry] == 0 && faulty_distance[entry] == 0) {
        num_inliers++;
        total_error += (error / (RES));
      }

      //total_error += ipow(RES*displacement[entry] - predicted_flow,2);
    }


    if ((num_inliers > num_inliers_wanted)) {
      errors[it] = total_error;
      counter_pass_check++;

    } else {

      errors[it] = UINT32_MAX;;
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
  int32_t new_est = (alpha * current_meas + (RES - alpha) * previous_est)
                    / RES;
  return new_est;
}

/* visuzlize_divergence This function is a visualization tool which visualizes the Edge filter in the one image and the histogram disparity with line fit in the second.
 * \param in[] is an array containing the pixel intensities of the (stereo)image
 * \param displacement is pixel displacement of edgehistograms
 * \param slope of the linefit
 * \param yint is the intercept of the line fit (From start of image)
 * \param image_width
 * \param image_height
 *
 * */
void visualize_divergence(uint8_t *in, int32_t *displacement, int32_t slope,
                          int32_t yInt, uint32_t image_width, uint32_t image_height)
{
  uint32_t y = 0;
  uint32_t x = 0;
  uint32_t idx = 0;

  for (y = 0; y < image_height; y++) {
    //line_check1=(uint32_t)(Slope*(float)x+(Yint)+(float)image_height/2);
    //line_check2=(uint32_t)(displacement[x]+image_height/2);
    for (x = 0; x < image_width; x++) {
      idx = 2 * (image_width * y + x);

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
  uint32_t min_err = a[min_ind];
  uint32_t min_err_tot = 0;
  for (i = 1; i < n; i++) {
    if (a[i] <= min_err) {
      min_ind = i;
      min_err = a[i];
      min_err_tot += min_err;
    }
  }
  *min_error = min_err;
  return min_ind;
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

/* getTotalIntensityImage: calculate summed intensity of image
 * \param in is the image buffer
 * \param image_height
 * \param image_width
 * \return mean summed up intensities
 * */
uint32_t getTotalIntensityImage(uint8_t *in, uint32_t image_height,
                                uint32_t image_width)
{

  uint32_t y = 0, x = 0;
  uint32_t idx = 0;
  uint32_t px_offset = 0;
  uint32_t totalIntensity = 0;
  for (x = 1; x < image_width - 1; x++) {
    for (y = 0; y < image_height; y++) {
      idx = 2 * (image_width * y + (x)); // 2 for interlace

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

