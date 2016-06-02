/*
 * divergence.c
 *
 *  Created on: Apr 9, 2015
 *      Author: knmcguire
 */

#include "edgeflow.h"
#include "sys_time.h"
#include <stdlib.h>

/*  divergence_total: The total function for the edgeflow algorithm
* divergenceArray is Array containing information to send to lisa-s
* stereocam_data_int16  is the data the stereocam receives from the lisa s
* stereocam_len is the length of the received data array, to make sure that it receives data.
* current_image_buffer is the image of the present image step
* edgeflow_parameters is a struct containing al the parameters for edgeflow
* edgeflow_results is a struct containing the resulting values of edgeflow
* */
void edgeflow_total(uint8_t divergenceArray[], int16_t *stereocam_data_int16, uint8_t stereocam_len,
                    uint8_t current_image_buffer[],
                    struct edgeflow_parameters_t *edgeflow_parameters, struct edgeflow_results_t *edgeflow_results)
{
  //TODO: get this timestap exactly on when the frame is captured!
  edgeflow_results->edge_hist[edgeflow_results->current_frame_nr].frame_time = sys_time_get();

  if (stereocam_len > 0) {
    edgeflow_results->edge_hist[edgeflow_results->current_frame_nr].pitch = stereocam_data_int16[0];
    edgeflow_results->edge_hist[edgeflow_results->current_frame_nr].roll = stereocam_data_int16[1];
    edgeflow_parameters->derotation = stereocam_data_int16[2];
    edgeflow_parameters->adapt_horizon = stereocam_data_int16[3];
    edgeflow_parameters->window_size = stereocam_data_int16[4];
    edgeflow_parameters->disparity_range = stereocam_data_int16[5];
    edgeflow_parameters->snapshot = stereocam_data_int16[6];
    edgeflow_parameters->autopilot_mode = stereocam_data_int16[7];


  } else {
    edgeflow_results->edge_hist[edgeflow_results->current_frame_nr].pitch = 0;
    edgeflow_results->edge_hist[edgeflow_results->current_frame_nr].roll = 0;
    edgeflow_parameters->alt_state_lisa = 0;
    edgeflow_parameters->dphi = 0;
    edgeflow_parameters->dtheta = 0;
  }

  calculate_edge_flow(current_image_buffer,   edgeflow_parameters, edgeflow_results);


  edgeflow_results->hz_x = edgeflow_calc_vel(edgeflow_parameters, edgeflow_results);


  edgeflow_to_sendarray(divergenceArray, edgeflow_parameters, edgeflow_results);
}

/*  divergence_init: Initialize structures edgeflow_parameters and results
* edgeflow_parameters is a struct containing al the parameters for edgeflow
* edgeflow_results is a struct containing the resulting values of edgeflow
* FOVX and FOVY are the field of view of the camera
* image_width and image_height are the pixel dimensions of the image
* use_monocam is a boolean that indicates if a monocam or stereocam is used
* */
void edgeflow_init(struct edgeflow_parameters_t *edgeflow_parameters, struct edgeflow_results_t *edgeflow_results,
                   const int8_t FOVX, const int8_t FOVY, int8_t image_width, int8_t image_height, int8_t use_monocam)
{
  edgeflow_parameters->FOVX = FOVX;
  edgeflow_parameters->FOVY = FOVY;
  edgeflow_parameters->image_height = IMAGE_HEIGHT;
  edgeflow_parameters->image_width = IMAGE_WIDTH;
  edgeflow_parameters->Q = 50;
  edgeflow_parameters->R = 100;
  edgeflow_parameters->max_horizon = MAX_HORIZON;
  edgeflow_parameters->max_disparity_range = DISP_RANGE_MAX;
  edgeflow_parameters->disparity_range = 20;
  edgeflow_parameters->window_size = 10;
  edgeflow_parameters->edge_flow_kalman = 1;
  edgeflow_parameters->RES = 100;
  edgeflow_parameters->use_monocam = use_monocam;

  edgeflow_results->R_height = 100;
  edgeflow_results->R_x = 100;
  edgeflow_results->R_y = 100;
  edgeflow_results->current_frame_nr = 0;
  edgeflow_results->previous_frame_offset[0] = 1;
  edgeflow_results->previous_frame_offset[1] = 1;
  edgeflow_results->edge_flow.flow_x =  edgeflow_results->prev_edge_flow.flow_x = 0;
  edgeflow_results->edge_flow.div_x =  edgeflow_results->prev_edge_flow.div_x = 0;
  edgeflow_results->edge_flow.flow_y =  edgeflow_results->prev_edge_flow.flow_y = 0;
  edgeflow_results->edge_flow.div_y =  edgeflow_results->prev_edge_flow.div_y = 0;

  edgeflow_results->covariance.C_flow_x = 20;
  edgeflow_results->covariance.C_flow_y = 20;
  edgeflow_results->covariance.C_div_x = 20;
  edgeflow_results->covariance.C_div_y = 20;
  edgeflow_results->covariance.C_height = 20;

  edgeflow_results->avg_dist = 0;
  edgeflow_results->avg_disp = 0;
  edgeflow_results->prev_avg_dist = 0;
  edgeflow_results->vel_x = 0;
  edgeflow_results->prev_vel_x = 0;
  edgeflow_results->vel_y = 0;
  edgeflow_results->prev_vel_y = 0;

  edgeflow_results->snapshot_is_taken = 0;
  edgeflow_parameters->snapshot_lenght = 300;
  edgeflow_results->snapshot_counter =   edgeflow_parameters->snapshot_lenght + 1;

}

/*  divergence_array: This function fills up the array that is send to the lisa -s
* Variables can be changed
* */
void edgeflow_to_sendarray(uint8_t edgeflow_array[24], struct edgeflow_parameters_t *edgeflow_parameters,
                           struct edgeflow_results_t *edgeflow_results)
{
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

  edgeflow_array[10] = (edgeflow_results->vel_x >> 8) & 0xff;
  edgeflow_array[11] = (edgeflow_results->vel_x) & 0xff;
  edgeflow_array[12] = (edgeflow_results->vel_y >> 8) & 0xff;
  edgeflow_array[13] = (edgeflow_results->vel_y) & 0xff;

//  memcpy(divergenceArray + 14, quality_measures_edgeflow,
  //       10 * sizeof(uint8_t)); // copy quality measures to output array
}

/*  divergence_calc_vel: calculate height by edgeflow and altitude
* edgeflow_parameters is a struct containing al the parameters for edgeflow
* edgeflow_results is a struct containing the resulting values of edgeflow
* */

int32_t edgeflow_calc_vel(struct edgeflow_parameters_t *edgeflow_parameters,
                          struct edgeflow_results_t *edgeflow_results)
{

  // Assign pointers to changable values in results (for smaller calculations)
  int32_t *vel_x = &edgeflow_results->vel_x;
  int32_t *vel_y = &edgeflow_results->vel_y;
  int32_t *prev_vel_x = &edgeflow_results->prev_vel_x;
  int32_t *prev_vel_y = &edgeflow_results->prev_vel_y;
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
  int8_t FOVX = edgeflow_parameters->FOVX;
  int8_t FOVY = edgeflow_parameters->FOVY;
  int32_t RES = edgeflow_parameters->RES;
  uint32_t R = edgeflow_parameters->R;
  uint32_t Q = edgeflow_parameters->Q;
  int32_t image_width = edgeflow_parameters->image_width;
  int32_t image_height = edgeflow_parameters->image_height;

  // disparity to distance in dm given 6cm dist between cams and Field of View (FOV) of 60deg
  // d =  Npix*cam_separation /(2*disp*tan(FOV/2))
  // d = 0.06*128 / (2*tan(disp*1.042/2))
  // d = 0.06*128 / (2*disp*1.042/2)
  // d = RES*0.06*128 / (disp*RES*1.042)
  // d = RES*0.06*PIX / (disp*FOVX)
  if (avg_disp > 0) {
    *avg_dist = RES * 3 * image_width / (*avg_disp * FOVX);
  } else {
    *avg_dist = 100; // 2 * RES * 6 * IMAGE_WIDTH / 104;
  }
  if (monocam) {
    *avg_dist = 1.0;
  }
  //filter height: TODO: use height directly from lisa s
  *avg_dist = simpleKalmanFilter(&(covariance->C_height), *prev_avg_dist,
                                 *avg_dist, Q, R, RES);
  //store the time of the frame
  // Calculate velocity TODO:: Find a way to use extract function correctly for this, since the results is different with than without
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
#if EDGEFLOW_USE_HEIGHT_AUTOPILOT
  int32_t alt_state_lisa =  edgeflow_parameters->alt_state_lisa;
  *vel_x = edge_flow->flow_x * (alt_state_lisa) * (*hz_x) * FOVX
           / (RES * RES * image_width);
  *vel_y = edge_flow->flow_y * (alt_state_lisa) * (*hz_y) * FOVY
           / (RES * RES * image_height);
#else
  *vel_x = edge_flow->flow_x * (*avg_dist) * (*hz_x) * FOVX
           / (RES * RES * image_width);
  *vel_y = edge_flow->flow_y * (*avg_dist) * (*hz_y) * FOVY
           / (RES * RES * image_height);
#endif
  *vel_x = simpleKalmanFilter(&(covariance->C_flow_x), *prev_vel_x, *vel_x,
                              Q, R, RES);
  *vel_y = simpleKalmanFilter(&(covariance->C_flow_y), *prev_vel_y, *vel_y,
                              Q, R, RES);
  // Store previous values
  *prev_avg_dist = *avg_dist;
  *prev_vel_x = *vel_x;
  *prev_vel_y = *vel_y;

  // move the indices for the edge hist structure
  * current_frame_nr = (*current_frame_nr + 1) % MAX_HORIZON;
  return *hz_x;
}

/*  calculate_edge_flow: calculate the global optical flow by edgflow
* uint8_t in[] is an array containing the pixel intensities of the (stereo)image
* edgeflow_parameters is a struct containing al the parameters for edgeflow
* edgeflow_results is a struct containing the resulting values of edgeflow
* */

void calculate_edge_flow(uint8_t in[], struct edgeflow_parameters_t *edgeflow_parameters,
                         struct edgeflow_results_t *edgeflow_results)
{

  struct displacement_t *displacement = &edgeflow_results->displacement;
  struct edge_hist_t *edge_hist = edgeflow_results->edge_hist;
  struct edge_flow_t *edge_flow = &edgeflow_results->edge_flow;

  int32_t *avg_disp = &edgeflow_results->avg_disp;
  uint8_t *previous_frame_offset = edgeflow_results->previous_frame_offset;
  uint8_t *current_frame_nr = &edgeflow_results->current_frame_nr;
  uint8_t *quality_measures = edgeflow_results->quality_measures_edgeflow;

  int32_t image_width = IMAGE_WIDTH;//edgeflow_parameters->image_width;
  int32_t image_height = IMAGE_HEIGHT;//edgeflow_parameters->image_height;
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
  int32_t edge_histogram_x_right[IMAGE_WIDTH];

  int32_t *edge_histogram_y = edge_hist[*current_frame_nr].y;
  int32_t *prev_edge_histogram_y;

  // Calculate previous frame number

  if (MAX_HORIZON > 2 && (edgeflow_parameters->adapt_horizon == 1) && edgeflow_parameters->snapshot == 0) {
    uint32_t flow_mag_x, flow_mag_y;
    flow_mag_x = abs(edge_flow->flow_x);
    flow_mag_y = abs(edge_flow->flow_y);

    uint32_t min_flow = 3;
    uint32_t max_flow = disp_range - 2;
    uint8_t previous_frame_offset_x = previous_frame_offset[0];
    uint8_t previous_frame_offset_y = previous_frame_offset[1];

    // Increment or deincrement previous frame offset based on previous measured flow.
    if (flow_mag_x > max_flow && previous_frame_offset_x > 1) {
      previous_frame_offset[0] = previous_frame_offset_x - 1;
    }
    if (flow_mag_x < min_flow && previous_frame_offset_x < MAX_HORIZON - 1) {
      previous_frame_offset[0] = previous_frame_offset_x + 1;
    }
    if (flow_mag_y > max_flow && previous_frame_offset_y > 1) {
      previous_frame_offset[1] = previous_frame_offset_y - 1;
    }
    if (flow_mag_y < min_flow && previous_frame_offset_y < MAX_HORIZON - 1) {
      previous_frame_offset[1] = previous_frame_offset_y + 1;
    }
  }


  // the previous frame number relative to dynamic parameters
  uint8_t previous_frame_x = (*current_frame_nr - previous_frame_offset[0] + MAX_HORIZON) %
                             MAX_HORIZON; // wrap index
  uint8_t previous_frame_y = (*current_frame_nr - previous_frame_offset[1] + MAX_HORIZON) %
                             MAX_HORIZON; // wrap index

  // copy previous edge histogram based on previous frame number
  if (edgeflow_results->snapshot_is_taken == 1 && edgeflow_parameters->snapshot == 1
      && (edgeflow_parameters->autopilot_mode == 10 || edgeflow_parameters->autopilot_mode == 11
          || edgeflow_parameters->autopilot_mode == 12)) {
    prev_edge_histogram_x = edgeflow_results->edge_hist_snapshot.x;
    prev_edge_histogram_y = edgeflow_results->edge_hist_snapshot.y;
    edgeflow_results->snapshot_counter++;
  } else {
    prev_edge_histogram_x = edge_hist[previous_frame_x].x;
    prev_edge_histogram_y = edge_hist[previous_frame_y].y;
  }

// int16_t der_shift_x = 1.5*(roll_prev - roll) * image_width / (FOVX);
// int16_t der_shift_y = 1.5*(pitch_prev - pitch) * image_height / (FOVY);

  // Calculate Edge Histogram
  calculate_edge_histogram(in, edge_histogram_x, image_width, image_height, 'x', 'l', edge_threshold);
  calculate_edge_histogram(in, edge_histogram_y, image_width, image_height, 'y', 'l', edge_threshold);
  calculate_edge_histogram(in, edge_histogram_x_right, image_width, image_height, 'x', 'r', edge_threshold);

  if (edgeflow_parameters->snapshot == 1 && (edgeflow_results->snapshot_is_taken == 0
      || edgeflow_results->snapshot_counter > edgeflow_parameters->snapshot_lenght)) {
    memcpy(&edgeflow_results->edge_hist_snapshot.x, edge_histogram_x, sizeof(int32_t) * image_width);
    memcpy(&edgeflow_results->edge_hist_snapshot.y, edge_histogram_y, sizeof(int32_t) * image_height);
    edgeflow_results->snapshot_counter = 0;
    edgeflow_results->snapshot_is_taken = 1;
  }



  //calculate angle diff [RAD * RES]


  int16_t der_shift_x = 0;
  int16_t der_shift_y = 0;
  if (edgeflow_parameters->derotation) {
    int16_t roll_prev = edge_hist[previous_frame_x].roll;
    int16_t pitch_prev = edge_hist[previous_frame_y].pitch;

    der_shift_x = (roll_prev - edge_hist[*current_frame_nr].roll) * image_width / (edgeflow_parameters->FOVX);
    der_shift_y = (pitch_prev - edge_hist[*current_frame_nr].pitch) * image_height / (edgeflow_parameters->FOVY);
  }
  // Calculate displacement
  uint32_t min_error_hor = calculate_displacement(edge_histogram_x, prev_edge_histogram_x, displacement->x,
                           image_width, window_size,
                           disp_range, der_shift_x);
  uint32_t min_error_ver = calculate_displacement(edge_histogram_y, prev_edge_histogram_y, displacement->y,
                           image_height, window_size,
                           disp_range, der_shift_y);
  *avg_disp = calculate_displacement_fullimage(edge_histogram_x, edge_histogram_x_right, image_width, disp_range);

  // Fit a linear line
  uint32_t line_error_fit_hor = line_fit(displacement->x, &edge_flow->div_x,
                                         &edge_flow->flow_x, image_width,
                                         window_size + disp_range, RES);
  uint32_t line_error_fit_ver = line_fit(displacement->y, &edge_flow->div_y, &edge_flow->flow_y,
                                         image_height,
                                         window_size + disp_range, RES);

  //Calculate and Store quality values
  uint32_t totalIntensity = getTotalIntensityImage(in, image_width, image_height);
  uint32_t mean_hor = getMean(edge_histogram_x, image_width);
  uint32_t mean_ver = getMean(edge_histogram_y, image_height);
  uint32_t median_hor = getMedian(edge_histogram_x, image_width);
  uint32_t median_ver = getMedian(edge_histogram_y, image_height);
  uint32_t amountPeaks_hor = getAmountPeaks(edge_histogram_x, 500 , image_width);
  uint32_t amountPeaks_ver = getAmountPeaks(edge_histogram_y, 500 , image_height);


  quality_measures[0] = boundint8(totalIntensity / 20000);
  quality_measures[1] = boundint8(mean_hor / 20);
  quality_measures[2] = boundint8(mean_ver / 20);
  quality_measures[3] = boundint8(median_hor / 20);
  quality_measures[4] = boundint8(median_ver / 20);
  quality_measures[5] = boundint8(amountPeaks_hor);
  quality_measures[6] = boundint8(amountPeaks_ver);
  quality_measures[7] = boundint8(line_error_fit_hor / 10);
  quality_measures[8] = boundint8(line_error_fit_ver / 10);

  int32_t *pointer = (int32_t *)quality_measures + 9;
  pointer[0] = min_error_hor;
  pointer[1] = min_error_ver;

  if (edgeflow_results->snapshot_is_taken == 1 && edgeflow_parameters->snapshot == 0) {
    edgeflow_results->snapshot_counter = edgeflow_parameters->snapshot_lenght + 1;
    edgeflow_results->snapshot_is_taken = 0;
  }
}

// calculate_edge_histogram calculates the image gradient of the images and makes a edge feature histogram
void calculate_edge_histogram(uint8_t *in, int32_t *edge_histogram, uint16_t image_width, uint16_t image_height,
                              char direction, char side, uint16_t edge_threshold)
{
  // TODO use arm_conv_q31()
  int32_t sobel_sum = 0;
  int32_t Sobel[3] = { -1, 0, 1};

  uint32_t y = 0, x = 0;
  int32_t c = 0;

  uint32_t idx = 0;

  // set pixel offset based on which image needed from interlaced image
  uint32_t px_offset = 0;
  if (side == 'l') {
    px_offset = 0;
  } else if (side == 'r') {
    px_offset = 1;
  } else
    while (1); // let user know something is wrong

  // compute edge histogram
  if (direction == 'x') {
    // set values that are not visited
    edge_histogram[0] = edge_histogram[image_width - 1] = 0;
    for (x = 1; x < image_width - 1; x++) {
      edge_histogram[x] = 0;
      for (y = 0; y < image_height; y++) {
        sobel_sum = 0;

        for (c = -1; c <= 1; c++) {
          idx = 2 * (image_width * y + (x + c)); // 2 for interlace

          sobel_sum += Sobel[c + 1] * (int32_t)in[idx + px_offset];
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

          sobel_sum += Sobel[c + 1] * (int32_t)in[idx + px_offset];
        }
        sobel_sum = abs(sobel_sum);
        if (sobel_sum > edge_threshold) {
          edge_histogram[y] += sobel_sum;
        }
      }
    }
  } else
    while (1);  // hang to show user something isn't right
}

// Calculate_displacement calculates the displacement between two histograms
// D should be half the search disparity range
// W is local search window
uint32_t calculate_displacement(int32_t *edge_histogram, int32_t *edge_histogram_prev, int32_t *displacement,
                                uint16_t size, uint8_t window, uint8_t disp_range, int32_t der_shift)
{
  int32_t c = 0, r = 0;
  uint32_t x = 0;
  uint32_t SAD_temp[2 * DISP_RANGE_MAX + 1]; // size must be at least 2*D + 1

  int32_t W = window;
  int32_t D = disp_range;
  uint32_t min_error = 0;
  uint32_t min_error_tot = 0;
  uint8_t SHIFT_TOO_FAR = 0;
  memset(displacement, 0, size);

  int32_t border[2];

  if (der_shift < 0) {
    border[0] =  W + D + der_shift;
    border[1] = size - W - D;
  } else if (der_shift > 0) {
    border[0] =  W + D;
    border[1] = size - W - D - der_shift;
  } else {
    border[0] =  W + D;
    border[1] = size - W - D;
  }

  if (border[0] >= border[1] || abs(der_shift) >= 10) {
    SHIFT_TOO_FAR = 1;
  }


  // TODO: replace with arm offset subtract
  if (!SHIFT_TOO_FAR) {

    for (x = border[0]; x < border[1]; x++) {
      displacement[x] = 0;

      for (c = -D; c <= D; c++) {
        SAD_temp[c + D] = 0;
        for (r = -W; r <= W; r++) {
          SAD_temp[c + D] += abs(edge_histogram[x + r] - edge_histogram_prev[x + r + c + der_shift]);
        }
      }
      displacement[x] = (int32_t)getMinimum2(SAD_temp, 2 * D + 1, &min_error) - D;

      min_error_tot += min_error;
    }

  }

  return min_error_tot;
}

// Calculate_displacement calculates the displacement between two histograms
// D should be disparity range
// TODO: for height should always look to the positive disparity range, can ignore negative
int32_t calculate_displacement_fullimage(int32_t *edge_histogram, int32_t *edge_histogram_2, uint16_t size,
    uint8_t disp_range)
{
  int32_t c = 0;
  uint32_t x = 0;
  uint32_t SAD_temp[2 * DISP_RANGE_MAX + 1]; // size must be at least 2*D + 1

  int32_t D = disp_range;
  uint32_t min_error = 0;

  // TODO check if one loop can be replaced by line diff
  for (c = -D; c <= D; c++) {
    SAD_temp[c + D] = 0;
    for (x = D; x < size - D; x++) {
      SAD_temp[c + D] += abs(edge_histogram[x] - edge_histogram_2[x + c]);
    }
  }

  return D - (int32_t)getMinimum2(SAD_temp, 2 * D + 1, &min_error);
}


// Line_fit fits a line using least squares to the histogram disparity map
uint32_t line_fit(int32_t *displacement, int32_t *divergence, int32_t *flow, uint32_t size, uint32_t border,
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
  int32_t border_int = (int32_t)border;
  int32_t size_int = (int32_t)size;
  uint32_t total_error = 0;

  *divergence = 0;
  *flow = 0;

  // compute fixed sums
  int32_t xend = size_int - border_int - 1;
  sumX = xend * (xend + 1) / 2 - border_int * (border_int + 1) / 2 + border_int;
  sumX2 = xend * (xend + 1) * (2 * xend + 1) / 6;
  xMean = (size_int - 1) / 2;
  count = size_int - 2 * border_int;

  for (x = border_int; x < size - border_int; x++) {
    sumY += displacement[x];
    sumXY += x * displacement[x];
  }

  yMean = RES * sumY / count;

  divergence_int = (RES * sumXY - sumX * yMean) / (sumX2 - sumX * xMean);    // compute slope of line ax + b
  *divergence = divergence_int;
  *flow = yMean - *divergence * xMean;  // compute b (or y) intercept of line ax + b

  for (x = border_int; x < size - border_int; x++) {
    total_error += abs(RES * displacement[x] - divergence_int * x + yMean);
  }

  return total_error / size;
}

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

void line_fit_RANSAC(int32_t *displacement, int32_t *divergence, int32_t *flow, uint16_t size, uint32_t border,
                     uint32_t RES)
{
  //Fit a linear line with RANSAC (from Guido's code)
  uint8_t ransac_iter = 20;
  int32_t it;
  uint32_t ind1, ind2, tmp, entry;
  uint32_t total_error, best_ind;
  int32_t dx, dflow, predicted_flow;
  // flow = a * x + b
  int32_t a[ransac_iter];
  int32_t b[ransac_iter];
  uint32_t errors[ransac_iter];

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
      //total_error += ipow(RES*displacement[entry] - predicted_flow,2);
      total_error += RES * displacement[entry] - predicted_flow;
    }
    errors[it] = total_error;
  }
  // select best fit:
  uint32_t min_error = 0;
  best_ind = getMinimum2(errors, ransac_iter, &min_error);

  *divergence = a[best_ind];
  *flow = b[best_ind];
}

void totalKalmanFilter(struct covariance_t *coveriance, struct edge_flow_t *prev_edge_flow,
                       struct edge_flow_t *edge_flow, uint32_t Q, uint32_t R, uint32_t RES)
{
  edge_flow->flow_x = simpleKalmanFilter(&(coveriance->C_flow_x), prev_edge_flow->flow_x,
                                         edge_flow->flow_x, Q, R, RES);
  edge_flow->flow_y = simpleKalmanFilter(&(coveriance->C_flow_y), prev_edge_flow->flow_y,
                                         edge_flow->flow_y, Q, R, RES);
  edge_flow->div_x = simpleKalmanFilter(&(coveriance->C_div_x), prev_edge_flow->div_x,
                                        edge_flow->div_x, Q, R, RES);
  edge_flow->div_y = simpleKalmanFilter(&(coveriance->C_div_y), prev_edge_flow->div_y,
                                        edge_flow->div_y, Q, R, RES);
}

int32_t simpleKalmanFilter(int32_t *cov, int32_t previous_est, int32_t current_meas, int32_t Q, int32_t R, int32_t RES)
{
  int32_t predict_cov = *cov + Q;
  int32_t K = RES * predict_cov / (*cov + R);

  *cov = ((RES - K) * predict_cov) / RES;

  return (previous_est + (K * (current_meas - previous_est)) / RES);
}


//This function is a visualization tool which visualizes the Edge filter in the one image and the histogram disparity with line fit in the second.
void visualize_divergence(uint8_t *in, int32_t *displacement, int32_t slope, int32_t yInt, uint32_t image_width,
                          uint32_t image_height)
{
  uint32_t y = 0;
  uint32_t x = 0;
  uint32_t idx = 0;

  for (y = 0; y < image_height; y++) {
    //line_check1=(uint32_t)(Slope*(float)x+(Yint)+(float)image_height/2);
    //line_check2=(uint32_t)(displacement[x]+image_height/2);
    for (x = 0; x < image_width; x++) {
      idx =  2 * (image_width * y + x);

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

// Small supporting functions

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
  *min_error = min_err_tot;
  return min_ind;
}

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

uint32_t getMedian(int32_t *daArray, int32_t iSize)
{
  // Allocate an array of the same size and sort it.
  int32_t i, j;

  int dpSorted[iSize];
  for (i = 0; i < iSize; ++i) {
    dpSorted[i] = daArray[i];
  }
  for (i = iSize - 1; i > 0; --i) {
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
  if ((iSize % 2) == 0) {
    dMedian = (dpSorted[iSize / 2] + dpSorted[(iSize / 2) - 1]) / 2.0;
  } else {
    dMedian = dpSorted[iSize / 2];
  }
  return dMedian;
}

uint32_t getMean(int32_t *daArray, int32_t iSize)
{
  int32_t dSum = daArray[0];
  int32_t i;
  for (i = 1; i < iSize; ++i) {
    dSum += daArray[i];
  }
  return dSum / iSize;
}

uint32_t getTotalIntensityImage(uint8_t *in, uint32_t image_height, uint32_t image_width)
{

  uint32_t y = 0, x = 0;
  uint32_t idx = 0;
  uint32_t px_offset = 0;
  uint32_t totalIntensity = 0;
  for (x = 1; x < image_width - 1; x++) {
    for (y = 0; y < image_height; y++) {
      idx = 2 * (image_width * y + (x)); // 2 for interlace


      totalIntensity += (uint32_t)in[idx + px_offset];
    }
  }
  return totalIntensity;
}

uint32_t getAmountPeaks(int32_t *edgehist, uint32_t median, int32_t size)
{
  uint32_t  amountPeaks = 0;
  uint32_t i = 0;

  for (i = 1; i < size + 1;  i ++) {
    if (edgehist[i - 1] < edgehist[i] && edgehist[i] > edgehist[i + 1] && edgehist[i] > median) {
      amountPeaks ++;
    }
  }
  return amountPeaks;
}

uint8_t boundint8(int32_t value)
{
  uint8_t value_uint8;
  if (value > 255) {
    value_uint8 = 255;
  } else if (value < 0) {
    value_uint8 = 0;
  } else {
    value_uint8 = (uint8_t)value;
  }

  return value_uint8;
}



