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
#include "math/filter.h"
#include "math/stats.h"

#ifdef USE_PPRZLINK
#include "pprz_datalink.h"
#include "pprzlink/intermcu_msg.h"
#else
#include "raw_digital_video_stream.h"
#endif

enum FilterType {
  NONE,
  KALMAN,
  MOVING_AVERAGE
};

#ifndef EDGEFLOW_DISPARITY_RANGE
#define EDGEFLOW_DISPARITY_RANGE 15
#endif

#ifndef EDGEFLOW_WINDOW_SIZE
#define EDGEFLOW_WINDOW_SIZE 5
#endif

#ifndef EDGEFLOW_USE_SNAPSHOT
#define EDGEFLOW_USE_SNAPSHOT false
#endif

//shared variables
struct edgeflow_parameters_t edgeflow_params;
struct edgeflow_t edgeflow;
struct snapshot_t edgeflow_snapshot;

struct cam_state_t *cam_state = NULL;

float sumN1, sumN2;

float coupled_flow_fit(struct displacement_t *disp, struct displacement_t *stereo_disp,
                       uint32_t RES, float scale_x, float scale_y, uint16_t w, uint16_t h, uint16_t border,
                       struct vec3_t *result);

/* edgeflow_total: The total function for the edgeflow algorithm
 * \param current_image_buffer is the image of the present image step.
 * \param iamge_time is the time the image was taken in us.
 * \param data_in is the data the stereocam receives from the autopilot.
 * \param data_len is the length of the received data array, to make sure that it receives data.
 * */
void edgeflow_total(uint8_t *current_image_buffer, uint32_t image_time)
{
  // move the indices for the edge hist structure
  edgeflow.current_frame_nr = (edgeflow.current_frame_nr + 1) % MAX_HORIZON;

  edgeflow.edge_hist[edgeflow.current_frame_nr].frame_time = image_time;

  if (cam_state != NULL &&
      abs(edgeflow.edge_hist[edgeflow.current_frame_nr].frame_time - cam_state->us_timestamp) > 1e6) {
    edgeflow_params.derotation = 1;
    edgeflow.edge_hist[edgeflow.current_frame_nr].phi = (int16_t)(cam_state->phi * edgeflow_params.RES);
    edgeflow.edge_hist[edgeflow.current_frame_nr].theta = (int16_t)(cam_state->theta * edgeflow_params.RES);
    edgeflow.edge_hist[edgeflow.current_frame_nr].psi = (int16_t)(cam_state->psi * edgeflow_params.RES);
    edgeflow.edge_hist[edgeflow.current_frame_nr].alt = (int16_t)(cam_state->alt * edgeflow_params.RES);
  } else {
    edgeflow_params.derotation = 0;
  }

  calculate_edge_flow(current_image_buffer, &edgeflow_params, &edgeflow);

  // skip first two frames from snapshot
  if (edgeflow_snapshot.snapshots < 2) {
    edgeflow_snapshot.keyframe = edgeflow.edge_hist[edgeflow.current_frame_nr];
    edgeflow_snapshot.snapshots++;
  } else if (EDGEFLOW_USE_SNAPSHOT) {
    calculate_snapshot_displacement(&edgeflow_params, &edgeflow, &edgeflow_snapshot);
  }
}

/*  edgeflow_init: Initialize structures edgeflow_params and results
 * \param edgeflow_params is a struct containing al the parameters for edgeflow
 * \param edgeflow is a struct containing the resulting values of edgeflow
 * \param FOVX and FOVY are the field of view of the camera
 * \param img_w and img_h are the pixel dimensions of the image
 * \param use_monocam is a boolean that indicates if a monocam or stereocam is used
 * \param cam_state_ref pointer to cam state struct, set to NULL for no derotation
 * */
void edgeflow_init(int16_t img_w, int16_t img_h, int8_t use_monocam, struct cam_state_t *cam_state_ref)
{
  cam_state = cam_state_ref;

  edgeflow_params.RES = 100;
  edgeflow_params.fovx = (int32_t)(FOVX * edgeflow_params.RES);
  edgeflow_params.fovy = (int32_t)(FOVY * edgeflow_params.RES);
  edgeflow_params.img_h = img_h;
  edgeflow_params.img_w = img_w;
  edgeflow_params.max_horizon = MAX_HORIZON;

#ifdef COMPILE_ON_LINUX
  edgeflow_params.stereo_shift =  0;
#else
  edgeflow_params.stereo_shift =  DISPARITY_OFFSET_HORIZONTAL;
#endif
  edgeflow_params.camera_seperation = (int16_t)(0.06 * edgeflow_params.RES);        // in m * RES

  edgeflow_params.Q = (int32_t)(0.5 * edgeflow_params.RES);
  edgeflow_params.R = (int32_t)(1. * edgeflow_params.RES);
  edgeflow_params.alpha = (int32_t)(0.75 * edgeflow_params.RES);

  edgeflow_params.use_monocam = use_monocam;

  edgeflow_params.disparity_range = EDGEFLOW_DISPARITY_RANGE; // this is not used as range but as max, maybe rename?
  edgeflow_params.window_size = EDGEFLOW_WINDOW_SIZE;
  edgeflow_params.derotation = 0;
  edgeflow_params.adapt_horizon = 1;
  edgeflow_params.filter_type = NONE;

  // Initialize variables
  edgeflow.current_frame_nr = 0;
  edgeflow.prev_frame_x = 0;
  edgeflow.prev_frame_y = 0;
  edgeflow.prev_frame_offset_x = 1;
  edgeflow.prev_frame_offset_y = 1;

  for (uint16_t i = 0; i < MAX_HORIZON; i++) {
    edgeflow.edge_hist[i].alt = 0;
    edgeflow.edge_hist[i].frame_time = 0;
    edgeflow.edge_hist[i].theta = 0;
    edgeflow.edge_hist[i].phi = 0;
    edgeflow.edge_hist[i].psi = 0;
    memset(edgeflow.edge_hist[i].x, 0, sizeof(edgeflow.edge_hist[i].x));
    memset(edgeflow.edge_hist[i].y, 0, sizeof(edgeflow.edge_hist[i].y));
  }

  memset(edgeflow.disp.x, 0, sizeof(edgeflow.disp.x));
  memset(edgeflow.disp.y, 0, sizeof(edgeflow.disp.y));
  memset(edgeflow.disp.stereo, 0, sizeof(edgeflow.disp.stereo));

  edgeflow.ran_twice = 0;
  edgeflow.dt = 0;
  edgeflow.hz.x = 25 * edgeflow_params.RES;
  edgeflow.hz.y = 25 * edgeflow_params.RES;

  // Edgeflow: initialize previous translational flow in x-direction (image coordinates)
  edgeflow.vel = (struct vec3_t) {0, 0, 0};
  edgeflow.prev_vel = (struct vec3_t) {0, 0, 0};

  // Initialize kalman
  edgeflow_params.cov.C_vel = (struct vec3_t) {(int32_t)(0.2 * edgeflow_params.RES), (int32_t)(0.2 * edgeflow_params.RES), (int32_t)(0.2 * edgeflow_params.RES)};
  edgeflow_params.cov.C_dist = (struct vec3_t) {(int32_t)(0.2 * edgeflow_params.RES), (int32_t)(0.2 * edgeflow_params.RES), (int32_t)(0.2 * edgeflow_params.RES)};

  edgeflow.avg_dist = 0;
  edgeflow.avg_disp = 0;
  edgeflow.prev_avg_dist = 0;

  edgeflow_snapshot.snapshots = 0;
  edgeflow_snapshot.dist = (struct vec3_t) {0, 0, 0};
  edgeflow_snapshot.dist_traveled = (struct vec3_t) {0, 0, 0};
}

/*  send_edgeflow: This function sends edgeflow message
 */
void send_edgeflow(void)
{
#ifdef EDGEFLOW_DEBUG
  /*EDGEFLOW_DEBUG defines which type of information is send through the edgelflowArray.
   For debugging, intermediate results are necessary to simplify the programming
   When EDGEFLOW_DEBUG is defined, it will send through the current histogram, the previous and the calculated displacement
   when it is not defined, it will send through flow, divergence and velocity*/
  static uint8_t edgeflow_debug_msg[128 * 5] = {0};
  uint8_t *current_frame_nr = &edgeflow->current_frame_nr;
  uint8_t *previous_frame_offset = &edgeflow->previous_frame_offset;

  uint8_t previous_frame_x = (*current_frame_nr - previous_frame_offset[0] + MAX_HORIZON) %
                             MAX_HORIZON; // wrap index

  uint8_t x = 0;
  uint8_t edge_hist_int8[128];
  uint8_t edge_hist_prev_int8[128];
  uint8_t displacement_int8[128];
  uint8_t plot2[128];
  uint8_t plot3[128];
  for (x = 0; x < 128; x++) {

    plot3[x] = boundint8((edgeflow->displacement.stereo[x] * 10 + 127));

    plot2[x] = boundint8((edgeflow->displacement.x[x] * 20 + 127));
    edge_hist_int8[x] = boundint8((edgeflow->vel_per_column[x] / 100 + 127));
    displacement_int8[x] = boundint8((edgeflow->stereo_distance_per_column[x] / 10  + 127));

    edge_hist_prev_int8[x] = boundint8((edgeflow->vel.x * (128 * 100 / 104)
                                        + edgeflow->vel.z * (x - 64)) / 100 + 127);

  }

  memcpy(edgeflow_debug_msg, &edge_hist_int8, 128 * sizeof(uint8_t)); // copy quality measures to output array
  memcpy(edgeflow_debug_msg + 128, &edge_hist_prev_int8,
         128 * sizeof(uint8_t));// copy quality measures to output array
  memcpy(edgeflow_debug_msg + 128 * 2, &displacement_int8,
         128 * sizeof(uint8_t));// copy quality measures to output array
  memcpy(edgeflow_debug_msg + 128 * 3, &plot2,
         128 * sizeof(uint8_t));// copy quality measures to output array
  memcpy(edgeflow_debug_msg + 128 * 4, &plot3,
         128 * sizeof(uint8_t));// copy quality measures to output array

  memcpy(edgeflow_msg, edgeflow_debug_msg, 128 * 5 * sizeof(uint8_t));

  SendArray(edgeflow_msg, 128, 5);

#elif defined(USE_PPRZLINK)
  uint8_t frame_freq = boundint8(edgeflow.hz.x / edgeflow_params.RES);
  uint8_t func_freq = boundint8(1e6 / edgeflow.dt);
  uint8_t res = bounduint8(edgeflow_params.RES);

  int16_t vx = edgeflow.vel.x, vy = edgeflow.vel.y, vz = edgeflow.vel.z;
  int16_t dx = edgeflow_snapshot.dist_traveled.x, dy = edgeflow_snapshot.dist_traveled.y, dz = edgeflow_snapshot.dist_traveled.z;

  uint16_t avg_dist = edgeflow.avg_dist;

  pprz_msg_send_STEREOCAM_VELOCITY(&(pprz.trans_tx), &dev,
      0, &(res), &frame_freq, &func_freq,
      &vx, &vy, &vz, &dx, &dy, &dz,
      &(edgeflow.flow_quality), &(edgeflow_snapshot.quality),
      &avg_dist);
#else
  static uint8_t edgeflow_msg[22] = {0};
  edgeflow_msg[0] = edgeflow_params.RES;
  edgeflow_msg[1] = boundint8(edgeflow.distance_closest_obstacle / 10);
  edgeflow_msg[2] = boundint8(edgeflow.hz.x / edgeflow_params.RES);
  edgeflow_msg[3] = boundint8(1e6 / edgeflow.dt);

  edgeflow_msg[4] = (edgeflow.vel.x >> 8) & 0xff;
  edgeflow_msg[5] = (edgeflow.vel.x) & 0xff;
  edgeflow_msg[6] = (edgeflow.vel.y >> 8) & 0xff;
  edgeflow_msg[7] = (edgeflow.vel.y) & 0xff;
  edgeflow_msg[8] = (edgeflow.vel.z >> 8) & 0xff;
  edgeflow_msg[9] = (edgeflow.vel.z) & 0xff;

  edgeflow_msg[10] = (edgeflow.vel_x_stereo_avoid_pixelwise >> 8) & 0xff;
  edgeflow_msg[11] = (edgeflow.vel_x_stereo_avoid_pixelwise) & 0xff;
  edgeflow_msg[12] = (edgeflow.vel_z_stereo_avoid_pixelwise >> 8) & 0xff;
  edgeflow_msg[13] = (edgeflow.vel_z_stereo_avoid_pixelwise) & 0xff;

  edgeflow_msg[14] = (edgeflow_snapshot.dist_traveled.x >> 8) & 0xff;
  edgeflow_msg[15] = (edgeflow_snapshot.dist_traveled.x) & 0xff;
  edgeflow_msg[16] = (edgeflow_snapshot.dist_traveled.y >> 8) & 0xff;
  edgeflow_msg[17] = (edgeflow_snapshot.dist_traveled.y) & 0xff;
  edgeflow_msg[18] = (edgeflow_snapshot.dist_traveled.z >> 8) & 0xff;
  edgeflow_msg[19] = (edgeflow_snapshot.dist_traveled.z) & 0xff;

  edgeflow_msg[20] = (edgeflow.flow_quality);
  edgeflow_msg[21] = (edgeflow_snapshot.quality);

  SendArray(edgeflow_msg, 22, 1);
#endif
}

/*  calculate_edge_flow: calculate the global optical flow by edgeflow
 * \param in[]            is an array containing the pixel intensities of the (stereo)image
 * \param edgeflow_params is a struct containing al the parameters for edgeflow
 * \param edgeflow        is a struct containing the resulting values of edgeflow
 * */
void calculate_edge_flow(uint8_t *in, struct edgeflow_parameters_t *params, struct edgeflow_t *edgeflow)
{
  int16_t border = params->window_size + params->disparity_range;

  // check that inputs within allowable ranges
  if (params->disparity_range > EDGEFLOW_DISPARITY_RANGE) {
    params->disparity_range = EDGEFLOW_DISPARITY_RANGE;
  }

  // Assign pointers to parameters
  int32_t monocam = params->use_monocam;
  int32_t fov_x = params->fovx;
  int32_t fov_y = params->fovy;
  int32_t RES = params->RES;
  uint32_t R = params->R;
  uint32_t Q = params->Q;
  int16_t img_w = params->img_w;
  int16_t img_h = params->img_h;

  // Define arrays and pointers for edge histogram and displacements
  int32_t *edge_hist_x = edgeflow->edge_hist[edgeflow->current_frame_nr].x;
  int32_t *edge_hist_x_right =  edgeflow->edge_hist_right;
  int32_t *edge_hist_y = edgeflow->edge_hist[edgeflow->current_frame_nr].y;

  // Calculate Edge Histogram
  calculate_edge_hist(in, edge_hist_x, img_w, img_h, 'x', 'l');
  calculate_edge_hist(in, edge_hist_y, img_w, img_h, 'y', 'l');
  calculate_edge_hist(in, edge_hist_x_right, img_w, img_h, 'x', 'r');

  if (edgeflow->ran_twice < 2) {
    edgeflow->ran_twice++;
    return;
  }

  // compute previous frame number relative to dynamic parameters
  edgeflow->prev_frame_x = (edgeflow->current_frame_nr - edgeflow->prev_frame_offset_x
                            + MAX_HORIZON) %  MAX_HORIZON; // wrap index
  edgeflow->prev_frame_y = (edgeflow->current_frame_nr - edgeflow->prev_frame_offset_y
                            + MAX_HORIZON) %  MAX_HORIZON; // wrap index

  //store the time of the frame
  edgeflow->dt = edgeflow->edge_hist[edgeflow->current_frame_nr].frame_time -
                 edgeflow->edge_hist[(edgeflow->current_frame_nr - 1 + MAX_HORIZON) %  MAX_HORIZON].frame_time;

  edgeflow->hz.x = 1e6 / ((edgeflow->edge_hist[edgeflow->current_frame_nr].frame_time - edgeflow->edge_hist[edgeflow->prev_frame_x].frame_time) / RES);
  edgeflow->hz.y = 1e6 / ((edgeflow->edge_hist[edgeflow->current_frame_nr].frame_time - edgeflow->edge_hist[edgeflow->prev_frame_y].frame_time) / RES);

  // Compute stereo image disparity
  calculate_disparity(edge_hist_x, edge_hist_x_right, edgeflow->disp.stereo, img_w, params->window_size,
                      params->stereo_shift * RES, edgeflow->disp.confidence_stereo, RES, 0, params->disparity_range);

  edgeflow->avg_disp = getMeanDisp(edgeflow->disp.stereo, img_w, border, RES / 4);

  /*edgeflow->avg_disp = calculate_disparity_fullimage(edge_hist_x, edge_hist_x_right, img_w, disp_range,
                               edgeflow_params->stereo_shift);*/

  // disparity to distance in dm given 6cm dist between cams and Field of View (FOV) of 60deg
  // d =  Npix*cam_separation /(2*disp*tan(FOV/2))
  // d = 0.06*128 / (2*tan(disp*1.042/2))
  // d = 0.06*128 / (2*disp*1.042/2)
  // d = RES*0.06*128 / (disp*RES*1.042)
  // d = RES*0.06*PIX / (disp*FOVX)

  // TODO replace with LUT
  if (edgeflow->avg_disp > 0) {
    edgeflow->avg_dist = (RES * RES * params->camera_seperation * img_w) / (edgeflow->avg_disp * fov_x);
  }
  if (monocam) {
    edgeflow->avg_dist = edgeflow->edge_hist->alt;
  }

  //filter height: TODO: use height directly from lisa s edgeflow_params->snapshot = 0;
  switch (params->filter_type) {
    case KALMAN:
      // todo use a variable covariance value maybe based on fit quality
      edgeflow->avg_dist = simpleKalmanFilter(&(params->cov.C_dist.z), edgeflow->prev_avg_dist, edgeflow->avg_dist, Q, R, RES);
      break;
    case MOVING_AVERAGE:
      edgeflow->avg_dist =  moving_average(edgeflow->prev_avg_dist, edgeflow->avg_dist, params->alpha, RES);
      break;
    default:
      break;
  }

  //calculate angle diff [pix/s * RES]
  int16_t der_shift_x = 0;
  int16_t der_shift_y = 0;

  if (params->derotation) {
    // TODO: consider adding the pixel dependent derotation (edge_x: qx^2, edge_y: py^2), may increase the fit quality under rotation
    der_shift_y = RES * (edgeflow->edge_hist[edgeflow->prev_frame_y].phi - edgeflow->edge_hist[edgeflow->current_frame_nr].phi) * img_h / params->fovy;
    der_shift_x = RES * (edgeflow->edge_hist[edgeflow->prev_frame_x].theta - edgeflow->edge_hist[edgeflow->current_frame_nr].theta) * img_w / params->fovx;
  }

  // Calculate displacement
  calculate_disparity(edgeflow->edge_hist[edgeflow->prev_frame_x].x, edge_hist_x, edgeflow->disp.x, img_w, params->window_size,
                      der_shift_x, edgeflow->disp.confidence_x, RES, -params->disparity_range, params->disparity_range);

  calculate_disparity(edgeflow->edge_hist[edgeflow->prev_frame_y].y, edge_hist_y, edgeflow->disp.y, img_h, params->window_size,
                      der_shift_y, edgeflow->disp.confidence_y, RES, -params->disparity_range, params->disparity_range);

  float scale_x = (float)(edgeflow->hz.x * params->camera_seperation * params->img_w) / params->fovx;
  float scale_y = (float)(edgeflow->hz.y * edgeflow->avg_dist) / (params->RES * params->RES);

  edgeflow->flow_quality = bounduint8((int32_t)(params->RES * coupled_flow_fit(&(edgeflow->disp), &(edgeflow->disp), RES,
                                      scale_x, scale_y, params->img_w, params->img_h, border, &(edgeflow->vel))));

  // change axis system from left-top to centre-centre
  edgeflow->vel.x += edgeflow->vel.z * img_w / 2;
  edgeflow->vel.y += edgeflow->vel.z * img_h / 2;

  // scale velocity by camera characteristics
  edgeflow->vel.x = edgeflow->vel.x * fov_x / (img_w * RES);
  edgeflow->vel.y = edgeflow->vel.y * fov_y / (img_h * RES);

  // z is facing forward
  edgeflow->vel.z = -edgeflow->vel.z;

  // filter output
  switch (params->filter_type) {
    case KALMAN:
      // todo use a variable covariance value maybe based on fit quality
      edgeflow->vel.x = simpleKalmanFilter(&(params->cov.C_vel.x), edgeflow->prev_vel.x, edgeflow->vel.x, Q, R, RES);
      edgeflow->vel.y = simpleKalmanFilter(&(params->cov.C_vel.y), edgeflow->prev_vel.y, edgeflow->vel.y, Q, R, RES);
      edgeflow->vel.z = simpleKalmanFilter(&(params->cov.C_vel.z), edgeflow->prev_vel.z, edgeflow->vel.z, Q, R, RES);
      break;
    case MOVING_AVERAGE:
      edgeflow->vel.x =  moving_average(edgeflow->prev_vel.x, edgeflow->vel.x, params->alpha, RES);
      edgeflow->vel.y =  moving_average(edgeflow->prev_vel.y, edgeflow->vel.y, params->alpha, RES);
      edgeflow->vel.z =  moving_average(edgeflow->prev_vel.z, edgeflow->vel.z, params->alpha, RES);
      break;
    default:
      break;
  }

  // Store previous values
  edgeflow->prev_avg_dist = edgeflow->avg_dist;
  edgeflow->prev_vel = edgeflow->vel;

  // update previous frame offset for next computation
  if (MAX_HORIZON > 2 && (params->adapt_horizon == 1)) {
    static uint32_t flow_mag_x, flow_mag_y;
    static int8_t change_x, change_y;
    flow_mag_x = abs(edgeflow->disp.x[abs_max((int32_t *)edgeflow->disp.x, (uint32_t)img_w)] - der_shift_x);
    flow_mag_y = abs(edgeflow->disp.y[abs_max((int32_t *)edgeflow->disp.y, (uint32_t)img_h)] - der_shift_y);

    // get similarity with previous frame
    int32_t similarity_x = SAD(edge_hist_x, edgeflow->edge_hist[(edgeflow->current_frame_nr - 1 + MAX_HORIZON) % MAX_HORIZON].x, img_w);
    int32_t similarity_y = SAD(edge_hist_y, edgeflow->edge_hist[(edgeflow->current_frame_nr - 1 + MAX_HORIZON) % MAX_HORIZON].y, img_h);

    const uint32_t min_flow = 300;//params->disparity_range * RES / 5;
    const uint32_t max_flow = 700;//params->disparity_range * RES / 2;

    change_x = change_y = 0;
    // Increment or decrement previous frame offset based on previous measured flow.
    if ((flow_mag_x >= max_flow) || similarity_x < 10000) {  // flow too high or previous frame very similar, likely not moving
      change_x = -1;
    } else if (flow_mag_x <= min_flow) {                     // flow low, can increase resolution
      change_x = 1;
    }

    if ((int8_t)edgeflow->prev_frame_offset_x + change_x > 0 && edgeflow->prev_frame_offset_x + change_x < MAX_HORIZON) {
      edgeflow->prev_frame_offset_x += change_x;
    }

    if ((flow_mag_y >= max_flow) || similarity_y < 10000) { // flow too high or previous frame very similar, likely not moving
      change_y = -1;
    } else if (flow_mag_y <= min_flow) {                    // flow low, can increase resolution
      change_y = 1;
    }

    if ((int8_t)edgeflow->prev_frame_offset_y + change_y > 0 && edgeflow->prev_frame_offset_y + change_y < MAX_HORIZON) {
      edgeflow->prev_frame_offset_y += change_y;
    }
  }
}

/*  calculate_edge_flow: calculate the global optical flow by edgeflow
 * \param params edgeflow settings
 * \param edgeflow is a struct containing the previously computed values of edgeflow
 * \param snapshot is a struct containting the resulting snapshot displacement
 * */
void calculate_snapshot_displacement(struct edgeflow_parameters_t *params, struct edgeflow_t *edgeflow, struct snapshot_t *snapshot)
{
  int16_t border = params->window_size + params->disparity_range;

  //calculate angle diff [pix/s * RES]
  int16_t der_shift_x = 0;
  int16_t der_shift_y = 0;

  if (params->derotation) {
    der_shift_y = params->RES * (snapshot->keyframe.phi - edgeflow->edge_hist[edgeflow->current_frame_nr].phi) * params->img_h / (params->fovy);
    der_shift_x = params->RES * (snapshot->keyframe.theta - edgeflow->edge_hist[edgeflow->current_frame_nr].theta) * params->img_w / (params->fovx);
  }

  // Calculate displacement
  calculate_disparity(snapshot->keyframe.x, edgeflow->edge_hist[edgeflow->current_frame_nr].x, snapshot->disp.x, params->img_w, params->window_size,
                      der_shift_x, snapshot->disp.confidence_x, params->RES, -params->disparity_range, params->disparity_range);

  calculate_disparity(snapshot->keyframe.y, edgeflow->edge_hist[edgeflow->current_frame_nr].y, snapshot->disp.y, params->img_h, params->window_size,
                      der_shift_y, snapshot->disp.confidence_y, params->RES, -params->disparity_range, params->disparity_range);

  float scale_x = (float)(params->RES * params->camera_seperation * params->img_w) / params->fovx;
  float scale_y = (float)edgeflow->avg_dist / params->RES;

  // solve fit
  snapshot->quality = bounduint8((int32_t)(params->RES * coupled_flow_fit(&(snapshot->disp), &(edgeflow->disp),
                                          params->RES, scale_x, scale_y, params->img_w, params->img_h, border, &(snapshot->dist))));

  // change axis system from left-top to centre-centre
  snapshot->dist.x += snapshot->dist.z * params->img_w / 2;
  snapshot->dist.y += snapshot->dist.z * params->img_h / 2;

  // scale velocity by camera characteristics
  snapshot->dist.x = snapshot->dist.x * params->fovx / (params->img_w * params->RES);
  snapshot->dist.y = snapshot->dist.y * params->fovy / (params->img_h * params->RES);

  // z axis should be pointing forward
  snapshot->dist.z = -snapshot->dist.z;

  // filter output
  switch (params->filter_type) {
    case KALMAN:
      // todo use a variable covariance value maybe based on fit quality
      snapshot->dist.x = simpleKalmanFilter(&(params->cov.C_dist.x),
                                            snapshot->prev_dist.x + edgeflow->vel.x * edgeflow->dt / 1e6, snapshot->dist.x, params->Q, params->R, params->RES);
      snapshot->dist.y = simpleKalmanFilter(&(params->cov.C_dist.y),
                                            snapshot->prev_dist.y + edgeflow->vel.y * edgeflow->dt / 1e6, snapshot->dist.y, params->Q, params->R, params->RES);
      snapshot->dist.z = simpleKalmanFilter(&(params->cov.C_dist.z),
                                            snapshot->prev_dist.z + edgeflow->vel.z * edgeflow->dt / 1e6, snapshot->dist.z, params->Q, params->R, params->RES);
      break;
    case MOVING_AVERAGE:
      snapshot->dist.x =  moving_average(snapshot->prev_dist.x + edgeflow->vel.x * edgeflow->dt / 1e6,
                                         snapshot->dist.x, params->alpha, params->RES);
      snapshot->dist.y =  moving_average(snapshot->prev_dist.y + edgeflow->vel.y * edgeflow->dt / 1e6,
                                         snapshot->dist.y, params->alpha, params->RES);
      snapshot->dist.z =  moving_average(snapshot->prev_dist.z + edgeflow->vel.z * edgeflow->dt / 1e6,
                                         snapshot->dist.z, params->alpha, params->RES);
      break;
    default:
      break;
  }

  // compute distance travelled since last snapshot
  snapshot->dist_traveled.x = snapshot->dist.x - snapshot->prev_dist.x;
  snapshot->dist_traveled.y = snapshot->dist.y - snapshot->prev_dist.y;
  snapshot->dist_traveled.z = snapshot->dist.z - snapshot->prev_dist.z;

  // determine if we need a new snapshot
  int32_t max_displacement = abs(snapshot->disp.x[abs_max(snapshot->disp.x, params->img_w)]);

  if ((edgeflow->edge_hist[edgeflow->current_frame_nr].frame_time - snapshot->keyframe.frame_time) > 5e6    // more than 5 seconds
      || max_displacement >= params->RES * params->disparity_range / 2                                      // max displacement aboe 50% of max
      //|| snapshot->quality < 20                                                                  // fit quality too low
      || sumN1 < IMAGE_WIDTH / 10                                                                           // not enough points for fit
      || sumN2 < IMAGE_HEIGHT / 10) {                                                                       // not enough points for fit
    snapshot->keyframe = edgeflow->edge_hist[edgeflow->current_frame_nr];
    snapshot->prev_dist = (struct vec3_t) {0, 0, 0};
  } else {
    // Store previous values
    snapshot->prev_dist = snapshot->dist;
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

// used as a temporary storage for Sum of Absolute difference computations
static int32_t __ccmram SAD1[IMAGE_WIDTH] = {0};
static int32_t __ccmram SAD2[IMAGE_WIDTH] = {0};
static int32_t __ccmram SAD_left[IMAGE_WIDTH] = {0};
static int32_t __ccmram SAD_right[IMAGE_WIDTH] = {0};
static int32_t __ccmram SAD_prev[IMAGE_WIDTH] = {0};
static int32_t __ccmram run_sum1[IMAGE_WIDTH] = {0};
static int32_t __ccmram run_sum2[IMAGE_WIDTH] = {0};

/* calculate_displacement_stereo: Calculate_displacement calculates the displacement between two histograms in stereo
 * \param edge_hist1
 * \param edge_hist2
 * \param disparity is an array that contains the pixel displacements of the compared edgehistograms
 * \param size indicates the size of array (important for x and y direction)
 * \param window indicates the pixel size of the window of neighboring pixels
 * \param pix_shift is the pixel shift between the stereocameras, determined by calibration in subpixel (scaled by RES)
 * \param min_disp minimum disparity to search
 * \param max_disp maximum disparity to search
 * */
uint32_t calculate_disparity(int32_t *edge_hist1, int32_t *edge_hist2, int32_t *disparity, uint16_t size,
                             uint8_t window, int16_t pix_shift, uint32_t *confidence,
                             int32_t RES, int32_t min_disp, int32_t max_disp)
{
  int32_t c, x1, x2;
  int32_t SAD_temp;

  uint32_t disp2[IMAGE_WIDTH] = {0};

  int32_t W = window;
  int32_t right_shift = max_disp + min_disp;    // This shift is used to add extra offset for array indexing with displacement check
  uint32_t sum_error = 0;

  int16_t border_left = max_disp;           //
  int16_t border_right = size + min_disp;   // todo min_disp > 0

  // for debugging derotation
  //int32_t sub_pix_shift = pix_shift;
  //pix_shift = 0;

  int32_t sub_pix_shift = pix_shift % RES;
  pix_shift /= RES;

  if (pix_shift > 0) {
    border_left += pix_shift;
  } else if (pix_shift < 0) {
    border_right += pix_shift;
  }

  arm_fill_q31(0, disparity, size);
  arm_fill_q31(0, confidence, size);

  if (border_right - border_left < 10) {
    return UINT32_MAX;    // too much pixel shift to compute result
  }

  // border with window
  int16_t border_leftW = border_left + W + 1;   // left border of output
  int16_t border_rightW = border_right - W;     // right left border of output

  arm_fill_q31(INT32_MAX, SAD1, IMAGE_WIDTH);
  arm_fill_q31(INT32_MAX, SAD2, IMAGE_WIDTH);
  arm_fill_q31(0, SAD_prev, IMAGE_WIDTH);

  for (c = min_disp; c <= max_disp; c++) {
    // compute running sum of SAD for all pixels at current disparity
    run_sum1[border_left] = abs(edge_hist1[border_left] - edge_hist2[border_left - c - pix_shift]);
    run_sum2[border_left - right_shift] = abs(edge_hist1[border_left - right_shift + c] -
                                          edge_hist2[border_left - right_shift - pix_shift]);
    for (x1 = border_left + 1, x2 = x1 - right_shift; x1 < border_right; x1++, x2++) {
      run_sum1[x1] = run_sum1[x1 - 1] + abs(edge_hist1[x1] - edge_hist2[x1 - c - pix_shift]);
      run_sum2[x2] = run_sum2[x2 - 1] + abs(edge_hist1[x2 + c] - edge_hist2[x2 - pix_shift]);
    }

    // compute SAD over window [x-W,x+W] for each pixel using the running sum
    for (x1 = border_leftW, x2 = x1 - right_shift; x1 < border_rightW; x1++, x2++) {
      SAD_temp = run_sum2[x1 + W] - run_sum2[x1 - W - 1];
      // store disparity with minimum SAD error for left image
      if (SAD_temp < SAD1[x1]) {
        SAD1[x1] = SAD_temp;
        disparity[x1] = c;
        SAD_left[x1] = SAD_prev[x1];
        SAD_right[x1] = 0;
      } else {
        if (!SAD_right[x1]) {
          SAD_right[x1] = SAD_temp;
        }
      }
      SAD_prev[x1] = SAD_temp;
      // Same for right image
      SAD_temp = run_sum2[x2 + W] - run_sum2[x2 - W - 1];
      if (SAD_temp < SAD2[x2]) {
        SAD2[x2] = SAD_temp;
        disp2[x2] = c;
      }
    }
  }

  for (x1 = border_leftW; x1 < border_rightW; x1++) {
    if (disparity[x1] != disp2[x1 - disparity[x1]]) { // occluded image column
      disparity[x1] = 0;
      confidence[x1] = 0;
    } else { // good fit
      sum_error += SAD1[x1];
      // subpixel disparity computation, fit parabola and assign minimum
      if (disparity[x1] > min_disp && disparity[x1] < max_disp) {
        disparity[x1] = RES * disparity[x1] + sub_pix_shift - RES * (SAD_right[x1] - SAD_left[x1]) / (SAD_left[x1] - 2 * SAD1[x1] + SAD_right[x1]) / 2;
      } else {
        disparity[x1] = RES * disparity[x1] + sub_pix_shift;
      }

      if (!SAD1[x1]) {
        SAD1[x1] = 1;
      }

      // confidence is the relative error of min and surrounding fit
      if ((SAD_right[x1] < SAD_left[x1] && disparity[x1] < max_disp) || disparity[x1] <= min_disp) {
        confidence[x1] = RES * SAD_right[x1] / SAD1[x1];
      } else {
        confidence[x1] = RES * SAD_left[x1] / SAD1[x1];
      }
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
  static int32_t SAD[EDGEFLOW_DISPARITY_RANGE] = {0};

  int32_t c, x;
  int32_t min_error;
  uint32_t min_index;

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

/** Returns the minimum value in an array with the accompanying index
 * If there are multiple indices containing the same minimum value,
 * the index closest to the min_index will be returned
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

/* getMean: calculate mean of array
 * \param a is an array containing the values
 * \param n is size of the array
 * \return mean of array
 * */
uint32_t getMeanDisp(uint32_t *a, int32_t n, int32_t border, uint32_t thresh)
{
  int32_t dSum = 0;
  int32_t i, count = 0;
  for (i = border; i < n - border; ++i) {
    if (a[i] >= thresh) {
      dSum += a[i];
      count++;
    }
  }
  if (count) {
    return dSum / count;
  } else {
    return 0;
  }
}

/** Compute divergence and ventral flow using coupled Bayesian linear regression
 * In short, we linearly solve the following set of equations with a bayesian prior
 * u * Z = -U + xW
 * v * Z = -V + yW
 * where u and v are the components of the flow vector at colomn x and row y respectively, Z is the distance to the row/colomn,
 * U and V are the relative velocities in the x and y directions respectively.
 * The solution to the Bayesian linear regression for [U V W]' is given by inv(X'X + P)*X'Y
 * in this case X is a vector of the indicies of the row and colomn and Y are the displacments
 *
 * @param disp displacement being fit
 * @param stereo_disp displacement sruct holding stereo distance and confidence
 * @param RES is resolution of displacement
 * @param scale_x is factor to scale dispalcement in x direction
 * @param scale_y is factor to scale dispalcement in y direction
 * @param w width of array
 * @param h heihgt of array
 * @param border usable array boundaries
 * @param result Resultant fit parameters
 */
float coupled_flow_fit(struct displacement_t *disp, struct displacement_t *stereo_disp,
                       uint32_t RES, float scale_x, float scale_y, uint16_t w, uint16_t h, uint16_t border,
                       struct vec3_t *result)
{

  static const float prior = 0.5f;
  static int32_t x, y;
  static float u, v;

  uint32_t conf_threshold = (int32_t)(1.2 * RES);  // confidence threshold

  sumN1 = 0.f; sumN2 = 0.f;
  float sumY1 = 0.f, sumY2 = 0.f;
  float sumY1Y1 = 0.f, sumY2Y2 = 0.f;
  float sumX1 = 0.f, sumX2 = 0.f;
  float sumX12 = 0.f, sumX22 = 0.f;
  float sumXY1 = 0.f, sumXY2 = 0.f;

  for (x = border; x < w - border; x++) {
    if (stereo_disp->stereo[x] >= RES && stereo_disp->confidence_stereo[x] >= conf_threshold && disp->confidence_x[x] >= conf_threshold) {
      sumN1++;
      sumX1 += x;
      sumX12 += x * x;

      // convert displacement to flow vector
      u = (float)(disp->x[x]) * scale_x / stereo_disp->stereo[x];

      sumY1 += u;
      sumY1Y1 += u * u;
      sumXY1 += u * x;
    }
  }

  for (y = border; y < h - border; y++) {
    if (disp->confidence_y[y] >= conf_threshold) {
      sumN2++;
      sumX2 += y;
      sumX22 += y * y;

      // convert displacement to flow vector
      v = (float) disp->y[y] * scale_y;

      sumY2 += v;
      sumY2Y2 += v * v;
      sumXY2 += v * y;
    }
  }

  // introduce prior and some variables for products used multiple times
  sumN1 += prior;
  sumN2 += prior;
  float sumX12X22 = sumX12 + sumX22 + prior;
  float sumX1X1 = sumX1 * sumX1;
  float sumX2X2 = sumX2 * sumX2;
  float sumX1X2 = sumX1 * sumX2;
  float sumXY1XY2 = sumXY1 + sumXY2;

  float det = sumN1 * sumN2 * sumX12X22 - sumN1 * sumX2X2 - sumN2 * sumX1X1;

  // solve least squares equation y\A
  float U = (sumN2 * (sumX12X22 * sumY1 - sumX1 * sumXY1XY2) + sumX1X2 * sumY2 - sumX2X2 * sumY1) / det;
  float V = (sumN1 * (sumX12X22 * sumY2 - sumX2 * sumXY1XY2) + sumX1X2 * sumY1 - sumX1X1 * sumY2) / det;
  float W = (sumN1 * sumN2 * sumXY1XY2 - sumN1 * sumX2 * sumY2 - sumN2 * sumX1 * sumY1) / det;

  // compute fit quality, coefficient of determination
  float R2 = 1.0f;
  if (sumY1Y1 + sumY2Y2 > 0) {
    R2 = (U * (U * sumN1 + W * sumX1) + V * (V * sumN2 + W * sumX2) + W * (U * sumX1 + V * sumX2 + W * sumX12X22)) / (sumY1Y1 + sumY2Y2);
  }

  result->x = (int32_t)U;
  result->y = (int32_t)V;
  result->z = (int32_t)W;

  return R2;
}
