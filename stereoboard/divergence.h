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

#define MAX_HORIZON 10
// #define MAX_FLOW 1.0
// #define RANSAC 1
// #define ADAPTIVE_THRES_EDGE 0

struct edge_hist_t{
	int32_t horizontal[IMAGE_WIDTH];
	int32_t vertical[IMAGE_HEIGHT];
};

struct edge_flow_t{
	int32_t horizontal_trans;
	int32_t horizontal_slope;
	int32_t vertical_slope;
	int32_t vertical_trans;
};

struct displacement_t{
	int32_t horizontal[IMAGE_WIDTH];
	int32_t vertical[IMAGE_HEIGHT];
};

struct coveriance_t{
	int32_t trans_x;
	int32_t trans_y;
	int32_t slope_x;
	int32_t slope_y;
};

void calculate_edge_flow_simple(uint8_t* in,uint32_t* edge_histogram,uint32_t* edge_histogram_prev,int32_t* displacement,float* slope,float* yint,uint32_t image_width, uint32_t image_height);
int32_t calculate_edge_flow(uint8_t* in, struct displacement_t* displacement, struct edge_flow_t* edge_flow, struct edge_hist_t* edge_hist, uint16_t* height, uint8_t current_frame_nr,
			int16_t windowsize, int16_t max_distance, int16_t edge_threshold, uint16_t image_width, uint16_t image_height, uint16_t RES);
void image_difference(uint8_t* in,uint8_t* in_prev,uint8_t* out, uint16_t image_width, uint16_t image_height);
void calculate_edge_histogram(uint8_t* in,int32_t* edge_histogram, uint16_t image_width, uint16_t image_height, char direction, char side);
void calculate_displacement(int32_t* edge_histogram,int32_t* edge_histogram_prev, int32_t* displacement, uint32_t size);
int32_t calculate_displacement_fullimage(int32_t* edge_histogram,int32_t* edge_histogram_2, uint32_t size);

void line_fit(int32_t* displacement, int32_t* Slope, int32_t* Yint, uint32_t image_width, uint16_t RES);
void line_fit_RANSAC( int32_t* displacement, int32_t *slope, int32_t* yInt, uint32_t size, uint32_t RES);

void totalKalmanFilter(struct coveriance_t* coveriance,struct edge_flow_t* prev_edge_flow, struct edge_flow_t* edge_flow, uint32_t Q, uint32_t R, uint32_t RES);
int32_t simpleKalmanFilter(int32_t* cov, int32_t previous_est, int32_t current_meas, int32_t Q, int32_t R, int32_t RES);

void visualize_divergence(uint8_t* in, int32_t* displacement, int32_t slope, int32_t yInt, uint32_t image_width, uint32_t image_height);

int32_t getMinimum2(uint32_t* flow_error, uint32_t  max_ind);

#endif /* DIVERGENCE_H_ */
