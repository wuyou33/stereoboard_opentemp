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

#define MAX_HORIZON 2
#define MAX_FLOW 1.0
#define RANSAC 1
#define ADAPTIVE_THRES_EDGE 0

struct edge_hist_t{
	int  horizontal[IMAGE_WIDTH];
	int  vertical[IMAGE_HEIGHT];
};

struct edge_flow_t{
	float horizontal[2];
	float vertical[2];
};

struct displacement_t{
	int  horizontal[IMAGE_WIDTH];
	int  vertical[IMAGE_HEIGHT];
};


void calculate_edge_flow_simple(uint8_t* in,uint32_t* edge_histogram,uint32_t* edge_histogram_prev,int32_t* displacement,float* slope,float* yint,uint32_t image_width, uint32_t image_height);
void calculate_edge_flow(uint8_t* in, struct displacement_t* displacement,struct edge_flow_t* edge_flow, struct edge_hist_t* edge_hist[],int front,int rear,int windowsize,int max_distance,int edge_threshold,uint32_t  image_width,uint32_t  image_height);
void image_difference(uint8_t* in,uint8_t* in_prev,uint8_t* out,uint32_t image_width, uint32_t image_height);
void calculate_edge_histogram(uint8_t* in, int* edge_histogram,uint32_t image_width, uint32_t image_height);
void calculate_displacement(int* edge_histogram,int* edge_histogram_prev,int* displacement,uint32_t image_width,uint32_t image_height);
int getMinimum2(int * flow_error, int  max_ind);
void line_fit(int32_t* displacement, float* Slope, float* Yint,uint32_t image_width);
void line_fit_RANSAC( int* displacement, float* Slope, float* Yint,int size);
void visualize_divergence(uint8_t* in,int32_t* displacement,float Slope, float Yint,uint32_t image_width, uint32_t image_height);
#endif /* DIVERGENCE_H_ */
