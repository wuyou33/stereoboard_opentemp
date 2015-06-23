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



void calculate_edge_flow_simple(uint8_t* in,uint32_t* edge_histogram,uint32_t* edge_histogram_prev,int32_t* displacement,float* slope,float* yint,uint32_t image_width, uint32_t image_height);
void image_difference(uint8_t* in,uint8_t* in_prev,uint8_t* out,uint32_t image_width, uint32_t image_height);
void calculate_edge_histogram(uint8_t* in,uint32_t* edge_histogram,uint32_t image_width, uint32_t image_height);
void calculate_displacement(uint32_t* edge_histogram,uint32_t* edge_histogram_prev,int32_t* displacement,uint32_t image_width,uint32_t image_height);
uint32_t getMinimum(uint32_t* flow_error, uint32_t max_ind);
void line_fit(int32_t* displacement, float* Slope, float* Yint,uint32_t image_width);
void line_fit_RANSAC( int32_t* displacement, float* Slope, float* Yint,uint16_t size);
void visualize_divergence(uint8_t* in,int32_t* displacement,float Slope, float Yint,uint32_t image_width, uint32_t image_height);
#endif /* DIVERGENCE_H_ */
