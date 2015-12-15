/*
 * distance_matrix.h
 *
 *  Created on: Jul 6, 2015
 *      Author: roland
 */

#ifndef DISTANCE_MATRIX_H_
#define DISTANCE_MATRIX_H_

#include <arm_math.h>
#include "../multigaze/stereoboard_parameters.h"
#include "main_parameters.h"


void calculateForwardVelocity(float distance,float alpha,  int MAX_SUBSEQUENT_OUTLIERS,int n_steps_velocity);
#endif /* DISTANCE_MATRIX_H_ */
