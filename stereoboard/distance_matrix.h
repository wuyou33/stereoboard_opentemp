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


void calculateDistanceMatrix(uint8_t disparity_image[],
                             uint8_t blackBorderSize, uint8_t pixelsPerLine, uint8_t widthPerBin,
                             uint8_t heightPerBin, uint8_t toSendBuffer[], uint32_t disparity_range);
uint8_t calculateHeadingFromHistogram(uint8_t *histogramBuffer);
uint8_t maxInArray(uint8_t *array, int length);


#endif /* DISTANCE_MATRIX_H_ */
