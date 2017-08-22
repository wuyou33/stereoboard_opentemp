/*
 * line_fit.h
 *
 *  Created on: 11 Jul 2017
 *      Author: kirk
 */

#ifndef STEREOBOARD_MATH_LINE_FIT_H_
#define STEREOBOARD_MATH_LINE_FIT_H_

#include <inttypes.h>
#include "image.h"

/* line_fit: fits a line using least squares to the histogram disparity map
 * \param points is an array that contains the pixel displacements of the compared edgehistograms
 * \param slope is slope of the optical intercept field
 * \param inercept is intercept of the optical intercept (calculated from middle from image)
 * \param size is the  size of stereo_distance_per_column
 * \param border is the search window + search distance used in blockmatching
 * \param RES is resolution used to calculate the line fit (int based math)
 * \param x_offset offset is x value for linefit
 * \return average fit error
 */
// Line_fit fits a line using least squares to the histogram disparity map
extern uint32_t line_fit(int32_t *points, int32_t *slope, int32_t *intercept,
                  uint32_t size, uint32_t border, int32_t RES, int32_t x_offset);

/* weighted_line_fit: fits a line using least squares to the histogram disparity map
 * \param points is an array that contains the pixel displacements of the compared edgehistograms
 * \param wieght is the weighting applied to each point
 * \param slope is slope of the optical intercept field
 * \param intercept is intercept of the optical intercept (calculated from middle from image)
 * \param size is the  size of stereo_distance_per_column
 * \param border is the search window + search distance used in blockmatching
 * \param RES is resolution used to calculate the line fit (int based math)
 * \param x_offset offset is x value for linefit
 * \return average fit error
 */
extern uint32_t weighted_line_fit(int32_t *points, int32_t *weight, int32_t *slope,
                           int32_t *intercept, uint32_t size, uint32_t border, uint16_t RES,
                           int32_t x_offset);

/* weighted_line_fit: fits a line using least squares to the histogram disparity map, excluding the areas that have faulty distance measurements
 * \param points is an array that contains the pixel displacements of the compared edgehistograms
 * \param wieght is the weighting applied to each point
 * \param slope is slope of the optical intercept field
 * \param intercept is intercept of the optical intercept (calculated from middle from image)
 * \param size is the  size of stereo_distance_per_column
 * \param border is the search window + search distance used in blockmatching
 * \param RES is resolution used to calculate the line fit (int based math)
 * \param p0 is the constraint condition, line must pass through (x0,y0), set (struct point_t){0,0} if not used
 */
extern uint32_t constrained_line_fit(int32_t *points, int32_t *weight, int32_t *slope,
                              int32_t *intercept, uint32_t size, uint32_t border, uint16_t RES,
                              struct point_t p0);

/* line_fit_RANSAC: fits a line using least squares to the histogram disparity map
 * \param points is an array that contains the pixel displacements of the compared edgehistograms
 * \param slope is slope of the optical intercept field
 * \paramintercept is intercept of the optical intercept (calculated from middle from image)
 * \param size is the  size of stereo_distance_per_column
 * \param border is the search window + search distance used in blockmatching
 * \param RES is resolution used to calculate the line fit (int based math)
 * TODO: Make the inlier_threshold and inlier ratio adaptable
 */
extern void line_fit_RANSAC(int8_t *ponts, int32_t *slope, int32_t *intercept, uint16_t size, uint32_t border, int32_t RES);


#endif /* STEREOBOARD_MATH_LINE_FIT_H_ */
