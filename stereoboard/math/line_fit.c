/*
 * line_fit.c
 *
 *  Created on: 11 Jul 2017
 *      Author: kirk
 */

#include "math/line_fit.h"

#include <stdint.h>
#include <stdlib.h>
#include "math/stats.h"

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

  // *slope = (sumXY - sumX * yMean) / (sumX2 - sumX * xMean); // compute slope of line ax + b
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

/* weighted_line_fit: fits a line using least squares to the histogram disparity map
 * \param points is an array that contains the pixel displacements of the compared edgehistograms
 * \param wieght is the weighting applied to each point
 * \param slope is slope of the optical intercept field
 * \param intercept is intercept of the optical intercept (calculated from middle from image)
 * \param size is the  size of stereo_distance_per_column
 * \param border is the search window + search distance used in blockmatching
 * \param RES is resolution used to calculate the line fit (int based math)
 * \param p0 is the constraint condition, line must pass through (x0,y0), set (struct point_t){0,0} if not used
 */
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
  if (weight == NULL) {
    error = line_fit(points, slope, intercept, size, border, RES, p0.x);
  } else {
    error = weighted_line_fit(points, weight, slope, intercept, size, border, RES, p0.x);
  }
  *intercept = p0.y - *slope * p0.x;
  return error;
}

/* line_fit_RANSAC: fits a line using least squares to the histogram disparity map
 * \param points is an array that contains the pixel displacements of the compared edgehistograms
 * \param slope is slope of the optical intercept field
 * \param intercept is intercept of the optical intercept (calculated from middle from image)
 * \param size is the  size of stereo_distance_per_column
 * \param border is the search window + search distance used in blockmatching
 * \param RES is resolution used to calculate the line fit (int based math)
 * TODO: Make the inlier_threshold and inlier ratio adaptable
 */
void line_fit_RANSAC(int8_t *points, int32_t *slope, int32_t *intercept, uint16_t size, uint32_t border, int32_t RES)
{
  static int16_t inlier_threshold = 2000;
  static int16_t inlier_ratio = 40;
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
  // intercept = a * x + b
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
    dflow = points[ind2] - points[ind1];

    // Fit line with two points
    a[it] = RES * dflow / dx;
    b[it] = RES * (int32_t)points[ind1] - (a[it] * ind1);
    // evaluate fit:

    total_error = 0;
    for (entry = border; entry < size - border; entry++) {
      predicted_flow = a[it] * entry + b[it];
      error = abs((RES * (int32_t)points[entry] - predicted_flow));

      if ((int32_t) error < inlier_threshold * RES) {
        num_inliers++;
        total_error += (error / (RES));
      }

      //total_error += ipow(RES*points[entry] - predicted_flow,2);
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
  best_ind = min(errors, ransac_iter);

  if (counter_pass_check > 0) {
    *slope = a[best_ind];
    *intercept = b[best_ind];
  } else {
    *slope = 0;
    *intercept = 0;
  }

}
