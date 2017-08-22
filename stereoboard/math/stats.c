/*
 * stats.c
 *
 *  Created on: 11 Jul 2017
 *      Author: kirk
 */

#include "math/stats.h"

#include <stdlib.h>

/* mean: computes the mean of array
 * \param a is an array containing the values
 * \param n is size of the array
 * \return mean of array
 * */
int32_t mean(int32_t *a, uint32_t n)
{
  int32_t dSum = a[0];
  uint32_t i;
  for (i = 1; i < n; ++i) {
    dSum += a[i];
  }
  return dSum / n;
}

/* median: computes the median value of array
 * \param a is an array containing the values
 * \param n is size of the array
 * \return median of array
 * */
int32_t median(int32_t *a, uint32_t n)
{
  // Allocate an array of the same size and sort it.
  uint32_t i, j;

  int32_t dpSorted[n];
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

/* max: finds the index of the maximum value in array
 * \param a is an array containing the values
 * \param n is size of the array
 * \return max_ind is the index of the maximum value located on the array
 * */
uint32_t max(int32_t *a, uint32_t n)
{
  uint32_t i;
  uint32_t ind = 0;
  int32_t max_error = a[0];

  for (i = 1; i < n; i++) {
    if (a[i] > max_error) {
      ind = i;
      max_error = a[i];
    }
  }
  return ind;
}

/* abs_max: finds the index of the absolute maximum value in array
 * \param a is an array containing the values
 * \param n is size of the array
 * \return max_ind is the index of the absolute maximum value located on the array
 * */
uint32_t abs_max(int32_t *a, uint32_t n)
{
  uint32_t i;
  uint32_t ind = 0;
  int32_t max_error = abs(a[0]);

  for (i = 1; i < n; i++) {
    if (abs(a[i]) > max_error) {
      ind = i;
      max_error = abs(a[i]);
    }
  }
  return ind;
}

/* min: finds the index of the minimum value in array
 * \param a is an array containing the values
 * \param n is size of the array
 * \return min_ind is the index of the minimum value located on the array
 * */
uint32_t min(int32_t *a, uint32_t n)
{
  uint32_t i;
  uint32_t ind = 0;
  int32_t min_error = a[ind];
  for (i = 1; i < n; i++) {
    if (a[i] < min_error) {
      ind = i;
      min_error = a[i];
    }
  }
  return ind;
}

/* abs_min: finds the index of the absolute maximum value in array
 * \param a is an array containing the values
 * \param n is size of the array
 * \return min_ind is the index of the absolute maximum value located on the array
 * */
uint32_t abs_min(int32_t *a, uint32_t n)
{
  uint32_t i;
  uint32_t ind = 0;
  int32_t min_error = abs(a[0]);

  for (i = 1; i < n; i++) {
    if (abs(a[i]) < min_error) {
      ind = i;
      min_error = abs(a[i]);
    }
  }
  return ind;
}

/* getPeaks: calculate number of peaks in an array
 * \param array is an edgehistogram
 * \param n is the number of elements in the array
 * \return number of peaks
 * */
uint32_t countPeaks(int32_t *array, int32_t n)
{
  uint32_t peaks = 0;
  int32_t i = 0;

  int32_t med = median(array, n);

  for (i = 1; i < n - 1; i++) {
    if (array[i - 1] < array[i] && array[i] > array[i + 1]
        && array[i] > med) {
      peaks++;
    }
  }
  return peaks;
}

/* sum: compute sum of array
 * \param array
 * \param n is the number of elements in array
 * \return sum of array elements
 */
uint32_t sum(uint8_t *array, uint32_t n)
{
  uint32_t i;
  uint32_t sum = 0;
  for (i = 0; i < n; i++) {
    sum += (uint32_t) array[i];
  }
  return sum;
}

/* SAD: Sum of Absolute Differences
 * \param array1
 * \param array2
 * \param n is the number of elements in array
 * \return sum of absoulte differences
 */
uint32_t SAD(int32_t *array1, int32_t *array2, uint32_t n)
{
  uint32_t i;
  uint32_t SAD = 0;
  for (i = 0; i < n; i++) {
    SAD += abs(array1[i] - array2[i]);
  }
  return SAD;
}
