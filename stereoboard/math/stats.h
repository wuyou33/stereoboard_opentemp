/*
 * stats.h
 *
 *  Created on: 11 Jul 2017
 *      Author: kirk
 */

#ifndef STEREOBOARD_MATH_STATS_H_
#define STEREOBOARD_MATH_STATS_H_

#include <inttypes.h>

/* mean: computes the mean of array
 * \param a is an array containing the values
 * \param n is size of the array
 * \return mean of array
 * */
extern int32_t mean(int32_t *a, uint32_t n);

/* median: computes the median value of array
 * \param a is an array containing the values
 * \param n is size of the array
 * \return median of array
 * */
extern int32_t median(int32_t *a, uint32_t n);

/* max: finds the index of the maximum value in array
 * \param a is an array containing the values
 * \param n is size of the array
 * \return max_ind is the index of the maximum value located on the array
 * */
extern uint32_t max(int32_t *a, uint32_t n);

/* abs_max: finds the index of the absolute maximum value in array
 * \param a is an array containing the values
 * \param n is size of the array
 * \return max_ind is the index of the absolute maximum value located on the array
 * */
uint32_t abs_max(int32_t *a, uint32_t n);

/* min: finds the index of the minimum value in array
 * \param a is an array containing the values
 * \param n is size of the array
 * \return min_ind is the index of the minimum value located on the array
 * */
extern uint32_t min(int32_t *a, uint32_t n);

/* abs_min: finds the index of the absolute maximum value in array
 * \param a is an array containing the values
 * \param n is size of the array
 * \return min_ind is the index of the absolute maximum value located on the array
 * */
uint32_t abs_min(int32_t *a, uint32_t n);

/* getPeaks: calculate number of peaks in an array
 * \param array is an edgehistogram
 * \param n is the number of elements in the array
 * \return number of peaks
 * */
extern uint32_t countPeaks(int32_t *array, int32_t n);

/* sum: compute sum of array
 * \param array
 * \param n is the number of elements in array
 * \return sum of array elements
 */
extern uint32_t sum(uint8_t *array, uint32_t n);

/* SAD: Sum of Absolute Differences
 * \param array1
 * \param array2
 * \param n is the number of elements in array
 * \return sum of absoulte differences
 */
uint32_t SAD(int32_t *array1, int32_t *array2, uint32_t n);

#endif /* STEREOBOARD_MATH_STATS_H_ */
