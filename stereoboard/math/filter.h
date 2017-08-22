/*
 * filter.h
 *
 *  Created on: 11 Jul 2017
 *      Author: kirk
 */

#ifndef STEREOBOARD_MATH_FILTER_H_
#define STEREOBOARD_MATH_FILTER_H_

#include <inttypes.h>

/* moving_average: a moving average filter (low pass filter)
 * \param prev is the previous smoothed variable
 * \param curr is current measurement update
 * \param alpha is the filter factor
 * \param RES is resolution used for the int based math
 * \return new variable estimate
 */
extern int32_t moving_average(int32_t prev, int32_t curr, int32_t alpha, int32_t RES);

/* simpleKalmanFilter: simple one dimension kalman filter
 * \param cov is covariance value of the measurement
 * \param prev is the previous estimate (from kalmanfilter previous timestep)
 * \param curr is current measurement update
 * \param Q is process noise
 * \param R is measurement noise
 * \param RES is resolution used for the int based math
 * \return new variable estimate
 */
extern int32_t simpleKalmanFilter(int32_t *cov, int32_t prev, int32_t curr, int32_t Q, int32_t R, int32_t RES);

#endif /* STEREOBOARD_MATH_FILTER_H_ */
