/*
 * filter.c
 *
 *  Created on: 11 Jul 2017
 *      Author: kirk
 */

#include "math/filter.h"

/* moving_average: an average filter with a forget function
 * \param previous_est is the previous smoothed variable
 * \param current_meas is current measurement update
 * \param alpha is the forget factor
 * \param RES is resolution used for the int based math
 * \return new variable estimate
 */
int32_t moving_average(int32_t prev, int32_t curr, int32_t alpha, int32_t RES)
{
  return (alpha * curr + (RES - alpha) * prev) / RES;
}

/* simpleKalmanFilter: simple one dimension kalman filter
 * \param cov is covariance value of the measurement
 * \param prev is the previous estimate (from kalmanfilter previous timestep)
 * \param curr is current measurement update
 * \param Q is process noise
 * \param R is measurement noise
 * \param RES is resolution used for the int based math
 * \return new variable estimate
 */
int32_t simpleKalmanFilter(int32_t *cov, int32_t prev,
                           int32_t curr, int32_t Q, int32_t R, int32_t RES)
{
  int32_t predict_cov = *cov + Q;
  int32_t K = RES * predict_cov / (*cov + R);

  *cov = ((RES - K) * predict_cov) / RES;

  return (prev + (K * (curr - prev)) / RES);
}
