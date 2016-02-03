/*
 * distance_matrix.c
 *
 * Makes a matrix of widthPerBin x heightPerBin with the COUNTER_THRESHOLD-th disparity value.
 *
 *  Created on: Jul 6, 2015
 *      Author: roland
 */

#include "distance_matrix.h"
#include "../common/led.h"



#define MAX_RECORDED_VELOCITY_EST 500
int disparity_velocity_max_time = MAX_RECORDED_VELOCITY_EST;
float distancesHistory[MAX_RECORDED_VELOCITY_EST];
float timeStepHistory[MAX_RECORDED_VELOCITY_EST];
int disparity_velocity_step = 0;
int distancesRecorded = 0;
int timeStepsRecorded = 0;
int velocity_disparity_outliers = 0;
void new_array_pop(float *array, int lengthArray)
{
  int index;
  for (index = 1; index < lengthArray; index++) {
    array[index - 1] = array[index];
  }
}
float new_fabs(float input)
{
  if (input < 0) {
    return -1.0 * input;
  }
  return input;
}
float calculateForwardVelocity(float distance, float alpha, int MAX_SUBSEQUENT_OUTLIERS, int n_steps_velocity)
{
  disparity_velocity_step += 1;
  float new_dist = 0.0;
  if (distancesRecorded > 0) {
    new_dist = alpha * distancesHistory[distancesRecorded - 1] + (1 - alpha) * distance;
  }
  // Deal with outliers:
  // Single outliers are discarded, while persisting outliers will lead to an array reset:
  if (distancesRecorded > 0 && new_fabs(new_dist - distancesHistory[distancesRecorded - 1]) > 0.5) {
    velocity_disparity_outliers += 1;
    if (velocity_disparity_outliers >= MAX_SUBSEQUENT_OUTLIERS) {
      // The drone has probably turned in a new direction
      distancesHistory[0] = new_dist;
      distancesRecorded = 1;

      timeStepHistory[0] = disparity_velocity_step;
      timeStepsRecorded = 1;
      velocity_disparity_outliers = 0;
    }
  } else {
    //append
    velocity_disparity_outliers = 0;
    timeStepHistory[timeStepsRecorded] = disparity_velocity_step;
    distancesHistory[distancesRecorded] = new_dist;
    distancesRecorded++;
    timeStepsRecorded++;
  }

  //determine velocity (very simple method):
  float velocityFound = 0.0;
  if (distancesRecorded > n_steps_velocity) {
    velocityFound = distancesHistory[distancesRecorded - n_steps_velocity] - distancesHistory[distancesRecorded - 1];
  }
  // keep maximum array size:
  if (distancesRecorded > disparity_velocity_max_time) {
    new_array_pop(distancesHistory, disparity_velocity_max_time);
  }
  if (timeStepsRecorded > disparity_velocity_max_time) {
    new_array_pop(timeStepHistory, disparity_velocity_max_time);
  }
  return velocityFound;
}

