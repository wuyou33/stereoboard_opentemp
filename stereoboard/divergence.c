/*
 * divergence.c
 *
 *  Created on: Apr 9, 2015
 *      Author: knmcguire
 */

#include "divergence.h"

#include "optic_flow.h"

#include <stdlib.h>

int32_t calculate_edge_flow(uint8_t *in, struct displacement_t *displacement, struct edge_flow_t *edge_flow,
                            struct edge_hist_t edge_hist[], uint16_t *height, uint8_t current_frame_nr, int16_t windowsize, int16_t max_distance,
                            int16_t edge_threshold, uint16_t image_width, uint16_t image_height, uint16_t RES)
{
  //Define arrays and pointers for edge histogram and displacements
  int32_t *edge_histogram_x = edge_hist[current_frame_nr].horizontal;
  int32_t *prev_edge_histogram_x;
  int32_t edge_histogram_right_x[IMAGE_WIDTH];

  int32_t *edge_histogram_y = edge_hist[current_frame_nr].vertical;
  int32_t *prev_edge_histogram_y;

  // Calculate previous frame number
  uint8_t previous_frame_offset_x = 1, previous_frame_offset_y = 1;

  if (MAX_HORIZON > 1) {
    uint32_t edge_flow_x, edge_flow_y;
    edge_flow_x = abs(edge_flow->horizontal_trans);
    edge_flow_y = abs(edge_flow->vertical_trans);

    if (edge_flow_x * (MAX_HORIZON - 1) > RES) {
      previous_frame_offset_x = RES / edge_flow_x + 1;
    } else {
      previous_frame_offset_x = MAX_HORIZON - 1;
    }

    if (edge_flow_y * (MAX_HORIZON - 1) > RES) {
      previous_frame_offset_y = RES / edge_flow_y + 1;
    } else {
      previous_frame_offset_y = MAX_HORIZON - 1;
    }
  }

  // the previous frame number relative to dynamic parameters
  uint8_t previous_frame_x = (current_frame_nr - previous_frame_offset_x + (MAX_HORIZON - 1)) %
                             (MAX_HORIZON - 1); // wrap index
  uint8_t previous_frame_y = (current_frame_nr - previous_frame_offset_y + (MAX_HORIZON - 1)) %
                             (MAX_HORIZON - 1); // wrap index

  // copy previous edge histogram based on previous frame number
  prev_edge_histogram_x = edge_hist[previous_frame_x].horizontal;
  prev_edge_histogram_y = edge_hist[previous_frame_y].vertical;

  //Calculate Edge Histogram
  calculate_edge_histogram(in, edge_histogram_x, image_width, image_height, 'x', 'l');
  calculate_edge_histogram(in, edge_histogram_y, image_width, image_height, 'y', 'l');
  calculate_edge_histogram(in, edge_histogram_right_x, image_width, image_height, 'x', 'r');

  //Calculate displacement
  //TODO add max distance and window size to function (is defined in function)
  calculate_displacement(edge_histogram_x, prev_edge_histogram_x, displacement->horizontal, image_width);
  calculate_displacement(edge_histogram_y, prev_edge_histogram_y, displacement->vertical, image_height);
  *height = abs(calculate_displacement_fullimage(edge_histogram_x, edge_histogram_right_x, image_width));

  //Fit a linear line
#ifdef RANSAC
  line_fit_RANSAC(displacement->horizontal, &edge_flow->horizontal_slope, &edge_flow->horizontal_trans, image_width, RES);
  line_fit_RANSAC(displacement->vertical, &edge_flow->horizontal_slope, &edge_flow->horizontal_trans, image_width, RES);
#else
  line_fit(displacement->horizontal, &edge_flow->horizontal_slope, &edge_flow->horizontal_trans, image_width, RES);
  line_fit(displacement->vertical, &edge_flow->vertical_slope, &edge_flow->vertical_trans, image_height, RES);
#endif

  // Correct Divergence slope and translation by the amount of frames skipped
  edge_flow->horizontal_slope /= previous_frame_offset_x;
  edge_flow->horizontal_trans /= previous_frame_offset_x;
  edge_flow->vertical_slope /= previous_frame_offset_y;
  edge_flow->vertical_trans /= previous_frame_offset_y;

  return previous_frame_offset_x;
}

//calculate_edge_histogram calculates the image gradient of the images and makes a edge feature histogram
void calculate_edge_histogram(uint8_t *in, int32_t *edge_histogram, uint16_t image_width, uint16_t image_height,
                              char direction, char side)
{
  int32_t  sobel_sum = 0;
  int32_t  Sobel[3] = { -1, 0, 1};

  uint16_t y = 0, x = 0;
  int8_t  c = 0;

  if (direction == 'x') {
    for (x = 0; x < image_width; x++) {
      edge_histogram[x] = 0;
      for (y = 0; y < image_height; y++) {
        sobel_sum = 0;

        for (c = -1; c <= 1; c++) {
          uint32_t idx = image_width * y * 2 + (x + c) * 2;

          if (side == 'l') {
            sobel_sum += Sobel[c + 1] * in[idx];
          } else { // default (side=='r')
            sobel_sum += Sobel[c + 1] * in[idx + 1];
          }
        }
        edge_histogram[x] += abs(sobel_sum);
      }
    }
  } else { // default (direction=='y')
    for (y = 0; y < image_height; y++) {
      edge_histogram[y] = 0;
      for (x = 0; x < image_width; x++) {
        sobel_sum = 0;

        for (c = -1; c <= 1; c++) {
          uint32_t idx = image_width * (y + c) * 2 + (x) * 2;

          if (side == 'l') {
            sobel_sum += Sobel[c + 1] * (int8_t)(in[idx]);
          } else { // default (side=='r')
            sobel_sum += Sobel[c + 1] * (int8_t)(in[idx + 1]);
          }
        }
        edge_histogram[y] += abs(sobel_sum);
      }
    }
  }
}

//Calculate_displacement calculates the displacement between two histograms
void calculate_displacement(int32_t *edge_histogram, int32_t *edge_histogram_prev, int32_t *displacement, uint32_t size)
{
  int8_t c = 0;
  int8_t r = 0;
  uint16_t x = 0;
  const uint8_t W = 10;   // local search window
  const uint8_t D = 10;   // half search disparity range
  uint32_t SAD_temp[20];    // size must be 2*W

  for (x = 0; x < size; x++) {
    if (x >= W + D && x <= size - W - D) {
      for (c = -D; c < D; c++) {
        SAD_temp[D + c] = 0;

        for (r = -W; r < W; r++) {
          SAD_temp[c + D] += abs(edge_histogram[x + r] - edge_histogram_prev[x + r + c]);
        }
      }

      displacement[x] = getMinimum(SAD_temp, 20) - W;
    } else {
      displacement[x] = 0;
    }
  }
}

//Calculate_displacement calculates the displacement between two histograms
int32_t calculate_displacement_fullimage(int32_t *edge_histogram, int32_t *edge_histogram_2, uint32_t size)
{
  int16_t r = 0;
  int8_t x = 0;
  const uint8_t D = 16;       // search disparity ranges
  uint32_t SAD_temp[32];    // size must be 2*W

  for (x = -D; x < D; x++) {
    SAD_temp[x + D] = 0;

    for (r = D; r < size - D; r++) {
      SAD_temp[D + x] += abs(edge_histogram[r] - edge_histogram_2[x + r]);
    }
  }

  return (getMinimum(SAD_temp, 32) - D);
}


//Line_fit fits a line using least squared to the histogram disparity map (displacement
void line_fit(int32_t *displacement, int32_t *Slope, int32_t *Yint, uint32_t image_width, uint16_t RES)
{
  uint16_t x;

  uint16_t count = 0;
  int32_t sumY = 0;
  int32_t sumX = 0;
  int32_t sumX2 = 0;
  int32_t sumXY = 0;
  int32_t xMean = 0;
  int32_t yMean = 0;

  *Slope = 0;
  *Yint = 0;

  for (x = 0; x < image_width; x++) {
    if (displacement[x] != 0) {
      sumX += x;
      sumX2 += x * x;
      sumY += displacement[x];
      sumXY += x * displacement[x];
      count++;
    }
  }

  if (count > 0) {
    /*sumX = ;
    xMean = image_width/2 - 1; //sumX/count;
    sumX2 = (image_width-1)*((image_width-1)+1)*(2*(image_width-1)+1)/6;  // http://www.trans4mind.com/personal_development/mathematics/series/sumNaturalSquares.htm
     */
    xMean = sumX / count;
    yMean = sumY / count;

    if ((sumX2 - sumX * xMean) != 0) {
      *Slope = RES * (sumXY - sumX * yMean) / (sumX2 - sumX * xMean);
    } else {
      *Slope = 10000;
    }
    *Yint = yMean * RES - *Slope * xMean;
  }
}


void line_fit_RANSAC(int32_t *displacement, int32_t *slope, int32_t *yInt, uint32_t size, uint32_t RES)
{
  //Fit a linear line with RANSAC (from Guido's code)
  const uint8_t ransac_iter = 20;
  int32_t it;
  uint32_t k;
  uint32_t ind1, ind2, tmp, entry;
  uint32_t total_error, best_ind;
  int32_t dx, dflow, predicted_flow;
  // flow = a * x + b
  int32_t a[ransac_iter];
  int32_t b[ransac_iter];
  uint32_t errors[ransac_iter];

  int32_t X[size];

  int32_t count_disp = 0;

  for (k = 0; k < size; k++) {
    X[k] = k;
    count_disp += displacement[k];
  }

  if (count_disp != 0) {
    for (it = 0; it < ransac_iter; it++) {
      ind1 = rand() % size;
      ind2 = rand() % size;

      while (ind1 == ind2) {
        ind2 = rand() % size;
      }

      if (X[ind1] > X[ind2]) {
        tmp = ind2;
        ind2 = ind1;
        ind1 = tmp;
      }
      /*while(displacement[ind1]==0)
        ind1 = rand() % size;
      while(displacement[ind2]==0)
        ind2 = rand() % size;*/

      dx = X[ind2] - X[ind1];
      dflow = displacement[ind2] - displacement[ind1];

      //Fit line with two points
      if (dx != 0) {
        a[it] = RES * dflow / dx;
      } else {
        a[it] = 100000;
      }

      b[it] = displacement[ind1] - (a[it] * X[ind1]);

      // evaluate fit:
      total_error = 0;
      for (entry = 0; entry < size; entry++) {
        predicted_flow = (a[it] * X[entry] + b[it]);
        total_error += abs(displacement[entry] - predicted_flow);
      }
      errors[it] = total_error;
    }
    // select best fit:
    best_ind = getMinimum2(errors, 20);

    *slope = a[best_ind];
    *yInt = b[best_ind];
  } else {
    *slope = 0;
    *yInt = 0;
  }
}

void totalKalmanFilter(struct coveriance_t *coveriance, struct edge_flow_t *prev_edge_flow,
                       struct edge_flow_t *edge_flow, uint32_t Q, uint32_t R, uint32_t RES)
{
  edge_flow->horizontal_trans = simpleKalmanFilter(&(coveriance->trans_x), prev_edge_flow->horizontal_trans,
                                edge_flow->horizontal_trans, Q, R, RES);
  edge_flow->vertical_trans = simpleKalmanFilter(&(coveriance->trans_y), prev_edge_flow->vertical_trans,
                              edge_flow->vertical_trans, Q, R, RES);
  edge_flow->horizontal_slope = simpleKalmanFilter(&(coveriance->slope_x), prev_edge_flow->horizontal_slope,
                                edge_flow->horizontal_slope, Q, R, RES);
  edge_flow->vertical_slope = simpleKalmanFilter(&(coveriance->slope_y), prev_edge_flow->vertical_slope,
                              edge_flow->vertical_slope, Q, R, RES);
}

int32_t simpleKalmanFilter(int32_t *cov, int32_t previous_est, int32_t current_meas, int32_t Q, int32_t R, int32_t RES)
{
  int32_t predict_cov = *cov + Q;
  int32_t K = RES * predict_cov / (*cov + R);

  *cov = ((RES - K) * predict_cov) / RES;

  return (previous_est + (K * (current_meas - previous_est)) / RES);
}


//This function is a visualization tool which visualizes the Edge filter in the one image and the histogram disparity with line fit in the second.
void visualize_divergence(uint8_t *in, int32_t *displacement, int32_t slope, int32_t yInt, uint32_t image_width,
                          uint32_t image_height)
{
  uint32_t y = 0;
  uint32_t x = 0;
  uint32_t idx = 0;

  uint32_t line_check1 = 0;
  uint32_t line_check2 = 0;

  for (y = 0; y < image_height; y++) {
    ////line_check1=(uint32_t)(Slope*(float)x+(Yint)+(float)image_height/2);
    //line_check2=(uint32_t)(displacement[x]+image_height/2);
    for (x = 0; x < image_width; x++) {
      idx =  image_width * y * 2 + (x) * 2;

      /*if(y==line_check1)
        out[idx]=255;
      else if(y==line_check2)
        out[idx]=100;
      else
        out[idx]=0;*/

      in[idx] = 100;
      in[idx + 1] = 0; //in[idx+1];
    }
  }
}


int32_t getMinimum2(uint32_t *flow_error, uint32_t max_ind)
{
  uint32_t i;
  uint32_t min_ind = 0;
  uint32_t min_err = flow_error[0];
  for (i = 1; i < max_ind; i++) {
    if (flow_error[i] < min_err && !isnan(flow_error[i])) {
      min_ind = i;
      min_err = flow_error[i];
    }
  }
  return min_ind;
}
