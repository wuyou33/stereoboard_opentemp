// Optic flow code

#include "optic_flow.h"
#include "usart.h"
#include <stdio.h>
#include <stdlib.h>

/* optic flow:
 * takes previous and current image and calculates displacement and divergence
 *
 * parameters:
 * image_width
 * image_height
 * max_flow (10 is a reasonable value)
 * */

#define IM_WIDTH 128
#define HALF_WIDTH 64
#define RANSAC_ITERATIONS 20
#define RESOLUTION 100

void optic_flow_horizontal(uint8_t *prev_im, uint8_t *curr_im, uint32_t image_width, uint32_t image_height,
                           uint32_t max_flow, uint8_t min_y, uint8_t max_y, int *divergence, int *displacement)
{
  /*
  * 1) Create histograms + determine average (slow, we should remember the last one)
  * 2) Histogram matching
  * 3) Fit linear model
  */

  uint8_t histogram1[IM_WIDTH];
  uint8_t histogram2[IM_WIDTH];
  uint32_t flow_error[IM_WIDTH]; // maximal max flow is half the image
  uint8_t avg_h1;
  uint8_t avg_h2;
  uint32_t x, f, flow_ind;

  // 1) Create histograms + determine average (slow, we should remember the last one)
  avg_h1 = createHistogramSobel(histogram1, prev_im, image_width, image_height, min_y, max_y);
  avg_h2 = createHistogramSobel(histogram2, curr_im, image_width, image_height, min_y, max_y);

  // 2) Histogram matching
  // Match all histogram entries that are larger than average

  uint32_t X[IM_WIDTH];
  int FLOW[IM_WIDTH];
  uint32_t n_entries = 0;

  for (x = max_flow; x < image_width - max_flow; x++) {
    // only start matching if it is a peak:
    if (histogram1[x] > avg_h1) {
      // determine errors for different flows (now only integer)
      for (f = -max_flow; f < max_flow; f++) {
        flow_error[f + max_flow] = abs(histogram1[x] - histogram2[x + f]);
      }

      // search minimum:
      flow_ind = getMinimum(flow_error, 2 * max_flow);

      // store the point plus its flow:
      X[n_entries] = x;
      FLOW[n_entries] = (int) flow_ind - max_flow;
      n_entries++;
    }
  }

  // 3) Fit linear model
  fitLinearModel(X, FLOW, n_entries, divergence, displacement);


}

void fitLinearModel(uint32_t *X, int *FLOW, uint32_t n_entries, int *divergence, int *displacement)
{
  // RANSAC fitting:
  // 2 points per fit
  uint8_t it;
  uint32_t ind1, ind2, tmp, entry, total_error, best_ind;
  int dx, dflow, predicted_flow;
  // flow = a * x + b
  int a[RANSAC_ITERATIONS];
  int b[RANSAC_ITERATIONS];
  uint32_t errors[RANSAC_ITERATIONS];

  for (it = 0; it < RANSAC_ITERATIONS; it++) {
    // make fit:
    ind1 = rand() % n_entries;
    ind2 = rand() % n_entries;

    while (ind1 == ind2) {
      ind2 = rand() % n_entries;
    }

    if (X[ind1] > X[ind2]) {
      tmp = ind2;
      ind2 = ind1;
      ind1 = tmp;
    }

    dx = X[ind2] - X[ind1];
    dflow = FLOW[ind2] - FLOW[ind1];
    // a and b both in resolution RESOLUTION:
    a[it] = dflow * RESOLUTION / dx;
    b[it] = FLOW[ind1] * RESOLUTION - (a[it] * (X[ind1] - HALF_WIDTH));

    // evaluate fit:
    for (entry = 0; entry < n_entries; entry++) {
      predicted_flow = a[it] * (X[entry] - HALF_WIDTH) + b[it];
      total_error += (uint32_t) abs(RESOLUTION * FLOW[entry] - predicted_flow);
    }
    errors[it] = total_error;
  }

  // select best fit:
  best_ind = getMinimum(errors, n_entries);
  (*divergence) = a[best_ind] - 100;
  (*displacement) = b[best_ind];
}

uint32_t getMinimum(uint32_t *flow_error, uint32_t max_ind)
{
  uint32_t i;
  uint32_t min_ind = 0;
  uint32_t min_err = flow_error[0];
  for (i = 1; i < max_ind; i++) {
    if (flow_error[i] < min_err) {
      min_ind = i;
      min_err = flow_error[i];
    }
  }
  return min_ind;
}

uint8_t createHistogramSobel(uint8_t *histogram, uint8_t *im, uint32_t image_width, uint32_t image_height,
                             uint8_t min_y, uint8_t max_y)
{
  uint8_t Sobel[3][3] = { { -1, 0, 1}, { -2, 0, 2}, { -1, 0, 1} };
  uint32_t sobel_threshold = 15;
  uint32_t x, y, r, c;
  int32_t sobel;
  uint32_t avg_hist = 0;

  // entries will not be filled:
  histogram[0] = 0; histogram[image_width - 1] = 0;

  // make histogram:
  for (x = 1; x < image_width - 1; x++) {
    histogram[x] = 0;

    for (y = min_y + 1; y < max_y - 1; y++) {
      // Convolution:
      sobel = 0;
      for (r = -1; r <= 1; r++) {
        for (c = -1; c <= 1; c++) {
          sobel += Sobel[r + 1][c + 1] * (int) im[(x + r) * 2 + (y + c) * image_width * 2];
        }
      }
      sobel = abs(sobel);
      if (sobel > sobel_threshold) {
        histogram[x] += sobel;
      }
    }

    avg_hist += histogram[x];
  }

  avg_hist /= (image_width - 2);

  return (uint8_t) avg_hist;
}
