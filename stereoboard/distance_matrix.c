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

void calculateDistanceMatrix(uint8_t *disparity_image,
                             int *matrixBuffer,
                             uint8_t blackBorderSize, uint8_t pixelsPerLine, uint8_t widthPerBin,
                             uint8_t heightPerBin, uint8_t *toSendBuffer, uint32_t disparity_range)
{

  uint8_t y;
  uint8_t valueInImageBuffer = 0;
  uint16_t positionInImageBuffer = 0;
  uint8_t positionInMatrix = 0;
  uint8_t x;
  //uint8_t z;

  uint16_t disparityHistogram[MATRIX_WIDTH_BINS * MATRIX_HEIGHT_BINS][disparity_range * RESOLUTION_FACTOR];
  for (x = 0; x < MATRIX_WIDTH_BINS * MATRIX_HEIGHT_BINS; x++) {
    for (y = 0; y < disparity_range * RESOLUTION_FACTOR; y++) {
      disparityHistogram[x][y] = 0;
    }
  }

  // Fill the disparity histogram:
  for (x = 0; x < MATRIX_WIDTH_BINS; x++) {
    for (y = 0; y < MATRIX_HEIGHT_BINS; y++) {
      int line;
      for (line = 0; line < heightPerBin; line++) {
        int bufferIndex = 0;
        for (bufferIndex = 0; bufferIndex < widthPerBin; bufferIndex++) {
          positionInImageBuffer = pixelsPerLine * (y * heightPerBin) + line * pixelsPerLine + widthPerBin * x + blackBorderSize +
                                  bufferIndex;
          valueInImageBuffer = disparity_image[positionInImageBuffer];

          positionInMatrix = y * MATRIX_WIDTH_BINS + x;

          disparityHistogram[positionInMatrix][valueInImageBuffer]++;
        }
      }
    }
  }


  // go over the cells in the matrix and determine per cell what the n-th disparity value is
  // n = COUNTER_THRESHOLD
  int COUNTER_THRESHOLD = 10;
  int bufferIndex;
  int anyOn = 0; // whether any cell in the matrix has a "close" object
  int disp;

  // go over all matrix cells
  for (bufferIndex = 0; bufferIndex < MATRIX_WIDTH_BINS * MATRIX_HEIGHT_BINS; bufferIndex++) {
    int n_disparities = 0;
    // go over all disparities
    for (disp = disparity_range * RESOLUTION_FACTOR - 1; disp >= 0; disp--) {
      n_disparities += disparityHistogram[bufferIndex][disp];
      // after n_disparities, we consider that the measurements are no longer noise
      if (n_disparities > COUNTER_THRESHOLD) {
        toSendBuffer[bufferIndex] = disp;
        if (disp > CLOSE_BOUNDARY) {
          anyOn = 1;
        }
        break;
      }
    }
  }
  if (anyOn == 1) {
    led_set();
  } else {
    led_clear();
  }
}
