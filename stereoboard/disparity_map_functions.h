
#ifndef DISPARITY_MAP_FUNCTIONS_H_
#define DISPARITY_MAP_FUNCTIONS_H_
#include "inttypes.h"
typedef enum {FOLLOW_ME_HISTOGRAM,AVOID_ME_HISTOGRAM} horizontal_histogram_type;

void histogram_x_direction(uint8_t *disparity_image, uint8_t *histogramBuffer, horizontal_histogram_type hist_type,uint8_t blackBorderSize, uint8_t pixelsPerLine, uint8_t heightPerLine);
void histogram_z_direction(uint8_t *disparity_image, uint8_t *histogramBuffer, uint8_t blackBorderSize, uint8_t pixelsPerLine, uint8_t heightPerLine);

#endif /* DISPARITY_MAP_FUNCTIONS_H_ */
