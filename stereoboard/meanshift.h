#ifndef _MEANSHIFT_H_
#define _MEANSHIFT_H_

#include "data_types.h"

//void meanshiftUpdate(uint8_t image[],int width, int height, int* trackPosX, int* trackPosY);

void meanshiftUpdate(uint8_t_image disparity_image, int_rectangle *searchrectangle, float *distanceToObject);

#endif
