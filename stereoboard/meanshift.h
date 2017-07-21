#ifndef _MEANSHIFT_H_
#define _MEANSHIFT_H_

#include "data_types.h"
#include "image.h"

void run_meanshift(struct image_t *disparity_image);

//void meanshiftUpdate(uint8_t image[],int width, int height, int* trackPosX, int* trackPosY);

float meanshiftUpdate(struct image_t *disparity_image, struct rectangle_i *searchrectangle);

#endif  // _MEANSHIFT_H_
