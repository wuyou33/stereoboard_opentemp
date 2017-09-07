/*
 * geometry.h
 *
 *  Created on: 6 Sep 2017
 *      Author: kirk
 */

#ifndef STEREOBOARD_MATH_GEOMETRY_H_
#define STEREOBOARD_MATH_GEOMETRY_H_

#include "image.h"

// Returns true if the point p lies inside the polygon[] with n vertices
bool isInside(struct point_t polygon[], int n, struct point_t p);

#endif /* STEREOBOARD_MATH_GEOMETRY_H_ */
