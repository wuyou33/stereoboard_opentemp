/*
 * filter_color.c
 *
 *  Created on: Sep 16, 2013
 *      Author: mavlab
 */

#include "filter_color.h"

// if defined, the filter for red color only checks two bounds, max_U and min_V
#define EFFICIENT

uint16_t filter_red_color(uint8_t *in, uint8_t *out, uint32_t image_width, uint32_t image_height, uint8_t min_U,
                          uint8_t max_U, uint8_t min_V, uint8_t max_V)
{
  uint16_t x, y;
  uint16_t n_channels;
  uint16_t count = 0;
  // color is a byte with both u and v:
  uint8_t ill, color, u, v;
  n_channels = 2;
#ifdef EFFICIENT
  // make a binary image out
  for (x = 0; x < image_width; x += 2) {
    for (y = 0; y < image_height; y++) {
      //ill = in[n_channels * (y*image_width+x) + 1];
      u = in[n_channels * (y * image_width + x)];
      v = in[n_channels * (y * image_width + x) + 2];
      out[y * image_width + x / 2] = 0;
      if (u <= max_U) {
        out[y * image_width + x / 2] = (v >= min_V);
      }
      //out[y*image_width+x/2] = u;

      //out[y*image_width+x] = 16 * ((u <= max_U) && (v >= min_V));
      if (out[y * image_width + x / 2]) { count++; }
    }
  }
#else
  // make a binary image out
  for (x = 0; x < image_width; x += 2) {
    for (y = 0; y < image_height; y++) {
      u = in[n_channels * (y * image_width + x)];
      v = in[n_channels * (y * image_width + x) + 2];
      //out[y*image_width+x/2] = 10 * ((u <= max_U) && (v >= min_V));
      //if(out[y*image_width+x/2]) count++;
      out[y * image_width + x / 2] = (u <= max_U) && (u >= min_U);
      out[y * image_width + x / 2] = out[y * image_width + x] && (v <= max_V) && (v >= min_V);
      if (out[y * image_width + x / 2]) { count++; }
    }
  }
#endif
  return count;
}


