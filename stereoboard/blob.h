#ifndef BLOB_H_
#define BLOB_H_

#include <sys/time.h>
#include "utils.h"
#include "image.h"

struct image_filter_t {
  uint8_t y_min;
  uint8_t y_max;
  uint8_t u_min;
  uint8_t u_max;
  uint8_t v_min;
  uint8_t v_max;
};

struct image_label_t {
  uint16_t id;
  uint8_t filter;

  uint16_t pixel_cnt;
  uint16_t x_min;
  uint16_t y_min;

  struct point_t contour[512];
  uint16_t contour_cnt;

  uint16_t corners[4];
};

#endif
