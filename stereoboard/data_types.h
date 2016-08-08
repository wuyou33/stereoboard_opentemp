
#ifndef _DATA_TYPES_H_
#define _DATA_TYPES_H_

#include <inttypes.h>

struct point_i {
  uint16_t x;
  uint16_t y;
};

struct point_f {
  float x;
  float y;
};

struct rectangle_i {
  uint16_t x;
  uint16_t y;
  uint16_t w;
  uint16_t h;
};

struct rectangle_f {
  float x;
  float y;
  uint16_t w;
  uint16_t h;
};

struct image_i {
  uint8_t *image;
  uint16_t w;
  uint16_t h;
};

#endif	// _DATA_TYPES_H_
