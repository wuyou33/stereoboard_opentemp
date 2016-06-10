
#ifndef _DATA_TYPES_H_
#define _DATA_TYPES_H_
#include <inttypes.h>
typedef struct{
  int x;
  int y;
}int_point;

typedef struct{
  float x;
  float y;
}float_point;

typedef struct{
  uint8_t x;
  uint8_t y;
}uint8_t_point;

typedef struct{
  int x;
  int y;
  int width;
  int height;
}int_rectangle;

typedef struct{
  float x;
  float y;
  float width;
  float height;
}float_rectangle;


typedef struct {
  uint8_t* image;
  int imageWidth;
  int imageHeight;
} uint8_t_image;
#endif
