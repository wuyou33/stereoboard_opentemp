#ifndef _MY_IMAGE_HEADER_
#define _MY_IMAGE_HEADER_

#include <inttypes.h>
#include "main_parameters.h"
struct img_struct {
  int seq;
  double timestamp;
  unsigned char *buf;
  int w;
  int h;
};

void setLineNumbers(uint8_t **givenImage, uint16_t width, uint16_t height);
void setLineNumbersImage(uint8_t *givenImage, uint16_t width, uint16_t height);

#endif
