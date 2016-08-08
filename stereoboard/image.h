#ifndef _MY_IMAGE_HEADER_
#define _MY_IMAGE_HEADER_

#include <inttypes.h>
#include "main_parameters.h"

struct img_struct {
  uint16_t seq;
  uint32_t timestamp;
  uint8_t *buf;
  uint8_t w;
  uint8_t h;
};

void setLineNumbers(uint8_t **givenImage, uint16_t width, uint16_t height);
void setLineNumbersImage(uint8_t *givenImage, uint16_t width, uint16_t height);

#endif
