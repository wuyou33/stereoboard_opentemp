/**
 *  \file tcm8230.h TCM8230 Driver in I2C
 *
 *  Copyright (c) See main file for copyright and liability
 */

#ifndef __MY_TCM8230_HEADER__
#define __MY_TCM8230_HEADER__

#include <stdint.h>

// Functions
void camera_tcm8230_i2c_init(void);
void camera_tcm8230_read(void);
void camera_tcm8230_config(void);

// Camera Settings (z means zoom)
#define IMG_SIZE_VGA    (0<<2)
#define IMG_SIZE_QVGA   (1<<2)
#define IMG_SIZE_QVGAz    (2<<2)
#define IMG_SIZE_QQVGA    (3<<2)
#define IMG_SIZE_QQVGAz   (4<<2)
#define IMG_SIZE_CIF    (5<<2)
#define IMG_SIZE_QCIF   (6<<2)
#define IMG_SIZE_QCIFz    (7<<2)
#define IMG_SIZE_subQCIF  (8<<2)
#define IMG_SIZE_subQCIFz (9<<2)

#define IMG_FORMAT_YUV422 (0<<1)
#define IMG_FORMAT_RGB565 (1<<1)

#define IMG_COLOR_COLOR   (0<<0)
#define IMG_COLOR_BW    (1<<0)

typedef struct {
  uint8_t framesize;
  uint8_t framerate;
  uint8_t sw_codes;
} tcm8230_settings;


#endif
