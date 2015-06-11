/**
 *  \file ov7670.h OV7670 Driver in I2C
 *
 *  Copyright (c) See main file for copyright and liability
 */

#ifndef __MY_OV7670_HEADER__
#define __MY_OV7670_HEADER__

#include <stdint.h>


// Functions
void camera_ov7670_i2c_init(void);
void camera_ov7670_read(void);
void camera_ov7670_config(void);


#define OV7670_AUTOMATIC_GAIN_CONTROL   0x00
#define OV7670_BLUE_GAIN                0x01
#define OV7679_RED_GAIN                 0x02
#define OV7670_VERTICAL_FRAME_CONTROL   0x03

#define OV7670_COMMON_CTRL1             0x04
#define OV7670_COMMON_CTRL2             0x09    // Sleep, Output drive
#define OV7670_COMMON_CTRL3             0x0C    // Tri-state on sleep
#define OV7670_COMMON_CTRL7             0x12    // IMAGE SIZE
#define OV7670_COMMON_CTRL8             0x13    // Auto gain/exposure
#define OV7670_COMMON_CTRL10            0x15    // PCLK
#define OV7670_COMMON_CTRL11            0x3B    // Night Mode
#define OV7670_COMMON_CTRL12            0x3C    // HREF
#define OV7670_COMMON_CTRL13            0x3D    // UV-swap
#define OV7670_COMMON_CTRL14            0x3E    // PCLK scaling
#define OV7670_COMMON_CTRL15            0x40    // RGB565/555
#define OV7670_DBLV                     0x6B    // PLL control
#define OV7670_TESTPATTERN              0x70    // Generate test pattern


#define OV7670_EDGE                     0x3F    // Edge enhancement
#define OV7670_BRIGHTNESS               0x55    // BRightness
#define OV7670_CONTRAST                 0x56    // Contrast
#define OV7670_CONTRAST_CENTER          0x57    // Contrast no change value

#define OV7670_PROD_ID                  0x0A
#define OV7670_PROD_VER                 0x0B
#define OV7670_PROD_MAN_ID_L            0x1C    // should read 7F
#define OV7670_PROD_MAN_ID_H            0x1D    // should read A2

#define OV7670_EXPOSURE_VALUE_H         0x07
#define OV7670_EXPOSURE_VALUE           0x10

#define OV7670_VFLIP                    0x1E   // Veritcal Flip

// Camera Settings (z means zoom)
#define IMG_OV_SIZE_VGA_YUV             (0<<0)
#define IMG_OV_SIZE_VGA_RGB             (4<<0)
#define IMG_OV_SIZE_VGA_BAYRAW          (1<<0)
#define IMG_OV_SIZE_VGA_BAYRAWPROC      (5<<0)
#define IMG_OV_SIZE_CIF                 (1<<5)
#define IMG_OV_SIZE_QVGA                (1<<4)
#define IMG_OV_SIZE_QCIF                (1<<3)

//#define IMG_FORMAT_YUV422 (0<<1)
//#define IMG_FORMAT_RGB565 (1<<1)

//#define IMG_COLOR_COLOR   (0<<0)
//#define IMG_COLOR_BW    (1<<0)

typedef struct {
  uint8_t framesize;
  uint8_t framerate;
  uint8_t sw_codes;
} ov7670_settings;


#endif
