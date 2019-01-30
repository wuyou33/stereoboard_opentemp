/*

// Created on: Dec 13, 2018
// Author: shushuai

// log_sd.c <-- ff.h <-- ff.c(fatfs_lib) <-- diskio.h(fatfs_lib) <-- ff_diskio.c(3) <-- spi_sd.h(2) <-- spi_sd.c(1)
// where files 1,2 and 3 are borrowed from ST examples (https://github.com/PaxInstruments/STM32CubeF4)
// But link functions for SD Card peripheral are added to file 1. These files are from following links.
// (1) STM32CubeF4-master/Drivers/BSP/Adafruit_Shield/stm32_adafruit_sd.c
// (2) STM32CubeF4-master/Drivers/BSP/Adafruit_Shield/stm32_adafruit_sd.h
// (3) STM32CubeF4-master/Middlewares/Third_Party/FatFs/src/drivers/sd_diskio.c
 
*/

#ifndef LOG_SD_H_
#define LOG_SD_H_

#include <stdint.h>
#include "image.h" // for logging image

void log_txt(void); // save txt file
void log_image_jpeg(struct image_t *img); // save compressed image
void log_image_bmp(struct image_t *img);  // save original image

#endif /* LOG_SD_H_ */
