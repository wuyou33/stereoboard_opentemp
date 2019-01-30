/*

// Created on: Dec 13, 2018
// Author: shushuai

// log_sd.c <-- ff.h(fatfs_lib) <-- ff.c(fatfs_lib) <-- diskio.h(fatfs_lib) <-- ff_diskio.c(3) <-- spi_sd.h(2) <-- spi_sd.c(1)
// where files 1,2 and 3 are borrowed from ST examples (https://github.com/PaxInstruments/STM32CubeF4)
// But link functions for SD Card peripheral are added to file 1. These files are from following links.
// (1) STM32CubeF4-master/Drivers/BSP/Adafruit_Shield/stm32_adafruit_sd.c
// (2) STM32CubeF4-master/Drivers/BSP/Adafruit_Shield/stm32_adafruit_sd.h
// (3) STM32CubeF4-master/Middlewares/Third_Party/FatFs/src/drivers/sd_diskio.c
 
*/

#include <string.h>
#include <stdio.h>
#include "log_sd.h"
#include "ff.h"
#include "encoding/jpeg.h"

void log_txt(void)
{
  FATFS FatFs;
  FIL MyFile;
  unsigned int numWrite;
  if (f_mount(&FatFs, "", 1) == FR_OK)
  {
    if(f_open(&MyFile,"ha.txt", FA_CREATE_ALWAYS | FA_WRITE) == FR_OK)
    {
      f_write(&MyFile, "Hello world!", 12, &numWrite);
      f_close(&MyFile);
      while(1);
    }
  }
}

void log_image_jpeg(struct image_t *img)
{
  FILINFO fno;
  char save_name[10];
  static int file_num=0;

  FATFS FatFs;
  FIL MyFile;
  struct image_t img_jpeg;
  unsigned int numWrite;
  img_jpeg.type = IMAGE_JPEG;
  img_jpeg.w = img->w;
  img_jpeg.h = img->h;
  img_jpeg.buf_size = 2 * img_jpeg.w * img_jpeg.h;
  uint8_t temp_buffer[img_jpeg.buf_size];
  img_jpeg.buf = temp_buffer;
  jpeg_encode_image(img, &img_jpeg, 99, true);

  if ( (f_mount(&FatFs, "", 1) == FR_OK) )
  {
    sprintf(save_name, "i%05d.jpg", file_num); // check if file is existed
    while(f_stat(save_name, &fno) != FR_NO_FILE)
    {
      file_num++;
      sprintf(save_name, "i%05d.jpg", file_num);
    }
    if(f_open(&MyFile, save_name, FA_CREATE_ALWAYS | FA_WRITE) == FR_OK)
    {
      f_write(&MyFile, img_jpeg.buf, img_jpeg.buf_size, &numWrite);
      f_close(&MyFile);
    }
  }
}

/* create bmp file with a buffer may be fast
  f_write(&MyFile, bmpfileheader, 14, &numWrite);
  f_write(&MyFile, bmpinfoheader, 40, &numWrite);
  for(int i=0; i<h; i++)
  {
    f_write(&MyFile, img_rgb+(w*(h-i-1)*3), 3*w, &numWrite);
    f_write(&MyFile, bmppad, (4-(w*3)%4)%4, &numWrite);
  }
*/
// following function stores bmp file with lines of pixels
void log_image_bmp(struct image_t *img)
{
  unsigned char bmpfileheader[14] = {'B','M', 0,0,0,0, 0,0, 0,0, 54,0,0,0};
  unsigned char bmpinfoheader[40] = {40,0,0,0, 0,0,0,0, 0,0,0,0, 1,0, 24,0};
  unsigned char bmppad[3] = {0,0,0};
  int w = img->w;
  int h = img->h;
  int filesize = 54 + 3*w*h; // bmp file
  unsigned char bmp_pixel[3];
  uint8_t *yuv400_ptr;

  FATFS FatFs;
  FIL MyFile;
  unsigned int numWrite;
  int i,j;

  FILINFO fno;
  char save_name[10];
  static int file_num=0;

  bmpfileheader[ 2] = (unsigned char)(filesize    );
  bmpfileheader[ 3] = (unsigned char)(filesize>> 8);
  bmpfileheader[ 4] = (unsigned char)(filesize>>16);
  bmpfileheader[ 5] = (unsigned char)(filesize>>24);
  bmpinfoheader[ 4] = (unsigned char)(       w    );
  bmpinfoheader[ 5] = (unsigned char)(       w>> 8);
  bmpinfoheader[ 6] = (unsigned char)(       w>>16);
  bmpinfoheader[ 7] = (unsigned char)(       w>>24);
  bmpinfoheader[ 8] = (unsigned char)(       h    );
  bmpinfoheader[ 9] = (unsigned char)(       h>> 8);
  bmpinfoheader[10] = (unsigned char)(       h>>16);
  bmpinfoheader[11] = (unsigned char)(       h>>24);

  if ( (f_mount(&FatFs, "", 1) == FR_OK) )
  {
    sprintf(save_name, "i%05d.bmp", file_num); // check if file is existed
    while(f_stat(save_name, &fno) != FR_NO_FILE)
    {
      file_num++;
      sprintf(save_name, "i%05d.bmp", file_num);
    }
    if(f_open(&MyFile, save_name, FA_CREATE_ALWAYS | FA_WRITE) == FR_OK)
    {
      f_write(&MyFile, bmpfileheader, 14, &numWrite);
      f_write(&MyFile, bmpinfoheader, 40, &numWrite);
      for(i=0; i<h; i++)
      {
        yuv400_ptr = img->buf + w*(h-i-1);
        for(j=0; j<w; j++)
        {
          bmp_pixel[0] = *yuv400_ptr;
          bmp_pixel[1] = *yuv400_ptr;
          bmp_pixel[2] = *yuv400_ptr;
          f_write(&MyFile, bmp_pixel, 3, &numWrite); // write pixel to bmp file
          yuv400_ptr++;
        }
        f_write(&MyFile, bmppad, (4-(w*3)%4)%4, &numWrite); // finish of a line
      }
      f_close(&MyFile);
    }
  }
}