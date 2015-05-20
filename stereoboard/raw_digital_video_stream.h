/*
 * raw_digital_video_stream.h
 *
 *  Created on: May 20, 2015
 *      Author: mavlab
 */

#ifndef RAW_DIGITAL_VIDEO_STREAM_H_
#define RAW_DIGITAL_VIDEO_STREAM_H_

#include "usart.h"

void Send(uint8_t *b, uint16_t width, uint16_t height)
{
  // New frame code
  uint8_t code[4];
  code[0] = 0xff;
  code[1] = 0x00;
  code[2] = 0x00;
  code[3] = 0xAF;
  while (UsartTx(code, 4) == 0)
    ;

  uint8_t msg = 0x07;
  while (UsartTx(&msg, 1) == 0)
    ;

#ifdef LARGE_IMAGE

  if (offset_crop == 0) {
    code[3] = 0xAC;
    while (UsartTx(code, 4) == 0)
      ;
  }

#endif

#ifdef SHOW_HMC
  {
    draw_mag_as_line(0);
    draw_mag_as_line(1);
    draw_mag_as_line(2);
  }
#endif

  int j = 0;
  for (j = 0; j < height; j++) {
    // Beginning of Line
    code[3] = 0x80;
    while (UsartTx(code, 4) == 0)
      ;

    // Line data
    while (UsartTx(b + width * j * 2, width * 2 + 1) == 0)
      ;

    // End of Line
    code[3] = 0xDA;
    while (UsartTx(code, 4) == 0)
      ;
  }

  // End of Frame
  code[3] = 0xAB;
  while (UsartTx(code, 4) == 0)
    ;
}




void SendDisparityMap(uint8_t *b)
{
  uint8_t code[4];
  code[0] = 0xff;
  code[1] = 0x00;
  code[2] = 0x00;

  uint16_t height = IMAGE_HEIGHT;
  int16_t width = IMAGE_WIDTH;

  int j = 0;
  for (j = 0; j < height; j++) {
    code[3] = 0x80;
    while (UsartTx(code, 4) == 0)
      ;
    while (UsartTx(b + width * j, width) == 0)
      ;

    code[3] = 0xDA;
    while (UsartTx(code, 4) == 0)
      ;
  }

  code[3] = 0xAB;
  while (UsartTx(code, 4) == 0)
    ;
}





#endif /* RAW_DIGITAL_VIDEO_STREAM_H_ */
