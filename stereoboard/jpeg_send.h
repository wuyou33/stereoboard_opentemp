/*
 * jpeg_send.h
 *
 *  Created on: May 20, 2015
 *      Author: mavlab
 */

#ifndef JPEG_SEND_H_
#define JPEG_SEND_H_

void SendJpeg(uint8_t *b, uint32_t size)
{
  uint8_t *p = (uint8_t *) & size;

  uint8_t code[10];
  code[0] = '#';
  code[1] = '#';
  code[2] = 'I';
  code[3] = 'M';
  code[4] = 'J';
  code[5] = '2';
  code[6] = p[0];
  code[7] = p[1];
  code[8] = p[2];
  code[9] = 0x00;

  while (UsartTx(code, 10) == 0)
    ;

  int j = 0;
  for (j = 0; j < size; j++) {
    while (UsartTx(b + j, 1) == 0)
      ;
  }
}




#endif /* JPEG_SEND_H_ */
