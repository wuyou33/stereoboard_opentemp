/*
 * raw_digital_video_stream.h
 *
 *  Created on: May 20, 2015
 *      Author: mavlab
 */

#ifndef RAW_DIGITAL_VIDEO_STREAM_H_
#define RAW_DIGITAL_VIDEO_STREAM_H_

#include "usart.h"

/** ITU-R BT.656 video transmission standard
 *
 * The embedded timing reference codes allow the video port to synchronize to the data stream using only
 * the parallel bus and the clock line. Each code starts with ‘0xFF 0x00 0x00’, and the final byte contains the
 * actual command. The command is contained within three bits of the final byte; the other bits of the final
 * byte are used for error checking. The three command bits are F for Field, V for Vertical Sync, and H for
 * Horizontal Sync. The final byte has the form ‘0b1FVHP3P2P1P0’. The four P bits are protection bits which
 * act like a checksum for the F, V, and H bits. The F bit determines which field is being transmitted for
 * interlaced video, field 1 (F = 0) or field 2 (F = 1). The V bit is 1 when blanking data is coming next. The H
 * bit determines if the code is a start of active video code (H = 0) or an end of active video code (H = 1).
 * The F, V, and H bits are used to form three basic commands which apply to either field. The commands
 * are listed below. Figure 3 puts these elements together to show how the timing codes work together to
 * transmit an interlaced video frame.
 * • Start of active video (SAV) indicates the beginning of a line of video. In this case, V = 0, as you are no
 *   longer in vertical blanking; H = 0 for SAV.
 * • End of Active Video (EAV) indicates the end of a line of video. V is also 0 in this case, as you are not
 *   yet entering vertical blanking. H = 1 for EAV.
 * • Last End of Active Video indicates the end of a field. Here, V = 1, as you are returning to vertical
 *   blanking. H = 1 to indicate EAV.
 */

#define BT656_SAV   0x80
#define BT656_EAV   0xDA

void Send(uint8_t *b, uint16_t width, uint16_t height)
{
  // New frame code: Vertical blanking = ON
  uint8_t code[4];
  code[0] = 0xff;
  code[1] = 0x00;
  code[2] = 0x00;
  code[3] = 0xAF;
  while (UsartTx(code, 4) == 0)
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
    // SAV: Beginning of Line
    code[3] = 0x80;
    while (UsartTx(code, 4) == 0)
      ;

    // Line data
    while (UsartTx(b + j * width * 2, width * 2 + 1) == 0)
      ;

    // EAV: End of Line
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

void SendMatrix(uint8_t* b) {
	uint8_t code[4];
	code[0] = 0xff;
	code[1] = 0x00;
	code[2] = 0x00;

	uint16_t height = 4;
	int16_t width = 4;

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
