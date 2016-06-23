/*
 * main_parameters.h
 *
 *  Created on: Oct-2014
 *  Author: mavlab
 */

#include "color.h"
#include "main_parameters.h"

uint32_t colorfilt_uyvy(struct img_struct *input, struct img_struct *output, uint8_t y_m, uint8_t y_M, uint8_t u_m,
                        uint8_t u_M, uint8_t v_m, uint8_t v_M, uint32_t *results)
{
  int x, y;
  int cnt = 0;
  results[0] = 0;
  results[1] = 0;

  uint8_t *source = input->buf;
  uint8_t *dest = output->buf;


  int linesubsampling = 1; // if 1, disabled

  for (y = 0; y < output->h - (linesubsampling - 1); y += linesubsampling) {
    for (x = 0; x < output->w << 1 ; x += 4) { // 2 bytes per pixel, 4 bytes per uyvy "sample"

      // Color Check:
      if ((source[1] >= y_m)
          && (source[1] <= y_M)
          && (source[0] >= u_m)
          && (source[0] <= u_M)
          && (source[2] >= v_m)
          && (source[2] <= v_M)
         ) {
        cnt ++; // total count not really used anymore

        //measure counts for left half and right half seperatedly, used to detect the roof color
        if (y < 25) { //hard coded = not so nice. Also, not very efficient. But works...
          if (x < 127) {
            results[0]++;
          } else {
            results[1]++;
          }
        }


#if (SEND_FILTER)
        //make it orange
        dest[0] = 64;         // U
        dest[1] = source[1];  // Y
        dest[2] = 255;        // V
        dest[3] = source[3];  // Y
      } else {
        //make it gray
        char u = source[0] - 127;
        u >> 2;
        dest[0] = 127;        // U
        dest[1] = source[1];  // Y
        u = source[2] - 127;
        u >> 2;
        dest[2] = 127;        // V
        dest[3] = source[3];  // Y
      }
#else
      } //if color check
#endif

      dest += 4;
      source += 4;
    }
    if (linesubsampling > 1) {
      dest += ((output->w) << 1) * (linesubsampling - 1);
      source += ((output->w) << 1) * (linesubsampling - 1);
    }
  }
  return cnt;
}

