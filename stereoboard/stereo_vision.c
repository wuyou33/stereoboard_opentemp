// Stereo vision code

#include "stereo_vision.h"

/* stereo_vision:
 * takes input image in, which contains both the left and the right image
 * generates disparity image out
 *
 * parameters:
 * image_width
 * image_height
 * disparity_range (20 is a reasonable value)
 * thr1: threshold for 1st check (4)
 * thr2: threshold for 2nd check (5)
 * */

void stereo_vision_Kirk(uint8_t *in, q7_t *out, uint32_t image_width, uint32_t image_height, uint32_t disparity_range,
                        uint8_t thr1, uint8_t thr2, uint8_t min_y, uint8_t max_y)
{
  uint32_t image_width_bytes = image_width * 2;
  uint32_t disparity_max = disparity_range - 1;

  q15_t line1[image_width];
  q15_t line2[image_width];

  q15_t cost_per_pixel[disparity_range];

  q7_t check1[image_width * disparity_range];
  q7_t check2[image_width * disparity_range];

  q7_t max_disps1[image_width];
  q7_t max_disps2[image_width];

  q7_t *upd_disps1;

  q7_t max_length1;
  q7_t max_length2;

  //uint32_t c = 0;
  uint32_t d = 0;
  //uint32_t e = 0;
  uint32_t i;
  uint32_t i2;
  //uint32_t j;
  //uint32_t k;
  uint32_t l;
  //uint32_t t;
  uint32_t cnt0;
  uint32_t cnt1;
  //int test_times;

  q7_t check_temp[disparity_range];
  q7_t seqLength;

  cnt0 = min_y * image_width_bytes;
  for (l = min_y; l < max_y; l++) {

    if ((l < image_height - 2) && (STEREO_CAM_NUMBER == 1)) {
      // separate the image line into a left and right one (line1, lin2 respectively):
      separate_image_line_offset(&in[cnt0], line1, line2, image_width_bytes);
    } else {
      // separate the image line into a left and right one with offset (line1, lin2 respectively):
      separate_image_line(&in[cnt0], line1, line2, image_width_bytes);
    }

    // the disparities will be put in upd_disps1:
    upd_disps1 = &out[cnt0 / 2];

    // for all 'l' image lines
    arm_fill_q7(0, check1, image_width * disparity_range); // set to zero
    arm_fill_q7(0, check2, image_width * disparity_range); // set to zero
    arm_fill_q7(0, check_temp, disparity_range);  // set to zero

    for (i = disparity_max; i < image_width; i++) { // for each pixel of the image line
      // compute cost
      arm_offset_q15(&line2[i - disparity_max], -line1[i], cost_per_pixel,
                     disparity_range);  // disparity_range should be multiple of 4 for efficiency
      arm_abs_q15(cost_per_pixel, cost_per_pixel, disparity_range);   // convert to absolute error

      // arm_min_q15( cost_per_pixel, disparity_range, &min_cost, &min_index ); // get minimum error for this pixel

      //if ( min_cost < thr2 ) // check if min_cost for pixel is not too high
      //{
      cnt1 = 0;
      for (d = 0; d < disparity_range; d++) {
        if (i == image_width - 1 || cost_per_pixel[disparity_max - d] > thr1) { // check if pixel cost exceeds error threshold
          seqLength = check_temp[d];
          // increment sequence length of all previous pixels in sequence
          while (check_temp[d] > 0) {
            check1[cnt1 + i - check_temp[d]] = seqLength;
            check2[cnt1 + i - d - check_temp[d]] = seqLength;
            check_temp[d]--;
          }
        }
        check_temp[d]++;
        cnt1 += image_width;
      }
      //}
    }

    arm_fill_q7(0, max_disps1, image_width);  // set to zero
    arm_fill_q7(0, max_disps2, image_width);  // set to zero

    for (i = disparity_max, i2 = 0; i < image_width; i++, i2++) { // for each pixel of the image line
      max_length1 = 0; max_length2 = 0;
      cnt1 = 0;
      for (d = 0; d < disparity_range; d++) {
        if (check1[cnt1 + i] > max_length1) { // determine for each pixel at which disparity the sequence of '0' is the longest
          max_length1 = check1[cnt1 + i];
          max_disps1[i] = d;        // the disparity with the longest '0' sequence around a pixel is chosen as initial disparity
        }

        if (check2[cnt1 + i2] > max_length2) { // and for the other image
          max_length2 = check2[cnt1 + i2];
          max_disps2[i2] = d;       // this is done for matching the left to the right image, and also vice-versa
        }
        cnt1 += image_width;
      }
    }

    arm_fill_q7(disparity_range, upd_disps1 + disparity_max, image_width - 2 * disparity_max); // set to disparity_max

    for (i = disparity_max; i < image_width - disparity_max; i++) { // for each pixel of the image line
      if (upd_disps1[i + max_disps2[i]] ==
          disparity_range) { // project the disparity map of the second image using initial disparities on the first image
        upd_disps1[i + max_disps2[i]] = max_disps2[i];
      }

      if (max_disps1[i] <
          upd_disps1[i]) { // compare the initial disparity map of the first image to the projection of the second image, choose smalles disparity
        upd_disps1[i] = max_disps1[i];
      }
    }
    cnt0 += image_width_bytes;
  } // for all image lines

}

void stereo_vision(uint8_t *in, q7_t *out, uint32_t image_width, uint32_t image_height, uint32_t disparity_range,
                   uint8_t thr1, uint8_t thr2, uint8_t min_y, uint8_t max_y)
{
  //uint32_t image_size = image_width*image_height;
  uint32_t image_width_bytes = image_width * 2;
  uint32_t disparity_max = disparity_range - 1;

  q15_t line1[image_width];
  q15_t line2[image_width];

  q15_t cost_per_pixel[disparity_range];

  q7_t check1[image_width * disparity_range];
  q7_t check2[image_width * disparity_range];

  q7_t max_disps1[image_width];
  q7_t max_disps2[image_width];

  q7_t *upd_disps1;

  q7_t max_length1;
  q7_t max_length2;

  //q15_t min_cost;
  //uint32_t min_index;

  //uint16_t start1;
  //q7_t length1;
  //q7_t disp1;

  //uint16_t start2;
  //q7_t length2;
  //q7_t disp2;

  //uint32_t c = 0;
  uint32_t d = 0;
  //uint32_t e = 0;
  uint32_t i;
  uint32_t i2;
  //uint32_t j;
  //uint32_t k;
  uint32_t l;
  //uint32_t t;
  uint32_t cnt0;
  uint32_t cnt1;
  //int test_times;

  q7_t check_temp[disparity_range];
  q7_t seqLength;

  cnt0 = min_y * image_width_bytes;
  for (l = min_y; l < max_y; l++) {
    // separate the image line into a left and right one (line1, lin2 respectively):
    //separate_image_line(&in[cnt0], line1, line2, image_width_bytes);


    if ((l < image_height - 2) && (STEREO_CAM_NUMBER == 1)) {
      // separate the image line into a left and right one (line1, lin2 respectively):
      separate_image_line_offset(&in[cnt0], line1, line2, image_width_bytes);
    } else {
      // separate the image line into a left and right one with offset (line1, lin2 respectively):
      separate_image_line(&in[cnt0], line1, line2, image_width_bytes);
    }

    // the disparities will be put in upd_disps1:
    upd_disps1 = &out[cnt0 / 2];

    // for all 'l' image lines
    arm_fill_q7(0, check1, image_width * disparity_range); // set to zero
    arm_fill_q7(0, check2, image_width * disparity_range); // set to zero
    arm_fill_q7(0, check_temp, disparity_range);  // set to zero

    for (i = disparity_max; i < image_width; i++) { // for each pixel of the image line
      // compute cost
      arm_offset_q15(&line2[i - disparity_max], -line1[i], cost_per_pixel,
                     disparity_range);  // disparity_range should be multiple of 4 for efficiency
      arm_abs_q15(cost_per_pixel, cost_per_pixel, disparity_range);   // convert to absolute error

      // arm_min_q15( cost_per_pixel, disparity_range, &min_cost, &min_index ); // get minimum error for this pixel

      //if ( min_cost < thr2 ) // check if min_cost for pixel is not too high
      //{
      cnt1 = 0;
      for (d = 0; d < disparity_range; d++) { // CHECK
        if (i == image_width - 1 || cost_per_pixel[disparity_max - d] > thr1) { // check if pixel cost exceeds error threshold
          seqLength = check_temp[d];
          // increment sequence length of all previous pixels in sequence
          while (check_temp[d] > 0) {
            check1[cnt1 + i - check_temp[d]] = seqLength;
            check2[cnt1 + i - d - check_temp[d]] = seqLength;
            check_temp[d]--;
          }
        }
        check_temp[d]++;
        cnt1 += image_width;
      }
      //}
    }

    arm_fill_q7(0, max_disps1, image_width);  // set to zero
    arm_fill_q7(0, max_disps2, image_width);  // set to zero

    for (i = disparity_max, i2 = 0; i < image_width; i++, i2++) { // for each pixel of the image line
      max_length1 = 0; max_length2 = 0;
      cnt1 = 0;
      for (d = 0; d < disparity_range; d++) { // CHECK
        if (check1[cnt1 + i] > max_length1) { // determine for each pixel at which disparity the sequence of '0' is the longest
          max_length1 = check1[cnt1 + i];
          max_disps1[i] = d;        // the disparity with the longest '0' sequence around a pixel is chosen as initial disparity
        }

        if (check2[cnt1 + i2] > max_length2) { // and for the other image
          max_length2 = check2[cnt1 + i2];
          max_disps2[i2] = d;       // this is done for matching the left to the right image, and also vice-versa
        }
        cnt1 += image_width;
      }
    }

    arm_fill_q7(disparity_range, upd_disps1 + disparity_max, image_width - 2 * disparity_max); // set to disparity_max

    for (i = disparity_max; i < image_width - disparity_max; i++) { // for each pixel of the image line
      if (upd_disps1[i + max_disps2[i]] ==
          disparity_range) { // project the disparity map of the second image using initial disparities on the first image
        upd_disps1[i + max_disps2[i]] = max_disps2[i];
      }

      if (max_disps1[i] <
          upd_disps1[i]) { // compare the initial disparity map of the first image to the projection of the second image, choose smalles disparity
        upd_disps1[i] = max_disps1[i];
      }
    }
    cnt0 += image_width_bytes;
  } // for all image lines

}

void separate_image_line(uint8_t *in, q15_t *line1, q15_t *line2, uint32_t image_width_bytes)
{
  uint32_t i, j;
  for (i = 0; i < image_width_bytes; i += 2) {
    j = i >> 1;
    line1[j] = (q15_t) in[i];
    line2[j] = (q15_t) in[i + 1];
  }
}

void separate_image_line_offset(uint8_t *in, q15_t *line1, q15_t *line2, uint32_t image_width_bytes)
{
  uint32_t i, j;
  for (i = 0; i < image_width_bytes; i += 2) {
    j = i >> 1;
    line1[j] = (q15_t) in[i];
    line2[j] = (q15_t) in[i + 1 + image_width_bytes + image_width_bytes];
  }
}

uint32_t evaluate_disparities(uint8_t *in, uint32_t image_width, uint32_t image_height, uint8_t disparity_threshold,
                              uint32_t disparities_high)
{
  uint32_t i, j;

  disparities_high = 0;

  for (i = 0; i < image_width; i++) {
    for (j = 0; j < image_height; j++) {
      if (in[i + j * image_width] > disparity_threshold) {
        disparities_high++;
      }
    }
  }

  return disparities_high;

}
/*
void evaluate_central_disparities(uint8_t* in, uint32_t image_width, uint32_t image_height, uint32_t* disparities, uint8_t n_disp_bins, uint8_t min_y, uint8_t max_y)
{
  uint16_t b, y, x;
  uint8_t border = 10;
  uint16_t bin_size = (image_width - 2 * border) / n_disp_bins + 1;
  uint32_t n_pixels[n_disp_bins];
  uint16_t RESOLUTION = 100;
  for(b = 0; b < n_disp_bins; b++)
  {
    disparities[b] = 0;
    n_pixels[b] = 0;
  }
  for(x = border; x < image_width - border; x++)
  {
    // determine the bin index
    b = (x-border) / bin_size;
    if(b >= n_disp_bins) b = n_disp_bins - 1;
    for(y = min_y; y < max_y; y++)
    {
      disparities[b] += in[x+y*image_width];
      n_pixels[b]++;
    }
  }
  for(b = 0; b < n_disp_bins; b++)
  {
    disparities[b] /= (n_pixels[b] / RESOLUTION);
  }
}

void evaluate_central_disparities2(uint8_t* in, uint32_t image_width, uint32_t image_height, uint32_t* disparities, uint8_t n_disp_bins, uint8_t min_y, uint8_t max_y, uint8_t disp_threshold)
{
  uint16_t b, y, x;
  uint8_t border = 10;
  uint16_t bin_size = (image_width - 2 * border) / n_disp_bins + 1;
  uint32_t n_pixels[n_disp_bins];
  uint16_t RESOLUTION = 100;
  uint32_t mean_x = 0;

  for(b = 0; b < n_disp_bins; b++)
  {
    disparities[b] = 0;
    //n_pixels[b] = 0;
  }
  for(x = border; x < image_width - border; x++)
  {
    for(y = min_y; y < max_y; y++)
    {
      if ( in[x+y*image_width] > disp_threshold)
      {
        disparities[0]++;
        mean_x += x;
      }
    }
  }
  disparities[1] = mean_x/disparities[0];

}



void evaluate_disparities_altitude(uint8_t* in, uint32_t image_width, uint32_t image_height, uint8_t disparity_threshold, uint32_t* disparities, uint8_t altitude_levels, uint16_t x_min, uint16_t x_max, uint32_t* bad_pixels)
{
  uint32_t i, j, k, l, n_pixels, RESOLUTION, min_d;

  uint32_t level_size = image_height/altitude_levels;
  for ( i = 0; i<altitude_levels; i++)
  {
    disparities[i] = 0;
    bad_pixels[i] = 0;
  }
  k = 0;
  l = 0;
  RESOLUTION = 100;
  n_pixels = 0;
  min_d = 3;

  for (j = 0; j<image_height; j++ )
  {
    if ( l == level_size && k <altitude_levels-1)
    {
      // take the average disparity in the image region, but at a certain resolution
      // if disparities go from 0 to 10, and the resolution is 10, then the values go from 0 to 100.
      if(n_pixels != 0)
      {
        disparities[k] *= RESOLUTION;
        disparities[k] /= n_pixels;
      }

      bad_pixels[k] = (bad_pixels[k]*RESOLUTION) / (n_pixels+bad_pixels[k]);

      if(k < altitude_levels)
      {
        k++;
      }
      n_pixels = 0;
      l = 0;
    }

    l++;

    for (i = x_min; i < x_max; i++)
    {
      // if (in[i+j*image_width]>disparity_threshold)
      //  disparities[k]++;
      if ( in[i+j*image_width]>=min_d )
      {
        disparities[k] += in[i+j*image_width];
        n_pixels++;
      }
      else
      {
        bad_pixels[k]++;
      }

    }
  }

  if(n_pixels != 0)
  {
    disparities[k] *= RESOLUTION;
    disparities[k] /= n_pixels;
  }
  bad_pixels[k] = (bad_pixels[k]*RESOLUTION) / (n_pixels+bad_pixels[k]);


  for ( i = 0; i<altitude_levels; i++)
    {
      disparities[i]-= min_d*RESOLUTION;

    }
}

void filter_disparity_map(uint8_t* in, uint8_t diff_threshold, uint32_t image_width, uint32_t image_height, uint8_t min_y, uint8_t max_y)
{
  uint16_t x, y, abs_diff;
  int diff;
  for (x = 0; x < image_width; x++)
  {
    for(y = min_y+1; y < max_y-1; y++)
    {
      diff = ((int) in[x+y*image_width]) - ((int) in[x+(y-1)*image_width]);
      diff = (diff < 0) ? -diff : diff;
      abs_diff = (uint16_t) (diff);
      if(abs_diff >= diff_threshold)
      {
        in[x+y*image_width] = 0;
      }
      diff = ((int) in[x+y*image_width]) - ((int) in[x+(y+1)*image_width]);
      diff = (diff < 0) ? -diff : diff;
      abs_diff = (uint16_t) (diff);
      if(abs_diff >= diff_threshold)
      {
        in[x+y*image_width] = 0;
      }
    }
  }
}
*/
