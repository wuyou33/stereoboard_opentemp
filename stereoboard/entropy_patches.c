// Entropy patches code

#include "entropy_patches.h"

#define RESOLUTION 250

static const int8_t log2_lookup_table[250] = {
  0, 8, 14, 19, 24, 28, 32, 36, 40, 43, 46, 50, 53, 55, 58, 61, 63, 66, 68, 71, 73, 75, 77, 79, 81, 83, 85, 87, 88, 90, 92, 93, 95, 96, 98, 99, 101, 102, 103, 105, 106, 107, 108, 109, 110, 111, 112, 113, 114, 115, 116, 117, 118, 119, 119, 120, 121, 122, 122, 123, 124, 124, 125, 125, 126, 126, 127, 127, 128, 128, 129, 129, 129, 130, 130, 130, 131, 131, 131, 131, 132, 132, 132, 132, 132, 132, 132, 132, 133, 133, 133, 133, 133, 133, 133, 133, 133, 132, 132, 132, 132, 132, 132, 132, 132, 131, 131, 131, 131, 131, 130, 130, 130, 129, 129, 129, 129, 128, 128, 127, 127, 127, 126, 126, 125, 125, 125, 124, 124, 123, 123, 122, 122, 121, 121, 120, 119, 119, 118, 118, 117, 116, 116, 115, 115, 114, 113, 113, 112, 111, 111, 110, 109, 108, 108, 107, 106, 105, 105, 104, 103, 102, 101, 101, 100, 99, 98, 97, 96, 95, 95, 94, 93, 92, 91, 90, 89, 88, 87, 86, 85, 84, 83, 82, 81, 80, 79, 78, 77, 76, 75, 74, 73, 72, 71, 70, 69, 68, 67, 66, 64, 63, 62, 61, 60, 59, 58, 56, 55, 54, 53, 52, 50, 49, 48, 47, 46, 44, 43, 42, 41, 39, 38, 37, 35, 34, 33, 32, 30, 29, 28, 26, 25, 24, 22, 21, 20, 18, 17, 16, 14, 13, 11, 10, 9, 7, 6, 4, 3, 1
};

/* Entropy patches:
 * takes input image in, which contains both the left and the right image
 * determines the entropy of patches in one of the two images.
 *
 * parameters:
 * right: whether to use the right image (1) or left image (0)
 * in: the input image(s)
 * image_width
 * image_height
 * min_x/y, max_x/y: defines a rectangle in which to take samples on a grid
 * patch_size: patch size in pixels of which to determine entropy
 * step_size: in pixels. The bigger the step size, the fewer samples
 * n_bins: number of bins in the histogram / pdf for entropy determination
 * */

uint32_t get_entropy_patches(uint8_t right, uint8_t *in, uint32_t image_width, uint32_t image_height,
                             uint8_t min_x, uint8_t max_x, uint8_t min_y, uint8_t max_y,
                             uint8_t patch_size, uint8_t step_size, uint8_t n_bins)
{
  uint32_t total_entropy = 0;
  uint32_t n_samples = 0;
  uint32_t n_elements = patch_size * patch_size;
  // initialize probability distribution P
  q15_t P[n_bins];
  q7_t bin_size = 255 / n_bins;

  // loop over the selected image part with step_size
  uint32_t W2 = image_width * 2; // number of pixels per image line in stereo image
  uint32_t x, y, xx, yy;
  uint8_t px, bin;
  uint32_t Mx = max_x - patch_size;
  uint32_t My = max_y - patch_size;

  for (x = min_x; x < Mx; x += step_size) {
    for (y = min_y; y < My; y += step_size) {
      // set probability distribution to zero:
      arm_fill_q15(0, P, n_bins);

      // loop over the pixels in the patch:
      for (xx = x; xx < x + patch_size; xx++) {
        for (yy = y; yy < y + patch_size; yy++) {
          px = in[yy * W2 + xx * 2 + right];
          bin = px / bin_size;
          P[bin]++;
        }
      }

      // get entropy for this patch:
      total_entropy += calculate_entropy(P, n_bins, n_elements);
      n_samples++;
    }
  }

  return (uint32_t) total_entropy / n_samples;
}

uint32_t calculate_entropy(q15_t *P, uint8_t n_bins, uint32_t n_elements)
{
  uint32_t entropy = 0;
  uint32_t table_index;
  uint8_t bin;
  for (bin = 0; bin < n_bins; bin++) {
    table_index = (P[bin] * RESOLUTION) / n_elements;
    entropy += log2_lookup_table[table_index];
  }
  return entropy;
}

/*
void stereo_vision(uint8_t* in, q7_t* out, uint32_t image_width, uint32_t image_height, uint32_t disparity_range, uint8_t thr1, uint8_t thr2)
{
  uint32_t image_size = image_width*image_height;
  uint32_t image_width_bytes = image_width * 2;
  uint32_t disparity_max = disparity_range-1;

  q15_t line1[image_width];
  q15_t line2[image_width];

  q15_t cost_per_pixel[disparity_range];

  q7_t check1[image_width*disparity_range];
  q7_t check2[image_width*disparity_range];

  q7_t max_disps1[image_width];
  q7_t max_disps2[image_width];

  q7_t* upd_disps1;
  q7_t upd_disps2[image_width];

  q7_t max_length1;
  q7_t max_length2;

  q15_t min_cost;
  uint32_t min_index;

  uint16_t start1;
  q7_t length1;
  q7_t disp1;

  uint16_t start2;
  q7_t length2;
  q7_t disp2;

  uint32_t c = 0;
  uint32_t d = 0;
  uint32_t e = 0;
  uint32_t i;
  uint32_t i2;
  uint32_t j;
  uint32_t k;
  uint32_t l;
  uint32_t t;
  uint32_t cnt0;
  uint32_t cnt1;
  int test_times;



  cnt0 = 0;
  for ( l = 0; l < image_height; l++)
  {
    // separate the image line into a left and right one (line1, lin2 respectively):
    separate_image_line(&in[cnt0], line1, line2, image_width_bytes);

    // the disparities will be put in upd_disps1:
    upd_disps1 = &out[cnt0/2];

    // for all 'l' image lines
    arm_fill_q7( 0, check1, image_width*disparity_range); // set to zero
    arm_fill_q7( 0, check2, image_width*disparity_range); // set to zero

    for ( i = disparity_max; i < image_width; i++) // for each pixel of the image line
    {
      // compute cost
      arm_offset_q15( &line2[i - disparity_max], -line1[i], cost_per_pixel, disparity_range); // disparity_range should be multiple of 4 for efficiency
      arm_abs_q15( cost_per_pixel, cost_per_pixel, disparity_range ); // convert to absolute error

      arm_min_q15( cost_per_pixel, disparity_range, &min_cost, &min_index ); // get minimum error for this pixel

      if ( min_cost<thr2 ) // check if min_cost for pixel is not too high
      {
        cnt1 = 0;
        for ( d = 0; d<disparity_range; d++)
        {
          if (cost_per_pixel[disparity_max-d]>thr1) // check if pixel cost exceeds error threshold
          {
            check1[cnt1+i] = 1; // setting pixel to 1 means that this pixel matches badly for this disparity
            check2[cnt1+i-d] = 1; // this is done for both left-right matching and right-left matching
          }

          cnt1+=image_width;
        }
      }

    }


    cnt1 = 0;
    for ( d = 0; d<disparity_range; d++)
    {
      length1 = 0;
      start1 = (cnt1)+disparity_max;

      length2 = 0;
      start2 = (cnt1);

      for ( i = disparity_max; i<=image_width; i++) // for each pixel of the image line
      {
        if ( check1[cnt1+i]==1 || i== image_width) // calculate the lengths of sequences of '0' (good matches)
        {
          if ( length1>0 )
          {
            arm_fill_q7( length1, &check1[start1], length1 );
            length1 = 0;
          }

          start1 = (cnt1)+i+1;

        }
        else
        {
          length1++;
        }

        i2 = i-disparity_max;

        if ( check2[cnt1+i2]==1 || i== image_width)  // do the same for the other image
        {
          if ( length2>0 )
          {
            arm_fill_q7( length2, &check2[start2], length2 );
            length2 = 0;
          }

          start2 = (cnt1)+i2+1;

        }
        else
        {
          length2++;
        }

      }

      cnt1+=image_width;

    }

    arm_fill_q7( 0, max_disps1, image_width); // set to zero
    arm_fill_q7( 0, max_disps2, image_width); // set to zero

    for ( i = disparity_max; i<image_width; i++) // for each pixel of the image line
    {
      max_length1 = 0;
      disp1 = 0;

      max_length2 = 0;
      disp2 = 0;

      i2 = i - disparity_max;

      cnt1 = 0;
      for ( d = 0; d<disparity_range; d++)
      {
        if ( check1[cnt1+i] > max_length1 )// determine for each pixel at which disparity the sequence of '0' is the longest
        {
          max_length1 = check1[cnt1+i];
          disp1 = d;
        }

        if ( check2[cnt1+i2] > max_length2 ) // and for the other image
        {
          max_length2 = check2[cnt1+i2];
          disp2 = d;
        }

        cnt1+=image_width;
      }

      max_disps1[i] = disp1; // the disparity with the longest '0' sequence around a pixel is chosen as initial disparity
      max_disps2[i2] = disp2; // this is done for matching the left to the right image, and also vice-versa

    }

    arm_fill_q7( 0, upd_disps1, image_width); // set to zero

    for ( i = 0; i<image_width; i++) // for each pixel of the image line
    {
      if ( upd_disps1[i+max_disps2[i]]==0 ) // project the disparity map of the second image using initial disparities on the first image
        upd_disps1[i+max_disps2[i]]=max_disps2[i];

    }

    for ( i = 0; i<image_width; i++) // for each pixel of the image line
    {
      if ( max_disps1[i] < upd_disps1[i] ) // compare the initial disparity map of the first image to the projection of the second image, choose smalles disparity
        upd_disps1[i] = max_disps1[i];
    }

    cnt0+=image_width_bytes;
  } // for all image lines

}

void separate_image_line(uint8_t* in, q15_t* line1, q15_t* line2, uint32_t image_width_bytes)
{
  uint32_t i, j;
  for(i = 0; i < image_width_bytes; i+=2)
  {
    j = i >> 1;
    line1[j] = (q15_t) in[i];
    line2[j] = (q15_t) in[i+1];
  }
}
*/
