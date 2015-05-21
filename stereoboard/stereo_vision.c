// Stereo vision code

#include "stereo_vision.h"

#define MIN(X, Y) (((X) < (Y)) ? (X) : (Y))

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

void stereo_vision_Kirk(uint8_t *in, q7_t *out, uint32_t image_width, uint32_t image_height, uint32_t disparity_min,
                        uint32_t disparity_range, uint32_t disparity_step, uint8_t thr1, uint8_t thr2, uint8_t min_y, uint8_t max_y)
{
  uint32_t image_width_bytes = image_width * 2;
  uint32_t disparity_max = disparity_range - 1 + disparity_min;

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
      separate_image_line_offset(&in[cnt0], line1, line2, image_width_bytes, 2);
    } else if ((l < image_height - 5) && (STEREO_CAM_NUMBER == 3)) {
      // separate the image line into a left and right one (line1, lin2 respectively):
      separate_image_line_offset(&in[cnt0], line1, line2, image_width_bytes, -2);
    } else if ((l < image_height - 5) && (STEREO_CAM_NUMBER == 7)) {
      // separate the image line into a left and right one (line1, lin2 respectively):
      separate_image_line_offset(&in[cnt0], line1, line2, image_width_bytes, 4);
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
      for (d = disparity_min; d <= disparity_max; d += disparity_step) {
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
      for (d = disparity_min; d <= disparity_max; d += disparity_step) {
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


#ifdef SMOOTH_DISPARITY_MAP

	int bufferIndex=0;
	uint8_t cleaner_image_buffer[image_width*image_height];
	for(bufferIndex=0; bufferIndex < image_width*image_height; bufferIndex++)
	{
		//out[bufferIndex]=10;
		if (bufferIndex > 2*image_width && bufferIndex < (image_width*image_height)-2*image_width_bytes){
			uint8_t pixel = out[bufferIndex];
			uint8_t above = out[bufferIndex-image_width];

			uint8_t underPixel = out[bufferIndex+image_width];
			cleaner_image_buffer[bufferIndex] = MIN(MIN(above,underPixel),pixel);
		}
		else
		{
			cleaner_image_buffer[bufferIndex] = 90;//out[bufferIndex];
			out[bufferIndex]=90;
		}

		// Time to write it to the out buffer as we surely do not access it anymore
		if (bufferIndex > 2*image_width)
		{
			out[bufferIndex-2*image_width] = cleaner_image_buffer[bufferIndex-2*image_width];
		}
	}

#endif
}

void stereo_vision(uint8_t *in, q7_t *out, uint32_t image_width, uint32_t image_height, uint32_t disparity_range,
                   uint8_t thr1, uint8_t thr2, uint8_t min_y, uint8_t max_y)
{
  uint32_t image_size = image_width * image_height;
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



  cnt0 = min_y * image_width_bytes;
  for (l = min_y; l < max_y; l++) {

    if ((l < image_height - 2) && (STEREO_CAM_NUMBER == 1)) {
      // separate the image line into a left and right one (line1, lin2 respectively):
      separate_image_line_offset(&in[cnt0], line1, line2, image_width_bytes, 2);
    } else {
      // separate the image line into a left and right one with offset (line1, lin2 respectively):
      separate_image_line(&in[cnt0], line1, line2, image_width_bytes);
    }

    // the disparities will be put in upd_disps1:
    upd_disps1 = &out[cnt0 / 2];

    // for all 'l' image lines
    arm_fill_q7(0, check1, image_width * disparity_range); // set to zero
    arm_fill_q7(0, check2, image_width * disparity_range); // set to zero

    for (i = disparity_max; i < image_width; i++) { // for each pixel of the image line
      // compute cost
      arm_offset_q15(&line2[i - disparity_max], -line1[i], cost_per_pixel,
                     disparity_range);  // disparity_range should be multiple of 4 for efficiency
      arm_abs_q15(cost_per_pixel, cost_per_pixel, disparity_range);   // convert to absolute error

      arm_min_q15(cost_per_pixel, disparity_range, &min_cost, &min_index);   // get minimum error for this pixel

      if (min_cost < thr2) { // check if min_cost for pixel is not too high
        cnt1 = 0;
        for (d = 0; d < disparity_range; d++) {
          if (cost_per_pixel[disparity_max - d] > thr1) { // check if pixel cost exceeds error threshold
            check1[cnt1 + i] = 1; // setting pixel to 1 means that this pixel matches badly for this disparity
            check2[cnt1 + i - d] = 1; // this is done for both left-right matching and right-left matching
          }

          cnt1 += image_width;
        }
      }

    }


    cnt1 = 0;
    for (d = 0; d < disparity_range; d++) {
      length1 = 0;
      start1 = (cnt1) + disparity_max;

      length2 = 0;
      start2 = (cnt1);

      for (i = disparity_max; i <= image_width; i++) { // for each pixel of the image line
        if (check1[cnt1 + i] == 1 || i == image_width) { // calculate the lengths of sequences of '0' (good matches)
          if (length1 > 0) {
            arm_fill_q7(length1, &check1[start1], length1);
            length1 = 0;
          }

          start1 = (cnt1) + i + 1;

        } else {
          length1++;
        }

        i2 = i - disparity_max;

        if (check2[cnt1 + i2] == 1 || i == image_width) { // do the same for the other image
          if (length2 > 0) {
            arm_fill_q7(length2, &check2[start2], length2);
            length2 = 0;
          }

          start2 = (cnt1) + i2 + 1;

        } else {
          length2++;
        }

      }

      cnt1 += image_width;

    }

    arm_fill_q7(0, max_disps1, image_width);  // set to zero
    arm_fill_q7(0, max_disps2, image_width);  // set to zero

    for (i = disparity_max; i < image_width; i++) { // for each pixel of the image line
      max_length1 = 0;
      disp1 = 0;

      max_length2 = 0;
      disp2 = 0;

      i2 = i - disparity_max;

      cnt1 = 0;
      for (d = 0; d < disparity_range; d++) {
        if (check1[cnt1 + i] > max_length1) { // determine for each pixel at which disparity the sequence of '0' is the longest
          max_length1 = check1[cnt1 + i];
          disp1 = d;
        }

        if (check2[cnt1 + i2] > max_length2) { // and for the other image
          max_length2 = check2[cnt1 + i2];
          disp2 = d;
        }

        cnt1 += image_width;
      }

      max_disps1[i] = disp1; // the disparity with the longest '0' sequence around a pixel is chosen as initial disparity
      max_disps2[i2] = disp2; // this is done for matching the left to the right image, and also vice-versa

    }

    arm_fill_q7(0, upd_disps1, image_width);  // set to zero

    for (i = 0; i < image_width; i++) { // for each pixel of the image line
      if (upd_disps1[i + max_disps2[i]] ==
          0) { // project the disparity map of the second image using initial disparities on the first image
        upd_disps1[i + max_disps2[i]] = max_disps2[i];
      }

    }

    for (i = 0; i < image_width; i++) { // for each pixel of the image line
      if (max_disps1[i] <
          upd_disps1[i]) { // compare the initial disparity map of the first image to the projection of the second image, choose smalles disparity
        upd_disps1[i] = max_disps1[i];
      }
    }

    cnt0 += image_width_bytes;
  } // for all image lines

}

void stereo_vision_cigla(uint8_t *in, q7_t *out, uint32_t image_width, uint32_t image_height, uint32_t disparity_range,
                         uint8_t sadWS, uint8_t sigma, uint32_t diff_threshold, uint8_t min_y, uint8_t max_y)
{
  uint32_t image_size = image_width * image_height;
  uint32_t image_width_bytes = image_width * 2;
  uint32_t disparity_max = disparity_range - 1;

  uint32_t image_width_disp = image_width - disparity_max;
  uint32_t image_width_disp_sad = image_width_disp - sadWS + 1;

  uint32_t sadWS2 = (sadWS - 1) / 2;

  q15_t line1[image_width];
  q15_t line2[image_width];
  q15_t absdiff[image_width_disp];
  q15_t sadCost;
  q31_t sadCostVec[image_width_disp_sad];
  q31_t costLeft[image_width_disp_sad * disparity_range];
  q31_t costRight[image_width_disp_sad * disparity_range];
  q31_t totalCostVec[image_width_disp_sad * disparity_range];
  q31_t minCostVec[image_width_disp_sad];
  q15_t absdiffNB[image_width_disp_sad - 1];

  q7_t *disps;

  uint32_t l; // line index
  uint32_t d; // disparity index
  uint32_t i; // general index counter
  uint32_t j; // general index counter
  uint32_t k; // general index counter

  q15_t expNums[256] = { 1, 119, 59, 39, 29, 23, 19, 202, 7, 173, 204, 164, 140, 62, 156, 169, 36, 58, 183, 51, 91, 105, 149, 137, 61, 89, 59, 139, 37, 140, 37, 34, 44, 15, 101, 125, 118, 129, 112, 71, 115, 51, 73, 21, 61, 94, 59, 53, 102, 19, 103, 53, 95, 74, 37, 2, 35, 70, 89, 49, 71, 72, 58, 7, 53, 22, 1, 55, 66, 19, 71, 49, 25, 69, 67, 51, 51, 23, 67, 26, 63, 7, 64, 64, 18, 57, 57, 19, 3, 27, 27, 52, 30, 52, 43, 31, 43, 28, 33, 29, 17, 47, 19, 30, 44, 41, 27, 20, 20, 33, 39, 25, 15, 33, 35, 5, 23, 35, 34, 15, 18, 33, 25, 30, 10, 31, 6, 23, 9, 29, 26, 8, 28, 17, 3, 25, 17, 21, 25, 7, 16, 23, 3, 19, 23, 19, 5, 17, 14, 1, 11, 18, 5, 16, 1, 17, 13, 13, 13, 13, 13, 11, 17, 15, 13, 14, 11, 6, 9, 11, 1, 7, 12, 8, 6, 13, 5, 9, 7, 4, 12, 7, 9, 9, 11, 6, 5, 7, 11, 3, 7, 8, 9, 10, 8, 5, 9, 3, 9, 7, 1, 2, 1, 2, 1, 2, 1, 2, 1, 7, 7, 3, 4, 5, 5, 1, 5, 5, 6, 2, 6, 5, 5, 6, 6, 2, 4, 1, 3, 2, 5, 1, 5, 5, 5, 4, 1, 1, 4, 3, 3, 2, 2, 3, 3, 3, 3, 3, 3, 2, 2, 3, 3, 3, 1, 1
                       };
  q15_t expDens[256] = { 1, 121, 61, 41, 31, 25, 21, 227, 8, 201, 241, 197, 171, 77, 197, 217, 47, 77, 247, 70, 127, 149, 215, 201, 91, 135, 91, 218, 59, 227, 61, 57, 75, 26, 178, 224, 215, 239, 211, 136, 224, 101, 147, 43, 127, 199, 127, 116, 227, 43, 237, 124, 226, 179, 91, 5, 89, 181, 234, 131, 193, 199, 163, 20, 154, 65, 3, 168, 205, 60, 228, 160, 83, 233, 230, 178, 181, 83, 246, 97, 239, 27, 251, 255, 73, 235, 239, 81, 13, 119, 121, 237, 139, 245, 206, 151, 213, 141, 169, 151, 90, 253, 104, 167, 249, 236, 158, 119, 121, 203, 244, 159, 97, 217, 234, 34, 159, 246, 243, 109, 133, 248, 191, 233, 79, 249, 49, 191, 76, 249, 227, 71, 253, 156, 28, 237, 164, 206, 249, 71, 165, 241, 32, 206, 254, 213, 57, 197, 165, 12, 134, 223, 63, 205, 13, 225, 175, 178, 181, 184, 187, 161, 253, 227, 200, 219, 175, 97, 148, 184, 17, 121, 211, 143, 109, 240, 94, 172, 136, 79, 241, 143, 187, 190, 236, 131, 111, 158, 252, 70, 166, 193, 221, 249, 203, 129, 236, 80, 244, 193, 28, 57, 29, 59, 30, 61, 31, 63, 32, 228, 232, 101, 137, 174, 177, 36, 183, 186, 227, 77, 235, 199, 202, 247, 251, 85, 173, 44, 134, 91, 231, 47, 239, 243, 247, 201, 51, 52, 211, 161, 164, 111, 113, 172, 175, 178, 181, 184, 187, 127, 129, 197, 200, 203, 69, 70
                       };



  disps = &out[0];

  arm_fill_q7(0, disps, image_size);


  for (l = min_y; l < max_y + 1; l++) {
    // separate the image line into a left and right one (line1, line2 respectively)
    separate_image_line(&in[l * image_width_bytes], line1, line2, image_width_bytes);

    for (d = 0; d < disparity_range; d++) {
      arm_sub_q15(&line1[disparity_max], &line2[disparity_max - d], absdiff,
                  image_width_disp); // compute difference between left and right image lines for disparity d
      arm_abs_q15(absdiff, absdiff, image_width_disp);   // convert to absolute error

      sadCost = 0;
      for (i = 0; i < sadWS; i++) {
        sadCost += absdiff[i]; // compute first SAD window in the row
      }

      j = 1;
      k = 0;
      sadCostVec[0] = sadCost; // initialize SAD array for this image line
      for (i = sadWS; i < image_width_disp; i++, j++, k++) {
        sadCost -= absdiff[k];
        sadCost += absdiff[i];
        sadCostVec[j] = sadCost; // compute SAD values of the whole image line by using the value from the previous pixel
      }

      arm_sub_q15(&line1[disparity_max + sadWS2], &line1[disparity_max + sadWS2 + 1], absdiffNB,
                  (image_width_disp_sad - 1)); // compute difference between left and right image lines at different disparities
      arm_abs_q15(absdiffNB, absdiffNB, (image_width_disp_sad - 1)); // convert to absolute error


      costLeft[(image_width_disp_sad * d) + 0] = sadCostVec[0];
      j = 0;
      for (i = 1; i < image_width_disp_sad; i++, j++) {
        k = absdiffNB[j];
        costLeft[(image_width_disp_sad * d) + i] = sadCostVec[i] + (costLeft[(image_width_disp_sad * d) + j] * expNums[k] /
            expDens[k]); // update SAD costs over the image line using Information Permeability
      }

      costRight[(image_width_disp_sad * d) + image_width_disp_sad - 1] = sadCostVec[image_width_disp_sad - 1];
      j = image_width_disp_sad - 1;
      for (i = image_width_disp_sad - 2; i > 0; i--, j--) {
        k = absdiffNB[i];
        costRight[(image_width_disp_sad * d) + i] = sadCostVec[i] + (costRight[(image_width_disp_sad * d) + j] * expNums[k] /
            expDens[k]); // update SAD costs over the image line using Information Permeability
      }

    }

    arm_add_q31(costLeft, costRight, totalCostVec,
                image_width_disp_sad * disparity_range); // add left and right permeability cost measures

    arm_copy_q31(&totalCostVec[0], minCostVec, image_width_disp_sad);

    disps = &out[ l * image_width ];

    // compute disparity map
    for (d = 1; d < disparity_range; d++) {
      for (i = 0; i < image_width_disp_sad; i++) {
        if (totalCostVec[image_width_disp_sad * d + i] < minCostVec[i]) {
          minCostVec[i] = totalCostVec[image_width_disp_sad * d + i]; // keep track of smallest cost per pixel ....
          disps[disparity_max + sadWS2 + i] = d; // ... and keep track of corresponding disparity value

        }
      }
    }

    //update disparity map
    uint8_t unreliable = 1;
    int32_t minCost = 100000000;
    int32_t maxCost = 0;
    i = 0;
    while (unreliable && i < image_width_disp_sad) {
      for (d = 0; d < disparity_range; d++) {
        if (costLeft[image_width_disp_sad * d + i] > maxCost) {
          maxCost = costLeft[image_width_disp_sad * d + i];
        }

        if (costLeft[image_width_disp_sad * d + i] < minCost) {
          minCost = costLeft[image_width_disp_sad * d + i];
        }
      }

      if (maxCost > minCost + diff_threshold) {
        unreliable = 0;
      } else {
        disps[disparity_max + sadWS2 + i] = disparity_range * 2;
        i++;
      }
    }

    unreliable = 1;
    minCost = 100000000;
    maxCost = 0;
    i = image_width_disp_sad - 1;


    while (unreliable && i > 0) {
      for (d = 1; d < disparity_range; d++) {
        if (costRight[image_width_disp_sad * d + i] > maxCost) {
          maxCost = costRight[image_width_disp_sad * d + i];
        }

        if (costRight[image_width_disp_sad * d + i] < minCost) {
          minCost = costRight[image_width_disp_sad * d + i];
        }
      }

      if (maxCost > minCost + diff_threshold) {
        unreliable = 0;
      } else {
        disps[disparity_max + sadWS2 + i] = disparity_range * 2;
        i--;
      }
    }

  }

}

void evaluate_disparities_control(uint8_t *in, uint32_t image_width, uint32_t image_height, uint32_t control_output,
                                  uint32_t disparity_range, uint8_t sadWS, uint8_t crop, uint8_t thr3)
{
  uint32_t sadWS2 = (sadWS - 1) / 2;
  uint32_t disparity_max = disparity_range - 1;
  uint32_t image_width_disp = image_width - disparity_max;
  uint32_t image_width_disp_sad = image_width_disp - sadWS + 1;

  uint32_t disp_counts[disparity_range];
  uint32_t disp_sums[disparity_range];
  uint32_t disp_indices[disparity_range];

  uint32_t i;
  uint32_t l;
  uint32_t x;

  for (i = 0; i < disparity_range; i++) {
    disp_counts[i] = 0;
    disp_sums[i] = 0;
  }

  for (l = crop; l < (image_height - crop); l++) {
    for (x = disparity_max + sadWS2; x < image_width_disp_sad + disparity_max + sadWS2; x++) {
      disp_counts[in[l * image_width + x]]++;
      disp_sums[in[l * image_width + x]] += x;
    }
  }

  for (i = 0; i < disparity_range; i++) {
    disp_indices[i] = disp_sums[i] / disp_counts[i];

    if ((control_output == 0) && (disp_counts[i] > thr3)) {
      control_output = disp_indices[i];
    }
  }

  control_output = 350;


}

uint32_t evaluate_disparities_control2(uint8_t *in, uint32_t image_width, uint32_t image_height,
                                       uint32_t disparity_range, uint8_t sadWS, uint8_t crop, uint32_t thr3)
{
  uint32_t sadWS2 = (sadWS - 1) / 2;
  uint32_t disparity_max = disparity_range - 1;
  uint32_t image_width_disp = image_width - disparity_max;
  uint32_t image_width_disp_sad = image_width_disp - sadWS + 1;

  uint32_t disp_counts[disparity_range + 1];
  uint32_t disp_sums[disparity_range + 1];
  uint32_t disp_indices[disparity_range + 1];

  uint32_t i;
  uint32_t l;
  uint32_t x;

  uint32_t control_output = 0;

  for (i = 0; i < disparity_range; i++) {
    disp_counts[i] = 0;
    disp_sums[i] = 0;
  }

  for (l = crop; l < (image_height - crop); l++) {
    for (x = disparity_max + sadWS2; x < image_width_disp_sad + disparity_max + sadWS2; x++) {
      disp_counts[in[l * image_width + x]]++;
      disp_sums[in[l * image_width + x]] += x;
    }
  }

  for (i = 0; i < disparity_range; i++) {
    disp_indices[i] = disp_sums[i] / disp_counts[i];

    if ((control_output == 0) && (disp_counts[i] > thr3)) {
      control_output = disp_indices[i];
    }
  }

  //control_output = 350;

  return control_output;


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

void separate_image_line_offset(uint8_t *in, q15_t *line1, q15_t *line2, uint32_t image_width_bytes,  int8_t offset)
{
  uint32_t i, j;

  if (offset > 0) {
    for (i = 0; i < image_width_bytes; i += 2) {
      j = i >> 1;
      //line1[j] = (q15_t) in[i];
      //line2[j] = (q15_t) in[i+1+image_width_bytes+image_width_bytes]; // corresponds to camera:1

      line1[j] = (q15_t) in[i];
      line2[j] = (q15_t) in[i + 1 + (image_width_bytes * offset)]; // corresponds to camera:7
    }
  }

  if (offset < 0) {
    for (i = 0; i < image_width_bytes; i += 2) {
      j = i >> 1;
      //line1[j] = (q15_t) in[i];
      //line2[j] = (q15_t) in[i+1+image_width_bytes+image_width_bytes]; // corresponds to camera:1

      line1[j] = (q15_t) in[i - (image_width_bytes * offset)];
      line2[j] = (q15_t) in[i + 1]; // corresponds to camera:7
    }
  }
}

void evaluate_central_disparities(uint8_t *in, uint32_t image_width, uint32_t image_height, uint32_t *disparities,
                                  uint8_t n_disp_bins, uint8_t min_y, uint8_t max_y, uint8_t border)
{
  uint16_t b, y, x;
  //uint8_t border = 10;
  uint16_t bin_size = (image_width - 2 * border) / n_disp_bins + 1;
  uint32_t n_pixels[n_disp_bins];
  uint16_t RESOLUTION = 100;
  for (b = 0; b < n_disp_bins; b++) {
    disparities[b] = 0;
    n_pixels[b] = 0;
  }
  for (x = border; x < image_width - border; x++) {
    // determine the bin index
    b = (x - border) / bin_size;
    if (b >= n_disp_bins) { b = n_disp_bins - 1; }
    for (y = min_y; y < max_y; y++) {
      disparities[b] += in[x + y * image_width];
      n_pixels[b]++;
    }
  }
  for (b = 0; b < n_disp_bins; b++) {
    disparities[b] /= (n_pixels[b] / RESOLUTION);
  }
}


// returns two values: disparities[0] = number of pixels over threshold, disparities[1] = weighted average of x-location
void evaluate_central_disparities2(uint8_t *in, uint32_t image_width, uint32_t image_height, uint32_t *disparities,
                                   uint8_t n_disp_bins, uint8_t min_y, uint8_t max_y, uint8_t disp_threshold, uint8_t border)
{
  uint16_t b, y, x;
  //uint8_t border = 10;
  uint16_t bin_size = (image_width - 2 * border) / n_disp_bins + 1;
  uint32_t n_pixels[n_disp_bins];
  uint16_t RESOLUTION = 100;
  uint32_t mean_x = 0;

  for (b = 0; b < n_disp_bins; b++) {
    disparities[b] = 0;
    //n_pixels[b] = 0;
  }
  for (x = border; x < image_width - border; x++) {
    for (y = min_y; y < max_y; y++) {
      if (in[x + y * image_width] > disp_threshold) {
        disparities[0]++;
        mean_x += x;
      }
    }
  }
  disparities[1] = mean_x / disparities[0];

  for (b = 0; b < n_disp_bins; b++) {
    //disparities[b] +=  (uint8_t) 48;

  }
}

void evaluate_central_disparities_bins(uint8_t *in, uint32_t image_width, uint32_t image_height,
                                       uint32_t disparity_range, uint32_t *disparities, uint8_t n_disp_bins, uint8_t min_y, uint8_t max_y, uint8_t border)
{
  uint16_t b, y, x, d, sum;
  //uint8_t border = 10;
  uint16_t bin_size = (image_width - 2 * border) / n_disp_bins + 1;
  uint32_t n_disps[disparity_range * n_disp_bins];
  uint16_t RESOLUTION = 100;
  for (b = 0; b < n_disp_bins; b++) {
    disparities[b] = (uint8_t) 0;
  }
  for (d = 0; d < disparity_range * n_disp_bins; d++) {
    n_disps[d] = 0;
  }



  for (x = border; x < image_width - border; x++) {
    // determine the bin index
    b = (x - border) / bin_size;
    if (b >= n_disp_bins) { b = n_disp_bins - 1; }
    for (y = min_y; y < max_y; y++) {
      n_disps[in[x + y * image_width] + (b * disparity_range)]++;

    }
  }
  for (b = 0; b < n_disp_bins; b++) {
    sum = 0;
    for (d = disparity_range - 1; d >= 0; d--) {
      sum += n_disps[d + (b * disparity_range)];
      if (sum > (900 / n_disp_bins)) { //700
        disparities[b] = d;
        break;

      }
    }
  }


  for (b = 0; b < n_disp_bins; b++) {
    //disparities[b] +=  (uint8_t) 48;

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

void evaluate_disparities_altitude(uint8_t *in, uint32_t image_width, uint32_t image_height,
                                   uint8_t disparity_threshold, uint32_t *disparities, uint8_t altitude_levels, uint16_t x_min, uint16_t x_max,
                                   uint32_t *bad_pixels)
{
  uint32_t i, j, k, l, n_pixels, RESOLUTION, min_d;

  uint32_t level_size = image_height / altitude_levels;
  for (i = 0; i < altitude_levels; i++) {
    disparities[i] = 0;
    bad_pixels[i] = 0;
  }
  k = 0;
  l = 0;
  RESOLUTION = 100;
  n_pixels = 0;
  min_d = 3;

  for (j = 0; j < image_height; j++) {
    if (l == level_size && k < altitude_levels - 1) {
      // take the average disparity in the image region, but at a certain resolution
      // if disparities go from 0 to 10, and the resolution is 10, then the values go from 0 to 100.
      if (n_pixels != 0) {
        disparities[k] *= RESOLUTION;
        disparities[k] /= n_pixels;
      }

      bad_pixels[k] = (bad_pixels[k] * RESOLUTION) / (n_pixels + bad_pixels[k]);

      if (k < altitude_levels) {
        k++;
      }
      n_pixels = 0;
      l = 0;
    }

    l++;

    for (i = x_min; i < x_max; i++) {
      // if (in[i+j*image_width]>disparity_threshold)
      //  disparities[k]++;
      if (in[i + j * image_width] >= min_d) {
        disparities[k] += in[i + j * image_width];
        n_pixels++;
      } else {
        bad_pixels[k]++;
      }

    }
  }

  if (n_pixels != 0) {
    disparities[k] *= RESOLUTION;
    disparities[k] /= n_pixels;
  }
  bad_pixels[k] = (bad_pixels[k] * RESOLUTION) / (n_pixels + bad_pixels[k]);


  for (i = 0; i < altitude_levels; i++) {
    disparities[i] -= min_d * RESOLUTION;

  }
}

void filter_disparity_map(uint8_t *in, uint8_t diff_threshold, uint32_t image_width, uint32_t image_height,
                          uint8_t min_y, uint8_t max_y)
{
  uint16_t x, y, abs_diff;
  int diff;
  for (x = 0; x < image_width; x++) {
    for (y = min_y + 1; y < max_y - 1; y++) {
      diff = ((int) in[x + y * image_width]) - ((int) in[x + (y - 1) * image_width]);
      diff = (diff < 0) ? -diff : diff;
      abs_diff = (uint16_t)(diff);
      if (abs_diff >= diff_threshold) {
        in[x + y * image_width] = 0;
      }
      diff = ((int) in[x + y * image_width]) - ((int) in[x + (y + 1) * image_width]);
      diff = (diff < 0) ? -diff : diff;
      abs_diff = (uint16_t)(diff);
      if (abs_diff >= diff_threshold) {
        in[x + y * image_width] = 0;
      }
    }
  }
}
