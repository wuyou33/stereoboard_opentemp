// Stereo vision code

#include "stereo_vision.h"
#include "led.h"
#include "sys_time.h"

#include "stereo_math.h"
#include "image.h"
#include "arm_math.h"
#include "main_parameters.h"
//#include "../multigaze/stereoboard_parameters.h"
#include "stm32f4xx_conf.h"

/**
 * Function takes input and calculates disparity using a block
 * @author Sjoerd + Roland
 * The function performs sparse matching, based on block matching, similar to stereo_vision_sparse_block
 *
 * _two_sided means that matching is performed as follows:
 *  -> the right halve of the left image is matched with the right image
 *  -> the left halve of the right image is matched with the left image
 * This approach is used to avoid issues when matching pixels at the border that are not present in the other image
 * As a result, the center region of the produced disparity map will contain duplicate stereo matches.
 * If good calibration parameters are used, these duplicate matches will (approximately) overlap
 *
 */
uint32_t stereo_vision_sparse_block_two_sided(uint8_t *in, q7_t *out, uint32_t image_width, uint32_t image_height,
    uint32_t disparity_min,
    uint32_t disparity_range, uint32_t disparity_step, uint8_t thr1, uint8_t thr2, uint8_t min_y, uint8_t max_y, q15_t *sub_disp_histogram)
{
  // init counter: number of matched points in disparity map
  uint32_t processed_pixels = 0;

  // compute start of disparity search range, which is based on calibration offset
#ifdef FULL_CALIBRATION
  disparity_min = -DISPARITY_OFFSET_HORIZONTAL;
#else
  disparity_min = -DISPARITY_OFFSET_HORIZONTAL / RESOLUTION_FACTOR;
#endif

  // number of bytes of one image line
  uint32_t image_width_bytes = image_width * 2;

  // compute end of disparity search range, which is based on disparity_min and the defined search range
  uint32_t disparity_max = disparity_range - 1 +
                           disparity_min;   // calculate maximum disparity value based on minimum and range

  // Stereo matching parameters
  int vertical_block_size = 5;    // vertical size of SAD-window
  int horizontal_block_size = 5;  // horizontal size of SAD-window
  int GRADIENT_THRESHOLD = 20;    // gradient threshold to define if there is sufficient texture at pixel location
  int PKRN_THRESHOLD = 140; // threshold that defines if matching-cost ratio between best match and second best match is significant/sufficient [in % to deal with fixed point (120 means a difference of 20%)]

  // Some more variable inits
  int half_vertical_block_size = (vertical_block_size - 1) / 2;
  int half_horizontal_block_size = (horizontal_block_size - 1) / 2;
  int half_imageWidth = image_width / 2;

  // index parameters that are incremented individually to prevent unnecessary computations
  int idx0 = 0; // starting point of line that is currently
  int idx_P = 0; // current pixel line starting point index
  int idx_SAD = -half_vertical_block_size - 1; // SAD block index
  int idx_line = 100; // SAD block index
  uint32_t lineIndex = 0;

  int i = 0; // iterator
  int ii = 0; // iterator
  int h = 0; // iterator
  int v = 0; // iterator

  // parabole fitting
  int x1 = 0;
  int x2 = 0;
  int x3 = 0;
  int y1 = 0;
  int y2 = 0;
  int y3 = 0;
  int32_t h31 = 0;
  int32_t h21 = 0;

  int32_t sub_disp;

  // arrays that contain de-interlaced image data -> pixels from left and right images are split
  // instead of splitting the whole image before processing, this is performed in parallel with processing to reduce memory usage
  // because we use SAD block matching, multiple image lines (vertical_block_size) need to be stored
  q15_t block_left[image_width * vertical_block_size]; // array of de-interlaced image lines from left camera
  q15_t block_right[image_width * vertical_block_size]; // array of de-interlaced image lines from right camera

  q15_t line_gradient[image_width - 1]; // horizontal image gradients for a single line
  q15_t cost[disparity_range]; // array to store pixel matching costs
  q15_t sum_cost[disparity_range]; // array to store sums of pixel matching costs
  q15_t sum_cost_opt[3]; // array to store sums of pixel matching costs

  q15_t c1;
  q15_t c2;
  uint32_t  c1_i;
  uint32_t  c2_i;

  // histogram that keeps track of how frequent each sub-disparity value was assigned to a match
  arm_fill_q15(0, sub_disp_histogram, disparity_range * RESOLUTION_FACTOR);

  // if images are shifted vertically for calibration, the vertical offset needs to be taken into account
  // this is to avoid accessing pixels outside the image array
  int8_t offset = DISPARITY_OFFSET_LEFT > DISPARITY_OFFSET_RIGHT ? DISPARITY_OFFSET_LEFT : DISPARITY_OFFSET_RIGHT;

  // the maximum line to process needs to be updated based on the value of offset to avoid accessing pixels outside the image array
  // note that only the value of 'max_y' is updated. Depending on the sign of 'offset' either the left or right image
  // is 'shifted' upwards, such that matching can always start from min_y = 0
  max_y = (max_y + offset) < image_height ? max_y : image_height - offset;


  // HERE WE GO
  for (lineIndex = min_y, idx0 = min_y * image_width_bytes, idx_P = (min_y * image_width) - (half_vertical_block_size * image_width) ; lineIndex < max_y + half_vertical_block_size; lineIndex += 1, idx0 += image_width_bytes, idx_P += image_width) {

    /// update index term to store this line at the right location in the left and right de-interlaced blocks
    idx_line++;
    if (idx_line >= vertical_block_size) {
      idx_line = 0;
    }

    // compute which line in the left and right de-intelaced blocks currently represents the center image line of the SAD window
    idx_SAD++;
    if (idx_line == half_vertical_block_size) {
      idx_SAD = 0;
    }

    // de-interlace image lines and put them at correct place in the image blocks
    if (lineIndex < max_y) {
      separate_image_line_offset_block(&in[idx0], block_left, block_right, image_width_bytes, idx_line, image_width);
    }

    // Only do further processing till at least the center-line of the SAD block has been reached for the first time
    if (idx_SAD > -1) {

      // calculate image gradient of right image by subtracting with one pixel offset (only for left half of the image)
      arm_sub_q15(&block_right[idx_SAD * image_width], &block_right[(idx_SAD * image_width) + 1], line_gradient,
                  half_imageWidth);

      // make image gradients absolute such that we can look for maximum values in the next step
      arm_abs_q15(line_gradient, line_gradient, half_imageWidth);

      // go over left half of this image, leaving a border because of SAD-window size and possibly because of horizontal calibration shift
      for (i = half_horizontal_block_size + MAX(disparity_min, 0); i < half_imageWidth; i++) {

        // 'i' is the index of the pixel that is currently processed

        // check for this pixel (i) if image gradient has a local maximum   AND   value of image gradient exceeds threshold.
        // this is purely based on single image line, only to detect potential candidates for matching based in image gradients
        if (line_gradient[i] > line_gradient[i - 1] && line_gradient[i] > line_gradient[i + 1]
            && line_gradient[i] > GRADIENT_THRESHOLD) {

          // set sum vector back to zero for new window
          arm_fill_q15(0, sum_cost, disparity_range);

          // perform SAD calculations
          for (h = i - half_horizontal_block_size; h < i + half_horizontal_block_size + 1; h++) {
            for (v = 0; v < vertical_block_size; v++) {
              // compute cost between: a single pixel from the window around current pixel (i) AND a range of pixels from other image (disparity range)
              // Note that disparity_min is taken into account here, so that the disparity range is [0 disparity_max]
              arm_offset_q15(&block_left[h + (v * image_width) + disparity_min], -block_right[h + (v * image_width)], cost,
                             disparity_range);
              // obtain absolute difference
              arm_abs_q15(cost, cost, disparity_range);
              // sum results of this pixel with other pixels in this window
              arm_add_q15(cost, sum_cost, sum_cost, disparity_range);

            }
          }

          // find minimum cost
          // c1 is smallest matching cost
          // c1_i is index with smallest matching cost
          arm_min_q15(sum_cost, disparity_range, &c1, &c1_i);

          // the index corresponds directly with the disparity value
          uint8_t disparity_value = (uint8_t) c1_i;

          // store smallest matching cost
          sum_cost_opt[1] = sum_cost[c1_i];

          // overwrite minimum cost with much higher value to find second minimum
          sum_cost[c1_i] = INT16_MAX;

          /// also do these two steps for direct neighbors, to find another 'local' minimum
          if (disparity_value > 0) {
            sum_cost_opt[0] = sum_cost[c1_i - 1];
            sum_cost[c1_i - 1] = INT16_MAX;
          }
          if (disparity_value < disparity_max) {
            sum_cost_opt[2] = sum_cost[c1_i + 1];
            sum_cost[c1_i + 1] = INT16_MAX;
          }


          // find second minimum cost
          arm_min_q15(sum_cost, disparity_range, &c2, &c2_i);

          // do the matching cost ratio check
          if ((c2 * 100) / c1 > PKRN_THRESHOLD) {

            // determine index of disparity pixel in output disparity map
            uint32_t locationInBuffer = (uint32_t) idx_P + i;

            // avoid accessing pixel outside image array
            if (locationInBuffer < image_width * image_height) {
              // initially assign integer-value-disparity as result
              sub_disp = disparity_value * RESOLUTION_FACTOR;
              out[locationInBuffer] = sub_disp;//c1_i;

              // if disparity value allows, perform hyperbola fitting based on three neighboring pixels
              if (disparity_value > 0 && disparity_value < disparity_max) {

                x1 = disparity_value - 1; // left neighbor
                x2 = disparity_value;
                x3 = disparity_value + 1; // right neighbor

                // corresponding matching costs of three neighbors
                y1 = sum_cost_opt[0];
                y2 = sum_cost_opt[1];
                y3 = sum_cost_opt[2];

                // by assuming a hyperbola shape, the x-location of the hyperbola minimum is computed
                // sub_disp gets a refined value here
                h31 = (y3 - y1);
                h21 = (y2 - y1) * 4;
                sub_disp = ((h21 - h31) * RESOLUTION_FACTOR * 10) / (h21 - h31 * 2) / 10 + (x1 * RESOLUTION_FACTOR);

              }

              // add the remaining sub-pixel calibration offset now
              // (the integer value offset was taken into account during SAD window matching
              sub_disp += DISPARITY_OFFSET_HORIZONTAL % RESOLUTION_FACTOR;

              // make sure disparity value is not negative, and store in output disparity map ('out')
              if (sub_disp < 0) {
                sub_disp = 0;
              }
              out[locationInBuffer] = sub_disp;
              processed_pixels++;

              // update the disparity histogram
              if (sub_disp >= 0 && sub_disp < disparity_range * RESOLUTION_FACTOR) {
                sub_disp_histogram[sub_disp]++;
              }

            }
          }
        }
      }

      // calculate image gradient of left image by subtracting with one pixel offset
      arm_sub_q15(&block_left[idx_SAD * image_width] + half_imageWidth - 1,
                  &block_left[(idx_SAD * image_width) + half_imageWidth], line_gradient, half_imageWidth);

      // make image gradients absolute such that we can look for maximum values in the next step
      arm_abs_q15(line_gradient, line_gradient, half_imageWidth);

      // cx_diff is the difference of principal-point-x-coordinate (cx) between left and right camera
      int cx_diff_compensation = -DISPARITY_OFFSET_HORIZONTAL / RESOLUTION_FACTOR;

      for (ii = half_imageWidth + cx_diff_compensation; ii < image_width - half_horizontal_block_size; ii++) {
        i = ii - half_imageWidth;

        // check if image gradient has a local maximum AND value of image gradient exceeds threshold.
        if (line_gradient[i] > line_gradient[i - 1] && line_gradient[i] > line_gradient[i + 1]
            && line_gradient[i] > GRADIENT_THRESHOLD) {


          // set sum vector back to zero for new window
          arm_fill_q15(0, sum_cost, disparity_range);

          // perform SAD calculations
          for (h = ii - half_horizontal_block_size; h < ii + half_horizontal_block_size + 1; h++) {
            for (v = 0; v < vertical_block_size; v++) {
              // compute difference between pixel from left image with (disparity) range of pixels from right image
              arm_offset_q15(&block_right[h + (v * image_width) - disparity_max], -block_left[h + (v * image_width)], cost,
                             disparity_range);
              // obtain absolute difference
              arm_abs_q15(cost, cost, disparity_range);
              // sum results of this pixel with other pixels in this window
              arm_add_q15(cost, sum_cost, sum_cost, disparity_range);

            }
          }

          // find minimum cost
          arm_min_q15(sum_cost, disparity_range, &c1, &c1_i);
          uint8_t disparity_value = (uint8_t) c1_i;
          // put minimum cost much higher to find second minimum
          sum_cost_opt[1] = sum_cost[c1_i];
          sum_cost[c1_i] = INT16_MAX;
          // also do this for direct neighbors
          if (disparity_value > 0) {
            sum_cost_opt[0] = sum_cost[c1_i - 1];
            sum_cost[c1_i - 1] = INT16_MAX;
          }
          if (disparity_value < disparity_max) {
            sum_cost_opt[2] = sum_cost[c1_i + 1];
            sum_cost[c1_i + 1] = INT16_MAX;
          }

          // find second minimum cost
          arm_min_q15(sum_cost, disparity_range, &c2, &c2_i);

          if ((c2 * 100) / c1 > PKRN_THRESHOLD) {

            uint32_t locationInBuffer = (uint32_t)(image_width * (lineIndex - half_vertical_block_size)) + ii;
            if (locationInBuffer < image_width * image_height) {

              sub_disp = (disparity_range - 1 - disparity_value) * RESOLUTION_FACTOR;
              //out[locationInBuffer+(sub_disp/RESOLUTION_FACTOR)] = sub_disp;//c1_i;

              if (disparity_value > 0 && disparity_value < disparity_max) {
                x1 = disparity_value - 1;
                x2 = disparity_value;
                x3 = disparity_value + 1;
                y1 = sum_cost_opt[0];
                y2 = sum_cost_opt[1];
                y3 = sum_cost_opt[2];

                h31 = (y3 - y1);
                h21 = (y2 - y1) * 4;
                sub_disp = ((h21 - h31) * RESOLUTION_FACTOR * 10) / (h21 - h31 * 2) / 10 + (x1 * RESOLUTION_FACTOR);
                sub_disp = ((disparity_range - 1) * RESOLUTION_FACTOR) - sub_disp;
              }

              sub_disp += DISPARITY_OFFSET_HORIZONTAL % RESOLUTION_FACTOR;

              if (sub_disp < 0) {
                out[locationInBuffer] = 0;
                sub_disp = 0;
              } else {
                out[locationInBuffer - (sub_disp / RESOLUTION_FACTOR)] = sub_disp;
                processed_pixels++;
              }

              if (sub_disp >= 0 && sub_disp < disparity_range * RESOLUTION_FACTOR) {
                sub_disp_histogram[sub_disp]++;
              }


              //sum_counts[disparity_value]++;

            }
            //          out[superIndexInBuffer++]=c1_i;
            //        out[0]=20;
          }


        }
      }
    }
  }



  return processed_pixels;

}

/**
 * Function takes input and calculates disparity using a block, plus it stores them as a list of image features
 * @author Sjoerd + Roland
 */
uint16_t stereo_vision_sparse_block_two_sided_features(uint8_t *in, q7_t *out, float *features, uint16_t features_max_number, uint32_t image_width, uint32_t image_height,
    uint32_t disparity_min,
    uint32_t disparity_range, uint32_t disparity_step, uint8_t thr1, uint8_t thr2, uint8_t min_y, uint8_t max_y, q15_t *sub_disp_histogram)
{

  uint16_t processed_pixels = 0; // number of points in disparity map
  uint32_t locationInBuffer = 0;

  // variables for feature computation
  int feature_count = 0;
  volatile int feature_index = 0;
  int data_size = 4; // data consists of [X Y Z disparity_map_index]
  volatile float scaling_factor;
  //uint32_t image_width_2 = image_width/2;
  //uint32_t image_height_2 = image_height/2;
  float baseline = 60; // real number [mm]
  float focal_length = 120; // real number, approximate estimate, might differ per camera [px], not millimeters

  // object filtering
  float histogram_bin_size = 350.0; // [mm]
  float Y_threshold = 500; // [mm], points are only stored if their Y coordinate is smaller (Y is positive down)
  float search_range = 4000; // [mm] maximum distance to search for the object
  volatile float X, Y, Z, locationInBuffer_f;

  int histogram [100];
  int histogram_acc [100];
  volatile int histogram_index;
  for (histogram_index = 0; histogram_index < 100; histogram_index++) {
    histogram[histogram_index] = 0;
    histogram_acc[histogram_index] = 0;
  }

  disparity_min = -DISPARITY_OFFSET_HORIZONTAL / RESOLUTION_FACTOR;

  uint32_t image_width_bytes = image_width * 2;           // number of bytes of 2 interlaced image lines
  // TODO check if disparity_min is still required
  uint32_t disparity_max = disparity_range - 1 +
                           disparity_min;   // calculate maximum diisparity value based on minimum and range

  int vertical_block_size = 5; // vertical size of SAD-window
  int horizontal_block_size = 5; // horizontal size of SAD-window
  int GRADIENT_THRESHOLD = 10; // defines if image gradient indicates sufficient texture
  int PKRN_THRESHOLD =
    130; // defines if best match is significantly better than second best match [in % to deal with fixed point (120 means a difference of 20%)]

  int half_vertical_block_size = (vertical_block_size - 1) / 2;
  int half_horizontal_block_size = (horizontal_block_size - 1) / 2;

  int fakeShitImageWidth = 128;
  int half_imageWidth = fakeShitImageWidth / 2;
  int half_imageHeight = 48;
  int idx0 = 0; // line starting point index
  int idx_SAD = -1; // SAD block index
  int idx_line = 100; // SAD block index
  uint32_t lineIndex = 0;
  int linenumber = 0;
  volatile int i = 0; // iterator
  volatile int ii = 0; // iterator
  // int d = 0; // iterator
  volatile int h = 0; // iterator
  volatile int v = 0; // iterator

  // parabole fitting
  volatile int x1 = 0;
  volatile int x2 = 0;
  volatile int x3 = 0;
  volatile int y1 = 0;
  volatile int y2 = 0;
  volatile int y3 = 0;
  volatile int32_t h31 = 0;
  volatile int32_t h21 = 0;

  volatile int32_t sub_disp;

  q15_t block_left[image_width * vertical_block_size]; // block that stores multiple image lines to handle SAD windows
  q15_t block_right[image_width * vertical_block_size]; // same
  q15_t line_gradient[fakeShitImageWidth - 1]; // horizontal image gradients for a single line
  q15_t cost[disparity_range]; // array to store pixel matching costs
  q15_t sum_cost[disparity_range]; // array to store sums of pixel matching costs
  q15_t sum_cost_opt[3]; // array to store sums of pixel matching costs
  //q15_t sum_counts[disparity_range];
  // Should be passed as a parameter:
  // q15_t sub_disp_histogram[disparity_range*RESOLUTION_FACTOR];
  q15_t c1;
  q15_t c2;
  uint32_t  c1_i;
  uint32_t  c2_i;

  // set sum vector back to zero for new window
  //arm_fill_q15(0, sum_counts, disparity_range);
  arm_fill_q15(0, sub_disp_histogram, disparity_range * RESOLUTION_FACTOR);
  // check that disparity search stays within the bounds of the input image
  int8_t offset = DISPARITY_OFFSET_LEFT > DISPARITY_OFFSET_RIGHT ? DISPARITY_OFFSET_LEFT : DISPARITY_OFFSET_RIGHT;
  max_y = (max_y + offset) < image_height ? max_y : image_height - offset;
  //int superIndexInBuffer = 0;
  for (lineIndex = min_y; lineIndex < max_y; lineIndex += 1) {
    linenumber = lineIndex;
    idx0 = lineIndex * image_width_bytes;

    // update index term to store this line at the right location in the left and right blocks
    idx_line++;
    if (idx_line >= vertical_block_size) {
      idx_line = 0;
    }

    idx_SAD++;
    if (idx_line == half_vertical_block_size) {
      idx_SAD = 0;
    }

    // de-interlace image lines and put them at correct place in the image blocks
    separate_image_line_offset_block(&in[idx0], block_right, block_left, image_width_bytes, idx_line, fakeShitImageWidth);

    if (idx_SAD > -1) {

      // calculate image gradient of left image by subtracting with one pixel offset
      arm_sub_q15(&block_left[idx_SAD * fakeShitImageWidth], &block_left[(idx_SAD * fakeShitImageWidth) + 1], line_gradient,
                  half_imageWidth);

      //    // make image gradients absolute such that we can look for maximum values in the next step
      arm_abs_q15(line_gradient, line_gradient, half_imageWidth);

      for (i = half_horizontal_block_size + MAX(disparity_min, 0); i < half_imageWidth; i++) {
        // check if image gradient has a local maximum AND value of image gradient exceeds threshold.
        if (line_gradient[i] > line_gradient[i - 1] && line_gradient[i] > line_gradient[i + 1]
            && line_gradient[i] > GRADIENT_THRESHOLD) {


          // set sum vector back to zero for new window
          arm_fill_q15(0, sum_cost, disparity_range);

          // perform SAD calculations
          for (h = i - half_horizontal_block_size; h < i + half_horizontal_block_size + 1; h++) {
            for (v = 0; v < vertical_block_size; v++) {
              // compute difference between pixel from left image with (disparity) range of pixels from right image
              arm_offset_q15(&block_right[h + (v * image_width) + disparity_min], -block_left[h + (v * image_width)], cost,
                             disparity_range);
              // obtain absolute difference
              arm_abs_q15(cost, cost, disparity_range);
              // sum results of this pixel with other pixels in this window
              arm_add_q15(cost, sum_cost, sum_cost, disparity_range);

            }
          }

          // find minimum cost
          arm_min_q15(sum_cost, disparity_range, &c1, &c1_i);
          uint8_t disparity_value = (uint8_t) c1_i;
          // put minimum cost much higher to find second minimum
          sum_cost_opt[1] = sum_cost[c1_i];
          sum_cost[c1_i] = 16384;
          // also do this for direct neighbors
          if (disparity_value > 0) {
            sum_cost_opt[0] = sum_cost[c1_i - 1];
            sum_cost[c1_i - 1] = 16384;
          }
          if (disparity_value < disparity_max) {
            sum_cost_opt[2] = sum_cost[c1_i + 1];
            sum_cost[c1_i + 1] = 16384;
          }


          // find second minimum cost
          arm_min_q15(sum_cost, disparity_range, &c2, &c2_i);

          if ((c2 * 100) / c1 > PKRN_THRESHOLD) {

            locationInBuffer = (uint32_t)(fakeShitImageWidth * (lineIndex - half_vertical_block_size)) + i;
            if (locationInBuffer < 12288) {

              sub_disp = disparity_value * RESOLUTION_FACTOR;
              //out[locationInBuffer] = sub_disp;//c1_i;

              if (disparity_value > 0 && disparity_value < disparity_max) {
                x1 = disparity_value - 1;
                x2 = disparity_value;
                x3 = disparity_value + 1;
                y1 = sum_cost_opt[0];
                y2 = sum_cost_opt[1];
                y3 = sum_cost_opt[2];

                h31 = (y3 - y1);
                h21 = (y2 - y1) * 4;
                sub_disp = ((h21 - h31) * RESOLUTION_FACTOR * 10) / (h21 - h31 * 2) / 10 + (x1 * RESOLUTION_FACTOR);

              }

              sub_disp += DISPARITY_OFFSET_HORIZONTAL % RESOLUTION_FACTOR;
              if (sub_disp < 0) {
                out[locationInBuffer] = 0;
                sub_disp = 0;
              } else {
                //out[locationInBuffer] = sub_disp;
                processed_pixels++;

                if (feature_count < features_max_number) {
                  feature_index = feature_count * data_size;
                  scaling_factor = (float) baseline / sub_disp * RESOLUTION_FACTOR;
                  X = (float)(i - half_imageWidth) * scaling_factor;
                  Y = (float)(linenumber - half_imageHeight) * scaling_factor;
                  Z = focal_length * scaling_factor;
                  locationInBuffer_f = (float) locationInBuffer;
                  features[feature_index] = X; // X (positive to the right)
                  features[feature_index + 1] = Y; // Y (positive down)
                  features[feature_index + 2] = Z; // Z (positive forward
                  features[feature_index + 3] = locationInBuffer_f;

                  if (Y < Y_threshold && Z < search_range) {
                    feature_count++;
                    histogram_index = (int) features[feature_index + 2] / histogram_bin_size;
                    histogram[histogram_index]++;
                    out[locationInBuffer] = sub_disp;
                  }
                }

              }
              //sum_counts[disparity_value]++;
              if (sub_disp >= 0 && sub_disp < disparity_range * RESOLUTION_FACTOR) {
                sub_disp_histogram[sub_disp]++;
              }

            }
          }


        }
      }

      // calculate image gradient of left image by subtracting with one pixel offset
      arm_sub_q15(&block_right[idx_SAD * fakeShitImageWidth] + half_imageWidth - 1,
                  &block_right[(idx_SAD * fakeShitImageWidth) + half_imageWidth], line_gradient, half_imageWidth);

      // make image gradients absolute such that we can look for maximum values in the next step
      arm_abs_q15(line_gradient, line_gradient, half_imageWidth);

      int cx_diff_compensation = -DISPARITY_OFFSET_HORIZONTAL / RESOLUTION_FACTOR;

      for (ii = half_imageWidth + cx_diff_compensation; ii < fakeShitImageWidth - half_horizontal_block_size; ii++) {
        i = ii - half_imageWidth + 1;

        // check if image gradient has a local maximum AND value of image gradient exceeds threshold.
        if (line_gradient[i] > line_gradient[i - 1] && line_gradient[i] > line_gradient[i + 1]
            && line_gradient[i] > GRADIENT_THRESHOLD) {


          // set sum vector back to zero for new window
          arm_fill_q15(0, sum_cost, disparity_range);

          // perform SAD calculations
          for (h = ii - half_horizontal_block_size; h < ii + half_horizontal_block_size + 1; h++) {
            for (v = 0; v < vertical_block_size; v++) {
              // compute difference between pixel from left image with (disparity) range of pixels from right image
              arm_offset_q15(&block_left[h + (v * image_width) - disparity_max], -block_right[h + (v * image_width)], cost,
                             disparity_range);
              // obtain absolute difference
              arm_abs_q15(cost, cost, disparity_range);
              // sum results of this pixel with other pixels in this window
              arm_add_q15(cost, sum_cost, sum_cost, disparity_range);

            }
          }

          // find minimum cost
          arm_min_q15(sum_cost, disparity_range, &c1, &c1_i);
          uint8_t disparity_value = (uint8_t) c1_i;
          // put minimum cost much higher to find second minimum
          sum_cost_opt[1] = sum_cost[c1_i];
          sum_cost[c1_i] = 16384;
          // also do this for direct neighbors
          if (disparity_value > 0) {
            sum_cost_opt[0] = sum_cost[c1_i - 1];
            sum_cost[c1_i - 1] = 16384;
          }
          if (disparity_value < disparity_max) {
            sum_cost_opt[2] = sum_cost[c1_i + 1];
            sum_cost[c1_i + 1] = 16384;
          }

          // find second minimum cost
          arm_min_q15(sum_cost, disparity_range, &c2, &c2_i);

          if ((c2 * 100) / c1 > PKRN_THRESHOLD) {

            locationInBuffer = (uint32_t)(fakeShitImageWidth * (lineIndex - half_vertical_block_size)) + ii;
            if (locationInBuffer < 12288) {

              sub_disp = (disparity_range - 1 - disparity_value) * RESOLUTION_FACTOR;

              if (disparity_value > 0 && disparity_value < disparity_max) {
                x1 = disparity_value - 1;
                x2 = disparity_value;
                x3 = disparity_value + 1;
                y1 = sum_cost_opt[0];
                y2 = sum_cost_opt[1];
                y3 = sum_cost_opt[2];

                h31 = (y3 - y1);
                h21 = (y2 - y1) * 4;
                sub_disp = ((h21 - h31) * RESOLUTION_FACTOR * 10) / (h21 - h31 * 2) / 10 + (x1 * RESOLUTION_FACTOR);
                sub_disp = ((disparity_range - 1) * RESOLUTION_FACTOR) - sub_disp;
              }

              sub_disp += DISPARITY_OFFSET_HORIZONTAL % RESOLUTION_FACTOR;

              if (sub_disp < 0) {
                out[locationInBuffer] = 0;
                sub_disp = 0;
              } else {

                processed_pixels++;

                if (feature_count < features_max_number) {
                  feature_index = feature_count * data_size;
                  scaling_factor = (float) baseline / sub_disp * RESOLUTION_FACTOR;
                  X = (float)(i - half_imageWidth) * scaling_factor;
                  Y = (float)(linenumber - half_imageHeight) * scaling_factor;
                  Z = focal_length * scaling_factor;
                  locationInBuffer_f = (float) locationInBuffer - (sub_disp / RESOLUTION_FACTOR);
                  features[feature_index] = X; // X (positive to the right)
                  features[feature_index + 1] = Y; // Y (positive down)
                  features[feature_index + 2] = Z; // Z (positive forward
                  features[feature_index + 3] = locationInBuffer_f;

                  if (Y < Y_threshold && Z < search_range) {
                    feature_count++;
                    histogram_index = (int) features[feature_index + 2] / histogram_bin_size;
                    histogram[histogram_index]++;
                    out[locationInBuffer - (sub_disp / RESOLUTION_FACTOR)] = sub_disp;
                  }
                }

              }

              if (sub_disp >= 0 && sub_disp < disparity_range * RESOLUTION_FACTOR) {
                sub_disp_histogram[sub_disp]++;
              }

            }
          }
        }
      }
    }
  }

  // Now search for peak in histogram within search range
  int max_histogram_index = (int) search_range / histogram_bin_size;
  for (histogram_index = 1; histogram_index < max_histogram_index - 1; histogram_index++) {
    histogram_acc[histogram_index] = histogram[histogram_index - 1] + histogram[histogram_index] + histogram[histogram_index + 1];
  }

  volatile int max_histogram_value = 0;
  volatile int peak_histogram_index = 0;
  for (histogram_index = 1; histogram_index < max_histogram_index; histogram_index++) {
    if (histogram_acc[histogram_index] > max_histogram_value) {
      max_histogram_value = histogram_acc[histogram_index];
      peak_histogram_index = histogram_index;
    }
  }

  float min_Z = (float)(peak_histogram_index - 1) * histogram_bin_size;
  float max_Z = (float)(peak_histogram_index + 2) * histogram_bin_size;

  for (feature_index = 0; feature_index < feature_count * data_size; feature_index += data_size) {
    Z = features[feature_index + 2];
    if (Z < min_Z || Z > max_Z) {
      locationInBuffer = (int) features[feature_index + 3];
      out[locationInBuffer] = 0; // remove point from disparity map
    }
  }

  return processed_pixels;

}


void stereo_vision_sparse_block_fast_version(uint8_t *in, q7_t *out, uint32_t image_width, uint32_t image_height,
    uint32_t disparity_min,
    uint32_t disparity_range, uint32_t disparity_step, uint8_t thr1, uint8_t thr2, uint8_t min_y, uint8_t max_y)
{

  disparity_min = -DISPARITY_OFFSET_HORIZONTAL / RESOLUTION_FACTOR;

  uint32_t image_width_bytes = image_width * 2;           // number of bytes of 2 interlaced image lines
  // TODO check if disparity_min is still required
  uint32_t disparity_max = disparity_range - 1 +
                           disparity_min;   // calculate maximum diisparity value based on minimum and range

  int vertical_block_size = 5; // vertical size of SAD-window
  int horizontal_block_size = 5; // horizontal size of SAD-window
  int GRADIENT_THRESHOLD = 10; // defines if image gradient indicates sufficient texture
  int PKRN_THRESHOLD =
    130; // defines if best match is significantly better than second best match [in % to deal with fixed point (120 means a difference of 20%)]

  int half_vertical_block_size = (vertical_block_size - 1) / 2;
  int half_horizontal_block_size = (horizontal_block_size - 1) / 2;

  int fakeShitImageWidth = 128;
  //int half_imageWidth = fakeShitImageWidth / 2;
  int idx0 = 0; // line starting point index
  int idx_SAD = -1; // SAD block index
  int idx_line = 100; // SAD block index
  uint32_t lineIndex = 0;
  volatile int i = 0; // iterator
  //volatile int ii = 0; // iterator
  // int d = 0; // iterator
  volatile int h = 0; // iterator
  volatile int v = 0; // iterator

  // parabole fitting
  volatile int x1 = 0;
  volatile int x2 = 0;
  volatile int x3 = 0;
  volatile int y1 = 0;
  volatile int y2 = 0;
  volatile int y3 = 0;
  volatile int32_t h31 = 0;
  volatile int32_t h21 = 0;
  volatile int32_t sub_disp;

  q15_t block_left[image_width * vertical_block_size]; // block that stores multiple image lines to handle SAD windows
  q15_t block_right[image_width * vertical_block_size]; // same
  q15_t line_gradient[fakeShitImageWidth - 1]; // horizontal image gradients for a single line
  q15_t cost[disparity_range]; // array to store pixel matching costs
  q15_t sum_cost[disparity_range]; // array to store sums of pixel matching costs
  q15_t sum_cost_opt[3]; // array to store sums of pixel matching costs
  //q15_t sum_counts[disparity_range];
  q15_t c1;
  q15_t c2;
  uint32_t  c1_i;
  uint32_t  c2_i;

  // set sum vector back to zero for new window
  //arm_fill_q15(0, sum_counts, disparity_range);

  // check that disparity search stays within the bounds of the input image
  int8_t offset = DISPARITY_OFFSET_LEFT > DISPARITY_OFFSET_RIGHT ? DISPARITY_OFFSET_LEFT : DISPARITY_OFFSET_RIGHT;
  max_y = (max_y + offset) < image_height ? max_y : image_height - offset;
  //int superIndexInBuffer = 0;
  for (lineIndex = min_y; lineIndex < max_y; lineIndex += 1) {
    idx0 = lineIndex * image_width_bytes;

    // update index term to store this line at the right location in the left and right blocks
    idx_line++;
    if (idx_line >= vertical_block_size) {
      idx_line = 0;
    }

    idx_SAD++;
    if (idx_line == half_vertical_block_size) {
      idx_SAD = 0;
    }

    // de-interlace image lines and put them at correct place in the image blocks
    separate_image_line_offset_block(&in[idx0], block_right, block_left, image_width_bytes, idx_line, fakeShitImageWidth);

    if (idx_SAD > -1) {

      // calculate image gradient of left image by subtracting with one pixel offset
      arm_sub_q15(&block_left[idx_SAD * fakeShitImageWidth], &block_left[(idx_SAD * fakeShitImageWidth) + 1], line_gradient,
                  fakeShitImageWidth - 1);

      //    // make image gradients absolute such that we can look for maximum values in the next step
      arm_abs_q15(line_gradient, line_gradient, fakeShitImageWidth - 1);


      for (i = half_horizontal_block_size + MAX(disparity_min, 0);
           i < fakeShitImageWidth - half_horizontal_block_size - disparity_range; i++) {
        // check if image gradient has a local maximum AND value of image gradient exceeds threshold.
        if (line_gradient[i] > line_gradient[i - 1] && line_gradient[i] > line_gradient[i + 1]
            && line_gradient[i] > GRADIENT_THRESHOLD) {
          // set sum vector back to zero for new window
          arm_fill_q15(0, sum_cost, disparity_range);

          // perform SAD calculations
          for (h = i - half_horizontal_block_size; h < i + half_horizontal_block_size + 1; h++) {
            for (v = 0; v < vertical_block_size; v++) {
              // compute difference between pixel from left image with (disparity) range of pixels from right image
              arm_offset_q15(&block_right[h + (v * image_width) + disparity_min], -block_left[h + (v * image_width)], cost,
                             disparity_range);
              // obtain absolute difference
              arm_abs_q15(cost, cost, disparity_range);
              // sum results of this pixel with other pixels in this window
              arm_add_q15(cost, sum_cost, sum_cost, disparity_range);

            }
          }

          // find minimum cost
          arm_min_q15(sum_cost, disparity_range, &c1, &c1_i);
          uint8_t disparity_value = (uint8_t) c1_i;
          // put minimum cost much higher to find second minimum
          sum_cost_opt[1] = sum_cost[c1_i];
          sum_cost[c1_i] = 16384;
          // also do this for direct neighbors
          if (disparity_value > 0) {
            sum_cost_opt[0] = sum_cost[c1_i - 1];
            sum_cost[c1_i - 1] = 16384;
          }
          if (disparity_value < disparity_max) {
            sum_cost_opt[2] = sum_cost[c1_i + 1];
            sum_cost[c1_i + 1] = 16384;
          }


          // find second minimum cost
          arm_min_q15(sum_cost, disparity_range, &c2, &c2_i);

          if ((c2 * 100) / c1 > PKRN_THRESHOLD) {

            uint32_t locationInBuffer = (uint32_t)(fakeShitImageWidth * (lineIndex - half_vertical_block_size)) + i;
            if (locationInBuffer < 12288) {

              sub_disp = disparity_value * RESOLUTION_FACTOR;
              out[locationInBuffer] = sub_disp;//c1_i;

              if (disparity_value > 0 && disparity_value < disparity_max) {
                x1 = disparity_value - 1;
                x2 = disparity_value;
                x3 = disparity_value + 1;
                y1 = sum_cost_opt[0];
                y2 = sum_cost_opt[1];
                y3 = sum_cost_opt[2];

                h31 = (y3 - y1);
                h21 = (y2 - y1) * 4;
                sub_disp = ((h21 - h31) * RESOLUTION_FACTOR * 10) / (h21 - h31 * 2) / 10 + (x1 * RESOLUTION_FACTOR);
              }

              sub_disp += DISPARITY_OFFSET_HORIZONTAL % RESOLUTION_FACTOR;
              if (sub_disp < 0) {
                out[locationInBuffer] = 0;
              } else {
                out[locationInBuffer] = sub_disp;
              }
            } // end-if inlier in buffer
          } // end-if peak ratio threshold
        } // end-if high image gradient
      } // horizontal line iterator


    }
  }



}

uint16_t stereo_vision_sparse_block_features(uint8_t *in, q7_t *out, uint8_t *features, uint16_t features_max_number,
    uint32_t image_width, uint32_t image_height,
    uint32_t disparity_min,
    uint32_t disparity_range, uint32_t disparity_step, uint8_t thr1, uint8_t thr2, uint8_t min_y, uint8_t max_y)
{

  disparity_min = -DISPARITY_OFFSET_HORIZONTAL / RESOLUTION_FACTOR;

  volatile uint16_t feature_count = 0;

  uint32_t image_width_bytes = image_width * 2;           // number of bytes of 2 interlaced image lines
  // TODO check if disparity_min is still required
  uint32_t disparity_max = disparity_range - 1 +
                           disparity_min;   // calculate maximum diisparity value based on minimum and range

  int vertical_block_size = 5; // vertical size of SAD-window
  int horizontal_block_size = 5; // horizontal size of SAD-window
  int GRADIENT_THRESHOLD = 10; // defines if image gradient indicates sufficient texture
  int PKRN_THRESHOLD =
    130; // defines if best match is significantly better than second best match [in % to deal with fixed point (120 means a difference of 20%)]

  int half_vertical_block_size = (vertical_block_size - 1) / 2;
  int half_horizontal_block_size = (horizontal_block_size - 1) / 2;

  int fakeShitImageWidth = 128;
  //int half_imageWidth = fakeShitImageWidth / 2;
  int idx0 = 0; // line starting point index
  int idx_SAD = -1; // SAD block index
  int idx_line = 100; // SAD block index
  uint32_t lineIndex = 0;
  volatile int i = 0; // iterator
  //volatile int ii = 0; // iterator
  // int d = 0; // iterator
  volatile int h = 0; // iterator
  volatile int v = 0; // iterator

  // parabole fitting
  volatile int x1 = 0;
  volatile int x2 = 0;
  volatile int x3 = 0;
  volatile int y1 = 0;
  volatile int y2 = 0;
  volatile int y3 = 0;
  volatile int32_t h31 = 0;
  volatile int32_t h21 = 0;
  volatile int32_t sub_disp;
  volatile uint32_t sys_time_ref;
  volatile uint32_t sys_time_start = sys_time_get();
  volatile uint32_t sys_time_passed;
  volatile uint16_t feature_index;
  volatile uint8_t data = 3; // data about features: [x y d]


  q15_t block_left[image_width * vertical_block_size]; // block that stores multiple image lines to handle SAD windows
  q15_t block_right[image_width * vertical_block_size]; // same
  q15_t line_gradient[fakeShitImageWidth - 1]; // horizontal image gradients for a single line
  q15_t cost[disparity_range]; // array to store pixel matching costs
  q15_t sum_cost[disparity_range]; // array to store sums of pixel matching costs
  q15_t sum_cost_opt[3]; // array to store sums of pixel matching costs
  //q15_t sum_counts[disparity_range];
  q15_t c1;
  q15_t c2;
  uint32_t  c1_i;
  uint32_t  c2_i;

  // set sum vector back to zero for new window
  //arm_fill_q15(0, sum_counts, disparity_range);

  // check that disparity search stays within the bounds of the input image
  int8_t offset = DISPARITY_OFFSET_LEFT > DISPARITY_OFFSET_RIGHT ? DISPARITY_OFFSET_LEFT : DISPARITY_OFFSET_RIGHT;
  max_y = (max_y + offset) < image_height ? max_y : image_height - offset;
  //int superIndexInBuffer = 0;
  for (lineIndex = min_y; lineIndex < max_y; lineIndex += 1) {
    idx0 = lineIndex * image_width_bytes;

    // update index term to store this line at the right location in the left and right blocks
    idx_line++;
    if (idx_line >= vertical_block_size) {
      idx_line = 0;
    }

    idx_SAD++;
    if (idx_line == half_vertical_block_size) {
      idx_SAD = 0;
    }

    // de-interlace image lines and put them at correct place in the image blocks
    separate_image_line_offset_block(&in[idx0], block_right, block_left, image_width_bytes, idx_line, fakeShitImageWidth);

    // 26 frames per second as desired
    sys_time_ref = TIMER_TICKS_PER_SEC * lineIndex / (26 * (max_y - min_y));
    sys_time_passed = get_timer_interval(sys_time_start);

    if (idx_SAD > -1 && sys_time_passed < sys_time_ref) {

      // calculate image gradient of left image by subtracting with one pixel offset
      arm_sub_q15(&block_left[idx_SAD * fakeShitImageWidth], &block_left[(idx_SAD * fakeShitImageWidth) + 1], line_gradient,
                  fakeShitImageWidth - 1);

      //    // make image gradients absolute such that we can look for maximum values in the next step
      arm_abs_q15(line_gradient, line_gradient, fakeShitImageWidth - 1);

      //int cx_diff_compensation = -DISPARITY_OFFSET_HORIZONTAL / RESOLUTION_FACTOR;

      for (i = half_horizontal_block_size + MAX(disparity_min, 0);
           i < fakeShitImageWidth - half_horizontal_block_size - disparity_range; i++) {
        // check if image gradient has a local maximum AND value of image gradient exceeds threshold.
        if (line_gradient[i] > line_gradient[i - 1] && line_gradient[i] > line_gradient[i + 1]
            && line_gradient[i] > GRADIENT_THRESHOLD) {
          // set sum vector back to zero for new window
          arm_fill_q15(0, sum_cost, disparity_range);

          // perform SAD calculations
          for (h = i - half_horizontal_block_size; h < i + half_horizontal_block_size + 1; h++) {
            for (v = 0; v < vertical_block_size; v++) {
              // compute difference between pixel from left image with (disparity) range of pixels from right image
              arm_offset_q15(&block_right[h + (v * image_width) + disparity_min], -block_left[h + (v * image_width)], cost,
                             disparity_range);
              // obtain absolute difference
              arm_abs_q15(cost, cost, disparity_range);
              // sum results of this pixel with other pixels in this window
              arm_add_q15(cost, sum_cost, sum_cost, disparity_range);

            }
          }

          // find minimum cost
          arm_min_q15(sum_cost, disparity_range, &c1, &c1_i);
          uint8_t disparity_value = (uint8_t) c1_i;
          // put minimum cost much higher to find second minimum
          sum_cost_opt[1] = sum_cost[c1_i];
          sum_cost[c1_i] = 16384;
          // also do this for direct neighbors
          if (disparity_value > 0) {
            sum_cost_opt[0] = sum_cost[c1_i - 1];
            sum_cost[c1_i - 1] = 16384;
          }
          if (disparity_value < disparity_max) {
            sum_cost_opt[2] = sum_cost[c1_i + 1];
            sum_cost[c1_i + 1] = 16384;
          }

          // find second minimum cost
          arm_min_q15(sum_cost, disparity_range, &c2, &c2_i);

          if ((c2 * 100) / c1 > PKRN_THRESHOLD) {

            uint32_t locationInBuffer = (uint32_t)(fakeShitImageWidth * (lineIndex - half_vertical_block_size)) + i;
            if (locationInBuffer < 12288) {

              sub_disp = disparity_value * RESOLUTION_FACTOR;
              out[locationInBuffer] = sub_disp;//c1_i;

              if (disparity_value > 0 && disparity_value < disparity_max) {
                x1 = disparity_value - 1;
                x2 = disparity_value;
                x3 = disparity_value + 1;
                y1 = sum_cost_opt[0];
                y2 = sum_cost_opt[1];
                y3 = sum_cost_opt[2];

                h31 = (y3 - y1);
                h21 = (y2 - y1) * 4;
                sub_disp = ((h21 - h31) * RESOLUTION_FACTOR * 10) / (h21 - h31 * 2) / 10 + (x1 * RESOLUTION_FACTOR);
                sub_disp = ((disparity_range - 1) * RESOLUTION_FACTOR) - sub_disp;
              }

              sub_disp += DISPARITY_OFFSET_HORIZONTAL % RESOLUTION_FACTOR;

              if (sub_disp < 0) {
                out[locationInBuffer] = 0;
              } else {
                //out[locationInBuffer] = sub_disp;

                if (feature_count < features_max_number) {
                  feature_index = feature_count * data;
                  features[feature_index] = (uint8_t) lineIndex; // line number
                  features[feature_index + 1] = (uint8_t) i; // column number
                  features[feature_index + 2] = (uint8_t) sub_disp; // disparity value
                  //features[feature_index+3] = (uint8_t) 0; // selection indicator
                  feature_count++;
                }

              }
            } // end-if inlier in buffer
          } // end-if peak ratio threshold
        } // end-if high image gradient
      } // horizontal line iterator


    }
  }

  return feature_count;

}

void stereo_vision_sparse_block(uint8_t *in, q7_t *out, uint32_t image_width, uint32_t image_height,
                                uint32_t disparity_min,
                                uint32_t disparity_range, uint32_t disparity_step, uint8_t thr1, uint8_t thr2, uint8_t min_y, uint8_t max_y)
{
  uint32_t image_width_bytes = image_width * 2;           // number of bytes of 2 interlaced image lines
  // TODO check if disparity_min is still required
  uint32_t disparity_max = disparity_range - 1 +
                           disparity_min;   // calculate maximum diisparity value based on minimum and range

  int vertical_block_size = 5; // vertical size of SAD-window
  int horizontal_block_size = 5; // horizontal size of SAD-window
  int GRADIENT_THRESHOLD = 5; // defines if image gradient indicates sufficient texture
  int PKRN_THRESHOLD =
    130; // defines if best match is significantly better than second best match [in % to deal with fixed point (120 means a difference of 20%)]

  int half_vertical_block_size = (vertical_block_size - 1) / 2;
  int half_horizontal_block_size = (horizontal_block_size - 1) / 2;

  int fakeShitImageWidth = 128;
  int idx0 = 0; // line starting point index
  int idx_SAD = -1; // SAD block index
  int idx_line = 100; // SAD block index
  uint32_t lineIndex = 0;
  volatile int i = 0; // iterator
  // int d = 0; // iterator
  volatile int h = 0; // iterator
  volatile int v = 0; // iterator


  q15_t block_left[image_width * vertical_block_size]; // block that stores multiple image lines to handle SAD windows
  q15_t block_right[image_width * vertical_block_size]; // same
  q15_t line_gradient[fakeShitImageWidth - 1]; // horizontal image gradients for a single line
  q15_t cost[disparity_range]; // array to store pixel matching costs
  q15_t sum_cost[disparity_range]; // array to store sums of pixel matching costs
  //q15_t sum_counts[disparity_range];
  q15_t c1;
  q15_t c2;
  uint32_t  c1_i;
  uint32_t  c2_i;

  // set sum vector back to zero for new window
  //arm_fill_q15(0, sum_counts, disparity_range);

  // check that disparity search stays within the bounds of the input image
  int8_t offset = DISPARITY_OFFSET_LEFT > DISPARITY_OFFSET_RIGHT ? DISPARITY_OFFSET_LEFT : DISPARITY_OFFSET_RIGHT;
  max_y = (max_y + offset) < image_height ? max_y : image_height - offset;
  //int superIndexInBuffer = 0;
  for (lineIndex = min_y; lineIndex < max_y; lineIndex++) {
    idx0 = lineIndex * image_width_bytes; // starting point of line in image buffer

    // update index term to store this line at the right location in the left and right blocks
    idx_line++;
    if (idx_line >= vertical_block_size) {
      idx_line = 0;
    }

    idx_SAD++;
    if (idx_line == half_vertical_block_size) {
      idx_SAD = 0;
    }

    // de-interlace image lines and put them at right place in the image blocks
    separate_image_line_offset_block(&in[idx0], block_right, block_left, image_width_bytes, idx_line,
                                     fakeShitImageWidth - 1);

    if (idx_SAD > -1) {

      // calculate image gradient of left image by subtracting with one pixel offset
      arm_sub_q15(&block_left[idx_SAD * fakeShitImageWidth], &block_left[(idx_SAD * fakeShitImageWidth) + 1], line_gradient,
                  fakeShitImageWidth - 1);

      //    // make image gradients absolute such that we can look for maximum values in the next step
      arm_abs_q15(line_gradient, line_gradient, fakeShitImageWidth - 1);

      for (i = half_horizontal_block_size; i < image_width - half_horizontal_block_size - disparity_max - 2; i++) {
        // check if image gradient has a local maximum AND value of image gradient exceeds threshold.
        if (line_gradient[i] > line_gradient[i - 1] && line_gradient[i] > line_gradient[i + 1]
            && line_gradient[i] > GRADIENT_THRESHOLD) {
          // set sum vector back to zero for new window
          arm_fill_q15(0, sum_cost, disparity_range);

          // perform SAD calculations
          for (h = i - half_horizontal_block_size; h < i + half_horizontal_block_size + 1; h++) {
            for (v = 0; v < vertical_block_size; v++) {
              // compute difference between pixel from left image with (disparity) range of pixels from right image
              arm_offset_q15(&block_right[h + (v * image_width)], -block_left[h + (v * image_width)], cost, disparity_range);
              // obtain absolute difference
              arm_abs_q15(cost, cost, disparity_range);
              // sum results of this pixel with other pixels in this window
              arm_add_q15(cost, sum_cost, sum_cost, disparity_range);

            }
          }

          // find minimum cost
          arm_min_q15(sum_cost, disparity_range, &c1, &c1_i);
          uint8_t disparity_value = (uint8_t) c1_i;
          // put minimum cost much higher to find second minimum
          sum_cost[c1_i] = 16384;
          // find second minimum cost
          arm_min_q15(sum_cost, disparity_range, &c2, &c2_i);

          if ((c2 * 100) / c1 > PKRN_THRESHOLD) {

            uint32_t locationInBuffer = (uint32_t)(fakeShitImageWidth * (lineIndex - half_vertical_block_size)) + i;
            if (locationInBuffer < 12288) {
              out[locationInBuffer] = disparity_value;//c1_i;
              //sum_counts[disparity_value]++;

            }
            //          out[superIndexInBuffer++]=c1_i;
            //        out[0]=20;
          }


        }
      }
    }
  }

  /*
  int sum_disparities = 0;
  for ( d = disparity_range-1; d>CLOSE_BOUNDARY; d--)
  {
    sum_disparities += sum_counts[d];
  }

  if(sum_disparities>5){
    led_set();
  }
  else
  {
    led_clear();
  }
  */

}


/* stereo_vision:
 * takes input image in, which contains both the left and the right image
 * generates disparity image out
 *
 * parameters:
 * image_width
 * image_height
 * disparity_range (20 is a reasonable value, should be multiple of 4 for efficiency)
 * thr1: threshold for 1st check (4)
 * thr2: threshold for 2nd check (5)
 * */
void stereo_vision_Kirk(uint8_t *in, q7_t *out, uint32_t image_width, uint32_t image_height, uint32_t disparity_min,
                        uint32_t disparity_range, uint32_t disparity_step, uint8_t thr1, uint8_t thr2 __attribute__((unused)), uint8_t min_y,
                        uint8_t max_y)
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

  q7_t max_length1 = 0;
  q7_t max_length2 = 0;

  uint32_t d = 0;
  uint32_t i = 0;
  uint32_t i2 = 0;
  uint32_t l = 0;
  uint32_t cnt0 = 0;
  uint32_t cnt1 = 0;

  q7_t check_temp[disparity_range];
  q7_t seqLength = 0;

  // check that disparity search stays within the bounds of the input image
  int8_t offset = DISPARITY_OFFSET_LEFT > DISPARITY_OFFSET_RIGHT ? DISPARITY_OFFSET_LEFT : DISPARITY_OFFSET_RIGHT;
  max_y = max_y + offset < image_height ? max_y : image_height - offset;

  // run through each line of image
  for (l = min_y; l < max_y; l++) {
    cnt0 = l * image_width_bytes;

    separate_image_line_offset(&in[cnt0], line1, line2, image_width_bytes);

    // the disparities will be put in upd_disps1:
    upd_disps1 = &out[l * image_width];

    // reinitialize temp arrays
    arm_fill_q7(0, check1, image_width * disparity_range);
    arm_fill_q7(0, check2, image_width * disparity_range);
    arm_fill_q7(0, check_temp, disparity_range);

    for (i = disparity_max; i < image_width; i++) {
      arm_offset_q15(&line2[i - disparity_max], -line1[i], cost_per_pixel, disparity_range);  // compute cost
      arm_abs_q15(cost_per_pixel, cost_per_pixel, disparity_range);   // convert to absolute error

      cnt1 = 0;
      for (d = disparity_min; d <= disparity_max; d += disparity_step) {
        if (cost_per_pixel[disparity_max - d] > thr1 || i == image_width - 1) { // check if pixel cost exceeds error threshold
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
    } // go to next pixel

    // reinitialize max disparity temp arrays
    arm_fill_q7(0, max_disps1, image_width);
    arm_fill_q7(0, max_disps2, image_width);

    for (i = disparity_max, i2 = 0; i < image_width; i++, i2++) {
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
      } // check next disparity value
    } // go to next pixel

    // set output to disparity_max
    arm_fill_q7(disparity_max, upd_disps1 + disparity_max, image_width - 2 * disparity_max);

    for (i = disparity_max; i < image_width - disparity_max; i++) {
      // project the disparity map of the second image using initial disparities on the first image
      if (upd_disps1[i + max_disps2[i]] == disparity_max) {
        upd_disps1[i + max_disps2[i]] = max_disps2[i];
      }

      // compare the initial disparity map of the first image to the projection of the second image, choose smallest disparity
      if (max_disps1[i] < upd_disps1[i]) {
        upd_disps1[i] = max_disps1[i];
      }
    } // go to next pixel
  } // go to next line


#if SMOOTH_DISPARITY_MAP

  int bufferIndex = 0;
  uint8_t cleaner_image_buffer[image_width * image_height];
  for (bufferIndex = 0; bufferIndex < image_width * image_height; bufferIndex++) {
    //out[bufferIndex]=10;
    if (bufferIndex > 2 * image_width && bufferIndex < (image_width * image_height) - 2 * image_width_bytes) {
      uint8_t pixel = out[bufferIndex];
      uint8_t above = out[bufferIndex - image_width];

      uint8_t underPixel = out[bufferIndex + image_width];
      cleaner_image_buffer[bufferIndex] = MIN(MIN(above, underPixel), pixel);
    } else {
      cleaner_image_buffer[bufferIndex] = out[bufferIndex];
    }

    // Write it to the out buffer when we surely do not access the outbuffer anymore
    if (bufferIndex > 2 * image_width) {
      out[bufferIndex - 2 * image_width] = cleaner_image_buffer[bufferIndex - 2 * image_width];
    }
  }

#endif
}


// TODO is this algorithm still used? what do we want to do with it?
void stereo_vision(uint8_t *in, q7_t *out, uint32_t image_width, uint32_t image_height, uint32_t disparity_range,
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

  q15_t min_cost;
  uint32_t min_index;

  uint16_t start1;
  q7_t length1;
  q7_t disp1;

  uint16_t start2;
  q7_t length2;
  q7_t disp2;

  uint32_t d = 0;
  uint32_t i;
  uint32_t i2;
  uint32_t l;
  uint32_t cnt0;
  uint32_t cnt1;

  cnt0 = min_y * image_width_bytes;
  for (l = min_y; l < max_y; l++) {

    // TODO same as above... why is this -5?
    if (l < image_height - 5) {
      // separate the image line into a left and right one (line1, lin2 respectively):
      separate_image_line_offset(&in[cnt0], line1, line2, image_width_bytes);
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

// TODO do we still want to use this stereo vision algorithm???
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
  q15_t expDens[256] = { 1, 121, 61, 41, 31, 25, 21, 227, 8, 201, 241, 197, 171, 77, 197, 217, 47, 77, 247, 70, 127, 149, 215, 201, 91, 135, 91, 218, 59, 227, 61, 57, 75, 26, 178, 224, 215, 239, 211, 136, 224, 101, 147, 43, 127, 199, 127, 116, 227, 43, 237, 124, 226, 179, 91, 5, 89, 181, 234, 131, 193, 199, 163, 20, 154, 65, 3, 168, 205, 60, 228, 160, 83, 233, 230, 178, 181, 83, 246, 97, 239, 27, 251, 255, 73, 235, 239, 81, 13, 119, 121, 237, 139, 245, 206, 151, 213, 141, 169, 151, 90, 253, 104, 167, 249, 236, 158, 119, 121, 203, 244, 159, 97, 217, 234, 34, 159, 246, 243, 109, 133, 248, 191, 233, 79, 249, 49, 191, 76, 249, 227, 71, 253, 156, 28, 237, 164, 206, 249, 71, 165, 241, 32, 206, 254, 213, 57, 197, 165, 12, 134, 223, 63, 205, 13, 225, 175, 178, 181, 184, 187, 161, 253, 227, 200, 219, 175, 97, 148, 184, 17, 121, 211, 143, 109, 240, 94, 172, 136, 79, 241, 143, 187, 190, 236, 131, 111, 158, 252, 70, 166, 193, 221, 249, 203, 129, 236, 80, 244, 193, 28, 57, 29, 59, 30, 61, 31, 63, 32, 228, 232, 101, 137,
                         174, 177, 36, 183, 186, 227, 77, 235, 199, 202, 247, 251, 85, 173, 44, 134, 91, 231, 47, 239, 243, 247, 201, 51, 52, 211, 161, 164, 111, 113, 172, 175, 178, 181, 184, 187, 127, 129, 197, 200, 203, 69, 70
                       };



  disps = &out[0];

  arm_fill_q7(0, disps, image_size);


  for (l = min_y; l < max_y + 1; l++) {
    // separate the image line into a left and right one (line1, line2 respectively)
    separate_image_line_offset(&in[l * image_width_bytes], line1, line2, image_width_bytes);

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
  // uint16_t bin_size = (image_width - 2 * border) / n_disp_bins + 1;
  //uint32_t n_pixels[n_disp_bins];
  //uint16_t RESOLUTION = 100;
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
  // uint16_t RESOLUTION = 100;
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

uint32_t evaluate_disparities_droplet(uint8_t *in, uint32_t image_width, uint32_t image_height,
                                      uint8_t max_height_check)
{
  volatile int x, y;

  volatile uint32_t disparities_close = 0;
  //uint8_t maximum_disparities[IMAGE_WIDTH] = { 29,28,28,28,27,27,26,26,25,25,25,24,24,24,23,23,23,22,22,22,22,21,21,21,21,20,20,20,20,20,19,19,19,19,19,18,18,18,18,18,18,18,17,17,17,17,17,17,17,17,16,16,16,16,16,16,16,16,16,16,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,16,16,16,16,16,16,16,16,17,17,17,17,17,18,18,18,19,19,19,19,20,20,20,21 };
  //uint8_t maximum_disparities[IMAGE_WIDTH] = { 25,24,24,23,23,23,22,22,22,21,21,21,20,20,20,20,19,19,19,19,19,18,18,18,18,18,18,18,17,17,17,17,17,17,17,17,16,16,16,16,16,16,16,16,16,16,16,16,16,16,16,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,16,16,16,16,16,16,16,16,16,16,16,16,16,16,16,17,17,17,17,17,17,17,17,18,18,18,18,18,18,18,19,19,19,19,19,20,20,20,20,21,21,21,22,22,22,23,23,23,24,24,25,25};
  uint8_t maximum_disparities[IMAGE_WIDTH] = { 31, 30, 30, 29, 29, 29, 28, 28, 28, 27, 27, 27, 26, 26, 26, 26, 25, 25, 25, 25, 25, 24, 24, 24, 24, 24, 24, 24, 23, 23, 23, 23, 23, 23, 23, 23, 22, 22, 22, 22, 22, 22, 22, 22, 22, 22, 22, 22, 22, 22, 22, 21, 21, 21, 21, 21, 21, 21, 21, 21, 21, 21, 21, 21, 21, 21, 21, 21, 21, 21, 21, 21, 21, 21, 21, 21, 22, 22, 22, 22, 22, 22, 22, 22, 22, 22, 22, 22, 22, 22, 22, 23, 23, 23, 23, 23, 23, 23, 23, 24, 24, 24, 24, 24, 24, 24, 25, 25, 25, 25, 25, 26, 26, 26, 26, 27, 27, 27, 28, 28, 28, 29, 29, 29, 30, 30, 31, 31};
  for (x = 0; x < image_width; x++) {
    for (y = 0; y < max_height_check; y++) {
      if (in[x + (y * image_width)] >= maximum_disparities[x]) {
        disparities_close++;
      }
    }
  }

  return disparities_close;
}

uint32_t evaluate_disparities_droplet_low_texture(struct image_t *disparity_map,
    uint32_t *count_disps_left, uint32_t *count_disps_right, uint32_t *hist_obs_sum)
{
#define HISTOGRAM_RESOLUTION 100       // [ms]
#define DROPLET_HISTOGRAM_LEN (TIMER_PERIOD / (TIMER_TICKS_PER_MSEC * HISTOGRAM_RESOLUTION))
  static const int32_t droplet_hist_len = DROPLET_HISTOGRAM_LEN;

  static uint32_t prev_histogram_bin = 0;
  static uint32_t curr_histogram_bin = 0;
  static uint32_t obst_time_histogram[DROPLET_HISTOGRAM_LEN] = {0};

  curr_histogram_bin = sys_time_get() / (TIMER_TICKS_PER_MSEC * HISTOGRAM_RESOLUTION);

  // reset bins in the past
  int32_t i;
  uint32_t num2clear = (curr_histogram_bin - prev_histogram_bin + droplet_hist_len) % droplet_hist_len;
  for (i = 0; i < num2clear; i++) {
    obst_time_histogram[(prev_histogram_bin + i) % droplet_hist_len] = 0;
  }
  prev_histogram_bin = curr_histogram_bin;

  uint8_t *disp_image = (uint8_t *)disparity_map->buf;

  int32_t x, y;
  int32_t image_width_2 = disparity_map->w / 2;

  // Look-up table for disparity to distance [mm] conversion using focal length of 115mm
  static const float disp2dist[180] = {0, 41400.000, 20700.000, 13800.000, 10350.000, 8280.000, 6900.000, 5914.286, 5175.000, 4600.000, 4140.000, 3763.636, 3450.000, 3184.615, 2957.143, 2760.000, 2587.500, 2435.294, 2300.000, 2178.947, 2070.000, 1971.429, 1881.818, 1800.000, 1725.000, 1656.000, 1592.308, 1533.333, 1478.571, 1427.586, 1380.000, 1335.484, 1293.750, 1254.545, 1217.647, 1182.857, 1150.000, 1118.919, 1089.474, 1061.538, 1035.000, 1009.756, 985.714, 962.791, 940.909, 920.000, 900.000, 880.851, 862.500, 844.898, 828.000, 811.765, 796.154, 781.132, 766.667, 752.727, 739.286, 726.316, 713.793, 701.695, 690.000, 678.689, 667.742, 657.143, 646.875, 636.923, 627.273, 617.910, 608.824, 600.000, 591.429, 583.099, 575.000, 567.123, 559.459, 552.000, 544.737, 537.662, 530.769, 524.051, 517.500, 511.111, 504.878, 498.795, 492.857, 487.059, 481.395, 475.862, 470.455, 465.169, 460.000, 454.945, 450.000, 445.161, 440.426, 435.789, 431.250, 426.804, 422.449, 418.182, 414.000, 409.901, 405.882, 401.942, 398.077, 394.286, 390.566, 386.916, 383.333, 379.817, 376.364, 372.973, 369.643, 366.372, 363.158, 360.000, 356.897, 353.846, 350.847, 347.899, 345.000, 342.149, 339.344, 336.585, 333.871, 331.200, 328.571, 325.984, 323.438, 320.930, 318.462, 316.031, 313.636, 311.278, 308.955, 306.667, 304.412, 302.190, 300.000, 297.842, 295.714, 293.617, 291.549, 289.510, 287.500, 285.517, 283.562, 281.633, 279.730, 277.852, 276.000, 274.172, 272.368, 270.588, 268.831, 267.097, 265.385, 263.694, 262.025, 260.377, 258.750, 257.143, 255.556, 253.988, 252.439, 250.909, 249.398, 247.904, 246.429, 244.970, 243.529, 242.105, 240.698, 239.306, 237.931, 236.571, 235.227, 233.898, 232.584, 231.285 };

  uint32_t disparities_close = 0;
  // max_Y = 500 mm
  // static const uint8_t maximum_disparities[12288] = { 34,34,34,34,34,34,34,34,34,34,34,34,34,34,34,34,34,34,34,34,34,34,34,34,34,34,34,34,34,34,34,34,34,34,34,34,34,34,34,34,34,34,34,34,34,34,34,34,34,34,34,34,34,34,34,34,34,34,34,34,34,34,34,34,34,34,34,34,34,34,34,34,34,34,34,34,34,34,34,34,34,34,34,34,34,34,34,34,34,34,34,34,34,34,34,34,34,34,34,34,34,34,34,34,34,34,34,34,34,34,34,34,34,34,34,34,34,34,34,34,34,34,34,34,34,34,34,34,34,34,34,34,34,34,34,34,34,34,34,34,34,34,34,34,34,34,34,34,34,34,34,34,34,34,34,34,34,34,34,34,34,34,34,34,34,34,34,34,34,34,34,34,34,34,34,34,34,34,34,34,34,34,34,34,34,34,34,34,34,34,34,34,34,34,34,34,34,34,34,34,34,34,34,34,34,34,34,34,34,34,34,34,34,34,34,34,34,34,34,34,34,34,34,34,34,34,34,34,34,34,34,34,34,34,34,34,34,34,34,34,34,34,34,34,34,34,34,34,34,34,34,34,34,34,34,34,33,33,33,33,33,33,33,33,33,33,33,33,33,33,33,33,33,33,33,33,33,33,33,33,33,33,33,33,33,33,33,33,33,33,33,33,33,33,33,33,33,33,33,33,33,33,33,33,33,33,33,33,33,33,33,33,33,33,33,33,33,33,33,33,33,33,33,33,33,33,33,33,33,33,33,33,33,33,33,33,33,33,33,33,33,33,33,33,33,33,33,33,33,33,33,33,33,33,33,33,33,33,33,33,33,33,33,33,33,33,33,33,33,33,33,33,33,33,33,33,33,33,33,33,33,33,33,33,32,32,32,32,32,32,32,32,32,32,32,32,32,32,32,32,32,32,32,32,32,32,32,32,32,32,32,32,32,32,32,32,32,32,32,32,32,32,32,32,32,32,32,32,32,32,32,32,32,32,32,32,32,32,32,32,32,32,32,32,32,32,32,32,32,32,32,32,32,32,32,32,32,32,32,32,32,32,32,32,32,32,32,32,32,32,32,32,32,32,32,32,32,32,32,32,32,32,32,32,32,32,32,32,32,32,32,32,32,32,32,32,32,32,32,32,32,32,32,32,32,32,32,32,32,32,32,32,31,31,31,31,31,31,31,31,31,31,31,31,31,31,31,31,31,31,31,31,31,31,31,31,31,31,31,31,31,31,31,31,31,31,31,31,31,31,31,31,31,31,31,31,31,31,31,31,31,31,31,31,31,31,31,31,31,31,31,31,31,31,31,31,31,31,31,31,31,31,31,31,31,31,31,31,31,31,31,31,31,31,31,31,31,31,31,31,31,31,31,31,31,31,31,31,31,31,31,31,31,31,31,31,31,31,31,31,31,31,31,31,31,31,31,31,31,31,31,31,31,31,31,31,31,31,31,31,31,31,31,31,31,31,31,31,31,31,31,31,31,31,31,31,31,31,31,31,31,31,31,31,31,31,31,31,31,31,31,31,31,31,31,31,31,31,31,31,31,31,31,31,31,31,31,31,31,31,31,31,31,31,31,31,31,31,31,31,31,31,31,31,31,31,31,31,31,31,31,31,31,31,31,31,31,31,31,31,31,31,31,31,31,31,31,31,31,31,31,31,31,31,31,31,31,31,31,31,31,31,31,31,31,31,31,31,31,31,31,31,31,31,31,31,31,31,31,31,31,31,31,31,31,31,31,31,30,30,30,30,30,30,30,30,30,30,30,30,30,30,30,30,30,30,30,30,30,30,30,30,30,30,30,30,30,30,30,30,30,30,30,30,30,30,30,30,30,30,30,30,30,30,30,30,30,30,30,30,30,30,30,30,30,30,30,30,30,30,30,30,30,30,30,30,30,30,30,30,30,30,30,30,30,30,30,30,30,30,30,30,30,30,30,30,30,30,30,30,30,30,30,30,30,30,30,30,30,30,30,30,30,30,30,30,30,30,30,30,30,30,30,30,30,30,30,30,30,30,30,30,30,30,30,30,29,29,29,29,29,29,29,29,29,29,29,29,29,29,29,29,29,29,29,29,29,29,29,29,29,29,29,29,29,29,29,29,29,29,29,29,29,29,29,29,29,29,29,29,29,29,29,29,29,29,29,29,29,29,29,29,29,29,29,29,29,29,29,29,29,29,29,29,29,29,29,29,29,29,29,29,29,29,29,29,29,29,29,29,29,29,29,29,29,29,29,29,29,29,29,29,29,29,29,29,29,29,29,29,29,29,29,29,29,29,29,29,29,29,29,29,29,29,29,29,29,29,29,29,29,29,29,29,29,29,29,29,29,29,29,29,29,29,29,29,29,29,29,29,29,29,29,29,29,29,29,29,29,29,29,29,29,29,29,29,29,29,29,29,29,29,29,29,29,29,29,29,29,29,29,29,29,29,29,29,29,29,29,29,29,29,29,29,29,29,29,29,29,29,29,29,29,29,29,29,29,29,29,29,29,29,29,29,29,29,29,29,29,29,29,29,29,29,29,29,29,29,29,29,29,29,29,29,29,29,29,29,29,29,29,29,29,29,29,29,29,29,29,29,29,29,29,29,29,29,29,29,29,29,29,29,28,28,28,28,28,28,28,28,28,28,28,28,28,28,28,28,28,28,28,28,28,28,28,28,28,28,28,28,28,28,28,28,28,28,28,28,28,28,28,28,28,28,28,28,28,28,28,28,28,28,28,28,28,28,28,28,28,28,28,28,28,28,28,28,28,28,28,28,28,28,28,28,28,28,28,28,28,28,28,28,28,28,28,28,28,28,28,28,28,28,28,28,28,28,28,28,28,28,28,28,28,28,28,28,28,28,28,28,28,28,28,28,28,28,28,28,28,28,28,28,28,28,28,28,28,28,28,28,27,27,27,27,27,27,27,27,27,27,27,27,27,27,27,27,27,27,27,27,27,27,27,27,27,27,27,27,27,27,27,27,27,27,27,27,27,27,27,27,27,27,27,27,27,27,27,27,27,27,27,27,27,27,27,27,27,27,27,27,27,27,27,27,27,27,27,27,27,27,27,27,27,27,27,27,27,27,27,27,27,27,27,27,27,27,27,27,27,27,27,27,27,27,27,27,27,27,27,27,27,27,27,27,27,27,27,27,27,27,27,27,27,27,27,27,27,27,27,27,27,27,27,27,27,27,27,27,26,26,26,26,26,26,26,26,26,26,26,26,26,26,26,26,26,26,26,26,26,26,26,26,26,26,26,26,26,26,26,26,26,26,26,26,26,26,26,26,26,26,26,26,26,26,26,26,26,26,26,26,26,26,26,26,26,26,26,26,26,26,26,26,26,26,26,26,26,26,26,26,26,26,26,26,26,26,26,26,26,26,26,26,26,26,26,26,26,26,26,26,26,26,26,26,26,26,26,26,26,26,26,26,26,26,26,26,26,26,26,26,26,26,26,26,26,26,26,26,26,26,26,26,26,26,26,26,26,26,26,26,26,26,26,26,26,26,26,26,26,26,26,26,26,26,26,26,26,26,26,26,26,26,26,26,26,26,26,26,26,26,26,26,26,26,26,26,26,26,26,26,26,26,26,26,26,26,26,26,26,26,26,26,26,26,26,26,26,26,26,26,26,26,26,26,26,26,26,26,26,26,26,26,26,26,26,26,26,26,26,26,26,26,26,26,26,26,26,26,26,26,26,26,26,26,26,26,26,26,26,26,26,26,26,26,26,26,26,26,26,26,26,26,26,26,26,26,26,26,26,26,26,26,26,26,25,25,25,25,25,25,25,25,25,25,25,25,25,25,25,25,25,25,25,25,25,25,25,25,25,25,25,25,25,25,25,25,25,25,25,25,25,25,25,25,25,25,25,25,25,25,25,25,25,25,25,25,25,25,25,25,25,25,25,25,25,25,25,25,25,25,25,25,25,25,25,25,25,25,25,25,25,25,25,25,25,25,25,25,25,25,25,25,25,25,25,25,25,25,25,25,25,25,25,25,25,25,25,25,25,25,25,25,25,25,25,25,25,25,25,25,25,25,25,25,25,25,25,25,25,25,25,25,25,24,24,24,24,24,24,24,24,24,24,24,24,24,24,24,24,24,24,24,24,24,24,24,24,24,24,24,24,24,24,24,24,24,24,24,24,24,24,24,24,24,24,24,24,24,24,24,24,24,24,24,24,24,24,24,24,24,24,24,24,24,24,24,24,24,24,24,24,24,24,24,24,24,24,24,24,24,24,24,24,24,24,24,24,24,24,24,24,24,24,24,24,24,24,24,24,24,24,24,24,24,24,24,24,24,24,24,24,24,24,24,24,24,24,24,24,24,24,24,24,24,24,24,24,24,25,25,25,24,24,24,24,24,24,24,24,24,24,24,24,24,24,24,24,24,24,24,24,24,24,24,24,24,24,24,24,24,24,24,24,24,24,24,24,24,24,24,24,24,24,24,24,24,24,24,24,24,24,24,24,24,24,24,24,24,24,24,24,24,24,24,24,24,24,24,24,24,24,24,24,24,24,24,24,24,24,24,24,24,24,24,24,24,24,24,24,24,24,24,24,24,24,24,24,24,24,24,24,24,24,24,24,24,24,24,24,24,24,24,24,24,24,24,24,24,24,24,24,24,24,24,24,24,25,25,25,24,24,23,23,23,23,23,23,23,23,23,23,23,23,23,23,23,23,23,23,23,23,23,23,23,23,23,23,23,23,23,23,23,23,23,23,23,23,23,23,23,23,23,23,23,23,23,23,23,23,23,23,23,23,23,23,23,23,23,23,23,23,23,23,23,23,23,23,23,23,23,23,23,23,23,23,23,23,23,23,23,23,23,23,23,23,23,23,23,23,23,23,23,23,23,23,23,23,23,23,23,23,23,23,23,23,23,23,23,23,23,23,23,23,23,23,23,23,23,23,23,23,23,24,24,25,25,25,24,24,23,23,23,22,22,22,22,22,22,22,22,22,22,22,22,22,22,22,22,22,22,22,22,22,22,22,22,22,22,22,22,22,22,22,22,22,22,22,22,22,22,22,22,22,22,22,22,22,22,22,22,22,22,22,22,22,22,22,22,22,22,22,22,22,22,22,22,22,22,22,22,22,22,22,22,22,22,22,22,22,22,22,22,22,22,22,22,22,22,22,22,22,22,22,22,22,22,22,22,22,22,22,22,22,22,22,22,22,22,22,22,22,22,22,22,22,22,22,23,23,23,24,24,25,25,25,24,24,23,23,23,22,22,22,21,21,21,21,21,21,21,21,21,21,21,21,21,21,21,21,21,21,21,21,21,21,21,21,21,21,21,21,21,21,21,21,21,21,21,21,21,21,21,21,21,21,21,21,21,21,21,21,21,21,21,21,21,21,21,21,21,21,21,21,21,21,21,21,21,21,21,21,21,21,21,21,21,21,21,21,21,21,21,21,21,21,21,21,21,21,21,21,21,21,21,21,21,21,21,21,21,21,21,21,21,21,21,21,21,21,21,21,21,22,22,22,23,23,23,24,24,25,25,25,24,24,23,23,23,22,22,22,21,21,21,21,21,21,21,21,21,21,21,21,21,21,21,21,21,21,21,21,21,21,21,21,21,21,21,21,21,21,21,21,21,21,21,21,21,21,21,21,21,21,21,21,21,21,21,21,21,21,21,21,21,21,21,21,21,21,21,21,21,21,21,21,21,21,21,21,21,21,21,21,21,21,21,21,21,21,21,21,21,21,21,21,21,21,21,21,21,21,21,21,21,21,21,21,21,21,21,21,21,21,21,21,21,21,21,21,21,22,22,22,23,23,23,24,24,25,25,25,24,24,23,23,23,22,22,22,21,21,21,20,20,20,20,20,20,20,20,20,20,20,20,20,20,20,20,20,20,20,20,20,20,20,20,20,20,20,20,20,20,20,20,20,20,20,20,20,20,20,20,20,20,20,20,20,20,20,20,20,20,20,20,20,20,20,20,20,20,20,20,20,20,20,20,20,20,20,20,20,20,20,20,20,20,20,20,20,20,20,20,20,20,20,20,20,20,20,20,20,20,20,20,20,20,20,20,20,20,20,20,20,20,20,21,21,21,22,22,22,23,23,23,24,24,25,25,25,24,24,23,23,23,22,22,22,21,21,21,20,20,20,20,19,19,19,19,19,19,19,19,19,19,19,19,19,19,19,19,19,19,19,19,19,19,19,19,19,19,19,19,19,19,19,19,19,19,19,19,19,19,19,19,19,19,19,19,19,19,19,19,19,19,19,19,19,19,19,19,19,19,19,19,19,19,19,19,19,19,19,19,19,19,19,19,19,19,19,19,19,19,19,19,19,19,19,19,19,19,19,19,19,19,19,19,19,19,19,20,20,20,20,21,21,21,22,22,22,23,23,23,24,24,25,25,25,24,24,23,23,23,22,22,22,21,21,21,20,20,20,20,19,19,19,19,19,19,19,19,19,19,19,19,19,19,19,19,19,19,19,19,19,19,19,19,19,19,19,19,19,19,19,19,19,19,19,19,19,19,19,19,19,19,19,19,19,19,19,19,19,19,19,19,19,19,19,19,19,19,19,19,19,19,19,19,19,19,19,19,19,19,19,19,19,19,19,19,19,19,19,19,19,19,19,19,19,19,19,19,19,19,19,19,19,19,19,20,20,20,20,21,21,21,22,22,22,23,23,23,24,24,25,25,25,24,24,23,23,23,22,22,22,21,21,21,20,20,20,20,19,19,19,19,18,18,18,18,18,18,18,18,18,18,18,18,18,18,18,18,18,18,18,18,18,18,18,18,18,18,18,18,18,18,18,18,18,18,18,18,18,18,18,18,18,18,18,18,18,18,18,18,18,18,18,18,18,18,18,18,18,18,18,18,18,18,18,18,18,18,18,18,18,18,18,18,18,18,18,18,18,18,18,18,18,18,18,18,18,18,18,19,19,19,19,20,20,20,20,21,21,21,22,22,22,23,23,23,24,24,25,25,25,24,24,23,23,23,22,22,22,21,21,21,20,20,20,20,19,19,19,19,18,18,18,18,18,18,17,17,17,17,17,17,17,17,17,17,17,17,17,17,17,17,17,17,17,17,17,17,17,17,17,17,17,17,17,17,17,17,17,17,17,17,17,17,17,17,17,17,17,17,17,17,17,17,17,17,17,17,17,17,17,17,17,17,17,17,17,17,17,17,17,17,17,17,17,17,17,17,17,17,17,18,18,18,18,18,18,19,19,19,19,20,20,20,20,21,21,21,22,22,22,23,23,23,24,24,25,25,25,24,24,23,23,23,22,22,22,21,21,21,20,20,20,20,19,19,19,19,18,18,18,18,18,18,17,17,17,17,17,17,17,16,16,16,16,16,16,16,16,16,16,16,16,16,16,16,16,16,16,16,16,16,16,16,16,16,16,16,16,16,16,16,16,16,16,16,16,16,16,16,16,16,16,16,16,16,16,16,16,16,16,16,16,16,16,16,16,16,16,16,16,16,17,17,17,17,17,17,17,18,18,18,18,18,18,19,19,19,19,20,20,20,20,21,21,21,22,22,22,23,23,23,24,24,25,25,25,24,24,23,23,23,22,22,22,21,21,21,20,20,20,20,19,19,19,19,18,18,18,18,18,18,17,17,17,17,17,17,17,16,16,16,16,16,16,16,16,16,16,16,16,16,16,16,16,16,16,16,16,16,16,16,16,16,16,16,16,16,16,16,16,16,16,16,16,16,16,16,16,16,16,16,16,16,16,16,16,16,16,16,16,16,16,16,16,16,16,16,16,16,17,17,17,17,17,17,17,18,18,18,18,18,18,19,19,19,19,20,20,20,20,21,21,21,22,22,22,23,23,23,24,24,25,25,25,24,24,23,23,23,22,22,22,21,21,21,20,20,20,20,19,19,19,19,18,18,18,18,18,18,17,17,17,17,17,17,17,16,16,16,16,16,16,16,16,16,16,16,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,16,16,16,16,16,16,16,16,16,16,16,17,17,17,17,17,17,17,18,18,18,18,18,18,19,19,19,19,20,20,20,20,21,21,21,22,22,22,23,23,23,24,24,25,25,25,24,24,23,23,23,22,22,22,21,21,21,20,20,20,20,19,19,19,19,18,18,18,18,18,18,17,17,17,17,17,17,17,16,16,16,16,16,16,16,16,16,16,16,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,16,16,16,16,16,16,16,16,16,16,16,17,17,17,17,17,17,17,18,18,18,18,18,18,19,19,19,19,20,20,20,20,21,21,21,22,22,22,23,23,23,24,24,25,25,25,24,24,23,23,23,22,22,22,21,21,21,20,20,20,20,19,19,19,19,18,18,18,18,18,18,17,17,17,17,17,17,17,16,16,16,16,16,16,16,16,16,16,16,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,16,16,16,16,16,16,16,16,16,16,16,17,17,17,17,17,17,17,18,18,18,18,18,18,19,19,19,19,20,20,20,20,21,21,21,22,22,22,23,23,23,24,24,25,25,25,24,24,23,23,23,22,22,22,21,21,21,20,20,20,20,19,19,19,19,18,18,18,18,18,18,17,17,17,17,17,17,17,16,16,16,16,16,16,16,16,16,16,16,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,16,16,16,16,16,16,16,16,16,16,16,17,17,17,17,17,17,17,18,18,18,18,18,18,19,19,19,19,20,20,20,20,21,21,21,22,22,22,23,23,23,24,24,25,25,25,24,24,23,23,23,22,22,22,21,21,21,20,20,20,20,19,19,19,19,18,18,18,18,18,18,17,17,17,17,17,17,17,16,16,16,16,16,16,16,16,16,16,16,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,16,16,16,16,16,16,16,16,16,16,16,17,17,17,17,17,17,17,18,18,18,18,18,18,19,19,19,19,20,20,20,20,21,21,21,22,22,22,23,23,23,24,24,25,25,25,24,24,23,23,23,22,22,22,21,21,21,20,20,20,20,19,19,19,19,18,18,18,18,18,18,17,17,17,17,17,17,17,16,16,16,16,16,16,16,16,16,16,16,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,16,16,16,16,16,16,16,16,16,16,16,17,17,17,17,17,17,17,18,18,18,18,18,18,19,19,19,19,20,20,20,20,21,21,21,22,22,22,23,23,23,24,24,25,25,25,24,24,23,23,23,22,22,22,21,21,21,20,20,20,20,19,19,19,19,18,18,18,18,18,18,17,17,17,17,17,17,17,16,16,16,16,16,16,16,16,16,16,16,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,16,16,16,16,16,16,16,16,16,16,16,17,17,17,17,17,17,17,18,18,18,18,18,18,19,19,19,19,20,20,20,20,21,21,21,22,22,22,23,23,23,24,24,25,25,25,24,24,23,23,23,22,22,22,21,21,21,20,20,20,20,19,19,19,19,18,18,18,18,18,18,17,17,17,17,17,17,17,16,16,16,16,16,16,16,16,16,16,16,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,16,16,16,16,16,16,16,16,16,16,16,17,17,17,17,17,17,17,18,18,18,18,18,18,19,19,19,19,20,20,20,20,21,21,21,22,22,22,23,23,23,24,24,25,25,25,24,24,23,23,23,22,22,22,21,21,21,20,20,20,20,19,19,19,19,18,18,18,18,18,18,17,17,17,17,17,17,17,16,16,16,16,16,16,16,16,16,16,16,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,16,16,16,16,16,16,16,16,16,16,16,17,17,17,17,17,17,17,18,18,18,18,18,18,19,19,19,19,20,20,20,20,21,21,21,22,22,22,23,23,23,24,24,25,25,25,24,24,23,23,23,22,22,22,21,21,21,20,20,20,20,19,19,19,19,18,18,18,18,18,18,17,17,17,17,17,17,17,16,16,16,16,16,16,16,16,16,16,16,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,16,16,16,16,16,16,16,16,16,16,16,17,17,17,17,17,17,17,18,18,18,18,18,18,19,19,19,19,20,20,20,20,21,21,21,22,22,22,23,23,23,24,24,25,25,25,24,24,23,23,23,22,22,22,21,21,21,20,20,20,20,19,19,19,19,18,18,18,18,18,18,17,17,17,17,17,17,17,16,16,16,16,16,16,16,16,16,16,16,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,16,16,16,16,16,16,16,16,16,16,16,17,17,17,17,17,17,17,18,18,18,18,18,18,19,19,19,19,20,20,20,20,21,21,21,22,22,22,23,23,23,24,24,25,25,25,24,24,23,23,23,22,22,22,21,21,21,20,20,20,20,19,19,19,19,18,18,18,18,18,18,17,17,17,17,17,17,17,16,16,16,16,16,16,16,16,16,16,16,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,16,16,16,16,16,16,16,16,16,16,16,17,17,17,17,17,17,17,18,18,18,18,18,18,19,19,19,19,20,20,20,20,21,21,21,22,22,22,23,23,23,24,24,25,25,25,24,24,23,23,23,22,22,22,21,21,21,20,20,20,20,19,19,19,19,18,18,18,18,18,18,17,17,17,17,17,17,17,16,16,16,16,16,16,16,16,16,16,16,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,16,16,16,16,16,16,16,16,16,16,16,17,17,17,17,17,17,17,18,18,18,18,18,18,19,19,19,19,20,20,20,20,21,21,21,22,22,22,23,23,23,24,24,25,25,25,24,24,23,23,23,22,22,22,21,21,21,20,20,20,20,19,19,19,19,18,18,18,18,18,18,17,17,17,17,17,17,17,16,16,16,16,16,16,16,16,16,16,16,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,16,16,16,16,16,16,16,16,16,16,16,17,17,17,17,17,17,17,18,18,18,18,18,18,19,19,19,19,20,20,20,20,21,21,21,22,22,22,23,23,23,24,24,25,25,25,24,24,23,23,23,22,22,22,21,21,21,20,20,20,20,19,19,19,19,18,18,18,18,18,18,17,17,17,17,17,17,17,16,16,16,16,16,16,16,16,16,16,16,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,16,16,16,16,16,16,16,16,16,16,16,17,17,17,17,17,17,17,18,18,18,18,18,18,19,19,19,19,20,20,20,20,21,21,21,22,22,22,23,23,23,24,24,25,25,25,24,24,23,23,23,22,22,22,21,21,21,20,20,20,20,19,19,19,19,18,18,18,18,18,18,17,17,17,17,17,17,17,16,16,16,16,16,16,16,16,16,16,16,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,16,16,16,16,16,16,16,16,16,16,16,17,17,17,17,17,17,17,18,18,18,18,18,18,19,19,19,19,20,20,20,20,21,21,21,22,22,22,23,23,23,24,24,25,25,25,24,24,23,23,23,22,22,22,21,21,21,20,20,20,20,19,19,19,19,18,18,18,18,18,18,17,17,17,17,17,17,17,16,16,16,16,16,16,16,16,16,16,16,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,16,16,16,16,16,16,16,16,16,16,16,17,17,17,17,17,17,17,18,18,18,18,18,18,19,19,19,19,20,20,20,20,21,21,21,22,22,22,23,23,23,24,24,25,25,25,24,24,23,23,23,22,22,22,21,21,21,20,20,20,20,19,19,19,19,18,18,18,18,18,18,17,17,17,17,17,17,17,16,16,16,16,16,16,16,16,16,16,16,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,16,16,16,16,16,16,16,16,16,16,16,17,17,17,17,17,17,17,18,18,18,18,18,18,19,19,19,19,20,20,20,20,21,21,21,22,22,22,23,23,23,24,24,25,25,25,24,24,23,23,23,22,22,22,21,21,21,20,20,20,20,19,19,19,19,18,18,18,18,18,18,17,17,17,17,17,17,17,16,16,16,16,16,16,16,16,16,16,16,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,16,16,16,16,16,16,16,16,16,16,16,17,17,17,17,17,17,17,18,18,18,18,18,18,19,19,19,19,20,20,20,20,21,21,21,22,22,22,23,23,23,24,24,25,25,25,24,24,23,23,23,22,22,22,21,21,21,20,20,20,20,19,19,19,19,18,18,18,18,18,18,17,17,17,17,17,17,17,16,16,16,16,16,16,16,16,16,16,16,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,16,16,16,16,16,16,16,16,16,16,16,17,17,17,17,17,17,17,18,18,18,18,18,18,19,19,19,19,20,20,20,20,21,21,21,22,22,22,23,23,23,24,24,25,25,25,24,24,23,23,23,22,22,22,21,21,21,20,20,20,20,19,19,19,19,18,18,18,18,18,18,17,17,17,17,17,17,17,16,16,16,16,16,16,16,16,16,16,16,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,16,16,16,16,16,16,16,16,16,16,16,17,17,17,17,17,17,17,18,18,18,18,18,18,19,19,19,19,20,20,20,20,21,21,21,22,22,22,23,23,23,24,24,25,25,25,24,24,23,23,23,22,22,22,21,21,21,20,20,20,20,19,19,19,19,18,18,18,18,18,18,17,17,17,17,17,17,17,16,16,16,16,16,16,16,16,16,16,16,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,16,16,16,16,16,16,16,16,16,16,16,17,17,17,17,17,17,17,18,18,18,18,18,18,19,19,19,19,20,20,20,20,21,21,21,22,22,22,23,23,23,24,24,25,25,25,24,24,23,23,23,22,22,22,21,21,21,20,20,20,20,19,19,19,19,18,18,18,18,18,18,17,17,17,17,17,17,17,16,16,16,16,16,16,16,16,16,16,16,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,16,16,16,16,16,16,16,16,16,16,16,17,17,17,17,17,17,17,18,18,18,18,18,18,19,19,19,19,20,20,20,20,21,21,21,22,22,22,23,23,23,24,24,25,25,25,24,24,23,23,23,22,22,22,21,21,21,20,20,20,20,19,19,19,19,18,18,18,18,18,18,17,17,17,17,17,17,17,16,16,16,16,16,16,16,16,16,16,16,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,16,16,16,16,16,16,16,16,16,16,16,17,17,17,17,17,17,17,18,18,18,18,18,18,19,19,19,19,20,20,20,20,21,21,21,22,22,22,23,23,23,24,24,25,25,25,24,24,23,23,23,22,22,22,21,21,21,20,20,20,20,19,19,19,19,18,18,18,18,18,18,17,17,17,17,17,17,17,16,16,16,16,16,16,16,16,16,16,16,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,16,16,16,16,16,16,16,16,16,16,16,17,17,17,17,17,17,17,18,18,18,18,18,18,19,19,19,19,20,20,20,20,21,21,21,22,22,22,23,23,23,24,24,25,25,25,24,24,23,23,23,22,22,22,21,21,21,20,20,20,20,19,19,19,19,18,18,18,18,18,18,17,17,17,17,17,17,17,16,16,16,16,16,16,16,16,16,16,16,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,16,16,16,16,16,16,16,16,16,16,16,17,17,17,17,17,17,17,18,18,18,18,18,18,19,19,19,19,20,20,20,20,21,21,21,22,22,22,23,23,23,24,24,25,25,25,24,24,23,23,23,22,22,22,21,21,21,20,20,20,20,19,19,19,19,18,18,18,18,18,18,17,17,17,17,17,17,17,16,16,16,16,16,16,16,16,16,16,16,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,16,16,16,16,16,16,16,16,16,16,16,17,17,17,17,17,17,17,18,18,18,18,18,18,19,19,19,19,20,20,20,20,21,21,21,22,22,22,23,23,23,24,24,25,25,25,24,24,23,23,23,22,22,22,21,21,21,20,20,20,20,19,19,19,19,18,18,18,18,18,18,17,17,17,17,17,17,17,16,16,16,16,16,16,16,16,16,16,16,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,16,16,16,16,16,16,16,16,16,16,16,17,17,17,17,17,17,17,18,18,18,18,18,18,19,19,19,19,20,20,20,20,21,21,21,22,22,22,23,23,23,24,24,25,25,25,24,24,23,23,23,22,22,22,21,21,21,20,20,20,20,19,19,19,19,18,18,18,18,18,18,17,17,17,17,17,17,17,16,16,16,16,16,16,16,16,16,16,16,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,16,16,16,16,16,16,16,16,16,16,16,17,17,17,17,17,17,17,18,18,18,18,18,18,19,19,19,19,20,20,20,20,21,21,21,22,22,22,23,23,23,24,24,25,25,25,24,24,23,23,23,22,22,22,21,21,21,20,20,20,20,19,19,19,19,18,18,18,18,18,18,17,17,17,17,17,17,17,16,16,16,16,16,16,16,16,16,16,16,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,16,16,16,16,16,16,16,16,16,16,16,17,17,17,17,17,17,17,18,18,18,18,18,18,19,19,19,19,20,20,20,20,21,21,21,22,22,22,23,23,23,24,24,25,25,25,24,24,23,23,23,22,22,22,21,21,21,20,20,20,20,19,19,19,19,18,18,18,18,18,18,17,17,17,17,17,17,17,16,16,16,16,16,16,16,16,16,16,16,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,16,16,16,16,16,16,16,16,16,16,16,17,17,17,17,17,17,17,18,18,18,18,18,18,19,19,19,19,20,20,20,20,21,21,21,22,22,22,23,23,23,24,24,25,25,25,24,24,23,23,23,22,22,22,21,21,21,20,20,20,20,19,19,19,19,18,18,18,18,18,18,17,17,17,17,17,17,17,16,16,16,16,16,16,16,16,16,16,16,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,16,16,16,16,16,16,16,16,16,16,16,17,17,17,17,17,17,17,18,18,18,18,18,18,19,19,19,19,20,20,20,20,21,21,21,22,22,22,23,23,23,24,24,25,25,25,24,24,23,23,23,22,22,22,21,21,21,20,20,20,20,19,19,19,19,18,18,18,18,18,18,17,17,17,17,17,17,17,16,16,16,16,16,16,16,16,16,16,16,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,16,16,16,16,16,16,16,16,16,16,16,17,17,17,17,17,17,17,18,18,18,18,18,18,19,19,19,19,20,20,20,20,21,21,21,22,22,22,23,23,23,24,24,25,25,25,24,24,23,23,23,22,22,22,21,21,21,20,20,20,20,19,19,19,19,18,18,18,18,18,18,17,17,17,17,17,17,17,16,16,16,16,16,16,16,16,16,16,16,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,16,16,16,16,16,16,16,16,16,16,16,17,17,17,17,17,17,17,18,18,18,18,18,18,19,19,19,19,20,20,20,20,21,21,21,22,22,22,23,23,23,24,24,25,25,25,24,24,23,23,23,22,22,22,21,21,21,20,20,20,20,19,19,19,19,18,18,18,18,18,18,17,17,17,17,17,17,17,16,16,16,16,16,16,16,16,16,16,16,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,16,16,16,16,16,16,16,16,16,16,16,17,17,17,17,17,17,17,18,18,18,18,18,18,19,19,19,19,20,20,20,20,21,21,21,22,22,22,23,23,23,24,24,25,25,25,24,24,23,23,23,22,22,22,21,21,21,20,20,20,20,19,19,19,19,18,18,18,18,18,18,17,17,17,17,17,17,17,16,16,16,16,16,16,16,16,16,16,16,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,16,16,16,16,16,16,16,16,16,16,16,17,17,17,17,17,17,17,18,18,18,18,18,18,19,19,19,19,20,20,20,20,21,21,21,22,22,22,23,23,23,24,24,25,25,25,24,24,23,23,23,22,22,22,21,21,21,20,20,20,20,19,19,19,19,18,18,18,18,18,18,17,17,17,17,17,17,17,16,16,16,16,16,16,16,16,16,16,16,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,16,16,16,16,16,16,16,16,16,16,16,17,17,17,17,17,17,17,18,18,18,18,18,18,19,19,19,19,20,20,20,20,21,21,21,22,22,22,23,23,23,24,24,25,25,25,24,24,23,23,23,22,22,22,21,21,21,20,20,20,20,19,19,19,19,18,18,18,18,18,18,17,17,17,17,17,17,17,16,16,16,16,16,16,16,16,16,16,16,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,16,16,16,16,16,16,16,16,16,16,16,17,17,17,17,17,17,17,18,18,18,18,18,18,19,19,19,19,20,20,20,20,21,21,21,22,22,22,23,23,23,24,24,25,25,25,24,24,23,23,23,22,22,22,21,21,21,20,20,20,20,19,19,19,19,18,18,18,18,18,18,17,17,17,17,17,17,17,16,16,16,16,16,16,16,16,16,16,16,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,16,16,16,16,16,16,16,16,16,16,16,17,17,17,17,17,17,17,18,18,18,18,18,18,19,19,19,19,20,20,20,20,21,21,21,22,22,22,23,23,23,24,24,25,25,25,24,24,23,23,23,22,22,22,21,21,21,20,20,20,20,19,19,19,19,18,18,18,18,18,18,17,17,17,17,17,17,17,16,16,16,16,16,16,16,16,16,16,16,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,16,16,16,16,16,16,16,16,16,16,16,17,17,17,17,17,17,17,18,18,18,18,18,18,19,19,19,19,20,20,20,20,21,21,21,22,22,22,23,23,23,24,24,25,25,25,24,24,23,23,23,22,22,22,21,21,21,20,20,20,20,19,19,19,19,18,18,18,18,18,18,17,17,17,17,17,17,17,16,16,16,16,16,16,16,16,16,16,16,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,16,16,16,16,16,16,16,16,16,16,16,17,17,17,17,17,17,17,18,18,18,18,18,18,19,19,19,19,20,20,20,20,21,21,21,22,22,22,23,23,23,24,24,25,25,25,24,24,23,23,23,22,22,22,21,21,21,20,20,20,20,19,19,19,19,18,18,18,18,18,18,17,17,17,17,17,17,17,16,16,16,16,16,16,16,16,16,16,16,16,16,16,16,16,16,16,16,16,16,16,16,16,16,16,16,16,16,16,16,16,16,16,16,16,16,16,16,16,16,16,16,16,16,16,16,16,16,16,16,16,16,16,16,16,16,16,16,16,16,17,17,17,17,17,17,17,18,18,18,18,18,18,19,19,19,19,20,20,20,20,21,21,21,22,22,22,23,23,23,24,24,25,25,25,24,24,23,23,23,22,22,22,21,21,21,20,20,20,20,19,19,19,19,18,18,18,18,18,18,17,17,17,17,17,17,17,16,16,16,16,16,16,16,16,16,16,16,16,16,16,16,16,16,16,16,16,16,16,16,16,16,16,16,16,16,16,16,16,16,16,16,16,16,16,16,16,16,16,16,16,16,16,16,16,16,16,16,16,16,16,16,16,16,16,16,16,16,17,17,17,17,17,17,17,18,18,18,18,18,18,19,19,19,19,20,20,20,20,21,21,21,22,22,22,23,23,23,24,24,25,25,25,24,24,23,23,23,22,22,22,21,21,21,20,20,20,20,19,19,19,19,18,18,18,18,18,18,17,17,17,17,17,17,17,17,17,17,17,17,17,17,17,17,17,17,17,17,17,17,17,17,17,17,17,17,17,17,17,17,17,17,17,17,17,17,17,17,17,17,17,17,17,17,17,17,17,17,17,17,17,17,17,17,17,17,17,17,17,17,17,17,17,17,17,17,17,17,17,17,17,17,17,18,18,18,18,18,18,19,19,19,19,20,20,20,20,21,21,21,22,22,22,23,23,23,24,24,25,25,25,24,24,23,23,23,22,22,22,21,21,21,20,20,20,20,19,19,19,19,18,18,18,18,18,18,18,18,18,18,18,18,18,18,18,18,18,18,18,18,18,18,18,18,18,18,18,18,18,18,18,18,18,18,18,18,18,18,18,18,18,18,18,18,18,18,18,18,18,18,18,18,18,18,18,18,18,18,18,18,18,18,18,18,18,18,18,18,18,18,18,18,18,18,18,18,18,18,18,18,18,18,18,18,18,18,18,19,19,19,19,20,20,20,20,21,21,21,22,22,22,23,23,23,24,24,25,25,25,24,24,23,23,23,22,22,22,21,21,21,20,20,20,20,19,19,19,19,19,19,19,19,19,19,19,19,19,19,19,19,19,19,19,19,19,19,19,19,19,19,19,19,19,19,19,19,19,19,19,19,19,19,19,19,19,19,19,19,19,19,19,19,19,19,19,19,19,19,19,19,19,19,19,19,19,19,19,19,19,19,19,19,19,19,19,19,19,19,19,19,19,19,19,19,19,19,19,19,19,19,19,19,19,19,19,19,19,19,19,20,20,20,20,21,21,21,22,22,22,23,23,23,24,24,25,25,25,24,24,23,23,23,22,22,22,21,21,21,20,20,20,20,19,19,19,19,19,19,19,19,19,19,19,19,19,19,19,19,19,19,19,19,19,19,19,19,19,19,19,19,19,19,19,19,19,19,19,19,19,19,19,19,19,19,19,19,19,19,19,19,19,19,19,19,19,19,19,19,19,19,19,19,19,19,19,19,19,19,19,19,19,19,19,19,19,19,19,19,19,19,19,19,19,19,19,19,19,19,19,19,19,19,19,19,19,19,19,20,20,20,20,21,21,21,22,22,22,23,23,23,24,24,25,25,25,24,24,23,23,23,22,22,22,21,21,21,20,20,20,20,20,20,20,20,20,20,20,20,20,20,20,20,20,20,20,20,20,20,20,20,20,20,20,20,20,20,20,20,20,20,20,20,20,20,20,20,20,20,20,20,20,20,20,20,20,20,20,20,20,20,20,20,20,20,20,20,20,20,20,20,20,20,20,20,20,20,20,20,20,20,20,20,20,20,20,20,20,20,20,20,20,20,20,20,20,20,20,20,20,20,20,20,20,20,20,20,20,20,20,21,21,21,22,22,22,23,23,23,24,24,25,25,25,24,24,23,23,23,22,22,22,21,21,21,21,21,21,21,21,21,21,21,21,21,21,21,21,21,21,21,21,21,21,21,21,21,21,21,21,21,21,21,21,21,21,21,21,21,21,21,21,21,21,21,21,21,21,21,21,21,21,21,21,21,21,21,21,21,21,21,21,21,21,21,21,21,21,21,21,21,21,21,21,21,21,21,21,21,21,21,21,21,21,21,21,21,21,21,21,21,21,21,21,21,21,21,21,21,21,21,21,21,21,21,21,21,21,21,21,21,22,22,22,23,23,23,24,24,25,25,25,24,24,23,23,23,22,22,22,21,21,21,21,21,21,21,21,21,21,21,21,21,21,21,21,21,21,21,21,21,21,21,21,21,21,21,21,21,21,21,21,21,21,21,21,21,21,21,21,21,21,21,21,21,21,21,21,21,21,21,21,21,21,21,21,21,21,21,21,21,21,21,21,21,21,21,21,21,21,21,21,21,21,21,21,21,21,21,21,21,21,21,21,21,21,21,21,21,21,21,21,21,21,21,21,21,21,21,21,21,21,21,21,21,21,21,21,21,22,22,22,23,23,23,24,24,25,25,25,24,24,23,23,23,22,22,22,22,22,22,22,22,22,22,22,22,22,22,22,22,22,22,22,22,22,22,22,22,22,22,22,22,22,22,22,22,22,22,22,22,22,22,22,22,22,22,22,22,22,22,22,22,22,22,22,22,22,22,22,22,22,22,22,22,22,22,22,22,22,22,22,22,22,22,22,22,22,22,22,22,22,22,22,22,22,22,22,22,22,22,22,22,22,22,22,22,22,22,22,22,22,22,22,22,22,22,22,22,22,22,22,22,22,22,22,22,22,22,22,23,23,23,24,24,25,25,25,24,24,23,23,23,23,23,23,23,23,23,23,23,23,23,23,23,23,23,23,23,23,23,23,23,23,23,23,23,23,23,23,23,23,23,23,23,23,23,23,23,23,23,23,23,23,23,23,23,23,23,23,23,23,23,23,23,23,23,23,23,23,23,23,23,23,23,23,23,23,23,23,23,23,23,23,23,23,23,23,23,23,23,23,23,23,23,23,23,23,23,23,23,23,23,23,23,23,23,23,23,23,23,23,23,23,23,23,23,23,23,23,23,23,23,23,23,23,23,23,23,23,23,24,24,25,25,25,24,24,24,24,24,24,24,24,24,24,24,24,24,24,24,24,24,24,24,24,24,24,24,24,24,24,24,24,24,24,24,24,24,24,24,24,24,24,24,24,24,24,24,24,24,24,24,24,24,24,24,24,24,24,24,24,24,24,24,24,24,24,24,24,24,24,24,24,24,24,24,24,24,24,24,24,24,24,24,24,24,24,24,24,24,24,24,24,24,24,24,24,24,24,24,24,24,24,24,24,24,24,24,24,24,24,24,24,24,24,24,24,24,24,24,24,24,24,24,24,24,24,24,24,24,25,25,25,24,24,24,24,24,24,24,24,24,24,24,24,24,24,24,24,24,24,24,24,24,24,24,24,24,24,24,24,24,24,24,24,24,24,24,24,24,24,24,24,24,24,24,24,24,24,24,24,24,24,24,24,24,24,24,24,24,24,24,24,24,24,24,24,24,24,24,24,24,24,24,24,24,24,24,24,24,24,24,24,24,24,24,24,24,24,24,24,24,24,24,24,24,24,24,24,24,24,24,24,24,24,24,24,24,24,24,24,24,24,24,24,24,24,24,24,24,24,24,24,24,24,24,24,24,25,25,25,25,25,25,25,25,25,25,25,25,25,25,25,25,25,25,25,25,25,25,25,25,25,25,25,25,25,25,25,25,25,25,25,25,25,25,25,25,25,25,25,25,25,25,25,25,25,25,25,25,25,25,25,25,25,25,25,25,25,25,25,25,25,25,25,25,25,25,25,25,25,25,25,25,25,25,25,25,25,25,25,25,25,25,25,25,25,25,25,25,25,25,25,25,25,25,25,25,25,25,25,25,25,25,25,25,25,25,25,25,25,25,25,25,25,25,25,25,25,25,25,25,25,25,25,25,25,25,26,26,26,26,26,26,26,26,26,26,26,26,26,26,26,26,26,26,26,26,26,26,26,26,26,26,26,26,26,26,26,26,26,26,26,26,26,26,26,26,26,26,26,26,26,26,26,26,26,26,26,26,26,26,26,26,26,26,26,26,26,26,26,26,26,26,26,26,26,26,26,26,26,26,26,26,26,26,26,26,26,26,26,26,26,26,26,26,26,26,26,26,26,26,26,26,26,26,26,26,26,26,26,26,26,26,26,26,26,26,26,26,26,26,26,26,26,26,26,26,26,26,26,26,26,26,26,26,26,26,26,26,26,26,26,26,26,26,26,26,26,26,26,26,26,26,26,26,26,26,26,26,26,26,26,26,26,26,26,26,26,26,26,26,26,26,26,26,26,26,26,26,26,26,26,26,26,26,26,26,26,26,26,26,26,26,26,26,26,26,26,26,26,26,26,26,26,26,26,26,26,26,26,26,26,26,26,26,26,26,26,26,26,26,26,26,26,26,26,26,26,26,26,26,26,26,26,26,26,26,26,26,26,26,26,26,26,26,26,26,26,26,26,26,26,26,26,26,26,26,26,26,26,26,26,26,27,27,27,27,27,27,27,27,27,27,27,27,27,27,27,27,27,27,27,27,27,27,27,27,27,27,27,27,27,27,27,27,27,27,27,27,27,27,27,27,27,27,27,27,27,27,27,27,27,27,27,27,27,27,27,27,27,27,27,27,27,27,27,27,27,27,27,27,27,27,27,27,27,27,27,27,27,27,27,27,27,27,27,27,27,27,27,27,27,27,27,27,27,27,27,27,27,27,27,27,27,27,27,27,27,27,27,27,27,27,27,27,27,27,27,27,27,27,27,27,27,27,27,27,27,27,27,27,28,28,28,28,28,28,28,28,28,28,28,28,28,28,28,28,28,28,28,28,28,28,28,28,28,28,28,28,28,28,28,28,28,28,28,28,28,28,28,28,28,28,28,28,28,28,28,28,28,28,28,28,28,28,28,28,28,28,28,28,28,28,28,28,28,28,28,28,28,28,28,28,28,28,28,28,28,28,28,28,28,28,28,28,28,28,28,28,28,28,28,28,28,28,28,28,28,28,28,28,28,28,28,28,28,28,28,28,28,28,28,28,28,28,28,28,28,28,28,28,28,28,28,28,28,28,28,28,29,29,29,29,29,29,29,29,29,29,29,29,29,29,29,29,29,29,29,29,29,29,29,29,29,29,29,29,29,29,29,29,29,29,29,29,29,29,29,29,29,29,29,29,29,29,29,29,29,29,29,29,29,29,29,29,29,29,29,29,29,29,29,29,29,29,29,29,29,29,29,29,29,29,29,29,29,29,29,29,29,29,29,29,29,29,29,29,29,29,29,29,29,29,29,29,29,29,29,29,29,29,29,29,29,29,29,29,29,29,29,29,29,29,29,29,29,29,29,29,29,29,29,29,29,29,29,29,29,29,29,29,29,29,29,29,29,29,29,29,29,29,29,29,29,29,29,29,29,29,29,29,29,29,29,29,29,29,29,29,29,29,29,29,29,29,29,29,29,29,29,29,29,29,29,29,29,29,29,29,29,29,29,29,29,29,29,29,29,29,29,29,29,29,29,29,29,29,29,29,29,29,29,29,29,29,29,29,29,29,29,29,29,29,29,29,29,29,29,29,29,29,29,29,29,29,29,29,29,29,29,29,29,29,29,29,29,29,29,29,29,29,29,29,29,29,29,29,29,29,29,29,29,29,29,29,30,30,30,30,30,30,30,30,30,30,30,30,30,30,30,30,30,30,30,30,30,30,30,30,30,30,30,30,30,30,30,30,30,30,30,30,30,30,30,30,30,30,30,30,30,30,30,30,30,30,30,30,30,30,30,30,30,30,30,30,30,30,30,30,30,30,30,30,30,30,30,30,30,30,30,30,30,30,30,30,30,30,30,30,30,30,30,30,30,30,30,30,30,30,30,30,30,30,30,30,30,30,30,30,30,30,30,30,30,30,30,30,30,30,30,30,30,30,30,30,30,30,30,30,30,30,30,30,31,31,31,31,31,31,31,31,31,31,31,31,31,31,31,31,31,31,31,31,31,31,31,31,31,31,31,31,31,31,31,31,31,31,31,31,31,31,31,31,31,31,31,31,31,31,31,31,31,31,31,31,31,31,31,31,31,31,31,31,31,31,31,31,31,31,31,31,31,31,31,31,31,31,31,31,31,31,31,31,31,31,31,31,31,31,31,31,31,31,31,31,31,31,31,31,31,31,31,31,31,31,31,31,31,31,31,31,31,31,31,31,31,31,31,31,31,31,31,31,31,31,31,31,31,31,31,31,31,31,31,31,31,31,31,31,31,31,31,31,31,31,31,31,31,31,31,31,31,31,31,31,31,31,31,31,31,31,31,31,31,31,31,31,31,31,31,31,31,31,31,31,31,31,31,31,31,31,31,31,31,31,31,31,31,31,31,31,31,31,31,31,31,31,31,31,31,31,31,31,31,31,31,31,31,31,31,31,31,31,31,31,31,31,31,31,31,31,31,31,31,31,31,31,31,31,31,31,31,31,31,31,31,31,31,31,31,31,31,31,31,31,31,31,31,31,31,31,31,31,31,31,31,31,31,31,32,32,32,32,32,32,32,32,32,32,32,32,32,32,32,32,32,32,32,32,32,32,32,32,32,32,32,32,32,32,32,32,32,32,32,32,32,32,32,32,32,32,32,32,32,32,32,32,32,32,32,32,32,32,32,32,32,32,32,32,32,32,32,32,32,32,32,32,32,32,32,32,32,32,32,32,32,32,32,32,32,32,32,32,32,32,32,32,32,32,32,32,32,32,32,32,32,32,32,32,32,32,32,32,32,32,32,32,32,32,32,32,32,32,32,32,32,32,32,32,32,32,32,32,32,32,32,32,33,33,33,33,33,33,33,33,33,33,33,33,33,33,33,33,33,33,33,33,33,33,33,33,33,33,33,33,33,33,33,33,33,33,33,33,33,33,33,33,33,33,33,33,33,33,33,33,33,33,33,33,33,33,33,33,33,33,33,33,33,33,33,33,33,33,33,33,33,33,33,33,33,33,33,33,33,33,33,33,33,33,33,33,33,33,33,33,33,33,33,33,33,33,33,33,33,33,33,33,33,33,33,33,33,33,33,33,33,33,33,33,33,33,33,33,33,33,33,33,33,33,33,33,33,33,33,33,34,34,34,34,34,34,34,34,34,34,34,34,34,34,34,34,34,34,34,34,34,34,34,34,34,34,34,34,34,34,34,34,34,34,34,34,34,34,34,34,34,34,34,34,34,34,34,34,34,34,34,34,34,34,34,34,34,34,34,34,34,34,34,34,34,34,34,34,34,34,34,34,34,34,34,34,34,34,34,34,34,34,34,34,34,34,34,34,34,34,34,34,34,34,34,34,34,34,34,34,34,34,34,34,34,34,34,34,34,34,34,34,34,34,34,34,34,34,34,34,34,34,34,34,34,34,34,34,34,34,34,34,34,34,34,34,34,34,34,34,34,34,34,34,34,34,34,34,34,34,34,34,34,34,34,34,34,34,34,34,34,34,34,34,34,34,34,34,34,34,34,34,34,34,34,34,34,34,34,34,34,34,34,34,34,34,34,34,34,34,34,34,34,34,34,34,34,34,34,34,34,34,34,34,34,34,34,34,34,34,34,34,34,34,34,34,34,34,34,34,34,34,34,34,34,34,34,34,34,34,34,34,34,34,34,34,34,34,34,34,34,34,34,34,34,34,34,34,34,34,34,34,34,34,34,34,35,35,35,35,35,35,35,35,35,35,35,35,35,35,35,35,35,35,35,35,35,35,35,35,35,35,35,35,35,35,35,35,35,35,35,35,35,35,35,35,35,35,35,35,35,35,35,35,35,35,35,35,35,35,35,35,35,35,35,35,35,35,35,35,35,35,35,35,35,35,35,35,35,35,35,35,35,35,35,35,35,35,35,35,35,35,35,35,35,35,35,35,35,35,35,35,35,35,35,35,35,35,35,35,35,35,35,35,35,35,35,35,35,35,35,35,35,35,35,35,35,35,35,35,35,35,35,35};

  // max_Y = 400 mm
  static const uint8_t maximum_disparities[12288] = {43, 43, 43, 43, 43, 43, 43, 43, 43, 43, 43, 43, 43, 43, 43, 43, 43, 43, 43, 43, 43, 43, 43, 43, 43, 43, 43, 43, 43, 43, 43, 43, 43, 43, 43, 43, 43, 43, 43, 43, 43, 43, 43, 43, 43, 43, 43, 43, 43, 43, 43, 43, 43, 43, 43, 43, 43, 43, 43, 43, 43, 43, 43, 43, 43, 43, 43, 43, 43, 43, 43, 43, 43, 43, 43, 43, 43, 43, 43, 43, 43, 43, 43, 43, 43, 43, 43, 43, 43, 43, 43, 43, 43, 43, 43, 43, 43, 43, 43, 43, 43, 43, 43, 43, 43, 43, 43, 43, 43, 43, 43, 43, 43, 43, 43, 43, 43, 43, 43, 43, 43, 43, 43, 43, 43, 43, 43, 43, 42, 42, 42, 42, 42, 42, 42, 42, 42, 42, 42, 42, 42, 42, 42, 42, 42, 42, 42, 42, 42, 42, 42, 42, 42, 42, 42, 42, 42, 42, 42, 42, 42, 42, 42, 42, 42, 42, 42, 42, 42, 42, 42, 42, 42, 42, 42, 42, 42, 42, 42, 42, 42, 42, 42, 42, 42, 42, 42, 42, 42, 42, 42, 42, 42, 42, 42, 42, 42, 42, 42, 42, 42, 42, 42, 42, 42, 42, 42, 42, 42, 42, 42, 42, 42, 42, 42, 42, 42, 42, 42, 42, 42, 42, 42, 42, 42, 42, 42, 42, 42, 42, 42, 42, 42, 42, 42, 42, 42, 42, 42, 42, 42, 42, 42, 42, 42, 42, 42, 42, 42, 42, 42, 42, 42, 42, 42, 42, 41, 41, 41, 41, 41, 41, 41, 41, 41, 41, 41, 41, 41, 41, 41, 41, 41, 41, 41, 41, 41, 41, 41, 41, 41, 41, 41, 41, 41, 41, 41, 41, 41, 41, 41, 41, 41, 41, 41, 41, 41, 41, 41, 41, 41, 41, 41, 41, 41, 41, 41, 41, 41, 41, 41, 41, 41, 41, 41, 41, 41, 41, 41, 41, 41, 41, 41, 41, 41, 41, 41, 41, 41, 41, 41, 41, 41, 41, 41, 41, 41, 41, 41, 41, 41, 41, 41, 41, 41, 41, 41, 41, 41, 41, 41, 41, 41, 41, 41, 41, 41, 41, 41, 41, 41, 41, 41, 41, 41, 41, 41, 41, 41, 41, 41, 41, 41, 41, 41, 41, 41, 41, 41, 41, 41, 41, 41, 41, 40, 40, 40, 40, 40, 40, 40, 40, 40, 40, 40, 40, 40, 40, 40, 40, 40, 40, 40, 40, 40, 40, 40, 40, 40, 40, 40, 40, 40, 40, 40, 40, 40, 40, 40, 40, 40, 40, 40, 40, 40, 40, 40, 40, 40, 40, 40, 40, 40, 40, 40, 40, 40, 40, 40, 40, 40, 40, 40, 40, 40, 40, 40, 40, 40, 40, 40, 40, 40, 40, 40, 40, 40, 40, 40, 40, 40, 40, 40, 40, 40, 40, 40, 40, 40, 40, 40, 40, 40, 40, 40, 40, 40, 40, 40, 40, 40, 40, 40, 40, 40, 40, 40, 40, 40, 40, 40, 40, 40, 40, 40, 40, 40, 40, 40, 40, 40, 40, 40, 40, 40, 40, 40, 40, 40, 40, 40, 40, 39, 39, 39, 39, 39, 39, 39, 39, 39, 39, 39, 39, 39, 39, 39, 39, 39, 39, 39, 39, 39, 39, 39, 39, 39, 39, 39, 39, 39, 39, 39, 39, 39, 39, 39, 39, 39, 39, 39, 39, 39, 39, 39, 39, 39, 39, 39, 39, 39, 39, 39, 39, 39, 39, 39, 39, 39, 39, 39, 39, 39, 39, 39, 39, 39, 39, 39, 39, 39, 39, 39, 39, 39, 39, 39, 39, 39, 39, 39, 39, 39, 39, 39, 39, 39, 39, 39, 39, 39, 39, 39, 39, 39, 39, 39, 39, 39, 39, 39, 39, 39, 39, 39, 39, 39, 39, 39, 39, 39, 39, 39, 39, 39, 39, 39, 39, 39, 39, 39, 39, 39, 39, 39, 39, 39, 39, 39, 39, 38, 38, 38, 38, 38, 38, 38, 38, 38, 38, 38, 38, 38, 38, 38, 38, 38, 38, 38, 38, 38, 38, 38, 38, 38, 38, 38, 38, 38, 38, 38, 38, 38, 38, 38, 38, 38, 38, 38, 38, 38, 38, 38, 38, 38, 38, 38, 38, 38, 38, 38, 38, 38, 38, 38, 38, 38, 38, 38, 38, 38, 38, 38, 38, 38, 38, 38, 38, 38, 38, 38, 38, 38, 38, 38, 38, 38, 38, 38, 38, 38, 38, 38, 38, 38, 38, 38, 38, 38, 38, 38, 38, 38, 38, 38, 38, 38, 38, 38, 38, 38, 38, 38, 38, 38, 38, 38, 38, 38, 38, 38, 38, 38, 38, 38, 38, 38, 38, 38, 38, 38, 38, 38, 38, 38, 38, 38, 38, 37, 37, 37, 37, 37, 37, 37, 37, 37, 37, 37, 37, 37, 37, 37, 37, 37, 37, 37, 37, 37, 37, 37, 37, 37, 37, 37, 37, 37, 37, 37, 37, 37, 37, 37, 37, 37, 37, 37, 37, 37, 37, 37, 37, 37, 37, 37, 37, 37, 37, 37, 37, 37, 37, 37, 37, 37, 37, 37, 37, 37, 37, 37, 37, 37, 37, 37, 37, 37, 37, 37, 37, 37, 37, 37, 37, 37, 37, 37, 37, 37, 37, 37, 37, 37, 37, 37, 37, 37, 37, 37, 37, 37, 37, 37, 37, 37, 37, 37, 37, 37, 37, 37, 37, 37, 37, 37, 37, 37, 37, 37, 37, 37, 37, 37, 37, 37, 37, 37, 37, 37, 37, 37, 37, 37, 37, 37, 37, 37, 37, 37, 37, 37, 37, 37, 37, 37, 37, 37, 37, 37, 37, 37, 37, 37, 37, 37, 37, 37, 37, 37, 37, 37, 37, 37, 37, 37, 37, 37, 37, 37, 37, 37, 37, 37, 37, 37, 37, 37, 37, 37, 37, 37, 37, 37, 37, 37, 37, 37, 37, 37, 37, 37, 37, 37, 37, 37, 37, 37, 37, 37, 37, 37, 37, 37, 37, 37, 37, 37, 37, 37, 37, 37, 37, 37, 37, 37, 37, 37, 37, 37, 37, 37, 37, 37, 37, 37, 37, 37, 37, 37, 37, 37, 37, 37, 37, 37, 37, 37, 37, 37, 37, 37, 37, 37, 37, 37, 37, 37, 37, 37, 37, 37, 37, 37, 37, 37, 37, 37, 37, 37, 37, 37, 37, 37, 37, 36, 36, 36, 36, 36, 36, 36, 36, 36, 36, 36, 36, 36, 36, 36, 36, 36, 36, 36, 36, 36, 36, 36, 36, 36, 36, 36, 36, 36, 36, 36, 36, 36, 36, 36, 36, 36, 36, 36, 36, 36, 36, 36, 36, 36, 36, 36, 36, 36, 36, 36, 36, 36, 36, 36, 36, 36, 36, 36, 36, 36, 36, 36, 36, 36, 36, 36, 36, 36, 36, 36, 36, 36, 36, 36, 36, 36, 36, 36, 36, 36, 36, 36, 36, 36, 36, 36, 36, 36, 36, 36, 36, 36, 36, 36, 36, 36, 36, 36, 36, 36, 36, 36, 36, 36, 36, 36, 36, 36, 36, 36, 36, 36, 36, 36, 36, 36, 36, 36, 36, 36, 36, 36, 36, 36, 36, 36, 36, 35, 35, 35, 35, 35, 35, 35, 35, 35, 35, 35, 35, 35, 35, 35, 35, 35, 35, 35, 35, 35, 35, 35, 35, 35, 35, 35, 35, 35, 35, 35, 35, 35, 35, 35, 35, 35, 35, 35, 35, 35, 35, 35, 35, 35, 35, 35, 35, 35, 35, 35, 35, 35, 35, 35, 35, 35, 35, 35, 35, 35, 35, 35, 35, 35, 35, 35, 35, 35, 35, 35, 35, 35, 35, 35, 35, 35, 35, 35, 35, 35, 35, 35, 35, 35, 35, 35, 35, 35, 35, 35, 35, 35, 35, 35, 35, 35, 35, 35, 35, 35, 35, 35, 35, 35, 35, 35, 35, 35, 35, 35, 35, 35, 35, 35, 35, 35, 35, 35, 35, 35, 35, 35, 35, 35, 35, 35, 35, 34, 34, 34, 34, 34, 34, 34, 34, 34, 34, 34, 34, 34, 34, 34, 34, 34, 34, 34, 34, 34, 34, 34, 34, 34, 34, 34, 34, 34, 34, 34, 34, 34, 34, 34, 34, 34, 34, 34, 34, 34, 34, 34, 34, 34, 34, 34, 34, 34, 34, 34, 34, 34, 34, 34, 34, 34, 34, 34, 34, 34, 34, 34, 34, 34, 34, 34, 34, 34, 34, 34, 34, 34, 34, 34, 34, 34, 34, 34, 34, 34, 34, 34, 34, 34, 34, 34, 34, 34, 34, 34, 34, 34, 34, 34, 34, 34, 34, 34, 34, 34, 34, 34, 34, 34, 34, 34, 34, 34, 34, 34, 34, 34, 34, 34, 34, 34, 34, 34, 34, 34, 34, 34, 34, 34, 34, 34, 34, 33, 33, 33, 33, 33, 33, 33, 33, 33, 33, 33, 33, 33, 33, 33, 33, 33, 33, 33, 33, 33, 33, 33, 33, 33, 33, 33, 33, 33, 33, 33, 33, 33, 33, 33, 33, 33, 33, 33, 33, 33, 33, 33, 33, 33, 33, 33, 33, 33, 33, 33, 33, 33, 33, 33, 33, 33, 33, 33, 33, 33, 33, 33, 33, 33, 33, 33, 33, 33, 33, 33, 33, 33, 33, 33, 33, 33, 33, 33, 33, 33, 33, 33, 33, 33, 33, 33, 33, 33, 33, 33, 33, 33, 33, 33, 33, 33, 33, 33, 33, 33, 33, 33, 33, 33, 33, 33, 33, 33, 33, 33, 33, 33, 33, 33, 33, 33, 33, 33, 33, 33, 33, 33, 33, 33, 33, 33, 33, 32, 32, 32, 32, 32, 32, 32, 32, 32, 32, 32, 32, 32, 32, 32, 32, 32, 32, 32, 32, 32, 32, 32, 32, 32, 32, 32, 32, 32, 32, 32, 32, 32, 32, 32, 32, 32, 32, 32, 32, 32, 32, 32, 32, 32, 32, 32, 32, 32, 32, 32, 32, 32, 32, 32, 32, 32, 32, 32, 32, 32, 32, 32, 32, 32, 32, 32, 32, 32, 32, 32, 32, 32, 32, 32, 32, 32, 32, 32, 32, 32, 32, 32, 32, 32, 32, 32, 32, 32, 32, 32, 32, 32, 32, 32, 32, 32, 32, 32, 32, 32, 32, 32, 32, 32, 32, 32, 32, 32, 32, 32, 32, 32, 32, 32, 32, 32, 32, 32, 32, 32, 32, 32, 32, 32, 32, 32, 32, 31, 31, 31, 31, 31, 31, 31, 31, 31, 31, 31, 31, 31, 31, 31, 31, 31, 31, 31, 31, 31, 31, 31, 31, 31, 31, 31, 31, 31, 31, 31, 31, 31, 31, 31, 31, 31, 31, 31, 31, 31, 31, 31, 31, 31, 31, 31, 31, 31, 31, 31, 31, 31, 31, 31, 31, 31, 31, 31, 31, 31, 31, 31, 31, 31, 31, 31, 31, 31, 31, 31, 31, 31, 31, 31, 31, 31, 31, 31, 31, 31, 31, 31, 31, 31, 31, 31, 31, 31, 31, 31, 31, 31, 31, 31, 31, 31, 31, 31, 31, 31, 31, 31, 31, 31, 31, 31, 31, 31, 31, 31, 31, 31, 31, 31, 31, 31, 31, 31, 31, 31, 31, 31, 31, 31, 31, 31, 31, 30, 30, 30, 30, 30, 30, 30, 30, 30, 30, 30, 30, 30, 30, 30, 30, 30, 30, 30, 30, 30, 30, 30, 30, 30, 30, 30, 30, 30, 30, 30, 30, 30, 30, 30, 30, 30, 30, 30, 30, 30, 30, 30, 30, 30, 30, 30, 30, 30, 30, 30, 30, 30, 30, 30, 30, 30, 30, 30, 30, 30, 30, 30, 30, 30, 30, 30, 30, 30, 30, 30, 30, 30, 30, 30, 30, 30, 30, 30, 30, 30, 30, 30, 30, 30, 30, 30, 30, 30, 30, 30, 30, 30, 30, 30, 30, 30, 30, 30, 30, 30, 30, 30, 30, 30, 30, 30, 30, 30, 30, 30, 30, 30, 30, 30, 30, 30, 30, 30, 30, 30, 30, 30, 30, 30, 30, 30, 30, 29, 29, 29, 29, 29, 29, 29, 29, 29, 29, 29, 29, 29, 29, 29, 29, 29, 29, 29, 29, 29, 29, 29, 29, 29, 29, 29, 29, 29, 29, 29, 29, 29, 29, 29, 29, 29, 29, 29, 29, 29, 29, 29, 29, 29, 29, 29, 29, 29, 29, 29, 29, 29, 29, 29, 29, 29, 29, 29, 29, 29, 29, 29, 29, 29, 29, 29, 29, 29, 29, 29, 29, 29, 29, 29, 29, 29, 29, 29, 29, 29, 29, 29, 29, 29, 29, 29, 29, 29, 29, 29, 29, 29, 29, 29, 29, 29, 29, 29, 29, 29, 29, 29, 29, 29, 29, 29, 29, 29, 29, 29, 29, 29, 29, 29, 29, 29, 29, 29, 29, 29, 29, 29, 29, 29, 29, 29, 29, 28, 28, 28, 28, 28, 28, 28, 28, 28, 28, 28, 28, 28, 28, 28, 28, 28, 28, 28, 28, 28, 28, 28, 28, 28, 28, 28, 28, 28, 28, 28, 28, 28, 28, 28, 28, 28, 28, 28, 28, 28, 28, 28, 28, 28, 28, 28, 28, 28, 28, 28, 28, 28, 28, 28, 28, 28, 28, 28, 28, 28, 28, 28, 28, 28, 28, 28, 28, 28, 28, 28, 28, 28, 28, 28, 28, 28, 28, 28, 28, 28, 28, 28, 28, 28, 28, 28, 28, 28, 28, 28, 28, 28, 28, 28, 28, 28, 28, 28, 28, 28, 28, 28, 28, 28, 28, 28, 28, 28, 28, 28, 28, 28, 28, 28, 28, 28, 28, 28, 28, 28, 28, 28, 28, 28, 28, 28, 28, 28, 28, 28, 28, 28, 28, 28, 28, 28, 28, 28, 28, 28, 28, 28, 28, 28, 28, 28, 28, 28, 28, 28, 28, 28, 28, 28, 28, 28, 28, 28, 28, 28, 28, 28, 28, 28, 28, 28, 28, 28, 28, 28, 28, 28, 28, 28, 28, 28, 28, 28, 28, 28, 28, 28, 28, 28, 28, 28, 28, 28, 28, 28, 28, 28, 28, 28, 28, 28, 28, 28, 28, 28, 28, 28, 28, 28, 28, 28, 28, 28, 28, 28, 28, 28, 28, 28, 28, 28, 28, 28, 28, 28, 28, 28, 28, 28, 28, 28, 28, 28, 28, 28, 28, 28, 28, 28, 28, 28, 28, 28, 28, 28, 28, 28, 28, 28, 28, 28, 28, 28, 28, 28, 28, 28, 28, 28, 28, 27, 27, 27, 27, 27, 27, 27, 27, 27, 27, 27, 27, 27, 27, 27, 27, 27, 27, 27, 27, 27, 27, 27, 27, 27, 27, 27, 27, 27, 27, 27, 27, 27, 27, 27, 27, 27, 27, 27, 27, 27, 27, 27, 27, 27, 27, 27, 27, 27, 27, 27, 27, 27, 27, 27, 27, 27, 27, 27, 27, 27, 27, 27, 27, 27, 27, 27, 27, 27, 27, 27, 27, 27, 27, 27, 27, 27, 27, 27, 27, 27, 27, 27, 27, 27, 27, 27, 27, 27, 27, 27, 27, 27, 27, 27, 27, 27, 27, 27, 27, 27, 27, 27, 27, 27, 27, 27, 27, 27, 27, 27, 27, 27, 27, 27, 27, 27, 27, 27, 27, 27, 27, 27, 27, 27, 27, 27, 27, 26, 26, 26, 26, 26, 26, 26, 26, 26, 26, 26, 26, 26, 26, 26, 26, 26, 26, 26, 26, 26, 26, 26, 26, 26, 26, 26, 26, 26, 26, 26, 26, 26, 26, 26, 26, 26, 26, 26, 26, 26, 26, 26, 26, 26, 26, 26, 26, 26, 26, 26, 26, 26, 26, 26, 26, 26, 26, 26, 26, 26, 26, 26, 26, 26, 26, 26, 26, 26, 26, 26, 26, 26, 26, 26, 26, 26, 26, 26, 26, 26, 26, 26, 26, 26, 26, 26, 26, 26, 26, 26, 26, 26, 26, 26, 26, 26, 26, 26, 26, 26, 26, 26, 26, 26, 26, 26, 26, 26, 26, 26, 26, 26, 26, 26, 26, 26, 26, 26, 26, 26, 26, 26, 26, 26, 26, 26, 26, 25, 25, 25, 25, 25, 25, 25, 25, 25, 25, 25, 25, 25, 25, 25, 25, 25, 25, 25, 25, 25, 25, 25, 25, 25, 25, 25, 25, 25, 25, 25, 25, 25, 25, 25, 25, 25, 25, 25, 25, 25, 25, 25, 25, 25, 25, 25, 25, 25, 25, 25, 25, 25, 25, 25, 25, 25, 25, 25, 25, 25, 25, 25, 25, 25, 25, 25, 25, 25, 25, 25, 25, 25, 25, 25, 25, 25, 25, 25, 25, 25, 25, 25, 25, 25, 25, 25, 25, 25, 25, 25, 25, 25, 25, 25, 25, 25, 25, 25, 25, 25, 25, 25, 25, 25, 25, 25, 25, 25, 25, 25, 25, 25, 25, 25, 25, 25, 25, 25, 25, 25, 25, 25, 25, 25, 25, 25, 25, 25, 24, 24, 24, 24, 24, 24, 24, 24, 24, 24, 24, 24, 24, 24, 24, 24, 24, 24, 24, 24, 24, 24, 24, 24, 24, 24, 24, 24, 24, 24, 24, 24, 24, 24, 24, 24, 24, 24, 24, 24, 24, 24, 24, 24, 24, 24, 24, 24, 24, 24, 24, 24, 24, 24, 24, 24, 24, 24, 24, 24, 24, 24, 24, 24, 24, 24, 24, 24, 24, 24, 24, 24, 24, 24, 24, 24, 24, 24, 24, 24, 24, 24, 24, 24, 24, 24, 24, 24, 24, 24, 24, 24, 24, 24, 24, 24, 24, 24, 24, 24, 24, 24, 24, 24, 24, 24, 24, 24, 24, 24, 24, 24, 24, 24, 24, 24, 24, 24, 24, 24, 24, 24, 24, 24, 24, 25, 25, 25, 24, 24, 23, 23, 23, 23, 23, 23, 23, 23, 23, 23, 23, 23, 23, 23, 23, 23, 23, 23, 23, 23, 23, 23, 23, 23, 23, 23, 23, 23, 23, 23, 23, 23, 23, 23, 23, 23, 23, 23, 23, 23, 23, 23, 23, 23, 23, 23, 23, 23, 23, 23, 23, 23, 23, 23, 23, 23, 23, 23, 23, 23, 23, 23, 23, 23, 23, 23, 23, 23, 23, 23, 23, 23, 23, 23, 23, 23, 23, 23, 23, 23, 23, 23, 23, 23, 23, 23, 23, 23, 23, 23, 23, 23, 23, 23, 23, 23, 23, 23, 23, 23, 23, 23, 23, 23, 23, 23, 23, 23, 23, 23, 23, 23, 23, 23, 23, 23, 23, 23, 23, 23, 23, 24, 24, 25, 25, 25, 24, 24, 23, 23, 23, 22, 22, 22, 22, 22, 22, 22, 22, 22, 22, 22, 22, 22, 22, 22, 22, 22, 22, 22, 22, 22, 22, 22, 22, 22, 22, 22, 22, 22, 22, 22, 22, 22, 22, 22, 22, 22, 22, 22, 22, 22, 22, 22, 22, 22, 22, 22, 22, 22, 22, 22, 22, 22, 22, 22, 22, 22, 22, 22, 22, 22, 22, 22, 22, 22, 22, 22, 22, 22, 22, 22, 22, 22, 22, 22, 22, 22, 22, 22, 22, 22, 22, 22, 22, 22, 22, 22, 22, 22, 22, 22, 22, 22, 22, 22, 22, 22, 22, 22, 22, 22, 22, 22, 22, 22, 22, 22, 22, 22, 22, 22, 22, 22, 22, 22, 23, 23, 23, 24, 24, 25, 25, 25, 24, 24, 23, 23, 23, 22, 22, 22, 21, 21, 21, 21, 21, 21, 21, 21, 21, 21, 21, 21, 21, 21, 21, 21, 21, 21, 21, 21, 21, 21, 21, 21, 21, 21, 21, 21, 21, 21, 21, 21, 21, 21, 21, 21, 21, 21, 21, 21, 21, 21, 21, 21, 21, 21, 21, 21, 21, 21, 21, 21, 21, 21, 21, 21, 21, 21, 21, 21, 21, 21, 21, 21, 21, 21, 21, 21, 21, 21, 21, 21, 21, 21, 21, 21, 21, 21, 21, 21, 21, 21, 21, 21, 21, 21, 21, 21, 21, 21, 21, 21, 21, 21, 21, 21, 21, 21, 21, 21, 21, 21, 21, 21, 21, 21, 21, 21, 21, 22, 22, 22, 23, 23, 23, 24, 24, 25, 25, 25, 24, 24, 23, 23, 23, 22, 22, 22, 21, 21, 21, 20, 20, 20, 20, 20, 20, 20, 20, 20, 20, 20, 20, 20, 20, 20, 20, 20, 20, 20, 20, 20, 20, 20, 20, 20, 20, 20, 20, 20, 20, 20, 20, 20, 20, 20, 20, 20, 20, 20, 20, 20, 20, 20, 20, 20, 20, 20, 20, 20, 20, 20, 20, 20, 20, 20, 20, 20, 20, 20, 20, 20, 20, 20, 20, 20, 20, 20, 20, 20, 20, 20, 20, 20, 20, 20, 20, 20, 20, 20, 20, 20, 20, 20, 20, 20, 20, 20, 20, 20, 20, 20, 20, 20, 20, 20, 20, 20, 20, 20, 20, 20, 20, 20, 21, 21, 21, 22, 22, 22, 23, 23, 23, 24, 24, 25, 25, 25, 24, 24, 23, 23, 23, 22, 22, 22, 21, 21, 21, 20, 20, 20, 20, 19, 19, 19, 19, 19, 19, 19, 19, 19, 19, 19, 19, 19, 19, 19, 19, 19, 19, 19, 19, 19, 19, 19, 19, 19, 19, 19, 19, 19, 19, 19, 19, 19, 19, 19, 19, 19, 19, 19, 19, 19, 19, 19, 19, 19, 19, 19, 19, 19, 19, 19, 19, 19, 19, 19, 19, 19, 19, 19, 19, 19, 19, 19, 19, 19, 19, 19, 19, 19, 19, 19, 19, 19, 19, 19, 19, 19, 19, 19, 19, 19, 19, 19, 19, 19, 19, 19, 19, 19, 19, 19, 19, 19, 19, 19, 20, 20, 20, 20, 21, 21, 21, 22, 22, 22, 23, 23, 23, 24, 24, 25, 25, 25, 24, 24, 23, 23, 23, 22, 22, 22, 21, 21, 21, 20, 20, 20, 20, 19, 19, 19, 19, 19, 19, 19, 19, 19, 19, 19, 19, 19, 19, 19, 19, 19, 19, 19, 19, 19, 19, 19, 19, 19, 19, 19, 19, 19, 19, 19, 19, 19, 19, 19, 19, 19, 19, 19, 19, 19, 19, 19, 19, 19, 19, 19, 19, 19, 19, 19, 19, 19, 19, 19, 19, 19, 19, 19, 19, 19, 19, 19, 19, 19, 19, 19, 19, 19, 19, 19, 19, 19, 19, 19, 19, 19, 19, 19, 19, 19, 19, 19, 19, 19, 19, 19, 19, 19, 19, 19, 19, 19, 19, 19, 20, 20, 20, 20, 21, 21, 21, 22, 22, 22, 23, 23, 23, 24, 24, 25, 25, 25, 24, 24, 23, 23, 23, 22, 22, 22, 21, 21, 21, 20, 20, 20, 20, 19, 19, 19, 19, 18, 18, 18, 18, 18, 18, 18, 18, 18, 18, 18, 18, 18, 18, 18, 18, 18, 18, 18, 18, 18, 18, 18, 18, 18, 18, 18, 18, 18, 18, 18, 18, 18, 18, 18, 18, 18, 18, 18, 18, 18, 18, 18, 18, 18, 18, 18, 18, 18, 18, 18, 18, 18, 18, 18, 18, 18, 18, 18, 18, 18, 18, 18, 18, 18, 18, 18, 18, 18, 18, 18, 18, 18, 18, 18, 18, 18, 18, 18, 18, 18, 18, 18, 18, 18, 18, 18, 19, 19, 19, 19, 20, 20, 20, 20, 21, 21, 21, 22, 22, 22, 23, 23, 23, 24, 24, 25, 25, 25, 24, 24, 23, 23, 23, 22, 22, 22, 21, 21, 21, 20, 20, 20, 20, 19, 19, 19, 19, 18, 18, 18, 18, 18, 18, 17, 17, 17, 17, 17, 17, 17, 17, 17, 17, 17, 17, 17, 17, 17, 17, 17, 17, 17, 17, 17, 17, 17, 17, 17, 17, 17, 17, 17, 17, 17, 17, 17, 17, 17, 17, 17, 17, 17, 17, 17, 17, 17, 17, 17, 17, 17, 17, 17, 17, 17, 17, 17, 17, 17, 17, 17, 17, 17, 17, 17, 17, 17, 17, 17, 17, 17, 17, 17, 17, 17, 17, 17, 17, 17, 18, 18, 18, 18, 18, 18, 19, 19, 19, 19, 20, 20, 20, 20, 21, 21, 21, 22, 22, 22, 23, 23, 23, 24, 24, 25, 25, 25, 24, 24, 23, 23, 23, 22, 22, 22, 21, 21, 21, 20, 20, 20, 20, 19, 19, 19, 19, 18, 18, 18, 18, 18, 18, 17, 17, 17, 17, 17, 17, 17, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 17, 17, 17, 17, 17, 17, 17, 18, 18, 18, 18, 18, 18, 19, 19, 19, 19, 20, 20, 20, 20, 21, 21, 21, 22, 22, 22, 23, 23, 23, 24, 24, 25, 25, 25, 24, 24, 23, 23, 23, 22, 22, 22, 21, 21, 21, 20, 20, 20, 20, 19, 19, 19, 19, 18, 18, 18, 18, 18, 18, 17, 17, 17, 17, 17, 17, 17, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 17, 17, 17, 17, 17, 17, 17, 18, 18, 18, 18, 18, 18, 19, 19, 19, 19, 20, 20, 20, 20, 21, 21, 21, 22, 22, 22, 23, 23, 23, 24, 24, 25, 25, 25, 24, 24, 23, 23, 23, 22, 22, 22, 21, 21, 21, 20, 20, 20, 20, 19, 19, 19, 19, 18, 18, 18, 18, 18, 18, 17, 17, 17, 17, 17, 17, 17, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 17, 17, 17, 17, 17, 17, 17, 18, 18, 18, 18, 18, 18, 19, 19, 19, 19, 20, 20, 20, 20, 21, 21, 21, 22, 22, 22, 23, 23, 23, 24, 24, 25, 25, 25, 24, 24, 23, 23, 23, 22, 22, 22, 21, 21, 21, 20, 20, 20, 20, 19, 19, 19, 19, 18, 18, 18, 18, 18, 18, 17, 17, 17, 17, 17, 17, 17, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 17, 17, 17, 17, 17, 17, 17, 18, 18, 18, 18, 18, 18, 19, 19, 19, 19, 20, 20, 20, 20, 21, 21, 21, 22, 22, 22, 23, 23, 23, 24, 24, 25, 25, 25, 24, 24, 23, 23, 23, 22, 22, 22, 21, 21, 21, 20, 20, 20, 20, 19, 19, 19, 19, 18, 18, 18, 18, 18, 18, 17, 17, 17, 17, 17, 17, 17, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 17, 17, 17, 17, 17, 17, 17, 18, 18, 18, 18, 18, 18, 19, 19, 19, 19, 20, 20, 20, 20, 21, 21, 21, 22, 22, 22, 23, 23, 23, 24, 24, 25, 25, 25, 24, 24, 23, 23, 23, 22, 22, 22, 21, 21, 21, 20, 20, 20, 20, 19, 19, 19, 19, 18, 18, 18, 18, 18, 18, 17, 17, 17, 17, 17, 17, 17, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 17, 17, 17, 17, 17, 17, 17, 18, 18, 18, 18, 18, 18, 19, 19, 19, 19, 20, 20, 20, 20, 21, 21, 21, 22, 22, 22, 23, 23, 23, 24, 24, 25, 25, 25, 24, 24, 23, 23, 23, 22, 22, 22, 21, 21, 21, 20, 20, 20, 20, 19, 19, 19, 19, 18, 18, 18, 18, 18, 18, 17, 17, 17, 17, 17, 17, 17, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 17, 17, 17, 17, 17, 17, 17, 18, 18, 18, 18, 18, 18, 19, 19, 19, 19, 20, 20, 20, 20, 21, 21, 21, 22, 22, 22, 23, 23, 23, 24, 24, 25, 25, 25, 24, 24, 23, 23, 23, 22, 22, 22, 21, 21, 21, 20, 20, 20, 20, 19, 19, 19, 19, 18, 18, 18, 18, 18, 18, 17, 17, 17, 17, 17, 17, 17, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 17, 17, 17, 17, 17, 17, 17, 18, 18, 18, 18, 18, 18, 19, 19, 19, 19, 20, 20, 20, 20, 21, 21, 21, 22, 22, 22, 23, 23, 23, 24, 24, 25, 25, 25, 24, 24, 23, 23, 23, 22, 22, 22, 21, 21, 21, 20, 20, 20, 20, 19, 19, 19, 19, 18, 18, 18, 18, 18, 18, 17, 17, 17, 17, 17, 17, 17, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 17, 17, 17, 17, 17, 17, 17, 18, 18, 18, 18, 18, 18, 19, 19, 19, 19, 20, 20, 20, 20, 21, 21, 21, 22, 22, 22, 23, 23, 23, 24, 24, 25, 25, 25, 24, 24, 23, 23, 23, 22, 22, 22, 21, 21, 21, 20, 20, 20, 20, 19, 19, 19, 19, 18, 18, 18, 18, 18, 18, 17, 17, 17, 17, 17, 17, 17, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 17, 17, 17, 17, 17, 17, 17, 18, 18, 18, 18, 18, 18, 19, 19, 19, 19, 20, 20, 20, 20, 21, 21, 21, 22, 22, 22, 23, 23, 23, 24, 24, 25, 25, 25, 24, 24, 23, 23, 23, 22, 22, 22, 21, 21, 21, 20, 20, 20, 20, 19, 19, 19, 19, 18, 18, 18, 18, 18, 18, 17, 17, 17, 17, 17, 17, 17, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 17, 17, 17, 17, 17, 17, 17, 18, 18, 18, 18, 18, 18, 19, 19, 19, 19, 20, 20, 20, 20, 21, 21, 21, 22, 22, 22, 23, 23, 23, 24, 24, 25, 25, 25, 24, 24, 23, 23, 23, 22, 22, 22, 21, 21, 21, 20, 20, 20, 20, 19, 19, 19, 19, 18, 18, 18, 18, 18, 18, 17, 17, 17, 17, 17, 17, 17, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 17, 17, 17, 17, 17, 17, 17, 18, 18, 18, 18, 18, 18, 19, 19, 19, 19, 20, 20, 20, 20, 21, 21, 21, 22, 22, 22, 23, 23, 23, 24, 24, 25, 25, 25, 24, 24, 23, 23, 23, 22, 22, 22, 21, 21, 21, 20, 20, 20, 20, 19, 19, 19, 19, 18, 18, 18, 18, 18, 18, 17, 17, 17, 17, 17, 17, 17, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 17, 17, 17, 17, 17, 17, 17, 18, 18, 18, 18, 18, 18, 19, 19, 19, 19, 20, 20, 20, 20, 21, 21, 21, 22, 22, 22, 23, 23, 23, 24, 24, 25, 25, 25, 24, 24, 23, 23, 23, 22, 22, 22, 21, 21, 21, 20, 20, 20, 20, 19, 19, 19, 19, 18, 18, 18, 18, 18, 18, 17, 17, 17, 17, 17, 17, 17, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 17, 17, 17, 17, 17, 17, 17, 18, 18, 18, 18, 18, 18, 19, 19, 19, 19, 20, 20, 20, 20, 21, 21, 21, 22, 22, 22, 23, 23, 23, 24, 24, 25, 25, 25, 24, 24, 23, 23, 23, 22, 22, 22, 21, 21, 21, 20, 20, 20, 20, 19, 19, 19, 19, 18, 18, 18, 18, 18, 18, 17, 17, 17, 17, 17, 17, 17, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 17, 17, 17, 17, 17, 17, 17, 18, 18, 18, 18, 18, 18, 19, 19, 19, 19, 20, 20, 20, 20, 21, 21, 21, 22, 22, 22, 23, 23, 23, 24, 24, 25, 25, 25, 24, 24, 23, 23, 23, 22, 22, 22, 21, 21, 21, 20, 20, 20, 20, 19, 19, 19, 19, 18, 18, 18, 18, 18, 18, 17, 17, 17, 17, 17, 17, 17, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 17, 17, 17, 17, 17, 17, 17, 18, 18, 18, 18, 18, 18, 19, 19, 19, 19, 20, 20, 20, 20, 21, 21, 21, 22, 22, 22, 23, 23, 23, 24, 24, 25, 25, 25, 24, 24, 23, 23, 23, 22, 22, 22, 21, 21, 21, 20, 20, 20, 20, 19, 19, 19, 19, 18, 18, 18, 18, 18, 18, 17, 17, 17, 17, 17, 17, 17, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 17, 17, 17, 17, 17, 17, 17, 18, 18, 18, 18, 18, 18, 19, 19, 19, 19, 20, 20, 20, 20, 21, 21, 21, 22, 22, 22, 23, 23, 23, 24, 24, 25, 25, 25, 24, 24, 23, 23, 23, 22, 22, 22, 21, 21, 21, 20, 20, 20, 20, 19, 19, 19, 19, 18, 18, 18, 18, 18, 18, 17, 17, 17, 17, 17, 17, 17, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 17, 17, 17, 17, 17, 17, 17, 18, 18, 18, 18, 18, 18, 19, 19, 19, 19, 20, 20, 20, 20, 21, 21, 21, 22, 22, 22, 23, 23, 23, 24, 24, 25, 25, 25, 24, 24, 23, 23, 23, 22, 22, 22, 21, 21, 21, 20, 20, 20, 20, 19, 19, 19, 19, 18, 18, 18, 18, 18, 18, 17, 17, 17, 17, 17, 17, 17, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 17, 17, 17, 17, 17, 17, 17, 18, 18, 18, 18, 18, 18, 19, 19, 19, 19, 20, 20, 20, 20, 21, 21, 21, 22, 22, 22, 23, 23, 23, 24, 24, 25, 25, 25, 24, 24, 23, 23, 23, 22, 22, 22, 21, 21, 21, 20, 20, 20, 20, 19, 19, 19, 19, 18, 18, 18, 18, 18, 18, 17, 17, 17, 17, 17, 17, 17, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 17, 17, 17, 17, 17, 17, 17, 18, 18, 18, 18, 18, 18, 19, 19, 19, 19, 20, 20, 20, 20, 21, 21, 21, 22, 22, 22, 23, 23, 23, 24, 24, 25, 25, 25, 24, 24, 23, 23, 23, 22, 22, 22, 21, 21, 21, 20, 20, 20, 20, 19, 19, 19, 19, 18, 18, 18, 18, 18, 18, 17, 17, 17, 17, 17, 17, 17, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 17, 17, 17, 17, 17, 17, 17, 18, 18, 18, 18, 18, 18, 19, 19, 19, 19, 20, 20, 20, 20, 21, 21, 21, 22, 22, 22, 23, 23, 23, 24, 24, 25, 25, 25, 24, 24, 23, 23, 23, 22, 22, 22, 21, 21, 21, 20, 20, 20, 20, 19, 19, 19, 19, 18, 18, 18, 18, 18, 18, 17, 17, 17, 17, 17, 17, 17, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 17, 17, 17, 17, 17, 17, 17, 18, 18, 18, 18, 18, 18, 19, 19, 19, 19, 20, 20, 20, 20, 21, 21, 21, 22, 22, 22, 23, 23, 23, 24, 24, 25, 25, 25, 24, 24, 23, 23, 23, 22, 22, 22, 21, 21, 21, 20, 20, 20, 20, 19, 19, 19, 19, 18, 18, 18, 18, 18, 18, 17, 17, 17, 17, 17, 17, 17, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 17, 17, 17, 17, 17, 17, 17, 18, 18, 18, 18, 18, 18, 19, 19, 19, 19, 20, 20, 20, 20, 21, 21, 21, 22, 22, 22, 23, 23, 23, 24, 24, 25, 25, 25, 24, 24, 23, 23, 23, 22, 22, 22, 21, 21, 21, 20, 20, 20, 20, 19, 19, 19, 19, 18, 18, 18, 18, 18, 18, 17, 17, 17, 17, 17, 17, 17, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 17, 17, 17, 17, 17, 17, 17, 18, 18, 18, 18, 18, 18, 19, 19, 19, 19, 20, 20, 20, 20, 21, 21, 21, 22, 22, 22, 23, 23, 23, 24, 24, 25, 25, 25, 24, 24, 23, 23, 23, 22, 22, 22, 21, 21, 21, 20, 20, 20, 20, 19, 19, 19, 19, 18, 18, 18, 18, 18, 18, 17, 17, 17, 17, 17, 17, 17, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 17, 17, 17, 17, 17, 17, 17, 18, 18, 18, 18, 18, 18, 19, 19, 19, 19, 20, 20, 20, 20, 21, 21, 21, 22, 22, 22, 23, 23, 23, 24, 24, 25, 25, 25, 24, 24, 23, 23, 23, 22, 22, 22, 21, 21, 21, 20, 20, 20, 20, 19, 19, 19, 19, 18, 18, 18, 18, 18, 18, 17, 17, 17, 17, 17, 17, 17, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 17, 17, 17, 17, 17, 17, 17, 18, 18, 18, 18, 18, 18, 19, 19, 19, 19, 20, 20, 20, 20, 21, 21, 21, 22, 22, 22, 23, 23, 23, 24, 24, 25, 25, 25, 24, 24, 23, 23, 23, 22, 22, 22, 21, 21, 21, 20, 20, 20, 20, 19, 19, 19, 19, 18, 18, 18, 18, 18, 18, 17, 17, 17, 17, 17, 17, 17, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 17, 17, 17, 17, 17, 17, 17, 18, 18, 18, 18, 18, 18, 19, 19, 19, 19, 20, 20, 20, 20, 21, 21, 21, 22, 22, 22, 23, 23, 23, 24, 24, 25, 25, 25, 24, 24, 23, 23, 23, 22, 22, 22, 21, 21, 21, 20, 20, 20, 20, 19, 19, 19, 19, 18, 18, 18, 18, 18, 18, 17, 17, 17, 17, 17, 17, 17, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 17, 17, 17, 17, 17, 17, 17, 18, 18, 18, 18, 18, 18, 19, 19, 19, 19, 20, 20, 20, 20, 21, 21, 21, 22, 22, 22, 23, 23, 23, 24, 24, 25, 25, 25, 24, 24, 23, 23, 23, 22, 22, 22, 21, 21, 21, 20, 20, 20, 20, 19, 19, 19, 19, 18, 18, 18, 18, 18, 18, 17, 17, 17, 17, 17, 17, 17, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 17, 17, 17, 17, 17, 17, 17, 18, 18, 18, 18, 18, 18, 19, 19, 19, 19, 20, 20, 20, 20, 21, 21, 21, 22, 22, 22, 23, 23, 23, 24, 24, 25, 25, 25, 24, 24, 23, 23, 23, 22, 22, 22, 21, 21, 21, 20, 20, 20, 20, 19, 19, 19, 19, 18, 18, 18, 18, 18, 18, 17, 17, 17, 17, 17, 17, 17, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 17, 17, 17, 17, 17, 17, 17, 18, 18, 18, 18, 18, 18, 19, 19, 19, 19, 20, 20, 20, 20, 21, 21, 21, 22, 22, 22, 23, 23, 23, 24, 24, 25, 25, 25, 24, 24, 23, 23, 23, 22, 22, 22, 21, 21, 21, 20, 20, 20, 20, 19, 19, 19, 19, 18, 18, 18, 18, 18, 18, 17, 17, 17, 17, 17, 17, 17, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 17, 17, 17, 17, 17, 17, 17, 18, 18, 18, 18, 18, 18, 19, 19, 19, 19, 20, 20, 20, 20, 21, 21, 21, 22, 22, 22, 23, 23, 23, 24, 24, 25, 25, 25, 24, 24, 23, 23, 23, 22, 22, 22, 21, 21, 21, 20, 20, 20, 20, 19, 19, 19, 19, 18, 18, 18, 18, 18, 18, 17, 17, 17, 17, 17, 17, 17, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 17, 17, 17, 17, 17, 17, 17, 18, 18, 18, 18, 18, 18, 19, 19, 19, 19, 20, 20, 20, 20, 21, 21, 21, 22, 22, 22, 23, 23, 23, 24, 24, 25, 25, 25, 24, 24, 23, 23, 23, 22, 22, 22, 21, 21, 21, 20, 20, 20, 20, 19, 19, 19, 19, 18, 18, 18, 18, 18, 18, 17, 17, 17, 17, 17, 17, 17, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 17, 17, 17, 17, 17, 17, 17, 18, 18, 18, 18, 18, 18, 19, 19, 19, 19, 20, 20, 20, 20, 21, 21, 21, 22, 22, 22, 23, 23, 23, 24, 24, 25, 25, 25, 24, 24, 23, 23, 23, 22, 22, 22, 21, 21, 21, 20, 20, 20, 20, 19, 19, 19, 19, 18, 18, 18, 18, 18, 18, 17, 17, 17, 17, 17, 17, 17, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 17, 17, 17, 17, 17, 17, 17, 18, 18, 18, 18, 18, 18, 19, 19, 19, 19, 20, 20, 20, 20, 21, 21, 21, 22, 22, 22, 23, 23, 23, 24, 24, 25, 25, 25, 24, 24, 23, 23, 23, 22, 22, 22, 21, 21, 21, 20, 20, 20, 20, 19, 19, 19, 19, 18, 18, 18, 18, 18, 18, 17, 17, 17, 17, 17, 17, 17, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 17, 17, 17, 17, 17, 17, 17, 18, 18, 18, 18, 18, 18, 19, 19, 19, 19, 20, 20, 20, 20, 21, 21, 21, 22, 22, 22, 23, 23, 23, 24, 24, 25, 25, 25, 24, 24, 23, 23, 23, 22, 22, 22, 21, 21, 21, 20, 20, 20, 20, 19, 19, 19, 19, 18, 18, 18, 18, 18, 18, 17, 17, 17, 17, 17, 17, 17, 17, 17, 17, 17, 17, 17, 17, 17, 17, 17, 17, 17, 17, 17, 17, 17, 17, 17, 17, 17, 17, 17, 17, 17, 17, 17, 17, 17, 17, 17, 17, 17, 17, 17, 17, 17, 17, 17, 17, 17, 17, 17, 17, 17, 17, 17, 17, 17, 17, 17, 17, 17, 17, 17, 17, 17, 17, 17, 17, 17, 17, 17, 17, 17, 17, 17, 17, 17, 18, 18, 18, 18, 18, 18, 19, 19, 19, 19, 20, 20, 20, 20, 21, 21, 21, 22, 22, 22, 23, 23, 23, 24, 24, 25, 25, 25, 24, 24, 23, 23, 23, 22, 22, 22, 21, 21, 21, 20, 20, 20, 20, 19, 19, 19, 19, 18, 18, 18, 18, 18, 18, 18, 18, 18, 18, 18, 18, 18, 18, 18, 18, 18, 18, 18, 18, 18, 18, 18, 18, 18, 18, 18, 18, 18, 18, 18, 18, 18, 18, 18, 18, 18, 18, 18, 18, 18, 18, 18, 18, 18, 18, 18, 18, 18, 18, 18, 18, 18, 18, 18, 18, 18, 18, 18, 18, 18, 18, 18, 18, 18, 18, 18, 18, 18, 18, 18, 18, 18, 18, 18, 18, 18, 18, 18, 18, 18, 18, 18, 18, 18, 18, 18, 19, 19, 19, 19, 20, 20, 20, 20, 21, 21, 21, 22, 22, 22, 23, 23, 23, 24, 24, 25, 25, 25, 24, 24, 23, 23, 23, 22, 22, 22, 21, 21, 21, 20, 20, 20, 20, 19, 19, 19, 19, 19, 19, 19, 19, 19, 19, 19, 19, 19, 19, 19, 19, 19, 19, 19, 19, 19, 19, 19, 19, 19, 19, 19, 19, 19, 19, 19, 19, 19, 19, 19, 19, 19, 19, 19, 19, 19, 19, 19, 19, 19, 19, 19, 19, 19, 19, 19, 19, 19, 19, 19, 19, 19, 19, 19, 19, 19, 19, 19, 19, 19, 19, 19, 19, 19, 19, 19, 19, 19, 19, 19, 19, 19, 19, 19, 19, 19, 19, 19, 19, 19, 19, 19, 19, 19, 19, 19, 19, 19, 19, 19, 20, 20, 20, 20, 21, 21, 21, 22, 22, 22, 23, 23, 23, 24, 24, 25, 25, 25, 24, 24, 23, 23, 23, 22, 22, 22, 21, 21, 21, 20, 20, 20, 20, 19, 19, 19, 19, 19, 19, 19, 19, 19, 19, 19, 19, 19, 19, 19, 19, 19, 19, 19, 19, 19, 19, 19, 19, 19, 19, 19, 19, 19, 19, 19, 19, 19, 19, 19, 19, 19, 19, 19, 19, 19, 19, 19, 19, 19, 19, 19, 19, 19, 19, 19, 19, 19, 19, 19, 19, 19, 19, 19, 19, 19, 19, 19, 19, 19, 19, 19, 19, 19, 19, 19, 19, 19, 19, 19, 19, 19, 19, 19, 19, 19, 19, 19, 19, 19, 19, 19, 19, 19, 19, 19, 19, 19, 19, 19, 20, 20, 20, 20, 21, 21, 21, 22, 22, 22, 23, 23, 23, 24, 24, 25, 25, 25, 24, 24, 23, 23, 23, 22, 22, 22, 21, 21, 21, 20, 20, 20, 20, 20, 20, 20, 20, 20, 20, 20, 20, 20, 20, 20, 20, 20, 20, 20, 20, 20, 20, 20, 20, 20, 20, 20, 20, 20, 20, 20, 20, 20, 20, 20, 20, 20, 20, 20, 20, 20, 20, 20, 20, 20, 20, 20, 20, 20, 20, 20, 20, 20, 20, 20, 20, 20, 20, 20, 20, 20, 20, 20, 20, 20, 20, 20, 20, 20, 20, 20, 20, 20, 20, 20, 20, 20, 20, 20, 20, 20, 20, 20, 20, 20, 20, 20, 20, 20, 20, 20, 20, 20, 20, 20, 20, 20, 20, 20, 20, 20, 20, 20, 21, 21, 21, 22, 22, 22, 23, 23, 23, 24, 24, 25, 25, 25, 24, 24, 23, 23, 23, 22, 22, 22, 21, 21, 21, 21, 21, 21, 21, 21, 21, 21, 21, 21, 21, 21, 21, 21, 21, 21, 21, 21, 21, 21, 21, 21, 21, 21, 21, 21, 21, 21, 21, 21, 21, 21, 21, 21, 21, 21, 21, 21, 21, 21, 21, 21, 21, 21, 21, 21, 21, 21, 21, 21, 21, 21, 21, 21, 21, 21, 21, 21, 21, 21, 21, 21, 21, 21, 21, 21, 21, 21, 21, 21, 21, 21, 21, 21, 21, 21, 21, 21, 21, 21, 21, 21, 21, 21, 21, 21, 21, 21, 21, 21, 21, 21, 21, 21, 21, 21, 21, 21, 21, 21, 21, 21, 21, 21, 21, 21, 21, 22, 22, 22, 23, 23, 23, 24, 24, 25, 25, 25, 24, 24, 23, 23, 23, 22, 22, 22, 22, 22, 22, 22, 22, 22, 22, 22, 22, 22, 22, 22, 22, 22, 22, 22, 22, 22, 22, 22, 22, 22, 22, 22, 22, 22, 22, 22, 22, 22, 22, 22, 22, 22, 22, 22, 22, 22, 22, 22, 22, 22, 22, 22, 22, 22, 22, 22, 22, 22, 22, 22, 22, 22, 22, 22, 22, 22, 22, 22, 22, 22, 22, 22, 22, 22, 22, 22, 22, 22, 22, 22, 22, 22, 22, 22, 22, 22, 22, 22, 22, 22, 22, 22, 22, 22, 22, 22, 22, 22, 22, 22, 22, 22, 22, 22, 22, 22, 22, 22, 22, 22, 22, 22, 22, 22, 22, 22, 22, 22, 22, 22, 23, 23, 23, 24, 24, 25, 25, 25, 24, 24, 23, 23, 23, 23, 23, 23, 23, 23, 23, 23, 23, 23, 23, 23, 23, 23, 23, 23, 23, 23, 23, 23, 23, 23, 23, 23, 23, 23, 23, 23, 23, 23, 23, 23, 23, 23, 23, 23, 23, 23, 23, 23, 23, 23, 23, 23, 23, 23, 23, 23, 23, 23, 23, 23, 23, 23, 23, 23, 23, 23, 23, 23, 23, 23, 23, 23, 23, 23, 23, 23, 23, 23, 23, 23, 23, 23, 23, 23, 23, 23, 23, 23, 23, 23, 23, 23, 23, 23, 23, 23, 23, 23, 23, 23, 23, 23, 23, 23, 23, 23, 23, 23, 23, 23, 23, 23, 23, 23, 23, 23, 23, 23, 23, 23, 23, 23, 23, 23, 23, 23, 23, 24, 24, 25, 25, 25, 24, 24, 24, 24, 24, 24, 24, 24, 24, 24, 24, 24, 24, 24, 24, 24, 24, 24, 24, 24, 24, 24, 24, 24, 24, 24, 24, 24, 24, 24, 24, 24, 24, 24, 24, 24, 24, 24, 24, 24, 24, 24, 24, 24, 24, 24, 24, 24, 24, 24, 24, 24, 24, 24, 24, 24, 24, 24, 24, 24, 24, 24, 24, 24, 24, 24, 24, 24, 24, 24, 24, 24, 24, 24, 24, 24, 24, 24, 24, 24, 24, 24, 24, 24, 24, 24, 24, 24, 24, 24, 24, 24, 24, 24, 24, 24, 24, 24, 24, 24, 24, 24, 24, 24, 24, 24, 24, 24, 24, 24, 24, 24, 24, 24, 24, 24, 24, 24, 24, 24, 24, 24, 24, 24, 24, 25, 25, 25, 25, 25, 25, 25, 25, 25, 25, 25, 25, 25, 25, 25, 25, 25, 25, 25, 25, 25, 25, 25, 25, 25, 25, 25, 25, 25, 25, 25, 25, 25, 25, 25, 25, 25, 25, 25, 25, 25, 25, 25, 25, 25, 25, 25, 25, 25, 25, 25, 25, 25, 25, 25, 25, 25, 25, 25, 25, 25, 25, 25, 25, 25, 25, 25, 25, 25, 25, 25, 25, 25, 25, 25, 25, 25, 25, 25, 25, 25, 25, 25, 25, 25, 25, 25, 25, 25, 25, 25, 25, 25, 25, 25, 25, 25, 25, 25, 25, 25, 25, 25, 25, 25, 25, 25, 25, 25, 25, 25, 25, 25, 25, 25, 25, 25, 25, 25, 25, 25, 25, 25, 25, 25, 25, 25, 25, 25, 25, 26, 26, 26, 26, 26, 26, 26, 26, 26, 26, 26, 26, 26, 26, 26, 26, 26, 26, 26, 26, 26, 26, 26, 26, 26, 26, 26, 26, 26, 26, 26, 26, 26, 26, 26, 26, 26, 26, 26, 26, 26, 26, 26, 26, 26, 26, 26, 26, 26, 26, 26, 26, 26, 26, 26, 26, 26, 26, 26, 26, 26, 26, 26, 26, 26, 26, 26, 26, 26, 26, 26, 26, 26, 26, 26, 26, 26, 26, 26, 26, 26, 26, 26, 26, 26, 26, 26, 26, 26, 26, 26, 26, 26, 26, 26, 26, 26, 26, 26, 26, 26, 26, 26, 26, 26, 26, 26, 26, 26, 26, 26, 26, 26, 26, 26, 26, 26, 26, 26, 26, 26, 26, 26, 26, 26, 26, 26, 26, 27, 27, 27, 27, 27, 27, 27, 27, 27, 27, 27, 27, 27, 27, 27, 27, 27, 27, 27, 27, 27, 27, 27, 27, 27, 27, 27, 27, 27, 27, 27, 27, 27, 27, 27, 27, 27, 27, 27, 27, 27, 27, 27, 27, 27, 27, 27, 27, 27, 27, 27, 27, 27, 27, 27, 27, 27, 27, 27, 27, 27, 27, 27, 27, 27, 27, 27, 27, 27, 27, 27, 27, 27, 27, 27, 27, 27, 27, 27, 27, 27, 27, 27, 27, 27, 27, 27, 27, 27, 27, 27, 27, 27, 27, 27, 27, 27, 27, 27, 27, 27, 27, 27, 27, 27, 27, 27, 27, 27, 27, 27, 27, 27, 27, 27, 27, 27, 27, 27, 27, 27, 27, 27, 27, 27, 27, 27, 27, 28, 28, 28, 28, 28, 28, 28, 28, 28, 28, 28, 28, 28, 28, 28, 28, 28, 28, 28, 28, 28, 28, 28, 28, 28, 28, 28, 28, 28, 28, 28, 28, 28, 28, 28, 28, 28, 28, 28, 28, 28, 28, 28, 28, 28, 28, 28, 28, 28, 28, 28, 28, 28, 28, 28, 28, 28, 28, 28, 28, 28, 28, 28, 28, 28, 28, 28, 28, 28, 28, 28, 28, 28, 28, 28, 28, 28, 28, 28, 28, 28, 28, 28, 28, 28, 28, 28, 28, 28, 28, 28, 28, 28, 28, 28, 28, 28, 28, 28, 28, 28, 28, 28, 28, 28, 28, 28, 28, 28, 28, 28, 28, 28, 28, 28, 28, 28, 28, 28, 28, 28, 28, 28, 28, 28, 28, 28, 28, 28, 28, 28, 28, 28, 28, 28, 28, 28, 28, 28, 28, 28, 28, 28, 28, 28, 28, 28, 28, 28, 28, 28, 28, 28, 28, 28, 28, 28, 28, 28, 28, 28, 28, 28, 28, 28, 28, 28, 28, 28, 28, 28, 28, 28, 28, 28, 28, 28, 28, 28, 28, 28, 28, 28, 28, 28, 28, 28, 28, 28, 28, 28, 28, 28, 28, 28, 28, 28, 28, 28, 28, 28, 28, 28, 28, 28, 28, 28, 28, 28, 28, 28, 28, 28, 28, 28, 28, 28, 28, 28, 28, 28, 28, 28, 28, 28, 28, 28, 28, 28, 28, 28, 28, 28, 28, 28, 28, 28, 28, 28, 28, 28, 28, 28, 28, 28, 28, 28, 28, 28, 28, 28, 28, 28, 28, 28, 28, 29, 29, 29, 29, 29, 29, 29, 29, 29, 29, 29, 29, 29, 29, 29, 29, 29, 29, 29, 29, 29, 29, 29, 29, 29, 29, 29, 29, 29, 29, 29, 29, 29, 29, 29, 29, 29, 29, 29, 29, 29, 29, 29, 29, 29, 29, 29, 29, 29, 29, 29, 29, 29, 29, 29, 29, 29, 29, 29, 29, 29, 29, 29, 29, 29, 29, 29, 29, 29, 29, 29, 29, 29, 29, 29, 29, 29, 29, 29, 29, 29, 29, 29, 29, 29, 29, 29, 29, 29, 29, 29, 29, 29, 29, 29, 29, 29, 29, 29, 29, 29, 29, 29, 29, 29, 29, 29, 29, 29, 29, 29, 29, 29, 29, 29, 29, 29, 29, 29, 29, 29, 29, 29, 29, 29, 29, 29, 29, 30, 30, 30, 30, 30, 30, 30, 30, 30, 30, 30, 30, 30, 30, 30, 30, 30, 30, 30, 30, 30, 30, 30, 30, 30, 30, 30, 30, 30, 30, 30, 30, 30, 30, 30, 30, 30, 30, 30, 30, 30, 30, 30, 30, 30, 30, 30, 30, 30, 30, 30, 30, 30, 30, 30, 30, 30, 30, 30, 30, 30, 30, 30, 30, 30, 30, 30, 30, 30, 30, 30, 30, 30, 30, 30, 30, 30, 30, 30, 30, 30, 30, 30, 30, 30, 30, 30, 30, 30, 30, 30, 30, 30, 30, 30, 30, 30, 30, 30, 30, 30, 30, 30, 30, 30, 30, 30, 30, 30, 30, 30, 30, 30, 30, 30, 30, 30, 30, 30, 30, 30, 30, 30, 30, 30, 30, 30, 30, 31, 31, 31, 31, 31, 31, 31, 31, 31, 31, 31, 31, 31, 31, 31, 31, 31, 31, 31, 31, 31, 31, 31, 31, 31, 31, 31, 31, 31, 31, 31, 31, 31, 31, 31, 31, 31, 31, 31, 31, 31, 31, 31, 31, 31, 31, 31, 31, 31, 31, 31, 31, 31, 31, 31, 31, 31, 31, 31, 31, 31, 31, 31, 31, 31, 31, 31, 31, 31, 31, 31, 31, 31, 31, 31, 31, 31, 31, 31, 31, 31, 31, 31, 31, 31, 31, 31, 31, 31, 31, 31, 31, 31, 31, 31, 31, 31, 31, 31, 31, 31, 31, 31, 31, 31, 31, 31, 31, 31, 31, 31, 31, 31, 31, 31, 31, 31, 31, 31, 31, 31, 31, 31, 31, 31, 31, 31, 31, 32, 32, 32, 32, 32, 32, 32, 32, 32, 32, 32, 32, 32, 32, 32, 32, 32, 32, 32, 32, 32, 32, 32, 32, 32, 32, 32, 32, 32, 32, 32, 32, 32, 32, 32, 32, 32, 32, 32, 32, 32, 32, 32, 32, 32, 32, 32, 32, 32, 32, 32, 32, 32, 32, 32, 32, 32, 32, 32, 32, 32, 32, 32, 32, 32, 32, 32, 32, 32, 32, 32, 32, 32, 32, 32, 32, 32, 32, 32, 32, 32, 32, 32, 32, 32, 32, 32, 32, 32, 32, 32, 32, 32, 32, 32, 32, 32, 32, 32, 32, 32, 32, 32, 32, 32, 32, 32, 32, 32, 32, 32, 32, 32, 32, 32, 32, 32, 32, 32, 32, 32, 32, 32, 32, 32, 32, 32, 32, 33, 33, 33, 33, 33, 33, 33, 33, 33, 33, 33, 33, 33, 33, 33, 33, 33, 33, 33, 33, 33, 33, 33, 33, 33, 33, 33, 33, 33, 33, 33, 33, 33, 33, 33, 33, 33, 33, 33, 33, 33, 33, 33, 33, 33, 33, 33, 33, 33, 33, 33, 33, 33, 33, 33, 33, 33, 33, 33, 33, 33, 33, 33, 33, 33, 33, 33, 33, 33, 33, 33, 33, 33, 33, 33, 33, 33, 33, 33, 33, 33, 33, 33, 33, 33, 33, 33, 33, 33, 33, 33, 33, 33, 33, 33, 33, 33, 33, 33, 33, 33, 33, 33, 33, 33, 33, 33, 33, 33, 33, 33, 33, 33, 33, 33, 33, 33, 33, 33, 33, 33, 33, 33, 33, 33, 33, 33, 33, 34, 34, 34, 34, 34, 34, 34, 34, 34, 34, 34, 34, 34, 34, 34, 34, 34, 34, 34, 34, 34, 34, 34, 34, 34, 34, 34, 34, 34, 34, 34, 34, 34, 34, 34, 34, 34, 34, 34, 34, 34, 34, 34, 34, 34, 34, 34, 34, 34, 34, 34, 34, 34, 34, 34, 34, 34, 34, 34, 34, 34, 34, 34, 34, 34, 34, 34, 34, 34, 34, 34, 34, 34, 34, 34, 34, 34, 34, 34, 34, 34, 34, 34, 34, 34, 34, 34, 34, 34, 34, 34, 34, 34, 34, 34, 34, 34, 34, 34, 34, 34, 34, 34, 34, 34, 34, 34, 34, 34, 34, 34, 34, 34, 34, 34, 34, 34, 34, 34, 34, 34, 34, 34, 34, 34, 34, 34, 34, 35, 35, 35, 35, 35, 35, 35, 35, 35, 35, 35, 35, 35, 35, 35, 35, 35, 35, 35, 35, 35, 35, 35, 35, 35, 35, 35, 35, 35, 35, 35, 35, 35, 35, 35, 35, 35, 35, 35, 35, 35, 35, 35, 35, 35, 35, 35, 35, 35, 35, 35, 35, 35, 35, 35, 35, 35, 35, 35, 35, 35, 35, 35, 35, 35, 35, 35, 35, 35, 35, 35, 35, 35, 35, 35, 35, 35, 35, 35, 35, 35, 35, 35, 35, 35, 35, 35, 35, 35, 35, 35, 35, 35, 35, 35, 35, 35, 35, 35, 35, 35, 35, 35, 35, 35, 35, 35, 35, 35, 35, 35, 35, 35, 35, 35, 35, 35, 35, 35, 35, 35, 35, 35, 35, 35, 35, 35, 35, 36, 36, 36, 36, 36, 36, 36, 36, 36, 36, 36, 36, 36, 36, 36, 36, 36, 36, 36, 36, 36, 36, 36, 36, 36, 36, 36, 36, 36, 36, 36, 36, 36, 36, 36, 36, 36, 36, 36, 36, 36, 36, 36, 36, 36, 36, 36, 36, 36, 36, 36, 36, 36, 36, 36, 36, 36, 36, 36, 36, 36, 36, 36, 36, 36, 36, 36, 36, 36, 36, 36, 36, 36, 36, 36, 36, 36, 36, 36, 36, 36, 36, 36, 36, 36, 36, 36, 36, 36, 36, 36, 36, 36, 36, 36, 36, 36, 36, 36, 36, 36, 36, 36, 36, 36, 36, 36, 36, 36, 36, 36, 36, 36, 36, 36, 36, 36, 36, 36, 36, 36, 36, 36, 36, 36, 36, 36, 36, 37, 37, 37, 37, 37, 37, 37, 37, 37, 37, 37, 37, 37, 37, 37, 37, 37, 37, 37, 37, 37, 37, 37, 37, 37, 37, 37, 37, 37, 37, 37, 37, 37, 37, 37, 37, 37, 37, 37, 37, 37, 37, 37, 37, 37, 37, 37, 37, 37, 37, 37, 37, 37, 37, 37, 37, 37, 37, 37, 37, 37, 37, 37, 37, 37, 37, 37, 37, 37, 37, 37, 37, 37, 37, 37, 37, 37, 37, 37, 37, 37, 37, 37, 37, 37, 37, 37, 37, 37, 37, 37, 37, 37, 37, 37, 37, 37, 37, 37, 37, 37, 37, 37, 37, 37, 37, 37, 37, 37, 37, 37, 37, 37, 37, 37, 37, 37, 37, 37, 37, 37, 37, 37, 37, 37, 37, 37, 37, 37, 37, 37, 37, 37, 37, 37, 37, 37, 37, 37, 37, 37, 37, 37, 37, 37, 37, 37, 37, 37, 37, 37, 37, 37, 37, 37, 37, 37, 37, 37, 37, 37, 37, 37, 37, 37, 37, 37, 37, 37, 37, 37, 37, 37, 37, 37, 37, 37, 37, 37, 37, 37, 37, 37, 37, 37, 37, 37, 37, 37, 37, 37, 37, 37, 37, 37, 37, 37, 37, 37, 37, 37, 37, 37, 37, 37, 37, 37, 37, 37, 37, 37, 37, 37, 37, 37, 37, 37, 37, 37, 37, 37, 37, 37, 37, 37, 37, 37, 37, 37, 37, 37, 37, 37, 37, 37, 37, 37, 37, 37, 37, 37, 37, 37, 37, 37, 37, 37, 37, 37, 37, 37, 37, 37, 37, 37, 37, 38, 38, 38, 38, 38, 38, 38, 38, 38, 38, 38, 38, 38, 38, 38, 38, 38, 38, 38, 38, 38, 38, 38, 38, 38, 38, 38, 38, 38, 38, 38, 38, 38, 38, 38, 38, 38, 38, 38, 38, 38, 38, 38, 38, 38, 38, 38, 38, 38, 38, 38, 38, 38, 38, 38, 38, 38, 38, 38, 38, 38, 38, 38, 38, 38, 38, 38, 38, 38, 38, 38, 38, 38, 38, 38, 38, 38, 38, 38, 38, 38, 38, 38, 38, 38, 38, 38, 38, 38, 38, 38, 38, 38, 38, 38, 38, 38, 38, 38, 38, 38, 38, 38, 38, 38, 38, 38, 38, 38, 38, 38, 38, 38, 38, 38, 38, 38, 38, 38, 38, 38, 38, 38, 38, 38, 38, 38, 38, 39, 39, 39, 39, 39, 39, 39, 39, 39, 39, 39, 39, 39, 39, 39, 39, 39, 39, 39, 39, 39, 39, 39, 39, 39, 39, 39, 39, 39, 39, 39, 39, 39, 39, 39, 39, 39, 39, 39, 39, 39, 39, 39, 39, 39, 39, 39, 39, 39, 39, 39, 39, 39, 39, 39, 39, 39, 39, 39, 39, 39, 39, 39, 39, 39, 39, 39, 39, 39, 39, 39, 39, 39, 39, 39, 39, 39, 39, 39, 39, 39, 39, 39, 39, 39, 39, 39, 39, 39, 39, 39, 39, 39, 39, 39, 39, 39, 39, 39, 39, 39, 39, 39, 39, 39, 39, 39, 39, 39, 39, 39, 39, 39, 39, 39, 39, 39, 39, 39, 39, 39, 39, 39, 39, 39, 39, 39, 39, 40, 40, 40, 40, 40, 40, 40, 40, 40, 40, 40, 40, 40, 40, 40, 40, 40, 40, 40, 40, 40, 40, 40, 40, 40, 40, 40, 40, 40, 40, 40, 40, 40, 40, 40, 40, 40, 40, 40, 40, 40, 40, 40, 40, 40, 40, 40, 40, 40, 40, 40, 40, 40, 40, 40, 40, 40, 40, 40, 40, 40, 40, 40, 40, 40, 40, 40, 40, 40, 40, 40, 40, 40, 40, 40, 40, 40, 40, 40, 40, 40, 40, 40, 40, 40, 40, 40, 40, 40, 40, 40, 40, 40, 40, 40, 40, 40, 40, 40, 40, 40, 40, 40, 40, 40, 40, 40, 40, 40, 40, 40, 40, 40, 40, 40, 40, 40, 40, 40, 40, 40, 40, 40, 40, 40, 40, 40, 40, 41, 41, 41, 41, 41, 41, 41, 41, 41, 41, 41, 41, 41, 41, 41, 41, 41, 41, 41, 41, 41, 41, 41, 41, 41, 41, 41, 41, 41, 41, 41, 41, 41, 41, 41, 41, 41, 41, 41, 41, 41, 41, 41, 41, 41, 41, 41, 41, 41, 41, 41, 41, 41, 41, 41, 41, 41, 41, 41, 41, 41, 41, 41, 41, 41, 41, 41, 41, 41, 41, 41, 41, 41, 41, 41, 41, 41, 41, 41, 41, 41, 41, 41, 41, 41, 41, 41, 41, 41, 41, 41, 41, 41, 41, 41, 41, 41, 41, 41, 41, 41, 41, 41, 41, 41, 41, 41, 41, 41, 41, 41, 41, 41, 41, 41, 41, 41, 41, 41, 41, 41, 41, 41, 41, 41, 41, 41, 41, 42, 42, 42, 42, 42, 42, 42, 42, 42, 42, 42, 42, 42, 42, 42, 42, 42, 42, 42, 42, 42, 42, 42, 42, 42, 42, 42, 42, 42, 42, 42, 42, 42, 42, 42, 42, 42, 42, 42, 42, 42, 42, 42, 42, 42, 42, 42, 42, 42, 42, 42, 42, 42, 42, 42, 42, 42, 42, 42, 42, 42, 42, 42, 42, 42, 42, 42, 42, 42, 42, 42, 42, 42, 42, 42, 42, 42, 42, 42, 42, 42, 42, 42, 42, 42, 42, 42, 42, 42, 42, 42, 42, 42, 42, 42, 42, 42, 42, 42, 42, 42, 42, 42, 42, 42, 42, 42, 42, 42, 42, 42, 42, 42, 42, 42, 42, 42, 42, 42, 42, 42, 42, 42, 42, 42, 42, 42, 42, 43, 43, 43, 43, 43, 43, 43, 43, 43, 43, 43, 43, 43, 43, 43, 43, 43, 43, 43, 43, 43, 43, 43, 43, 43, 43, 43, 43, 43, 43, 43, 43, 43, 43, 43, 43, 43, 43, 43, 43, 43, 43, 43, 43, 43, 43, 43, 43, 43, 43, 43, 43, 43, 43, 43, 43, 43, 43, 43, 43, 43, 43, 43, 43, 43, 43, 43, 43, 43, 43, 43, 43, 43, 43, 43, 43, 43, 43, 43, 43, 43, 43, 43, 43, 43, 43, 43, 43, 43, 43, 43, 43, 43, 43, 43, 43, 43, 43, 43, 43, 43, 43, 43, 43, 43, 43, 43, 43, 43, 43, 43, 43, 43, 43, 43, 43, 43, 43, 43, 43, 43, 43, 43, 43, 43, 43, 43, 43, 44, 44, 44, 44, 44, 44, 44, 44, 44, 44, 44, 44, 44, 44, 44, 44, 44, 44, 44, 44, 44, 44, 44, 44, 44, 44, 44, 44, 44, 44, 44, 44, 44, 44, 44, 44, 44, 44, 44, 44, 44, 44, 44, 44, 44, 44, 44, 44, 44, 44, 44, 44, 44, 44, 44, 44, 44, 44, 44, 44, 44, 44, 44, 44, 44, 44, 44, 44, 44, 44, 44, 44, 44, 44, 44, 44, 44, 44, 44, 44, 44, 44, 44, 44, 44, 44, 44, 44, 44, 44, 44, 44, 44, 44, 44, 44, 44, 44, 44, 44, 44, 44, 44, 44, 44, 44, 44, 44, 44, 44, 44, 44, 44, 44, 44, 44, 44, 44, 44, 44, 44, 44, 44, 44, 44, 44, 44, 44};

  // max_Y = 300 mm
  // static const uint8_t maximum_disparities[12288] = {57,57,57,57,57,57,57,57,57,57,57,57,57,57,57,57,57,57,57,57,57,57,57,57,57,57,57,57,57,57,57,57,57,57,57,57,57,57,57,57,57,57,57,57,57,57,57,57,57,57,57,57,57,57,57,57,57,57,57,57,57,57,57,57,57,57,57,57,57,57,57,57,57,57,57,57,57,57,57,57,57,57,57,57,57,57,57,57,57,57,57,57,57,57,57,57,57,57,57,57,57,57,57,57,57,57,57,57,57,57,57,57,57,57,57,57,57,57,57,57,57,57,57,57,57,57,57,57,56,56,56,56,56,56,56,56,56,56,56,56,56,56,56,56,56,56,56,56,56,56,56,56,56,56,56,56,56,56,56,56,56,56,56,56,56,56,56,56,56,56,56,56,56,56,56,56,56,56,56,56,56,56,56,56,56,56,56,56,56,56,56,56,56,56,56,56,56,56,56,56,56,56,56,56,56,56,56,56,56,56,56,56,56,56,56,56,56,56,56,56,56,56,56,56,56,56,56,56,56,56,56,56,56,56,56,56,56,56,56,56,56,56,56,56,56,56,56,56,56,56,56,56,56,56,56,56,55,55,55,55,55,55,55,55,55,55,55,55,55,55,55,55,55,55,55,55,55,55,55,55,55,55,55,55,55,55,55,55,55,55,55,55,55,55,55,55,55,55,55,55,55,55,55,55,55,55,55,55,55,55,55,55,55,55,55,55,55,55,55,55,55,55,55,55,55,55,55,55,55,55,55,55,55,55,55,55,55,55,55,55,55,55,55,55,55,55,55,55,55,55,55,55,55,55,55,55,55,55,55,55,55,55,55,55,55,55,55,55,55,55,55,55,55,55,55,55,55,55,55,55,55,55,55,55,53,53,53,53,53,53,53,53,53,53,53,53,53,53,53,53,53,53,53,53,53,53,53,53,53,53,53,53,53,53,53,53,53,53,53,53,53,53,53,53,53,53,53,53,53,53,53,53,53,53,53,53,53,53,53,53,53,53,53,53,53,53,53,53,53,53,53,53,53,53,53,53,53,53,53,53,53,53,53,53,53,53,53,53,53,53,53,53,53,53,53,53,53,53,53,53,53,53,53,53,53,53,53,53,53,53,53,53,53,53,53,53,53,53,53,53,53,53,53,53,53,53,53,53,53,53,53,53,52,52,52,52,52,52,52,52,52,52,52,52,52,52,52,52,52,52,52,52,52,52,52,52,52,52,52,52,52,52,52,52,52,52,52,52,52,52,52,52,52,52,52,52,52,52,52,52,52,52,52,52,52,52,52,52,52,52,52,52,52,52,52,52,52,52,52,52,52,52,52,52,52,52,52,52,52,52,52,52,52,52,52,52,52,52,52,52,52,52,52,52,52,52,52,52,52,52,52,52,52,52,52,52,52,52,52,52,52,52,52,52,52,52,52,52,52,52,52,52,52,52,52,52,52,52,52,52,51,51,51,51,51,51,51,51,51,51,51,51,51,51,51,51,51,51,51,51,51,51,51,51,51,51,51,51,51,51,51,51,51,51,51,51,51,51,51,51,51,51,51,51,51,51,51,51,51,51,51,51,51,51,51,51,51,51,51,51,51,51,51,51,51,51,51,51,51,51,51,51,51,51,51,51,51,51,51,51,51,51,51,51,51,51,51,51,51,51,51,51,51,51,51,51,51,51,51,51,51,51,51,51,51,51,51,51,51,51,51,51,51,51,51,51,51,51,51,51,51,51,51,51,51,51,51,51,50,50,50,50,50,50,50,50,50,50,50,50,50,50,50,50,50,50,50,50,50,50,50,50,50,50,50,50,50,50,50,50,50,50,50,50,50,50,50,50,50,50,50,50,50,50,50,50,50,50,50,50,50,50,50,50,50,50,50,50,50,50,50,50,50,50,50,50,50,50,50,50,50,50,50,50,50,50,50,50,50,50,50,50,50,50,50,50,50,50,50,50,50,50,50,50,50,50,50,50,50,50,50,50,50,50,50,50,50,50,50,50,50,50,50,50,50,50,50,50,50,50,50,50,50,50,50,50,49,49,49,49,49,49,49,49,49,49,49,49,49,49,49,49,49,49,49,49,49,49,49,49,49,49,49,49,49,49,49,49,49,49,49,49,49,49,49,49,49,49,49,49,49,49,49,49,49,49,49,49,49,49,49,49,49,49,49,49,49,49,49,49,49,49,49,49,49,49,49,49,49,49,49,49,49,49,49,49,49,49,49,49,49,49,49,49,49,49,49,49,49,49,49,49,49,49,49,49,49,49,49,49,49,49,49,49,49,49,49,49,49,49,49,49,49,49,49,49,49,49,49,49,49,49,49,49,47,47,47,47,47,47,47,47,47,47,47,47,47,47,47,47,47,47,47,47,47,47,47,47,47,47,47,47,47,47,47,47,47,47,47,47,47,47,47,47,47,47,47,47,47,47,47,47,47,47,47,47,47,47,47,47,47,47,47,47,47,47,47,47,47,47,47,47,47,47,47,47,47,47,47,47,47,47,47,47,47,47,47,47,47,47,47,47,47,47,47,47,47,47,47,47,47,47,47,47,47,47,47,47,47,47,47,47,47,47,47,47,47,47,47,47,47,47,47,47,47,47,47,47,47,47,47,47,46,46,46,46,46,46,46,46,46,46,46,46,46,46,46,46,46,46,46,46,46,46,46,46,46,46,46,46,46,46,46,46,46,46,46,46,46,46,46,46,46,46,46,46,46,46,46,46,46,46,46,46,46,46,46,46,46,46,46,46,46,46,46,46,46,46,46,46,46,46,46,46,46,46,46,46,46,46,46,46,46,46,46,46,46,46,46,46,46,46,46,46,46,46,46,46,46,46,46,46,46,46,46,46,46,46,46,46,46,46,46,46,46,46,46,46,46,46,46,46,46,46,46,46,46,46,46,46,45,45,45,45,45,45,45,45,45,45,45,45,45,45,45,45,45,45,45,45,45,45,45,45,45,45,45,45,45,45,45,45,45,45,45,45,45,45,45,45,45,45,45,45,45,45,45,45,45,45,45,45,45,45,45,45,45,45,45,45,45,45,45,45,45,45,45,45,45,45,45,45,45,45,45,45,45,45,45,45,45,45,45,45,45,45,45,45,45,45,45,45,45,45,45,45,45,45,45,45,45,45,45,45,45,45,45,45,45,45,45,45,45,45,45,45,45,45,45,45,45,45,45,45,45,45,45,45,44,44,44,44,44,44,44,44,44,44,44,44,44,44,44,44,44,44,44,44,44,44,44,44,44,44,44,44,44,44,44,44,44,44,44,44,44,44,44,44,44,44,44,44,44,44,44,44,44,44,44,44,44,44,44,44,44,44,44,44,44,44,44,44,44,44,44,44,44,44,44,44,44,44,44,44,44,44,44,44,44,44,44,44,44,44,44,44,44,44,44,44,44,44,44,44,44,44,44,44,44,44,44,44,44,44,44,44,44,44,44,44,44,44,44,44,44,44,44,44,44,44,44,44,44,44,44,44,43,43,43,43,43,43,43,43,43,43,43,43,43,43,43,43,43,43,43,43,43,43,43,43,43,43,43,43,43,43,43,43,43,43,43,43,43,43,43,43,43,43,43,43,43,43,43,43,43,43,43,43,43,43,43,43,43,43,43,43,43,43,43,43,43,43,43,43,43,43,43,43,43,43,43,43,43,43,43,43,43,43,43,43,43,43,43,43,43,43,43,43,43,43,43,43,43,43,43,43,43,43,43,43,43,43,43,43,43,43,43,43,43,43,43,43,43,43,43,43,43,43,43,43,43,43,43,43,41,41,41,41,41,41,41,41,41,41,41,41,41,41,41,41,41,41,41,41,41,41,41,41,41,41,41,41,41,41,41,41,41,41,41,41,41,41,41,41,41,41,41,41,41,41,41,41,41,41,41,41,41,41,41,41,41,41,41,41,41,41,41,41,41,41,41,41,41,41,41,41,41,41,41,41,41,41,41,41,41,41,41,41,41,41,41,41,41,41,41,41,41,41,41,41,41,41,41,41,41,41,41,41,41,41,41,41,41,41,41,41,41,41,41,41,41,41,41,41,41,41,41,41,41,41,41,41,40,40,40,40,40,40,40,40,40,40,40,40,40,40,40,40,40,40,40,40,40,40,40,40,40,40,40,40,40,40,40,40,40,40,40,40,40,40,40,40,40,40,40,40,40,40,40,40,40,40,40,40,40,40,40,40,40,40,40,40,40,40,40,40,40,40,40,40,40,40,40,40,40,40,40,40,40,40,40,40,40,40,40,40,40,40,40,40,40,40,40,40,40,40,40,40,40,40,40,40,40,40,40,40,40,40,40,40,40,40,40,40,40,40,40,40,40,40,40,40,40,40,40,40,40,40,40,40,39,39,39,39,39,39,39,39,39,39,39,39,39,39,39,39,39,39,39,39,39,39,39,39,39,39,39,39,39,39,39,39,39,39,39,39,39,39,39,39,39,39,39,39,39,39,39,39,39,39,39,39,39,39,39,39,39,39,39,39,39,39,39,39,39,39,39,39,39,39,39,39,39,39,39,39,39,39,39,39,39,39,39,39,39,39,39,39,39,39,39,39,39,39,39,39,39,39,39,39,39,39,39,39,39,39,39,39,39,39,39,39,39,39,39,39,39,39,39,39,39,39,39,39,39,39,39,39,38,38,38,38,38,38,38,38,38,38,38,38,38,38,38,38,38,38,38,38,38,38,38,38,38,38,38,38,38,38,38,38,38,38,38,38,38,38,38,38,38,38,38,38,38,38,38,38,38,38,38,38,38,38,38,38,38,38,38,38,38,38,38,38,38,38,38,38,38,38,38,38,38,38,38,38,38,38,38,38,38,38,38,38,38,38,38,38,38,38,38,38,38,38,38,38,38,38,38,38,38,38,38,38,38,38,38,38,38,38,38,38,38,38,38,38,38,38,38,38,38,38,38,38,38,38,38,38,37,37,37,37,37,37,37,37,37,37,37,37,37,37,37,37,37,37,37,37,37,37,37,37,37,37,37,37,37,37,37,37,37,37,37,37,37,37,37,37,37,37,37,37,37,37,37,37,37,37,37,37,37,37,37,37,37,37,37,37,37,37,37,37,37,37,37,37,37,37,37,37,37,37,37,37,37,37,37,37,37,37,37,37,37,37,37,37,37,37,37,37,37,37,37,37,37,37,37,37,37,37,37,37,37,37,37,37,37,37,37,37,37,37,37,37,37,37,37,37,37,37,37,37,37,37,37,37,35,35,35,35,35,35,35,35,35,35,35,35,35,35,35,35,35,35,35,35,35,35,35,35,35,35,35,35,35,35,35,35,35,35,35,35,35,35,35,35,35,35,35,35,35,35,35,35,35,35,35,35,35,35,35,35,35,35,35,35,35,35,35,35,35,35,35,35,35,35,35,35,35,35,35,35,35,35,35,35,35,35,35,35,35,35,35,35,35,35,35,35,35,35,35,35,35,35,35,35,35,35,35,35,35,35,35,35,35,35,35,35,35,35,35,35,35,35,35,35,35,35,35,35,35,35,35,35,34,34,34,34,34,34,34,34,34,34,34,34,34,34,34,34,34,34,34,34,34,34,34,34,34,34,34,34,34,34,34,34,34,34,34,34,34,34,34,34,34,34,34,34,34,34,34,34,34,34,34,34,34,34,34,34,34,34,34,34,34,34,34,34,34,34,34,34,34,34,34,34,34,34,34,34,34,34,34,34,34,34,34,34,34,34,34,34,34,34,34,34,34,34,34,34,34,34,34,34,34,34,34,34,34,34,34,34,34,34,34,34,34,34,34,34,34,34,34,34,34,34,34,34,34,34,34,34,33,33,33,33,33,33,33,33,33,33,33,33,33,33,33,33,33,33,33,33,33,33,33,33,33,33,33,33,33,33,33,33,33,33,33,33,33,33,33,33,33,33,33,33,33,33,33,33,33,33,33,33,33,33,33,33,33,33,33,33,33,33,33,33,33,33,33,33,33,33,33,33,33,33,33,33,33,33,33,33,33,33,33,33,33,33,33,33,33,33,33,33,33,33,33,33,33,33,33,33,33,33,33,33,33,33,33,33,33,33,33,33,33,33,33,33,33,33,33,33,33,33,33,33,33,33,33,33,32,32,32,32,32,32,32,32,32,32,32,32,32,32,32,32,32,32,32,32,32,32,32,32,32,32,32,32,32,32,32,32,32,32,32,32,32,32,32,32,32,32,32,32,32,32,32,32,32,32,32,32,32,32,32,32,32,32,32,32,32,32,32,32,32,32,32,32,32,32,32,32,32,32,32,32,32,32,32,32,32,32,32,32,32,32,32,32,32,32,32,32,32,32,32,32,32,32,32,32,32,32,32,32,32,32,32,32,32,32,32,32,32,32,32,32,32,32,32,32,32,32,32,32,32,32,32,32,31,31,31,31,31,31,31,31,31,31,31,31,31,31,31,31,31,31,31,31,31,31,31,31,31,31,31,31,31,31,31,31,31,31,31,31,31,31,31,31,31,31,31,31,31,31,31,31,31,31,31,31,31,31,31,31,31,31,31,31,31,31,31,31,31,31,31,31,31,31,31,31,31,31,31,31,31,31,31,31,31,31,31,31,31,31,31,31,31,31,31,31,31,31,31,31,31,31,31,31,31,31,31,31,31,31,31,31,31,31,31,31,31,31,31,31,31,31,31,31,31,31,31,31,31,31,31,31,29,29,29,29,29,29,29,29,29,29,29,29,29,29,29,29,29,29,29,29,29,29,29,29,29,29,29,29,29,29,29,29,29,29,29,29,29,29,29,29,29,29,29,29,29,29,29,29,29,29,29,29,29,29,29,29,29,29,29,29,29,29,29,29,29,29,29,29,29,29,29,29,29,29,29,29,29,29,29,29,29,29,29,29,29,29,29,29,29,29,29,29,29,29,29,29,29,29,29,29,29,29,29,29,29,29,29,29,29,29,29,29,29,29,29,29,29,29,29,29,29,29,29,29,29,29,29,29,28,28,28,28,28,28,28,28,28,28,28,28,28,28,28,28,28,28,28,28,28,28,28,28,28,28,28,28,28,28,28,28,28,28,28,28,28,28,28,28,28,28,28,28,28,28,28,28,28,28,28,28,28,28,28,28,28,28,28,28,28,28,28,28,28,28,28,28,28,28,28,28,28,28,28,28,28,28,28,28,28,28,28,28,28,28,28,28,28,28,28,28,28,28,28,28,28,28,28,28,28,28,28,28,28,28,28,28,28,28,28,28,28,28,28,28,28,28,28,28,28,28,28,28,28,28,28,28,27,27,27,27,27,27,27,27,27,27,27,27,27,27,27,27,27,27,27,27,27,27,27,27,27,27,27,27,27,27,27,27,27,27,27,27,27,27,27,27,27,27,27,27,27,27,27,27,27,27,27,27,27,27,27,27,27,27,27,27,27,27,27,27,27,27,27,27,27,27,27,27,27,27,27,27,27,27,27,27,27,27,27,27,27,27,27,27,27,27,27,27,27,27,27,27,27,27,27,27,27,27,27,27,27,27,27,27,27,27,27,27,27,27,27,27,27,27,27,27,27,27,27,27,27,27,27,27,26,26,26,26,26,26,26,26,26,26,26,26,26,26,26,26,26,26,26,26,26,26,26,26,26,26,26,26,26,26,26,26,26,26,26,26,26,26,26,26,26,26,26,26,26,26,26,26,26,26,26,26,26,26,26,26,26,26,26,26,26,26,26,26,26,26,26,26,26,26,26,26,26,26,26,26,26,26,26,26,26,26,26,26,26,26,26,26,26,26,26,26,26,26,26,26,26,26,26,26,26,26,26,26,26,26,26,26,26,26,26,26,26,26,26,26,26,26,26,26,26,26,26,26,26,26,26,26,25,25,25,25,25,25,25,25,25,25,25,25,25,25,25,25,25,25,25,25,25,25,25,25,25,25,25,25,25,25,25,25,25,25,25,25,25,25,25,25,25,25,25,25,25,25,25,25,25,25,25,25,25,25,25,25,25,25,25,25,25,25,25,25,25,25,25,25,25,25,25,25,25,25,25,25,25,25,25,25,25,25,25,25,25,25,25,25,25,25,25,25,25,25,25,25,25,25,25,25,25,25,25,25,25,25,25,25,25,25,25,25,25,25,25,25,25,25,25,25,25,25,25,25,25,25,25,25,25,24,24,23,23,23,23,23,23,23,23,23,23,23,23,23,23,23,23,23,23,23,23,23,23,23,23,23,23,23,23,23,23,23,23,23,23,23,23,23,23,23,23,23,23,23,23,23,23,23,23,23,23,23,23,23,23,23,23,23,23,23,23,23,23,23,23,23,23,23,23,23,23,23,23,23,23,23,23,23,23,23,23,23,23,23,23,23,23,23,23,23,23,23,23,23,23,23,23,23,23,23,23,23,23,23,23,23,23,23,23,23,23,23,23,23,23,23,23,23,23,23,23,23,24,24,25,25,25,24,24,23,23,23,22,22,22,22,22,22,22,22,22,22,22,22,22,22,22,22,22,22,22,22,22,22,22,22,22,22,22,22,22,22,22,22,22,22,22,22,22,22,22,22,22,22,22,22,22,22,22,22,22,22,22,22,22,22,22,22,22,22,22,22,22,22,22,22,22,22,22,22,22,22,22,22,22,22,22,22,22,22,22,22,22,22,22,22,22,22,22,22,22,22,22,22,22,22,22,22,22,22,22,22,22,22,22,22,22,22,22,22,22,22,22,22,22,22,22,23,23,23,24,24,25,25,25,24,24,23,23,23,22,22,22,21,21,21,21,21,21,21,21,21,21,21,21,21,21,21,21,21,21,21,21,21,21,21,21,21,21,21,21,21,21,21,21,21,21,21,21,21,21,21,21,21,21,21,21,21,21,21,21,21,21,21,21,21,21,21,21,21,21,21,21,21,21,21,21,21,21,21,21,21,21,21,21,21,21,21,21,21,21,21,21,21,21,21,21,21,21,21,21,21,21,21,21,21,21,21,21,21,21,21,21,21,21,21,21,21,21,21,21,21,22,22,22,23,23,23,24,24,25,25,25,24,24,23,23,23,22,22,22,21,21,21,20,20,20,20,20,20,20,20,20,20,20,20,20,20,20,20,20,20,20,20,20,20,20,20,20,20,20,20,20,20,20,20,20,20,20,20,20,20,20,20,20,20,20,20,20,20,20,20,20,20,20,20,20,20,20,20,20,20,20,20,20,20,20,20,20,20,20,20,20,20,20,20,20,20,20,20,20,20,20,20,20,20,20,20,20,20,20,20,20,20,20,20,20,20,20,20,20,20,20,20,20,20,20,21,21,21,22,22,22,23,23,23,24,24,25,25,25,24,24,23,23,23,22,22,22,21,21,21,20,20,20,20,19,19,19,19,19,19,19,19,19,19,19,19,19,19,19,19,19,19,19,19,19,19,19,19,19,19,19,19,19,19,19,19,19,19,19,19,19,19,19,19,19,19,19,19,19,19,19,19,19,19,19,19,19,19,19,19,19,19,19,19,19,19,19,19,19,19,19,19,19,19,19,19,19,19,19,19,19,19,19,19,19,19,19,19,19,19,19,19,19,19,19,19,19,19,19,20,20,20,20,21,21,21,22,22,22,23,23,23,24,24,25,25,25,24,24,23,23,23,22,22,22,21,21,21,20,20,20,20,19,19,19,19,18,18,18,18,18,18,17,17,17,17,17,17,17,17,17,17,17,17,17,17,17,17,17,17,17,17,17,17,17,17,17,17,17,17,17,17,17,17,17,17,17,17,17,17,17,17,17,17,17,17,17,17,17,17,17,17,17,17,17,17,17,17,17,17,17,17,17,17,17,17,17,17,17,17,17,17,17,17,17,17,17,18,18,18,18,18,18,19,19,19,19,20,20,20,20,21,21,21,22,22,22,23,23,23,24,24,25,25,25,24,24,23,23,23,22,22,22,21,21,21,20,20,20,20,19,19,19,19,18,18,18,18,18,18,17,17,17,17,17,17,17,16,16,16,16,16,16,16,16,16,16,16,16,16,16,16,16,16,16,16,16,16,16,16,16,16,16,16,16,16,16,16,16,16,16,16,16,16,16,16,16,16,16,16,16,16,16,16,16,16,16,16,16,16,16,16,16,16,16,16,16,16,17,17,17,17,17,17,17,18,18,18,18,18,18,19,19,19,19,20,20,20,20,21,21,21,22,22,22,23,23,23,24,24,25,25,25,24,24,23,23,23,22,22,22,21,21,21,20,20,20,20,19,19,19,19,18,18,18,18,18,18,17,17,17,17,17,17,17,16,16,16,16,16,16,16,16,16,16,16,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,16,16,16,16,16,16,16,16,16,16,16,17,17,17,17,17,17,17,18,18,18,18,18,18,19,19,19,19,20,20,20,20,21,21,21,22,22,22,23,23,23,24,24,25,25,25,24,24,23,23,23,22,22,22,21,21,21,20,20,20,20,19,19,19,19,18,18,18,18,18,18,17,17,17,17,17,17,17,16,16,16,16,16,16,16,16,16,16,16,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,16,16,16,16,16,16,16,16,16,16,16,17,17,17,17,17,17,17,18,18,18,18,18,18,19,19,19,19,20,20,20,20,21,21,21,22,22,22,23,23,23,24,24,25,25,25,24,24,23,23,23,22,22,22,21,21,21,20,20,20,20,19,19,19,19,18,18,18,18,18,18,17,17,17,17,17,17,17,16,16,16,16,16,16,16,16,16,16,16,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,16,16,16,16,16,16,16,16,16,16,16,17,17,17,17,17,17,17,18,18,18,18,18,18,19,19,19,19,20,20,20,20,21,21,21,22,22,22,23,23,23,24,24,25,25,25,24,24,23,23,23,22,22,22,21,21,21,20,20,20,20,19,19,19,19,18,18,18,18,18,18,17,17,17,17,17,17,17,16,16,16,16,16,16,16,16,16,16,16,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,16,16,16,16,16,16,16,16,16,16,16,17,17,17,17,17,17,17,18,18,18,18,18,18,19,19,19,19,20,20,20,20,21,21,21,22,22,22,23,23,23,24,24,25,25,25,24,24,23,23,23,22,22,22,21,21,21,20,20,20,20,19,19,19,19,18,18,18,18,18,18,17,17,17,17,17,17,17,16,16,16,16,16,16,16,16,16,16,16,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,16,16,16,16,16,16,16,16,16,16,16,17,17,17,17,17,17,17,18,18,18,18,18,18,19,19,19,19,20,20,20,20,21,21,21,22,22,22,23,23,23,24,24,25,25,25,24,24,23,23,23,22,22,22,21,21,21,20,20,20,20,19,19,19,19,18,18,18,18,18,18,17,17,17,17,17,17,17,16,16,16,16,16,16,16,16,16,16,16,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,16,16,16,16,16,16,16,16,16,16,16,17,17,17,17,17,17,17,18,18,18,18,18,18,19,19,19,19,20,20,20,20,21,21,21,22,22,22,23,23,23,24,24,25,25,25,24,24,23,23,23,22,22,22,21,21,21,20,20,20,20,19,19,19,19,18,18,18,18,18,18,17,17,17,17,17,17,17,16,16,16,16,16,16,16,16,16,16,16,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,16,16,16,16,16,16,16,16,16,16,16,17,17,17,17,17,17,17,18,18,18,18,18,18,19,19,19,19,20,20,20,20,21,21,21,22,22,22,23,23,23,24,24,25,25,25,24,24,23,23,23,22,22,22,21,21,21,20,20,20,20,19,19,19,19,18,18,18,18,18,18,17,17,17,17,17,17,17,16,16,16,16,16,16,16,16,16,16,16,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,16,16,16,16,16,16,16,16,16,16,16,17,17,17,17,17,17,17,18,18,18,18,18,18,19,19,19,19,20,20,20,20,21,21,21,22,22,22,23,23,23,24,24,25,25,25,24,24,23,23,23,22,22,22,21,21,21,20,20,20,20,19,19,19,19,18,18,18,18,18,18,17,17,17,17,17,17,17,16,16,16,16,16,16,16,16,16,16,16,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,16,16,16,16,16,16,16,16,16,16,16,17,17,17,17,17,17,17,18,18,18,18,18,18,19,19,19,19,20,20,20,20,21,21,21,22,22,22,23,23,23,24,24,25,25,25,24,24,23,23,23,22,22,22,21,21,21,20,20,20,20,19,19,19,19,18,18,18,18,18,18,17,17,17,17,17,17,17,16,16,16,16,16,16,16,16,16,16,16,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,16,16,16,16,16,16,16,16,16,16,16,17,17,17,17,17,17,17,18,18,18,18,18,18,19,19,19,19,20,20,20,20,21,21,21,22,22,22,23,23,23,24,24,25,25,25,24,24,23,23,23,22,22,22,21,21,21,20,20,20,20,19,19,19,19,18,18,18,18,18,18,17,17,17,17,17,17,17,16,16,16,16,16,16,16,16,16,16,16,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,16,16,16,16,16,16,16,16,16,16,16,17,17,17,17,17,17,17,18,18,18,18,18,18,19,19,19,19,20,20,20,20,21,21,21,22,22,22,23,23,23,24,24,25,25,25,24,24,23,23,23,22,22,22,21,21,21,20,20,20,20,19,19,19,19,18,18,18,18,18,18,17,17,17,17,17,17,17,16,16,16,16,16,16,16,16,16,16,16,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,16,16,16,16,16,16,16,16,16,16,16,17,17,17,17,17,17,17,18,18,18,18,18,18,19,19,19,19,20,20,20,20,21,21,21,22,22,22,23,23,23,24,24,25,25,25,24,24,23,23,23,22,22,22,21,21,21,20,20,20,20,19,19,19,19,18,18,18,18,18,18,17,17,17,17,17,17,17,16,16,16,16,16,16,16,16,16,16,16,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,16,16,16,16,16,16,16,16,16,16,16,17,17,17,17,17,17,17,18,18,18,18,18,18,19,19,19,19,20,20,20,20,21,21,21,22,22,22,23,23,23,24,24,25,25,25,24,24,23,23,23,22,22,22,21,21,21,20,20,20,20,19,19,19,19,18,18,18,18,18,18,17,17,17,17,17,17,17,16,16,16,16,16,16,16,16,16,16,16,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,16,16,16,16,16,16,16,16,16,16,16,17,17,17,17,17,17,17,18,18,18,18,18,18,19,19,19,19,20,20,20,20,21,21,21,22,22,22,23,23,23,24,24,25,25,25,24,24,23,23,23,22,22,22,21,21,21,20,20,20,20,19,19,19,19,18,18,18,18,18,18,17,17,17,17,17,17,17,16,16,16,16,16,16,16,16,16,16,16,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,16,16,16,16,16,16,16,16,16,16,16,17,17,17,17,17,17,17,18,18,18,18,18,18,19,19,19,19,20,20,20,20,21,21,21,22,22,22,23,23,23,24,24,25,25,25,24,24,23,23,23,22,22,22,21,21,21,20,20,20,20,19,19,19,19,18,18,18,18,18,18,17,17,17,17,17,17,17,16,16,16,16,16,16,16,16,16,16,16,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,16,16,16,16,16,16,16,16,16,16,16,17,17,17,17,17,17,17,18,18,18,18,18,18,19,19,19,19,20,20,20,20,21,21,21,22,22,22,23,23,23,24,24,25,25,25,24,24,23,23,23,22,22,22,21,21,21,20,20,20,20,19,19,19,19,18,18,18,18,18,18,17,17,17,17,17,17,17,16,16,16,16,16,16,16,16,16,16,16,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,16,16,16,16,16,16,16,16,16,16,16,17,17,17,17,17,17,17,18,18,18,18,18,18,19,19,19,19,20,20,20,20,21,21,21,22,22,22,23,23,23,24,24,25,25,25,24,24,23,23,23,22,22,22,21,21,21,20,20,20,20,19,19,19,19,18,18,18,18,18,18,17,17,17,17,17,17,17,16,16,16,16,16,16,16,16,16,16,16,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,16,16,16,16,16,16,16,16,16,16,16,17,17,17,17,17,17,17,18,18,18,18,18,18,19,19,19,19,20,20,20,20,21,21,21,22,22,22,23,23,23,24,24,25,25,25,24,24,23,23,23,22,22,22,21,21,21,20,20,20,20,19,19,19,19,18,18,18,18,18,18,17,17,17,17,17,17,17,16,16,16,16,16,16,16,16,16,16,16,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,16,16,16,16,16,16,16,16,16,16,16,17,17,17,17,17,17,17,18,18,18,18,18,18,19,19,19,19,20,20,20,20,21,21,21,22,22,22,23,23,23,24,24,25,25,25,24,24,23,23,23,22,22,22,21,21,21,20,20,20,20,19,19,19,19,18,18,18,18,18,18,17,17,17,17,17,17,17,16,16,16,16,16,16,16,16,16,16,16,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,16,16,16,16,16,16,16,16,16,16,16,17,17,17,17,17,17,17,18,18,18,18,18,18,19,19,19,19,20,20,20,20,21,21,21,22,22,22,23,23,23,24,24,25,25,25,24,24,23,23,23,22,22,22,21,21,21,20,20,20,20,19,19,19,19,18,18,18,18,18,18,17,17,17,17,17,17,17,16,16,16,16,16,16,16,16,16,16,16,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,16,16,16,16,16,16,16,16,16,16,16,17,17,17,17,17,17,17,18,18,18,18,18,18,19,19,19,19,20,20,20,20,21,21,21,22,22,22,23,23,23,24,24,25,25,25,24,24,23,23,23,22,22,22,21,21,21,20,20,20,20,19,19,19,19,18,18,18,18,18,18,17,17,17,17,17,17,17,16,16,16,16,16,16,16,16,16,16,16,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,16,16,16,16,16,16,16,16,16,16,16,17,17,17,17,17,17,17,18,18,18,18,18,18,19,19,19,19,20,20,20,20,21,21,21,22,22,22,23,23,23,24,24,25,25,25,24,24,23,23,23,22,22,22,21,21,21,20,20,20,20,19,19,19,19,18,18,18,18,18,18,17,17,17,17,17,17,17,16,16,16,16,16,16,16,16,16,16,16,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,16,16,16,16,16,16,16,16,16,16,16,17,17,17,17,17,17,17,18,18,18,18,18,18,19,19,19,19,20,20,20,20,21,21,21,22,22,22,23,23,23,24,24,25,25,25,24,24,23,23,23,22,22,22,21,21,21,20,20,20,20,19,19,19,19,18,18,18,18,18,18,17,17,17,17,17,17,17,16,16,16,16,16,16,16,16,16,16,16,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,16,16,16,16,16,16,16,16,16,16,16,17,17,17,17,17,17,17,18,18,18,18,18,18,19,19,19,19,20,20,20,20,21,21,21,22,22,22,23,23,23,24,24,25,25,25,24,24,23,23,23,22,22,22,21,21,21,20,20,20,20,19,19,19,19,18,18,18,18,18,18,17,17,17,17,17,17,17,16,16,16,16,16,16,16,16,16,16,16,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,16,16,16,16,16,16,16,16,16,16,16,17,17,17,17,17,17,17,18,18,18,18,18,18,19,19,19,19,20,20,20,20,21,21,21,22,22,22,23,23,23,24,24,25,25,25,24,24,23,23,23,22,22,22,21,21,21,20,20,20,20,19,19,19,19,18,18,18,18,18,18,17,17,17,17,17,17,17,16,16,16,16,16,16,16,16,16,16,16,16,16,16,16,16,16,16,16,16,16,16,16,16,16,16,16,16,16,16,16,16,16,16,16,16,16,16,16,16,16,16,16,16,16,16,16,16,16,16,16,16,16,16,16,16,16,16,16,16,16,17,17,17,17,17,17,17,18,18,18,18,18,18,19,19,19,19,20,20,20,20,21,21,21,22,22,22,23,23,23,24,24,25,25,25,24,24,23,23,23,22,22,22,21,21,21,20,20,20,20,19,19,19,19,18,18,18,18,18,18,17,17,17,17,17,17,17,17,17,17,17,17,17,17,17,17,17,17,17,17,17,17,17,17,17,17,17,17,17,17,17,17,17,17,17,17,17,17,17,17,17,17,17,17,17,17,17,17,17,17,17,17,17,17,17,17,17,17,17,17,17,17,17,17,17,17,17,17,17,17,17,17,17,17,17,18,18,18,18,18,18,19,19,19,19,20,20,20,20,21,21,21,22,22,22,23,23,23,24,24,25,25,25,24,24,23,23,23,22,22,22,21,21,21,20,20,20,20,19,19,19,19,19,19,19,19,19,19,19,19,19,19,19,19,19,19,19,19,19,19,19,19,19,19,19,19,19,19,19,19,19,19,19,19,19,19,19,19,19,19,19,19,19,19,19,19,19,19,19,19,19,19,19,19,19,19,19,19,19,19,19,19,19,19,19,19,19,19,19,19,19,19,19,19,19,19,19,19,19,19,19,19,19,19,19,19,19,19,19,19,19,19,19,20,20,20,20,21,21,21,22,22,22,23,23,23,24,24,25,25,25,24,24,23,23,23,22,22,22,21,21,21,20,20,20,20,20,20,20,20,20,20,20,20,20,20,20,20,20,20,20,20,20,20,20,20,20,20,20,20,20,20,20,20,20,20,20,20,20,20,20,20,20,20,20,20,20,20,20,20,20,20,20,20,20,20,20,20,20,20,20,20,20,20,20,20,20,20,20,20,20,20,20,20,20,20,20,20,20,20,20,20,20,20,20,20,20,20,20,20,20,20,20,20,20,20,20,20,20,20,20,20,20,20,20,21,21,21,22,22,22,23,23,23,24,24,25,25,25,24,24,23,23,23,22,22,22,21,21,21,21,21,21,21,21,21,21,21,21,21,21,21,21,21,21,21,21,21,21,21,21,21,21,21,21,21,21,21,21,21,21,21,21,21,21,21,21,21,21,21,21,21,21,21,21,21,21,21,21,21,21,21,21,21,21,21,21,21,21,21,21,21,21,21,21,21,21,21,21,21,21,21,21,21,21,21,21,21,21,21,21,21,21,21,21,21,21,21,21,21,21,21,21,21,21,21,21,21,21,21,21,21,21,21,21,21,22,22,22,23,23,23,24,24,25,25,25,24,24,23,23,23,22,22,22,22,22,22,22,22,22,22,22,22,22,22,22,22,22,22,22,22,22,22,22,22,22,22,22,22,22,22,22,22,22,22,22,22,22,22,22,22,22,22,22,22,22,22,22,22,22,22,22,22,22,22,22,22,22,22,22,22,22,22,22,22,22,22,22,22,22,22,22,22,22,22,22,22,22,22,22,22,22,22,22,22,22,22,22,22,22,22,22,22,22,22,22,22,22,22,22,22,22,22,22,22,22,22,22,22,22,22,22,22,22,22,22,23,23,23,24,24,25,25,25,24,24,23,23,23,23,23,23,23,23,23,23,23,23,23,23,23,23,23,23,23,23,23,23,23,23,23,23,23,23,23,23,23,23,23,23,23,23,23,23,23,23,23,23,23,23,23,23,23,23,23,23,23,23,23,23,23,23,23,23,23,23,23,23,23,23,23,23,23,23,23,23,23,23,23,23,23,23,23,23,23,23,23,23,23,23,23,23,23,23,23,23,23,23,23,23,23,23,23,23,23,23,23,23,23,23,23,23,23,23,23,23,23,23,23,23,23,23,23,23,23,23,23,24,24,25,25,25,25,25,25,25,25,25,25,25,25,25,25,25,25,25,25,25,25,25,25,25,25,25,25,25,25,25,25,25,25,25,25,25,25,25,25,25,25,25,25,25,25,25,25,25,25,25,25,25,25,25,25,25,25,25,25,25,25,25,25,25,25,25,25,25,25,25,25,25,25,25,25,25,25,25,25,25,25,25,25,25,25,25,25,25,25,25,25,25,25,25,25,25,25,25,25,25,25,25,25,25,25,25,25,25,25,25,25,25,25,25,25,25,25,25,25,25,25,25,25,25,25,25,25,25,25,25,25,26,26,26,26,26,26,26,26,26,26,26,26,26,26,26,26,26,26,26,26,26,26,26,26,26,26,26,26,26,26,26,26,26,26,26,26,26,26,26,26,26,26,26,26,26,26,26,26,26,26,26,26,26,26,26,26,26,26,26,26,26,26,26,26,26,26,26,26,26,26,26,26,26,26,26,26,26,26,26,26,26,26,26,26,26,26,26,26,26,26,26,26,26,26,26,26,26,26,26,26,26,26,26,26,26,26,26,26,26,26,26,26,26,26,26,26,26,26,26,26,26,26,26,26,26,26,26,26,27,27,27,27,27,27,27,27,27,27,27,27,27,27,27,27,27,27,27,27,27,27,27,27,27,27,27,27,27,27,27,27,27,27,27,27,27,27,27,27,27,27,27,27,27,27,27,27,27,27,27,27,27,27,27,27,27,27,27,27,27,27,27,27,27,27,27,27,27,27,27,27,27,27,27,27,27,27,27,27,27,27,27,27,27,27,27,27,27,27,27,27,27,27,27,27,27,27,27,27,27,27,27,27,27,27,27,27,27,27,27,27,27,27,27,27,27,27,27,27,27,27,27,27,27,27,27,27,28,28,28,28,28,28,28,28,28,28,28,28,28,28,28,28,28,28,28,28,28,28,28,28,28,28,28,28,28,28,28,28,28,28,28,28,28,28,28,28,28,28,28,28,28,28,28,28,28,28,28,28,28,28,28,28,28,28,28,28,28,28,28,28,28,28,28,28,28,28,28,28,28,28,28,28,28,28,28,28,28,28,28,28,28,28,28,28,28,28,28,28,28,28,28,28,28,28,28,28,28,28,28,28,28,28,28,28,28,28,28,28,28,28,28,28,28,28,28,28,28,28,28,28,28,28,28,28,29,29,29,29,29,29,29,29,29,29,29,29,29,29,29,29,29,29,29,29,29,29,29,29,29,29,29,29,29,29,29,29,29,29,29,29,29,29,29,29,29,29,29,29,29,29,29,29,29,29,29,29,29,29,29,29,29,29,29,29,29,29,29,29,29,29,29,29,29,29,29,29,29,29,29,29,29,29,29,29,29,29,29,29,29,29,29,29,29,29,29,29,29,29,29,29,29,29,29,29,29,29,29,29,29,29,29,29,29,29,29,29,29,29,29,29,29,29,29,29,29,29,29,29,29,29,29,29,31,31,31,31,31,31,31,31,31,31,31,31,31,31,31,31,31,31,31,31,31,31,31,31,31,31,31,31,31,31,31,31,31,31,31,31,31,31,31,31,31,31,31,31,31,31,31,31,31,31,31,31,31,31,31,31,31,31,31,31,31,31,31,31,31,31,31,31,31,31,31,31,31,31,31,31,31,31,31,31,31,31,31,31,31,31,31,31,31,31,31,31,31,31,31,31,31,31,31,31,31,31,31,31,31,31,31,31,31,31,31,31,31,31,31,31,31,31,31,31,31,31,31,31,31,31,31,31,32,32,32,32,32,32,32,32,32,32,32,32,32,32,32,32,32,32,32,32,32,32,32,32,32,32,32,32,32,32,32,32,32,32,32,32,32,32,32,32,32,32,32,32,32,32,32,32,32,32,32,32,32,32,32,32,32,32,32,32,32,32,32,32,32,32,32,32,32,32,32,32,32,32,32,32,32,32,32,32,32,32,32,32,32,32,32,32,32,32,32,32,32,32,32,32,32,32,32,32,32,32,32,32,32,32,32,32,32,32,32,32,32,32,32,32,32,32,32,32,32,32,32,32,32,32,32,32,33,33,33,33,33,33,33,33,33,33,33,33,33,33,33,33,33,33,33,33,33,33,33,33,33,33,33,33,33,33,33,33,33,33,33,33,33,33,33,33,33,33,33,33,33,33,33,33,33,33,33,33,33,33,33,33,33,33,33,33,33,33,33,33,33,33,33,33,33,33,33,33,33,33,33,33,33,33,33,33,33,33,33,33,33,33,33,33,33,33,33,33,33,33,33,33,33,33,33,33,33,33,33,33,33,33,33,33,33,33,33,33,33,33,33,33,33,33,33,33,33,33,33,33,33,33,33,33,34,34,34,34,34,34,34,34,34,34,34,34,34,34,34,34,34,34,34,34,34,34,34,34,34,34,34,34,34,34,34,34,34,34,34,34,34,34,34,34,34,34,34,34,34,34,34,34,34,34,34,34,34,34,34,34,34,34,34,34,34,34,34,34,34,34,34,34,34,34,34,34,34,34,34,34,34,34,34,34,34,34,34,34,34,34,34,34,34,34,34,34,34,34,34,34,34,34,34,34,34,34,34,34,34,34,34,34,34,34,34,34,34,34,34,34,34,34,34,34,34,34,34,34,34,34,34,34,35,35,35,35,35,35,35,35,35,35,35,35,35,35,35,35,35,35,35,35,35,35,35,35,35,35,35,35,35,35,35,35,35,35,35,35,35,35,35,35,35,35,35,35,35,35,35,35,35,35,35,35,35,35,35,35,35,35,35,35,35,35,35,35,35,35,35,35,35,35,35,35,35,35,35,35,35,35,35,35,35,35,35,35,35,35,35,35,35,35,35,35,35,35,35,35,35,35,35,35,35,35,35,35,35,35,35,35,35,35,35,35,35,35,35,35,35,35,35,35,35,35,35,35,35,35,35,35,37,37,37,37,37,37,37,37,37,37,37,37,37,37,37,37,37,37,37,37,37,37,37,37,37,37,37,37,37,37,37,37,37,37,37,37,37,37,37,37,37,37,37,37,37,37,37,37,37,37,37,37,37,37,37,37,37,37,37,37,37,37,37,37,37,37,37,37,37,37,37,37,37,37,37,37,37,37,37,37,37,37,37,37,37,37,37,37,37,37,37,37,37,37,37,37,37,37,37,37,37,37,37,37,37,37,37,37,37,37,37,37,37,37,37,37,37,37,37,37,37,37,37,37,37,37,37,37,38,38,38,38,38,38,38,38,38,38,38,38,38,38,38,38,38,38,38,38,38,38,38,38,38,38,38,38,38,38,38,38,38,38,38,38,38,38,38,38,38,38,38,38,38,38,38,38,38,38,38,38,38,38,38,38,38,38,38,38,38,38,38,38,38,38,38,38,38,38,38,38,38,38,38,38,38,38,38,38,38,38,38,38,38,38,38,38,38,38,38,38,38,38,38,38,38,38,38,38,38,38,38,38,38,38,38,38,38,38,38,38,38,38,38,38,38,38,38,38,38,38,38,38,38,38,38,38,39,39,39,39,39,39,39,39,39,39,39,39,39,39,39,39,39,39,39,39,39,39,39,39,39,39,39,39,39,39,39,39,39,39,39,39,39,39,39,39,39,39,39,39,39,39,39,39,39,39,39,39,39,39,39,39,39,39,39,39,39,39,39,39,39,39,39,39,39,39,39,39,39,39,39,39,39,39,39,39,39,39,39,39,39,39,39,39,39,39,39,39,39,39,39,39,39,39,39,39,39,39,39,39,39,39,39,39,39,39,39,39,39,39,39,39,39,39,39,39,39,39,39,39,39,39,39,39,40,40,40,40,40,40,40,40,40,40,40,40,40,40,40,40,40,40,40,40,40,40,40,40,40,40,40,40,40,40,40,40,40,40,40,40,40,40,40,40,40,40,40,40,40,40,40,40,40,40,40,40,40,40,40,40,40,40,40,40,40,40,40,40,40,40,40,40,40,40,40,40,40,40,40,40,40,40,40,40,40,40,40,40,40,40,40,40,40,40,40,40,40,40,40,40,40,40,40,40,40,40,40,40,40,40,40,40,40,40,40,40,40,40,40,40,40,40,40,40,40,40,40,40,40,40,40,40,41,41,41,41,41,41,41,41,41,41,41,41,41,41,41,41,41,41,41,41,41,41,41,41,41,41,41,41,41,41,41,41,41,41,41,41,41,41,41,41,41,41,41,41,41,41,41,41,41,41,41,41,41,41,41,41,41,41,41,41,41,41,41,41,41,41,41,41,41,41,41,41,41,41,41,41,41,41,41,41,41,41,41,41,41,41,41,41,41,41,41,41,41,41,41,41,41,41,41,41,41,41,41,41,41,41,41,41,41,41,41,41,41,41,41,41,41,41,41,41,41,41,41,41,41,41,41,41,43,43,43,43,43,43,43,43,43,43,43,43,43,43,43,43,43,43,43,43,43,43,43,43,43,43,43,43,43,43,43,43,43,43,43,43,43,43,43,43,43,43,43,43,43,43,43,43,43,43,43,43,43,43,43,43,43,43,43,43,43,43,43,43,43,43,43,43,43,43,43,43,43,43,43,43,43,43,43,43,43,43,43,43,43,43,43,43,43,43,43,43,43,43,43,43,43,43,43,43,43,43,43,43,43,43,43,43,43,43,43,43,43,43,43,43,43,43,43,43,43,43,43,43,43,43,43,43,44,44,44,44,44,44,44,44,44,44,44,44,44,44,44,44,44,44,44,44,44,44,44,44,44,44,44,44,44,44,44,44,44,44,44,44,44,44,44,44,44,44,44,44,44,44,44,44,44,44,44,44,44,44,44,44,44,44,44,44,44,44,44,44,44,44,44,44,44,44,44,44,44,44,44,44,44,44,44,44,44,44,44,44,44,44,44,44,44,44,44,44,44,44,44,44,44,44,44,44,44,44,44,44,44,44,44,44,44,44,44,44,44,44,44,44,44,44,44,44,44,44,44,44,44,44,44,44,45,45,45,45,45,45,45,45,45,45,45,45,45,45,45,45,45,45,45,45,45,45,45,45,45,45,45,45,45,45,45,45,45,45,45,45,45,45,45,45,45,45,45,45,45,45,45,45,45,45,45,45,45,45,45,45,45,45,45,45,45,45,45,45,45,45,45,45,45,45,45,45,45,45,45,45,45,45,45,45,45,45,45,45,45,45,45,45,45,45,45,45,45,45,45,45,45,45,45,45,45,45,45,45,45,45,45,45,45,45,45,45,45,45,45,45,45,45,45,45,45,45,45,45,45,45,45,45,46,46,46,46,46,46,46,46,46,46,46,46,46,46,46,46,46,46,46,46,46,46,46,46,46,46,46,46,46,46,46,46,46,46,46,46,46,46,46,46,46,46,46,46,46,46,46,46,46,46,46,46,46,46,46,46,46,46,46,46,46,46,46,46,46,46,46,46,46,46,46,46,46,46,46,46,46,46,46,46,46,46,46,46,46,46,46,46,46,46,46,46,46,46,46,46,46,46,46,46,46,46,46,46,46,46,46,46,46,46,46,46,46,46,46,46,46,46,46,46,46,46,46,46,46,46,46,46,47,47,47,47,47,47,47,47,47,47,47,47,47,47,47,47,47,47,47,47,47,47,47,47,47,47,47,47,47,47,47,47,47,47,47,47,47,47,47,47,47,47,47,47,47,47,47,47,47,47,47,47,47,47,47,47,47,47,47,47,47,47,47,47,47,47,47,47,47,47,47,47,47,47,47,47,47,47,47,47,47,47,47,47,47,47,47,47,47,47,47,47,47,47,47,47,47,47,47,47,47,47,47,47,47,47,47,47,47,47,47,47,47,47,47,47,47,47,47,47,47,47,47,47,47,47,47,47,49,49,49,49,49,49,49,49,49,49,49,49,49,49,49,49,49,49,49,49,49,49,49,49,49,49,49,49,49,49,49,49,49,49,49,49,49,49,49,49,49,49,49,49,49,49,49,49,49,49,49,49,49,49,49,49,49,49,49,49,49,49,49,49,49,49,49,49,49,49,49,49,49,49,49,49,49,49,49,49,49,49,49,49,49,49,49,49,49,49,49,49,49,49,49,49,49,49,49,49,49,49,49,49,49,49,49,49,49,49,49,49,49,49,49,49,49,49,49,49,49,49,49,49,49,49,49,49,50,50,50,50,50,50,50,50,50,50,50,50,50,50,50,50,50,50,50,50,50,50,50,50,50,50,50,50,50,50,50,50,50,50,50,50,50,50,50,50,50,50,50,50,50,50,50,50,50,50,50,50,50,50,50,50,50,50,50,50,50,50,50,50,50,50,50,50,50,50,50,50,50,50,50,50,50,50,50,50,50,50,50,50,50,50,50,50,50,50,50,50,50,50,50,50,50,50,50,50,50,50,50,50,50,50,50,50,50,50,50,50,50,50,50,50,50,50,50,50,50,50,50,50,50,50,50,50,51,51,51,51,51,51,51,51,51,51,51,51,51,51,51,51,51,51,51,51,51,51,51,51,51,51,51,51,51,51,51,51,51,51,51,51,51,51,51,51,51,51,51,51,51,51,51,51,51,51,51,51,51,51,51,51,51,51,51,51,51,51,51,51,51,51,51,51,51,51,51,51,51,51,51,51,51,51,51,51,51,51,51,51,51,51,51,51,51,51,51,51,51,51,51,51,51,51,51,51,51,51,51,51,51,51,51,51,51,51,51,51,51,51,51,51,51,51,51,51,51,51,51,51,51,51,51,51,52,52,52,52,52,52,52,52,52,52,52,52,52,52,52,52,52,52,52,52,52,52,52,52,52,52,52,52,52,52,52,52,52,52,52,52,52,52,52,52,52,52,52,52,52,52,52,52,52,52,52,52,52,52,52,52,52,52,52,52,52,52,52,52,52,52,52,52,52,52,52,52,52,52,52,52,52,52,52,52,52,52,52,52,52,52,52,52,52,52,52,52,52,52,52,52,52,52,52,52,52,52,52,52,52,52,52,52,52,52,52,52,52,52,52,52,52,52,52,52,52,52,52,52,52,52,52,52,53,53,53,53,53,53,53,53,53,53,53,53,53,53,53,53,53,53,53,53,53,53,53,53,53,53,53,53,53,53,53,53,53,53,53,53,53,53,53,53,53,53,53,53,53,53,53,53,53,53,53,53,53,53,53,53,53,53,53,53,53,53,53,53,53,53,53,53,53,53,53,53,53,53,53,53,53,53,53,53,53,53,53,53,53,53,53,53,53,53,53,53,53,53,53,53,53,53,53,53,53,53,53,53,53,53,53,53,53,53,53,53,53,53,53,53,53,53,53,53,53,53,53,53,53,53,53,53,55,55,55,55,55,55,55,55,55,55,55,55,55,55,55,55,55,55,55,55,55,55,55,55,55,55,55,55,55,55,55,55,55,55,55,55,55,55,55,55,55,55,55,55,55,55,55,55,55,55,55,55,55,55,55,55,55,55,55,55,55,55,55,55,55,55,55,55,55,55,55,55,55,55,55,55,55,55,55,55,55,55,55,55,55,55,55,55,55,55,55,55,55,55,55,55,55,55,55,55,55,55,55,55,55,55,55,55,55,55,55,55,55,55,55,55,55,55,55,55,55,55,55,55,55,55,55,55,56,56,56,56,56,56,56,56,56,56,56,56,56,56,56,56,56,56,56,56,56,56,56,56,56,56,56,56,56,56,56,56,56,56,56,56,56,56,56,56,56,56,56,56,56,56,56,56,56,56,56,56,56,56,56,56,56,56,56,56,56,56,56,56,56,56,56,56,56,56,56,56,56,56,56,56,56,56,56,56,56,56,56,56,56,56,56,56,56,56,56,56,56,56,56,56,56,56,56,56,56,56,56,56,56,56,56,56,56,56,56,56,56,56,56,56,56,56,56,56,56,56,56,56,56,56,56,56,57,57,57,57,57,57,57,57,57,57,57,57,57,57,57,57,57,57,57,57,57,57,57,57,57,57,57,57,57,57,57,57,57,57,57,57,57,57,57,57,57,57,57,57,57,57,57,57,57,57,57,57,57,57,57,57,57,57,57,57,57,57,57,57,57,57,57,57,57,57,57,57,57,57,57,57,57,57,57,57,57,57,57,57,57,57,57,57,57,57,57,57,57,57,57,57,57,57,57,57,57,57,57,57,57,57,57,57,57,57,57,57,57,57,57,57,57,57,57,57,57,57,57,57,57,57,57,57,58,58,58,58,58,58,58,58,58,58,58,58,58,58,58,58,58,58,58,58,58,58,58,58,58,58,58,58,58,58,58,58,58,58,58,58,58,58,58,58,58,58,58,58,58,58,58,58,58,58,58,58,58,58,58,58,58,58,58,58,58,58,58,58,58,58,58,58,58,58,58,58,58,58,58,58,58,58,58,58,58,58,58,58,58,58,58,58,58,58,58,58,58,58,58,58,58,58,58,58,58,58,58,58,58,58,58,58,58,58,58,58,58,58,58,58,58,58,58,58,58,58,58,58,58,58,58,58 };

  static const float distance_reference_lookup [39] = { 1971.4286, 2178.9474, 2300, 2435.2941, 2435.2941, 2435.2941, 2587.5, 2587.5, 2587.5, 2587.5, 2760, 2760, 2760, 2760, 2760, 2760, 2760, 2760, 2760, 2760, 2760, 2760, 2760, 2760, 2760, 2760, 2760, 2760, 2760, 2587.5, 2587.5, 2587.5, 2587.5, 2587.5, 2435.2941, 2435.2941, 2300, 2178.9474, 1971.4286};

  static const int32_t table_resolution = 50;           // [mm]
  static const int32_t table_length = 39;
  static const int32_t table_resolution_2 = table_resolution / 2;
  //static const float focal_length = 115;            // [px]
  static const int32_t baseline_distance = 60;        // [mm]
  static const int32_t forward_velocity = 750;        // [mm/s]
  static const int32_t disparity_scale = RESOLUTION_FACTOR;         // [-]
  // todo, if we want to implement true droplet with forward looking camera, this should be wait to get to TP2
  // should be set around 1m, after triggered need to wait sqrt((CP-TP2)^2 - R_turn^2)/V
  // obst wait should also be parametrically linked to the settings above
  static const int32_t obst_wait = 2000 * 1000 / forward_velocity;            // obstacle avoidance at 2m [ms]

  volatile int32_t distance_difference, obst_time_update;
  int32_t x_pos_detection, distance_reference_lookup_index;
  uint32_t obst_time_histogram_index_update;

  uint32_t image_index;
  *count_disps_left = 0; *count_disps_right = 0;
  for (x = 0; x < disparity_map->w; x++) {
    for (y = 0; y < disparity_map->h; y++) {
      image_index = x + (y * disparity_map->w);
      if (disp_image[image_index] > 0) {
        // count number of stereo macosthes in left and right image halve
        // used for detecting low-texture situations (such as non-textured walls)
        if (x < image_width_2) {
          (*count_disps_left)++;
        } else {
          (*count_disps_right)++;
        }

        // check if disparity value indicates object inside the Droplet area
        //  - count number of occurrences
        //  - estimate how far the object has penetrated the Droplet area (to deal with late detections)
        if (disp_image[image_index] >= maximum_disparities[image_index]) {
          disparities_close++;

          // compute detection distance
          //(float) focal_length*baseline_distance*disparity_scale/disparity_detection;

          // compute detection x-position
          x_pos_detection = (x - image_width_2) * baseline_distance * disparity_scale /
                            (disp_image[image_index] * table_resolution);

          // obtain reference distance
          distance_reference_lookup_index = x_pos_detection + table_resolution_2;
          Bound(distance_reference_lookup_index, 0, table_length - 1);

          // compute distance of the penetrating pixel
          distance_difference = distance_reference_lookup[distance_reference_lookup_index] -
                                disp2dist[disp_image[image_index]];

          obst_time_update = obst_wait - (distance_difference * 1000 / forward_velocity);
          obst_time_update /= HISTOGRAM_RESOLUTION;
          Bound(obst_time_update, 0, droplet_hist_len - 1);

          obst_time_histogram_index_update = (obst_time_update + curr_histogram_bin) % droplet_hist_len;
          obst_time_histogram[obst_time_histogram_index_update]++;
        }
      } // end if stereo match (disp > 0)
    } // end for y
  } // end for x

  // We pick max histogram value in a small range around now to improve to robustness of the detection
  // in case of small speed estimate errors
  *hist_obs_sum = 0;
  static int32_t index;
  for (i = 0; i < 3; i++) {
    index = (curr_histogram_bin + i) % droplet_hist_len;
    if (obst_time_histogram[index] > *hist_obs_sum) {
      *hist_obs_sum = obst_time_histogram[index];
    }
  }

  return disparities_close;
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

uint16_t getFeatureImageLocations(uint8_t *current_image_buffer, uint8_t *disparity_image_buffer,
                                  uint8_t *feature_image_locations, uint8_t *target_location, uint32_t image_width, uint32_t image_height, uint8_t min_y,
                                  uint8_t max_y, uint16_t feature_count_limit)
{
  uint8_t maxDisparityInHistogram = 200;
  volatile uint8_t disp_min = 13;
  volatile uint8_t max_disp_threshold = 10;
  volatile uint8_t disp_window = 7;

  volatile uint16_t x = 0;
  volatile uint16_t y = 0;
  volatile uint16_t i = 0;
  volatile uint8_t disp = 0;
  volatile uint8_t max_disp = 0;
  volatile uint8_t max_disp_count = 0;
  volatile uint8_t disp_hist [maxDisparityInHistogram];
  volatile uint8_t disp_range_min = 0;
  volatile uint8_t disp_range_max = 0;
  volatile uint8_t Y_hist [image_height];
  volatile uint8_t X_hist [image_width];
  volatile uint8_t X_hist_max = 0;
  volatile uint8_t X_hist_max_count = 0;
  volatile uint8_t min_X = 0;
  volatile uint8_t max_X = 0;
  volatile uint16_t mean_X = 0;
  volatile int16_t disp_mean = 0;
  volatile uint16_t disp_count = 0;
  volatile uint16_t distance = 0;

  volatile int image_width_bytes = image_width * 2;




  // initialize histogram
  for (i = 0; i < maxDisparityInHistogram; i++) {
    disp_hist[i] = 0;
  }

  // compute disparity histogram
  for (y = min_y; y < max_y; y++) {
    for (x = 0; x < image_width; x++) {

      disp = disparity_image_buffer[x + y * image_width];
      if (disp > disp_min && disp < maxDisparityInHistogram) {
        disp_hist[disp]++;
        disp_hist[disp - 1]++;
        disp_hist[disp + 1]++;
      }
    }
  }

  // find disparity peak in histogram
  for (i = disp_min + disp_window; i < maxDisparityInHistogram; i++) {
    if (disp_hist[i] > max_disp_count) {
      max_disp_count = disp_hist[i];
      max_disp = i;
    }
  }

  led_clear();

  target_location[0] = (uint8_t) 65;
  target_location[1] = (uint8_t) 48;
  target_location[2] = (uint8_t) 100;

  // check if there is a clear object
  if ((max_disp_count > max_disp_threshold) && (max_disp > disp_min + disp_window)) {

    led_set();

    disp_range_min = max_disp - disp_window;
    disp_range_max = max_disp + disp_window;
    // create histogram for Y-direction
    for (i = 0; i < image_height; i++) {
      Y_hist[i] = 0;
    }

    for (y = min_y; y < max_y; y++) {
      for (x = 0; x < image_width; x++) {

        disp = disparity_image_buffer[x + y * image_width];
        if ((disp > disp_range_min) && (disp < disp_range_max)) {
          Y_hist[y]++;
        }
      }
    }

    for (i = 4; i < image_height - 10; i++) {
      if (Y_hist[i] == 0 && Y_hist[i - 1] == 0 && Y_hist[i - 2] == 0 && Y_hist[i - 3] == 0 && Y_hist[i - 4] == 0) {
        min_y = i;
      }
    }

    // initialize histogram for X-direction
    for (i = 0; i < image_width; i++) {
      X_hist[i] = 0;
    }

    // compute histogram for X-direction
    for (y = min_y; y < max_y; y++) {
      for (x = 0; x < image_width; x++) {

        disp = disparity_image_buffer[x + y * image_width];
        if ((disp > disp_range_min) && (disp < disp_range_max)) {
          X_hist[x]++;
        }
      }
    }

    // find peak and its location of X-histogram
    for (i = 0; i < image_width; i++) {
      if (X_hist[i] > X_hist_max_count) {
        X_hist_max_count = X_hist[i];
        X_hist_max = i;
      }
    }

    // find most-right zero index on left side of X-histogram [
    for (i = 4; i < X_hist_max; i++) {
      if (X_hist[i] == 0 && X_hist[i - 1] == 0 && X_hist[i - 2] == 0 && X_hist[i - 3] == 0 && X_hist[i - 4] == 0) {
        min_X = i;
      }
    }

    // find most-left zero index on right side of X-histogram ]
    for (i = image_width - 5; i > X_hist_max; i--) {
      if (X_hist[i] == 0 && X_hist[i + 1] == 0 && X_hist[i + 2] == 0 && X_hist[i + 3] == 0 && X_hist[i + 4] == 0) {
        max_X = i;
      }
    }

    // visualize left and right borders of object in image
    for (y = min_y; y < max_y; y++) {
      current_image_buffer[(min_X * 2) + 1 + (y * image_width_bytes)] = 255;
      current_image_buffer[(max_X * 2) + 1 + (y * image_width_bytes)] = 255;

    }

    // compute and visualize middle line of object
    mean_X = (min_X + max_X) / 2;
    current_image_buffer[(mean_X * 2) + 1 + (min_y * image_width_bytes)] = 255;


    // compute average disparity of object
    for (y = min_y; y < max_y; y++) {
      for (x = min_X; x < max_X; x++) {

        disp = disparity_image_buffer[x + y * image_width];
        if ((disp > disp_range_min) && (disp < disp_range_max)) {
          disp_mean += disp;
          disp_count++;
        }
      }
    }

    disp_mean = disp_mean / disp_count;

    // convert disparity to distance
    distance = (120 * 6) / ((disp_mean / RESOLUTION_FACTOR) * 2); // [cm] divided by 2

    target_location[0] = (uint8_t) mean_X;
    target_location[1] = (uint8_t) min_y;
    target_location[2] = (uint8_t) distance;


  } // endif object found

  return disp_count;

}


uint16_t getFeatureImageLocations_old(uint8_t *disparity_image_buffer, uint8_t *feature_image_locations,
                                      uint32_t image_width, uint32_t image_height, uint8_t min_y, uint8_t max_y, uint16_t feature_count_limit)
{
  // set minimum value for disparity
  uint8_t disp_far = 19; // 29; // approx 1.5m
  uint8_t disp_close = 86; //86; // approx 0.5m
  const uint8_t nr_bins = 10;
  volatile uint8_t bin_min_value = 30;
  volatile uint8_t disp = 0;
  volatile uint8_t disp_bins [nr_bins];
  volatile uint8_t bin_width = (disp_close - disp_far) / nr_bins;
  volatile uint8_t bin_nr = 0;
  volatile uint8_t bin_index = 0;
  volatile uint8_t disp_start = 0;
  volatile uint8_t disp_end = 0;
  volatile uint8_t sum_bins = 0;

  volatile uint16_t feature_count = 0;
  volatile uint16_t x = 0;
  volatile uint16_t y = 0;



  for (x = 0; x < nr_bins; x++) {
    disp_bins[x] = 0;
  }

  for (y = min_y; y < max_y; y++) {
    for (x = 0; x < image_width; x++) {

      disp = disparity_image_buffer[x + y * image_width];
      if ((disp > disp_far) && (disp < disp_close)) {
        bin_nr = (disp - disp_far) / bin_width;
        disp_bins[bin_nr]++;

        if (bin_nr < nr_bins - 1) {
          disp_bins[bin_nr + 1]++;
        }
      }
    }
  }

  sum_bins = 0;
  bin_index = 0;
  for (x = nr_bins - 1; x > 0; x--) {
    sum_bins += disp_bins[x];
    if ((sum_bins > bin_min_value) && (bin_index == 0)) {
      bin_index = x;
    }

  }

  if (bin_index > 0) {
    disp_start = ((bin_index - 1) * bin_width) + disp_far;
    disp_end = ((bin_index + 1) * bin_width) + disp_far;

    for (y = min_y; (y < max_y) && (feature_count < feature_count_limit); y++) {
      for (x = 0; (x < image_width) && (feature_count < feature_count_limit); x++) {

        disp = disparity_image_buffer[x + y * image_width];
        if ((disp > disp_start) && (disp < disp_end)) {
          feature_image_locations[feature_count] = x;
          feature_image_locations[feature_count_limit + feature_count] = y;
          feature_image_locations[(feature_count_limit * 2) + feature_count] = disparity_image_buffer[x + y * image_width];

          feature_count++;
        }
      }
    }
  }

  return feature_count;
}

void visualizeFeatureImageLocations(uint8_t *inI, uint8_t *inF, uint16_t nr_of_features, uint32_t image_width,
                                    uint16_t feature_count_limit)
{
  uint16_t i, x, y = 0;

  int image_width_bytes = image_width * 2;
  for (i = 0; i < nr_of_features; i++) {
    x = inF[i];
    y = inF[i + feature_count_limit];
    inI[(x * 2) + 1 + (y * image_width_bytes)] = 255;
  }
}

uint16_t visualizeBlobImageLocation(uint8_t *inI, uint8_t *inF, uint8_t *target_location,
                                    volatile uint16_t nr_of_features3, uint32_t image_width, uint16_t feature_count_limit)
{
  volatile uint16_t i = 0;
  volatile uint16_t j = 0;
  volatile int16_t x = 0;
  volatile int16_t y = 0;
  volatile int16_t disp = 0;
  volatile int16_t fx = 0;
  volatile int16_t fy = 0;
  volatile int16_t fdisp = 0;
  volatile int16_t x_error = 0;
  volatile int16_t y_error = 0;
  volatile int16_t x_mean = 0;
  volatile int16_t y_mean = 0;
  volatile int16_t disp_mean = 0;
  volatile uint16_t z_mean = 0;

  //volatile uint16_t nr_of_features = feature_count_limit;
  volatile uint16_t nr_of_features2 = feature_count_limit;
  volatile uint16_t count_features = 0;

  int image_width_bytes = image_width * 2;

  for (i = 0; i < nr_of_features2; i++) {
    x = inF[i];
    y = inF[i + feature_count_limit];
    disp = inF[i + (2 * feature_count_limit)];
    count_features = 0;
    x_mean = x;
    y_mean = y;
    disp_mean = disp;

    for (j = 0; j < nr_of_features2; j++) {
      fx = inF[j];
      fy = inF[j + feature_count_limit];
      fdisp = inF[i + (2 * feature_count_limit)];
      x_error = x - fx;
      y_error = y - fy;

      if ((x_error < 5) && (x_error > -5) && (y_error < 1) && (y_error > -8)) {
        count_features++;
        x_mean += fx;
        y_mean += fy;
        disp_mean += fdisp;
      }
    }

    target_location[0] = (uint8_t) 65;
    target_location[1] = (uint8_t) 48;
    target_location[2] = (uint8_t) 100;

    led_clear();

    if (count_features > 5) {
      x_mean = x_mean / nr_of_features2;
      y_mean = y_mean / nr_of_features2;
      disp_mean = disp_mean / nr_of_features2;
      z_mean = (120 * 6) / ((disp_mean / RESOLUTION_FACTOR) * 2); // [cm] divided by 2

      target_location[0] = (uint8_t) x_mean;
      target_location[1] = (uint8_t) y_mean;
      target_location[2] = (uint8_t) z_mean;


      inI[(x * 2) + 1 + (y * image_width_bytes)] = 255;
      inI[((x - 1) * 2) + 1 + ((y - 1) * image_width_bytes)] = 255;
      inI[((x + 1) * 2) + 1 + ((y - 1) * image_width_bytes)] = 255;
      inI[((x - 1) * 2) + 1 + ((y + 1) * image_width_bytes)] = 255;
      inI[((x + 1) * 2) + 1 + ((y + 1) * image_width_bytes)] = 255;

      led_set();

      /*
      target_location[0] = 0;
      target_location[1] = 1;
      target_location[2] = 2;
      */

      break;
    } else {
      count_features = 0;
    }


  }





  return count_features;
}

void getFeatureXYZLocations(uint8_t *in, float *out, uint16_t nr_of_features, uint32_t image_width,
                            uint32_t image_height)
{
  uint16_t i = 0;

  for (i = 0; i < nr_of_features; i++) {
    // Rotate image points based on camera roll offset and vehicle-attitude


    // convert image points to 3D world points defined by camera frame of reference


  }

}