/*
 * gate_detection.c
 *
 *  Created on: Sep 5, 2016
 *      Author: Guido de Croon
 ** @brief
 ** gate matching assumes that the gate will be seen from more or less from the centre of the gate in y
 ** direction.
 */

#include "gate_detection.h"

#include <math.h>
#include <stdlib.h>

#include "main_parameters.h"
#include "camera_type.h"
#include "stereo_math.h"
#include "math/stats.h"
#include "math/geometry.h"
#include "stereo_image.h"
#include "sys_time.h"

#ifdef USE_PPRZLINK
#include "pprzlink/intermcu_msg.h"
#endif

#ifdef COMPILE_ON_LINUX
#include <string.h>
#include <stdio.h>

void arm_sqrt_f32(float in, float *out)
{
  *out = sqrtf(in);
}
#else
#include "arm_math.h"
#endif

// gate params: TODO: would also be better to put elsewhere, centrally:
// TODO KIRK find correct
#define GATE_SIZE 1.1f  // m
#define HALF_GATE_SIZE 0.55f

// variables that have to be remembered in between function calls:

// since MAX_POINTS means that the algorithm will stop gathering points after MAX_POINTS, we sample according to a "moving" grid
// these starting points are made for a grid with step size 4
#define GRID_STEP 4
int Y0[16] = {0, 2, 1, 3, 0, 2, 1, 3, 0, 2, 1, 3, 0, 2, 1, 3};
int X0[16] = {0, 0, 0, 0, 2, 2, 2, 2, 1, 1, 1, 1, 3, 3, 3, 3};

struct point_t points[MAX_POINTS];
uint32_t n_points = 0;

// Settings for the fitting:
uint8_t weights[MAX_POINTS];
const uint8_t min_points = 50;
int WEIGHTED = 0; // color has no weight at the moment, since it is thresholded
//#define STICK // the stick we assume not to be red
#define CIRCLE 0
#define SQUARE 1
#define TRAPEZIUM 2
#define RECTANGLE 3
#define BOUNDING_BOX 4
#define POLYGON 5
#define DOOR 6

#ifndef GATE_SHAPE
#define GATE_SHAPE SQUARE
#endif
float outlier_threshold = 20.0f;

bool find_gate_from_side(struct image_t *img, struct roi_t *roi, uint16_t roi_w);
bool find_gate_from_top(struct image_t *img, struct roi_t *roi, uint16_t roi_w);

// Settings for the evolution:
// 10 individuals 30 generations is a normal setting
#define N_INDIVIDUALS 10
// The number of genes depends on the shape:
#if GATE_SHAPE == CIRCLE || GATE_SHAPE == SQUARE
#define N_GENES 3
#elif GATE_SHAPE == RECTANGLE || GATE_SHAPE == BOUNDING_BOX
#define N_GENES 4
#else
#define N_GENES 5
#endif
uint16_t n_generations = 15; // could be reduced for instance when there are many points
int16_t population[N_INDIVIDUALS][N_GENES];

// mutations will give a number in the range [-MUTATION_RANGE, MUTATION_RANGE]
#define MUTATION_RANGE 6

// watch out: inliers fit does not work so well...
#define DISTANCE_FIT 0
#define INLIERS_FIT 1
#define FF DISTANCE_FIT

// whether to draw on the disparity image:
#ifndef GATE_DETECTION_GRAPHICS
#define GATE_DETECTION_GRAPHICS 0
#endif

uint8_t color[3] = {128, 64, 64};
uint8_t gate_color[3] = {128, 64, 128};

#if !defined(GATE_Y_MIN) || !defined(GATE_Y_MAX)
#define GATE_Y_MIN 0
#define GATE_Y_MAX 255
#endif

#if !defined(GATE_U_MIN) || !defined(GATE_U_MAX) || !defined(GATE_V_MIN) || !defined(GATE_V_MAX)
#define GATE_U_MIN 0
#define GATE_U_MAX 255
#define GATE_V_MIN 0
#define GATE_V_MAX 255
#endif

uint8_t y_m = GATE_Y_MIN, y_M = GATE_Y_MAX;
uint8_t u_m = GATE_U_MAX, u_M = GATE_U_MAX;
uint8_t v_m = GATE_V_MAX, v_M = GATE_V_MAX;

void gate_set_intensity(uint8_t Y_m, uint8_t Y_M)
{
  y_m = Y_m;
  y_M = Y_M;
}

void gate_set_color(uint8_t Y_m, uint8_t Y_M, uint8_t U_m, uint8_t U_M, uint8_t V_m, uint8_t V_M)
{
  y_m = Y_m;
  y_M = Y_M;
  u_m = U_m;
  u_M = U_M;
  v_m = V_m;
  v_M = V_M;
}

/**************************************************************************************************/

// Gate detection settings:
#ifdef GATE_NSAMPLES
uint16_t n_samples        = GATE_NSAMPLES;
#else
uint16_t n_samples        = 2500;
#endif
uint8_t min_gate_size     = 10;
uint8_t min_gate_quality  = 14;
float gate_thickness      = 0.; // as fraction of gate size
float angle_to_gate       = 0;

// persistent results
#define MAX_GATES 25
struct gate_t gates[MAX_GATES];
struct image_t img_result;
uint8_t n_gates = 0;

/**************************************************************************************************/

void draw_gate(struct image_t *im, struct gate_t *gate, uint8_t shape, uint8_t *color)
{

  // check the four lines of which the gate consists:
  struct point_t tl, tr, bl, br;  // corner points of polygon

#ifdef GATE_ROTATE
  float s_rot = sinf(gate->rot), c_rot = cosf(gate->rot);
  tl.x = (int32_t)(gate->x - (float)gate->sz * c_rot - (float)gate->sz_left * s_rot); // --
  tl.y = (int32_t)(gate->y + (float)gate->sz * s_rot - (float)gate->sz_left * c_rot);
  bl.x = (int32_t)(gate->x - (float)gate->sz * c_rot + (float)gate->sz_left * s_rot); // -+
  bl.y = (int32_t)(gate->y + (float)gate->sz * s_rot + (float)gate->sz_left * c_rot);
  br.x = (int32_t)(gate->x + (float)gate->sz * c_rot + (float)gate->sz_right * s_rot); // ++
  br.y = (int32_t)(gate->y - (float)gate->sz * s_rot + (float)gate->sz_right * c_rot);
  tr.x = (int32_t)(gate->x + (float)gate->sz * c_rot - (float)gate->sz_right * s_rot); // +-
  tr.y = (int32_t)(gate->y - (float)gate->sz * s_rot - (float)gate->sz_right * c_rot);
#else
  tl.x = gate->x - gate->sz;
  tl.y = gate->y - gate->sz_left;
  bl.x = gate->x - gate->sz;
  bl.y = gate->y + gate->sz_left;
  br.x = gate->x + gate->sz;
  br.y = gate->y + gate->sz_right;
  tr.x = gate->x + gate->sz;
  tr.y = gate->y - gate->sz_right;
#endif

  // draw four lines on the image:
  if (shape == SQUARE || shape == RECTANGLE || shape == DOOR || shape == TRAPEZIUM) {
    image_draw_line(im, &bl, &tl, color);
    image_draw_line(im, &tl, &tr, color);
    image_draw_line(im, &tr, &br, color);
    image_draw_line(im, &bl, &br, color);
  } else if (GATE_SHAPE == CIRCLE) {
    tl.x = gate->x; tl.y = gate->y;
    image_draw_circle(im, &tl, gate->sz, color);
  } else if (GATE_SHAPE == BOUNDING_BOX) {
    image_draw_line(im, &bl, &tl, color);
    image_draw_line(im, &tl, &tr, color);
    image_draw_line(im, &tr, &br, color);
    image_draw_line(im, &bl, &br, color);

#ifdef GATE_ROTATE
    tl.x = gate->x + gate->sz_left * (-c_rot - s_rot);  // --
    tl.y = gate->y + gate->sz_left * (s_rot - c_rot);
    bl.x = gate->x + gate->sz_left * (-c_rot - s_rot);  // -+
    bl.y = gate->y + gate->sz_left * (s_rot + c_rot);
    br.x = gate->x + gate->sz_left * (c_rot + s_rot);    // ++
    br.y = gate->y + gate->sz_left * (-s_rot + c_rot);
    tr.x = gate->x + gate->sz_left * (c_rot + s_rot);    // +-
    tr.y = gate->y + gate->sz_left * (-s_rot - c_rot);
#else
    bl.x = gate->x - gate->sz_left;
    bl.y = gate->y - gate->sz_left;
    tl.x = gate->x - gate->sz_left;
    tl.y = gate->y + gate->sz_left;
    tr.x = gate->x + gate->sz_left;
    tr.y = gate->y + gate->sz_left;
    br.x = gate->x + gate->sz_left;
    br.y = gate->y - gate->sz_left;
#endif
    image_draw_line(im, &bl, &tl, color);
    image_draw_line(im, &tl, &tr, color);
    image_draw_line(im, &tr, &br, color);
    image_draw_line(im, &bl, &br, color);
  }
}


// Checks for a single pixel if it is the right color
// 1 means that it passes the filter
bool check_pixel(struct image_t *im, uint16_t x, uint16_t y, struct roi_t *roi)
{
  if (roi != NULL)
  {
    if (x < roi->tl.x || x >= roi->br.x ||
        y < roi->tl.y || y >= roi->br.y)
    {
      return false;
    }
  } else if (x >= im->w || y >= im->h) {
    return false;
  }

  uint8_t *buf;
  if (im->type == IMAGE_GRAYSCALE) {
    buf = (uint8_t *)im->buf + y * im->w + x;
    if (buf[0] >= y_m && buf[0] <= y_M) {
      return true;
    } else if (GATE_DETECTION_GRAPHICS) {
      // make the pixel black, so that we can see it on the ground station:
      buf[0] = 0;
    }
  } else if (im->type == IMAGE_YUV422) {
    uint8_t y_offset = 1;
    if (x % 2) {
      x--;
      y_offset += 2;
    }
    buf = (uint8_t *)im->buf + BYTES_PER_PIXEL * (y * im->w + x);
    // Check if the color is inside the specified values
    if ((buf[y_offset] >= y_m) && (buf[y_offset] <= y_M) &&
        (buf[0] >= u_m) && (buf[0] <= u_M) &&
        (buf[2] >= v_m) && (buf[2] <= v_M)) {
      return true;
    } else if (GATE_DETECTION_GRAPHICS) {
      // make the pixel black, so that we can see it on the ground station:
      buf[y_offset] = 0;
    }
  }

  return false;
}

// Checks for a single pixel if it is the right color and counts the pixels that passed the filter in 3 vertical bins
// 1 means that it passes the filter
bool check_pixel_and_count(struct image_t *im, uint16_t x, uint16_t y, uint16_t *counter, struct roi_t *roi)
{
  if (roi != NULL)
  {
    if (x < roi->tl.x || x >= roi->br.x ||
        y < roi->tl.y || y >= roi->br.y)
    {
      return false;
    }
  } else if (x >= im->w || y >= im->h) {
    return false;
  }

  int8_t Nbin = 3;
  uint8_t *buf;
  if (im->type == IMAGE_GRAYSCALE) {
    buf = (uint8_t *)im->buf + y * im->w + x;
    if (buf[0] >= y_m && buf[0] <= y_M) {
      for (uint8_t j = 0; j < Nbin; j++) {
        if (x < im->w * (j + 1) / Nbin) {
          counter[j]++;
          break;
        }
      }

      return true;
    } else if (GATE_DETECTION_GRAPHICS) {
      // make the pixel black, so that we can see it on the ground station:
      buf[0] = 0;
    }
  } else if (im->type == IMAGE_YUV422) {
    uint8_t y_offset = 1;
    if (x % 2) {
      x--;
      y_offset += 2;
    }
    buf = (uint8_t *)im->buf + 2 * (y * im->w + x); // each pixel has two bytes
    // Check if the color is inside the specified values
    if ((buf[y_offset] >= y_m) && (buf[y_offset] <= y_M) &&
        (buf[0] >= u_m) && (buf[0] <= u_M) &&
        (buf[2] >= v_m) && (buf[2] <= v_M)) {
      for (uint8_t j = 0; j < Nbin; j++) {
        if (x < im->w * (j + 1) / Nbin) {
          counter[j]++;
          break;
        }
      }

      return true;
    } else if (GATE_DETECTION_GRAPHICS) {
      // make the pixel black, so that we can see it on the ground station:
      buf[y_offset] = 0;
    }
  }

  return false;
}

// TODO: find a better way to evaluate a line, this way will work better for short lines than long lines due to integer rounding
void check_line(struct image_t *im, struct point_t Q1, struct point_t Q2, uint32_t *n_points,
                uint32_t *n_colored_points, struct roi_t *roi)
{
  static float t_step = 0.2;//0.05;
  static uint16_t x, y;
  static float t;

  (*n_points) = 0;
  (*n_colored_points) = 0;

  struct point_t vec = {Q2.x - Q1.x, Q2.y - Q1.y};
  // go from Q1 to Q2 in 1/t_step steps:
  for (t = 0.0f; t < 1.0f; t += t_step) {
    // determine integer coordinate on the line:
    x = (uint16_t)(Q1.x + (1.0f - t) * vec.x);
    y = (uint16_t)(Q1.y + (1.0f - t) * vec.y);

    if (x < im->w - 1 && y < im->h - 1) {
      // augment number of checked points:
      (*n_points)++;

      // due to rounding in line estimation, its possible that correct pixel is actually in a one pixel vicinity of us
      if (check_pixel(im, x, y, roi)
          || (x + 1 < im->w && check_pixel(im, x + 1, y, roi))
          || (y + 1 < im->h && check_pixel(im, x, y + 1, roi))
          || (x + 1 < im->w && y + 1 < im->h && check_pixel(im, x + 1, y + 1, roi))) {
        // the point is of the right color:
        (*n_colored_points)++;
      }
    }
  }
}

//#define USE_QUALITY_SIDES
void check_gate(struct image_t *im, struct gate_t *gate, struct roi_t *roi)
{
  uint32_t n_points = 0, n_colored_points = 0;
  uint32_t np, nc;
  // how much of the side should be visible to count as a detected side?
#ifdef USE_QUALITY_SIDES
  float min_ratio_side = 0.30;
  gate->n_sides = 0;
#endif

  // check the four lines of which the gate consists:
  struct point_t tl, tr, bl, br;  // corner points of polygon

#ifdef GATE_ROTATE
  float s_rot = sinf(M_PI - gate->rot), c_rot = cosf(M_PI - gate->rot);
  bl.x = (int32_t)(gate->x - (float)gate->sz * c_rot + (float)gate->sz_left * s_rot); // --
  bl.y = (int32_t)(gate->y - (float)gate->sz * s_rot - (float)gate->sz_left * c_rot);
  tl.x = (int32_t)(gate->x - (float)gate->sz * c_rot - (float)gate->sz_left * s_rot); // -+
  tl.y = (int32_t)(gate->y - (float)gate->sz * s_rot + (float)gate->sz_left * c_rot);
  tr.x = (int32_t)(gate->x + (float)gate->sz * c_rot - (float)gate->sz_right * s_rot); // ++
  tr.y = (int32_t)(gate->y + (float)gate->sz * s_rot + (float)gate->sz_right * c_rot);
  br.x = (int32_t)(gate->x + (float)gate->sz * c_rot + (float)gate->sz_right * s_rot); // +-
  br.y = (int32_t)(gate->y + (float)gate->sz * s_rot - (float)gate->sz_right * c_rot);
#else
  bl.x = gate->x - gate->sz;
  bl.y = gate->y - gate->sz_left;
  tl.x = gate->x - gate->sz;
  tl.y = gate->y + gate->sz_left;
  tr.x = gate->x + gate->sz;
  tr.y = gate->y + gate->sz_right;
  br.x = gate->x + gate->sz;
  br.y = gate->y - gate->sz_right;
#endif

  // left
  check_line(im, bl, tl, &np, &nc, roi);
#ifdef USE_QUALITY_SIDES
  if (nc >= min_ratio_side * np) {
    gate->n_sides++;
  }
#endif
  n_points += np;
  n_colored_points += nc;

  // top
  check_line(im, tl, tr, &np, &nc, roi);
#ifdef USE_QUALITY_SIDES
  if (nc >= min_ratio_side * np) {
    gate->n_sides++;
  }
#endif
  n_points += np;
  n_colored_points += nc;

  // right
  check_line(im, tr, br, &np, &nc, roi);
#ifdef USE_QUALITY_SIDES
  if (nc >= min_ratio_side * np) {
    gate->n_sides++;
  }
#endif
  n_points += np;
  n_colored_points += nc;

  if(GATE_SHAPE != DOOR)
  {
    // bottom
    check_line(im, bl, br, &np, &nc, roi);
#ifdef USE_QUALITY_SIDES
    if (nc >= min_ratio_side * np) {
      gate->n_sides++;
    }
#endif
    n_points += np;
    n_colored_points += nc;
  }

  struct point_t from, to;
  // confirm that we have a gate and not a coloured plate
  // must have less than 30% coloured points in centre
  // middle lr
  from.x = gate->x - gate->sz;
  from.y = gate->y;
  to.x = gate->x + gate->sz;
  to.y = gate->y;
  check_line(im, from, to, &np, &nc, roi);
  if (np && 100 * nc / np > 50) {
    n_points = 0;
  }

  // middle ud
  from.x = gate->x;
  from.y = gate->y - (gate->sz_left + gate->sz_right) / 2;
  to.x = gate->x;
  to.y = gate->y + (gate->sz_left + gate->sz_right) / 2;
  check_line(im, from, to, &np, &nc, roi);
  if (np && 100 * nc / np > 50) {
    n_points = 0;
  }

  // the quality is the ratio of colored points / number of points:
  if (n_points == 0) {
    gate->q = 0;
  } else {
    gate->q = (100 * n_colored_points) / n_points;
  }
}

/** snake_up_down will snake along a string of pixels setting the min and max of the snake
 *  y direction is up down. Minimum y is at top of image
 */
uint32_t snake_up_down(struct image_t *im, uint16_t x, uint16_t y, uint16_t *y_tl, uint16_t *y_br, uint16_t *x_tl,
                       uint16_t *x_br, struct roi_t *roi)
{
  struct roi_t limit = {.tl={0,0}, .br={im->w,im->h}};
  if (roi == NULL)
  {
    roi = &limit;
  }

  uint32_t num_points = 0;
  (*x_tl) = x;
  (*y_tl) = y;

  // snake towards negative y (up)
  while ((*y_tl) > roi->tl.y + 1) {
    num_points++;
    if (check_pixel(im, (*x_tl), (*y_tl) - 1, roi)) {   // check next pixel
      (*y_tl)--;
    } else if (check_pixel(im, (*x_tl), (*y_tl) - 2, roi)) {   // check next pixel skipping one
      (*y_tl) -= 2;
    } else if (check_pixel(im, (*x_tl) + 1, (*y_tl) - 1, roi)) {
      (*x_tl)++;
      (*y_tl)--;
    } else if (check_pixel(im, (*x_tl) - 1, (*y_tl) - 1, roi)) {
      (*x_tl)--;
      (*y_tl)--;
    } else if (check_pixel(im, (*x_tl) + 1, (*y_tl) - 2, roi)) {
      (*x_tl)++;
      (*y_tl) -= 2;
    } else if (check_pixel(im, (*x_tl) - 1, (*y_tl) - 2, roi)) {
      (*x_tl)--;
      (*y_tl) -= 2;
    } else if (check_pixel(im, (*x_tl) + 2, (*y_tl) - 2, roi)) {
      (*x_tl) += 2;
      (*y_tl) -= 2;
    } else if (check_pixel(im, (*x_tl) - 2, (*y_tl) - 2, roi)) {
      (*x_tl) -= 2;
      (*y_tl) -= 2;
    } else {
      num_points--;
      break;
    }
  }

  (*x_br) = x;
  (*y_br) = y;
  // snake towards positive y (down)
  while ((*y_br) < roi->br.y - 2) {
    num_points++;
    if (check_pixel(im, (*x_br), (*y_br) + 1, roi)) {
      (*y_br)++;
    } else if (check_pixel(im, (*x_br), (*y_br) + 2, roi)) {   // check next pixel skipping one
      (*y_br) += 2;
    } else if (check_pixel(im, (*x_br) + 1, (*y_br) + 1, roi)) {
      (*x_br)++;
      (*y_br)++;
    } else if (check_pixel(im, (*x_br) - 1, (*y_br) + 1, roi)) {
      (*x_br)--;
      (*y_br)++;
    } else if (check_pixel(im, (*x_br) + 1, (*y_br) + 2, roi)) {
      (*x_br)++;
      (*y_br) += 2;
    } else if (check_pixel(im, (*x_br) - 1, (*y_br) + 2, roi)) {
      (*x_br)--;
      (*y_br) += 2;
    } else if (check_pixel(im, (*x_br) + 2, (*y_br) + 2, roi)) {
      (*x_br) += 2;
      (*y_br) += 2;
    } else if (check_pixel(im, (*x_br) - 2, (*y_br) + 2, roi)) {
      (*x_br) -= 2;
      (*y_br) += 2;
    } else {
      num_points--;
      break;
    }
  }
  return num_points;
}

/** snake_up_down will snake along a string of pixels setting the min and max of the snake
 *  x direction is left right
 */
uint32_t snake_left_right(struct image_t *im, uint16_t x, uint16_t y, uint16_t *x_tl, uint16_t *x_br, uint16_t *y_tl,
    uint16_t *y_br, struct roi_t *roi)
{
  struct roi_t limit = {.tl={0,0}, .br={im->w,im->h}};
  if (roi == NULL)
  {
    roi = &limit;
  }

  uint32_t num_points = 0;
  (*x_tl) = x;
  (*y_tl) = y;

  // snake towards negative x (left)
  while ((*x_tl) > roi->tl.x + 1) {
    num_points++;
    if (check_pixel(im, (*x_tl) - 1, (*y_tl), roi)) {
      (*x_tl)--;
    } else if (check_pixel(im, (*x_tl) - 2, (*y_tl), roi)) {
      (*x_tl) -= 2;
    } else if (check_pixel(im, (*x_tl) - 1, (*y_tl) + 1, roi)) {
      (*y_tl)++;
      (*x_tl)--;
    } else if (check_pixel(im, (*x_tl) - 1, (*y_tl) - 1, roi)) {
      (*y_tl)--;
      (*x_tl)--;
    } else if (check_pixel(im, (*x_tl) - 2, (*y_tl) + 1, roi)) {
      (*y_tl)++;
      (*x_tl) -= 2;
    } else if (check_pixel(im, (*x_tl) - 2, (*y_tl) - 1, roi)) {
      (*y_tl)--;
      (*x_tl) -= 2;
    } else if (check_pixel(im, (*x_tl) - 2, (*y_tl) + 2, roi)) {
      (*y_tl) += 2;
      (*x_tl) -= 2;
    } else if (check_pixel(im, (*x_tl) - 2, (*y_tl) - 2, roi)) {
      (*y_tl) -= 2;
      (*x_tl) -= 2;
    } else {
      num_points--;
      break;
    }
  }

  (*y_br) = y;
  (*x_br) = x;
  // snake towards positive x (right)
  while ((*x_br) < roi->br.x - 2) {
    num_points++;
    if (check_pixel(im, (*x_br) + 1, (*y_br), roi)) {
      (*x_br)++;
    } else if (check_pixel(im, (*x_br) + 2, (*y_br), roi)) {
      (*x_br) += 2;
    } else if (check_pixel(im, (*x_br) + 1, (*y_br) + 1, roi)) {
      (*y_br)++;
      (*x_br)++;
    } else if (check_pixel(im, (*x_br) + 1, (*y_br) - 1, roi)) {
      (*y_br)--;
      (*x_br)++;
    } else if (check_pixel(im, (*x_br) + 2, (*y_br) + 1, roi)) {
      (*y_br)++;
      (*x_br) += 2;
    } else if (check_pixel(im, (*x_br) + 2, (*y_br) - 1, roi)) {
      (*y_br)--;
      (*x_br) += 2;
    } else if (check_pixel(im, (*x_br) + 2, (*y_br) + 2, roi)) {
      (*y_br) += 2;
      (*x_br) += 2;
    } else if (check_pixel(im, (*x_br) + 2, (*y_br) - 2, roi)) {
      (*y_br) -= 2;
      (*x_br) += 2;
    } else {
      num_points--;
      break;
    }
  }
  return num_points;
}

/*
void calculate_gate_position(int x_pix, int y_pix, int sz_pix, struct image_t *img, struct gate_t_img gate)
{
  // pixel distance conversion
  static float hor_angle = 0., vert_angle = 0.;

  // calculate angles here, rotate camera pixels 90 deg
  vert_angle = -(x_pix - img->h / 2) * radians_per_pix_h - stateGetNedToBodyEulers_f()->theta;
  hor_angle = (y_pix - img->w / 2) * radians_per_pix_w;

  // in body frame
  gate_x_dist = gate_size_m / (gate.sz * 2 * radians_per_pix_w);
  gate_y_dist = gate_x_dist * sin(hor_angle);
  gate_z_dist = gate_x_dist * sin(vert_angle);
}
*/

#ifdef USE_PPRZLINK
#include "pprz_datalink.h"
#include "pprzlink/intermcu_msg.h"

/* Send gate over pprzlink
 * @param gate Gate to send
 * @param depth Distance to gate, if not known set to 0
 */
void pprz_send_gate(struct gate_t *gate, float depth)
{
  float w = 2.f * gate->sz * FOVX / IMAGE_WIDTH;
  float h = 2.f * (float)(gate->sz_left > gate->sz_right ? gate->sz_left : gate->sz_right)
      * FOVY / IMAGE_HEIGHT;

  float theta = pix2angle((gate->x - IMAGE_WIDTH/2), 0);
  float phi = -pix2angle((gate->y - IMAGE_HEIGHT/2), 1); // positive y, causes negative phi

  pprz_msg_send_STEREOCAM_GATE(&(pprz.trans_tx), &dev, 0, &(gate->q), &w, &h,
      &phi, &theta, &depth);
}
#else
void pprz_send_gate(struct gate_t *gate, float depth){}
#endif

uint16_t x, y;
uint16_t x_tl, x_br, x_topl, x_bottoml, x_topr, x_bottomr;
uint16_t y_tl, y_br, y_topl, y_bottoml, y_topr, y_bottomr;
uint8_t init_x, init_y;

// Function
// Samples from the image and checks if the pixel is the right color.
// If yes, it "snakes" up and down to see if it is the side of a gate.
// If this stretch is long enough, it "snakes" also left and right.
// If the left/right stretch is also long enough, add the coords as a
// candidate square, optionally drawing it on the image.
bool snake_gate_detection(struct image_t *img, struct gate_t *best_gate, bool run_gen_alg, uint16_t *bins,
                          struct roi_t *roi, uint32_t *integral_image)
{
  uint32_t start_time = sys_time_get();
  struct roi_t ROI = {.tl = {0,0}, .br = {img->w, img->h}};
  // ensure roi is positive area grater that 1
  if (roi == NULL || roi->br.x - roi->tl.x < 1 ||
      roi->br.y - roi->tl.y < 1) {
    roi = &ROI;
  }

  uint16_t roi_w = roi->br.x - roi->tl.x;
  uint16_t roi_h = roi->br.y - roi->tl.y;

  best_gate->q = min_gate_quality;
  best_gate->sz = min_gate_size / 2;

  uint16_t i, j;

  uint16_t bin_cnt[3] = {0}; // initialize the counters for each bin with zero

  n_gates = 0;
  uint8_t best_x = 0, best_y = 0;
  for (i = 0; i < n_samples; i++) {
    // limit maximum computation time to 50ms
    if (get_timer_interval_ms(start_time) > 50){
      break;
    }

    // get a random coordinate:
    x = (rand() % roi_w) + roi->tl.x;
    y = (rand() % roi_h) + roi->tl.y;

    if (!check_pixel_and_count(img, x, y, bin_cnt, roi))
    {
      continue;
    }

    // if point is inside of an identified gate, exclude it
    static struct point_t polygon[4];
    bool inside_gate = false;
    struct point_t p = {.x = x, .y = y};
    for (j = 0; j < n_gates; j++)
    {
      polygon[0].x = gates[j].x - gates[j].sz - 2; polygon[0].y = gates[j].y - gates[j].sz_left - 2;
      polygon[1].x = gates[j].x - gates[j].sz - 2; polygon[1].y = gates[j].y + gates[j].sz_left + 2;
      polygon[2].x = gates[j].x + gates[j].sz + 2; polygon[2].y = gates[j].y - gates[j].sz_right - 2;
      polygon[3].x = gates[j].x + gates[j].sz + 2; polygon[3].y = gates[j].y + gates[j].sz_right + 2;

      if (isInside(polygon, 4, p))
      {
        inside_gate = true;
        break;
      }
    }
    if (inside_gate)
    {
      continue;
    }

    // check if the pixel has the right color/intensity
    if(find_gate_from_top(img, roi, roi_w) || find_gate_from_side(img, roi, roi_h)){
      // apply some limits based on shape needed to be fit
      if (GATE_SHAPE == SQUARE || GATE_SHAPE == CIRCLE) {
        gates[n_gates].sz = (gates[n_gates].sz + gates[n_gates].sz_left + gates[n_gates].sz_right) / 3;
        gates[n_gates].sz_left = gates[n_gates].sz_right = gates[n_gates].sz;
      } else if (GATE_SHAPE == RECTANGLE || GATE_SHAPE == DOOR || GATE_SHAPE == BOUNDING_BOX) {
        if (gates[n_gates].sz_left > gates[n_gates].sz_right) {
          gates[n_gates].sz_right = gates[n_gates].sz_left;
        } else if (gates[n_gates].sz_left < gates[n_gates].sz_right) {
          gates[n_gates].sz_left = gates[n_gates].sz_right;
        }
      } else if (GATE_SHAPE == TRAPEZIUM) {
        //gates[n_gates].y = (y_bottoml + y_topl + y_bottomr + y_topr) / 4;
      }

      // Door must be taller than it is wide
      if (GATE_SHAPE == DOOR && gates[n_gates].sz_left < 1.2 * gates[n_gates].sz) {
        continue;
      }

      // determine quality of gate
      if (gates[n_gates].n_sides > 2) {
        // check the gate quality:
        check_gate(img, &gates[n_gates], roi);
        // only increment the number of gates if the quality is better than previous gate
        // if same quality but bigger then also add gate
        // else it will be overwritten by the next one
        if (gates[n_gates].q >= best_gate->q) {
          best_x = init_x;
          best_y = init_y;
          *best_gate = gates[n_gates];
          n_gates++;
        }
      }

      if (n_gates >= MAX_GATES) {
        break;
      }
    }
  }

  if (n_gates > 0) {
    // do an additional fit to improve the gate detection:
    if (run_gen_alg) {
      // temporary variables:
      float fitness;//, angle_1, angle_2;
      // int clock_arms = 0;

      // prepare the Region of Interest (ROI), which is larger than the gate:
      float size_factor = 1.25;

      uint8_t max_candidate_gates = 10;
      uint8_t start = 0;
      if (n_gates > max_candidate_gates) {
        // only check max_candidate_gate gates, last gates are best quality
        start = n_gates - max_candidate_gates;
      }

      struct roi_t gen_roi;
      uint16_t gate_nr;
      for (gate_nr = start; gate_nr < n_gates; gate_nr++) {
        int16_t ROI_size = (int16_t)(((float) gates[gate_nr].sz) * size_factor);
        gen_roi.tl.x = gates[gate_nr].x - ROI_size;
        gen_roi.tl.x = (gen_roi.tl.x < 0) ? 0 : gen_roi.tl.x;
        gen_roi.br.x = gates[gate_nr].x + ROI_size;
        gen_roi.br.x = (gen_roi.br.x < img->w) ? gen_roi.br.x : img->h;
        gen_roi.tl.y = gates[gate_nr].y - ROI_size;
        gen_roi.tl.y = (gen_roi.tl.y < 0) ? 0 : gen_roi.tl.y;
        gen_roi.br.y = gates[gate_nr].y + ROI_size;
        gen_roi.br.y = (gen_roi.br.y < img->h) ? gen_roi.br.y : img->w;

        // use best gate a seed for elite population
        struct gate_t gen_gate = gates[gate_nr];
        // detect the gate:
        // TODO: instead of define, use a parameter to decide on GATE_SHAPE:
        fitness = gen_gate_detection(img, &gen_roi, &gen_gate, integral_image);
        check_gate(img, &gen_gate, roi);

        if (gen_gate.n_sides > 2 && gen_gate.q > best_gate->q) {
          // store the information in the gate:
          *best_gate = gen_gate;
        }
      }
    } else if (GATE_DETECTION_GRAPHICS) {
      // draw crosshair centre of gate:
      struct point_t top;
      struct point_t bottom;
      struct point_t left;
      struct point_t right;
      /*top.x = best_gate->x; top.y = best_gate->y - 5;
      bottom.x = best_gate->x; bottom.y = best_gate->y + 5;
      left.x = best_gate->x - 5; left.y = best_gate->y;
      right.x = best_gate->x + 5; right.y = best_gate->y;*/
      top.x = best_x; top.y = best_y - 5;
      bottom.x = best_x; bottom.y = best_y + 5;
      left.x = best_x - 5; left.y = best_y;
      right.x = best_x + 5; right.y = best_y;
      image_draw_line(img, &top, &bottom, color);
      image_draw_line(img, &left, &right, color);
    }
    // calculate_gate_position(best_gate.x, best_gate.y, best_gate.sz, img, best_gate);

    if (GATE_DETECTION_GRAPHICS) {
      draw_gate(img, best_gate, GATE_SHAPE, color);
    }

    if (bins != NULL) {
      bins[0] = bin_cnt[0];
      bins[1] = bin_cnt[1];
      bins[2] = bin_cnt[2];
    }

    return true;
  }
  return false;
}


bool find_gate_from_top(struct image_t *img, struct roi_t *roi, uint16_t roi_w)
{
  uint16_t sz = 0, szy1 = 0, szy2 = 0;
  uint32_t tot_nc;

  tot_nc = snake_left_right(img, x, y, &x_tl, &x_br, &y_tl, &y_br, roi);
  sz = x_br - x_tl;

  // check if we found a size that is long enough
  if (sz > min_gate_size && sz < roi_w) {
    x_tl += (uint16_t)(sz * gate_thickness);
    x_br -= (uint16_t)(sz * gate_thickness);

    init_x = x;
    init_y = y;

    // create the gate:
    gates[n_gates].n_sides = 1;
    gates[n_gates].sz = (x_br - x_tl) / 2;
    gates[n_gates].rot = 0;
    gates[n_gates].x = (x_br + x_tl) / 2;

    tot_nc += snake_up_down(img, x_tl, y_tl, &y_topl, &y_bottoml, &x_topl, &x_bottoml, roi);
    tot_nc += snake_up_down(img, x_br, y_br, &y_topr, &y_bottomr, &x_topr, &x_bottomr, roi);

    y_topl += (uint16_t)(sz * gate_thickness);
    y_bottoml -= (uint16_t)(sz * gate_thickness);
    y_topr += (uint16_t)(sz * gate_thickness);
    y_bottomr -= (uint16_t)(sz * gate_thickness);

    // sizes of the left-right stretches: in y pixel coordinates
    szy1 = y_bottoml - y_topl;
    szy2 = y_bottomr - y_topr;

    if (szy1 >= min_gate_size && szy2 >= min_gate_size) {
      gates[n_gates].sz_left = szy1 / 2;
      gates[n_gates].sz_right = szy2 / 2;
      //gates[n_gates].y = (y_bottom1 + y_top1 + y_bottom2 + y_top2) / 4;
      if (gates[n_gates].sz_left > gates[n_gates].sz_right) {
        gates[n_gates].y = (y_bottoml + y_topl) / 2;
      } else {
        gates[n_gates].y = (y_bottomr + y_topr) / 2;
      }
      // check if convex, i.e. check doesnt for a half H
      if (((y_bottoml > y_br + min_gate_size) && (y_topr + min_gate_size < y_tl)) ||
          ((y_bottomr > y_br + min_gate_size) && (y_topl + min_gate_size < y_tl))) {
        gates[n_gates].sz_left = gates[n_gates].sz_right = gates[n_gates].sz = 0;
        return false;
      }
      gates[n_gates].n_sides += 2;
#ifdef GATE_ROTATE
      gates[n_gates].rot = (atan2f((float)(x_bottoml) - (float)(x_topl), (float)(y_bottoml) - (float)(y_topl)) +
                            atan2f((float)(x_bottomr) - (float)(x_topr), (float)(y_bottomr) - (float)(y_topr))) / 2;
#endif
    } else if (szy1 >= min_gate_size) {
      gates[n_gates].y = (y_bottoml + y_topl) / 2;
      gates[n_gates].sz_left = gates[n_gates].sz_right = szy1 / 2;
      gates[n_gates].n_sides++;
#ifdef GATE_ROTATE
      gates[n_gates].rot = atan2f((float)(x_bottoml) - (float)(x_topl), (float)(y_bottoml) - (float)(y_topl));
#endif
    } else if (szy2 >= min_gate_size) {
      gates[n_gates].y = (y_bottomr + y_topr) / 2;
      gates[n_gates].sz_left = gates[n_gates].sz_right = szy2 / 2;
      gates[n_gates].n_sides++;
#ifdef GATE_ROTATE
      gates[n_gates].rot = atan2f((float)(x_bottomr) - (float)(x_topr), (float)(y_bottomr) - (float)(y_topr));
#endif
    } else {
      gates[n_gates].sz_left = gates[n_gates].sz_right = 0;
      return false;
    }

    // Need both sides of door and not find the bottom and not a P
    if (GATE_SHAPE == DOOR && (gates[n_gates].n_sides < 3 || y > gates[n_gates].y ||
        abs(gates[n_gates].sz_left - gates[n_gates].sz_right) > min_gate_size/2)) {
      gates[n_gates].sz_left = gates[n_gates].sz_right =  gates[n_gates].sz = 0;
      return false;
    }

    static uint8_t side_used = 0;
    // try to identify other horizontal side of the gate
    if (y > gates[n_gates].y && szy1 >= min_gate_size && szy1 > szy2) {  // use top left
      x = x_topl;
      y = y_topl;
      side_used = 0;
    } else if (y <= gates[n_gates].y && szy1 >= min_gate_size && szy1 > szy2) {  // use bottom left
      x = x_bottoml;
      y = y_bottoml;
      side_used = 0;
    } else if (y > gates[n_gates].y && szy2 >= min_gate_size) {  // use top right
      x = x_topr;
      y = y_topr;
      side_used = 1;
    } else if (y <= gates[n_gates].y && szy2 >= min_gate_size) { // use bottom right
      x = x_bottoml;
      y = y_bottoml;
      side_used = 1;
    }

    tot_nc += snake_left_right(img, x, y, &x_tl, &x_br, &y_tl, &y_br, roi);
    // check if length long enough and horizontal closes shape (ie not an s)
    if ((x_br - x_tl >= min_gate_size) && (x_br - x_tl > 3 * gates[n_gates].sz / 2) &&
        ((side_used == 0 && (x_tl + min_gate_size / 2 < x)) || (side_used == 1 && !(x_br < x + min_gate_size / 2)))) {
      gates[n_gates].n_sides++;
      sz = x_br - x_tl;
      x_tl += (uint16_t)(sz * gate_thickness);
      x_br -= (uint16_t)(sz * gate_thickness);
      if (y > gates[n_gates].y) {
        if (x_tl < x_topl) {
          x_topl = x_tl;
          y_topl = y_tl;
        }
        if (x_br > x_topr) {
          x_topr = x_br;
          y_topr = y_br;
        }
      } else {
        if (x_tl < x_bottoml) {
          x_bottoml = x_tl;
          y_bottoml = y_tl;
        }
        if (x_br > x_topr) {
          x_bottomr = x_br;
          y_bottomr = y_br;
        }
      }
      gates[n_gates].sz = (gates[n_gates].sz * 2 + (x_br - x_tl)) / 4;
      gates[n_gates].x = (gates[n_gates].x * 2 + x_br + x_tl) / 4;
    }

    if (GATE_SHAPE == DOOR && gates[n_gates].n_sides > 3) {
      // found bottom so not a door
      gates[n_gates].sz_left = gates[n_gates].sz_right =  gates[n_gates].sz = 0;
      return false;
    }

#ifdef GATE_ROTATE
    gates[n_gates].x += gates[n_gates].sz_left * tanf(gates[n_gates].rot);
#endif
    return true;
  }
  return false;
}

bool find_gate_from_side(struct image_t *img, struct roi_t *roi, uint16_t roi_h)
{
  uint16_t sz = 0, szx1 = 0, szx2 = 0;
  uint32_t tot_nc;

  tot_nc = snake_up_down(img, x, y, &y_tl, &y_br, &x_tl, &x_br, roi);
  sz = y_br - y_tl;

  // check if we found a size that is long enough
  if (sz > min_gate_size && sz < roi_h) {
    y_tl += (uint16_t)(sz * gate_thickness);
    y_br -= (uint16_t)(sz * gate_thickness);

    init_x = x;
    init_y = y;

    // create the gate:
    gates[n_gates].n_sides = 1;
    gates[n_gates].sz_left = gates[n_gates].sz_right = (y_br - y_tl) / 2;
    gates[n_gates].rot = 0;
    gates[n_gates].y = (y_br + y_tl) / 2;

#ifdef GATE_ROTATE
    gates[n_gates].rot = atan2f((float)(x_br - x_tl), (float)(y_br - y_tl));
#endif

    tot_nc += snake_left_right(img, x_tl, y_tl, &x_topl, &x_topr, &y_topl, &y_topr, roi);
    tot_nc += snake_left_right(img, x_br, y_br, &x_bottoml, &x_bottomr, &y_bottoml, &y_bottomr, roi);

    x_topl += (uint16_t)(sz * gate_thickness);
    x_bottoml -= (uint16_t)(sz * gate_thickness);
    x_topr += (uint16_t)(sz * gate_thickness);
    x_bottomr -= (uint16_t)(sz * gate_thickness);

    // sizes of the left-right stretches: in y pixel coordinates
    szx1 = x_bottomr - x_bottoml;
    szx2 = x_topr - x_topl;

    if (szx1 > min_gate_size && szx2 > min_gate_size) {
      gates[n_gates].x = (x_bottoml + x_topl + x_bottomr + x_topr) / 4;
      if (szx1 > szx2) {
        //gates[n_gates].x = (x_bottomr + x_bottoml) / 2;
        gates[n_gates].sz = szx1 / 2;
      } else {
        //gates[n_gates].x = (x_topr + x_topl) / 2;
        gates[n_gates].sz = szx2 / 2;
      }
      // check if convex
      if (((x_bottoml + min_gate_size <= x_br) && (x_topr >= x_tl + min_gate_size)) ||
          ((x_bottomr >= x_br + min_gate_size) && (x_topl + min_gate_size <= x_tl))) {
        gates[n_gates].sz_left = gates[n_gates].sz_right = gates[n_gates].sz = 0;
        return false;
      }
      gates[n_gates].n_sides += 2;
    } else if (szx1 > min_gate_size) {
      gates[n_gates].x = (x_bottomr + x_bottoml) / 2;
      gates[n_gates].sz = szx1 / 2;
      gates[n_gates].n_sides++;
    } else if (szx2 > min_gate_size) {
      gates[n_gates].x = (x_topr + x_topl) / 2;
      gates[n_gates].sz = szx2 / 2;
      gates[n_gates].n_sides++;
    } else {
      gates[n_gates].sz_left = gates[n_gates].sz_right = 0;
      return false;
    }

    // Found a bottom, not a door
    if (GATE_SHAPE == DOOR && szx1 > min_gate_size) {
      gates[n_gates].sz_left = gates[n_gates].sz_right = gates[n_gates].sz = 0;
      return false;
    }

    static uint8_t side_used = 0;
    // try to identify other vertical side of the gate
    if (x >= gates[n_gates].x && szx1 > min_gate_size && szx1 > szx2) {  // use bottom left
      x = x_bottoml;
      y = y_bottoml;
      side_used = 0;
    } else if (x < gates[n_gates].x && szx1 > min_gate_size && szx1 > szx2) {  // use bottom right
      x = x_bottomr;
      y = y_bottomr;
      side_used = 0;
    } else if (x >= gates[n_gates].x && szx2 > min_gate_size) {  // use top left
      x = x_topl;
      y = y_topl;
      side_used = 1;
    } else if (x < gates[n_gates].x && szx2 > min_gate_size) { // use top right
      x = x_topr;
      y = y_topr;
      side_used = 1;
    }

    tot_nc = snake_up_down(img, x, y, &y_tl, &y_br, &x_tl, &x_br, roi);
    // check if length long enough and vertical closes shape (ie not an s)
    if ((y_br - y_tl > min_gate_size) &&
        ((side_used == 0 && !(y_br > y + min_gate_size / 2)) || (side_used == 1 && (y_tl + min_gate_size / 2 > y)))) {
      gates[n_gates].n_sides++;
      y_tl += (uint16_t)(sz * gate_thickness);
      y_br -= (uint16_t)(sz * gate_thickness);

      if (x > gates[n_gates].x) {
        if (y_br > y_bottoml) {
          y_bottoml = y_br;
          x_bottoml = x_br;
        }
        if (y_tl < y_topl) {
          y_topl = y_tl;
          x_topl = x_tl;
        }
        gates[n_gates].sz_left = (y_bottoml - y_topl) / 2;
      } else {
        if (y_br > y_bottomr) {
          y_bottomr = y_br;
          x_bottomr = x_br;
        }
        if (y_tl < y_topr) {
          y_topr = y_tl;
          x_topr = x_tl;
        }
        gates[n_gates].sz_right = (y_bottomr - y_topr) / 2;
      }

      gates[n_gates].y = (gates[n_gates].y * 2 + y_br + y_tl) / 4;
#ifdef GATE_ROTATE
      gates[n_gates].rot = (gates[n_gates].rot + atan2f((float)(x_br - x_tl), (float)(y_br - y_tl))) / 2;
#endif
    }
    // Check if found both left and right
    if (GATE_SHAPE == DOOR && (gates[n_gates].n_sides < 3 ||
        abs(gates[n_gates].sz_left - gates[n_gates].sz_right) > min_gate_size/2)) {
      gates[n_gates].sz_left = gates[n_gates].sz_right =  gates[n_gates].sz = 0;
      return false;
    }
#ifdef GATE_ROTATE
    gates[n_gates].x += gates[n_gates].sz_left * tanf(gates[n_gates].rot);
#endif

    return true;
  }
  return false;
}

/**************************************************************************************************/

/**
 * Function takes a disparity image and fits a circular gate to the close-by points.
 * - x0, y0, size0 contain an initial guess and indicate where the closest object probably is
 * - x_center, y_center, and radius are also used for initialising the optimisation. The optimised results are put back in these parameters.
 * - fitness: how good is the hypothesis of x_center, etc.
 * @author Guido
 */
uint16_t w, h;  // todo, temp remove later
float gen_gate_detection(struct image_t *image, struct roi_t *roi, struct gate_t *gate, uint32_t *integral_image)
{
  struct roi_t ROI = {.tl = {0,0}, .br = {image->w, image->h}};
  // ensure roi is positive area grater that 1
  if (roi == NULL || roi->br.x - roi->tl.x < 1 || roi->br.y - roi->tl.y < 1) {
    roi = &ROI;
  }

  if (integral_image == NULL) {
    return -1.f;
  }

  get_integral_image((uint8_t *)image->buf, image->w, image->h, integral_image);
  w = image->w;
  h = image->h;

  float fitness;
  struct gate_t gate0;
  //float *angle_1, *angle_2;
  //float *psi;
  // 1) convert the disparity map to a vector of points:
  n_points = convert_image_to_points(image, roi, points, weights);

  // if there are enough points close by:
  if (n_points > min_points) {
    // 2) fit a window to the points
    // determine initial guess (we always do):
    // Initialise with average position and size
    int32_t sum_x = 0.;
    int32_t sum_y = 0.;

    int32_t mean_left = 0., n_points_left = 0.;
    int32_t mean_right = 0., n_points_right = 0.;

    uint16_t i;
    for (i = 0; i < n_points; i++) {
      sum_x += points[i].x; // could still do a weighted average
      sum_y += points[i].y;
    }
    gate0.x = sum_x / n_points;
    gate0.y = sum_y / n_points;

    for (i = 0; i < n_points; i++) {
      if (points[i].x < gate0.x) {
        mean_left += points[i].x;
        n_points_left++;
      } else if (points[i].x > gate0.x) {
        mean_right += points[i].x;
        n_points_right++;
      }
    }
    mean_left /= n_points_left;
    mean_right /= n_points_right;

    gate0.sz = (mean_right - mean_left) / 2;
    // bound the size
    BoundLower(gate0.sz, min_gate_size);
    if (GATE_SHAPE == BOUNDING_BOX) {
      gate0.sz_left = gate0.sz_right = gate0.sz / 2;
      BoundLower(gate0.sz_left, min_gate_size);
      BoundLower(gate0.sz_right, min_gate_size);
    } else {
      gate0.sz_left = gate0.sz_right = gate0.sz;
    }

    if (GATE_DETECTION_GRAPHICS) {
      // draw initial guess as a crosshair:
      struct point_t top;
      struct point_t bottom;
      struct point_t left;
      struct point_t right;
      top.x = gate0.x; top.y = gate0.y - gate0.sz_left;
      bottom.x = gate0.x; bottom.y = gate0.y + gate0.sz_left;
      left.x = gate0.x - gate0.sz; left.y = gate0.y;
      right.x = gate0.x + gate0.sz; right.y = gate0.y;
      image_draw_line(image, &top, &bottom, color);
      image_draw_line(image, &left, &right, color);
    }

    // run the fit procedure:
    fitness = gen_run(&gate0, gate, integral_image);

    if (GATE_SHAPE == TRAPEZIUM) {
      // the sizes of the two sides (together with knowledge on the real-world size of the gate and FOV etc. of the camera)
      // tells us the angle to the center of the gate.
      //(*psi) = get_angle_from_polygon(*s_left, *s_right, color_image);
    }

    if (GATE_DETECTION_GRAPHICS) {
      draw_gate(image, gate, GATE_SHAPE, color);
#ifdef STICK
      draw_stick(image, (*x_center), (*y_center), (*radius), color);
#endif
    }
  } else {
    fitness = BAD_FIT;
  }
  return fitness;
}

int16_t get_mutation(void)
{
  return (int16_t)((rand() % (MUTATION_RANGE * 2 + 1)) - MUTATION_RANGE);
}

float gen_run(struct gate_t *gate0, struct gate_t *gen_gate, uint32_t *integral_image)
{
  float fitness = 1000000;  // large number, since we will minimise it:
  uint16_t i, g, ge, j, k;
  for (i = 0, j = N_INDIVIDUALS / 2; i < N_INDIVIDUALS / 2; i++, j++) {
    population[i][0] = gate0->x + get_mutation(); Bound(population[i][0], 0, IMAGE_WIDTH);
    population[i][1] = gate0->y + get_mutation(); Bound(population[i][1], 0, IMAGE_WIDTH);
    population[i][2] = gate0->sz + get_mutation(); Bound(population[i][2], 0, IMAGE_WIDTH / 2);
    if (GATE_SHAPE == RECTANGLE || GATE_SHAPE == BOUNDING_BOX) {
      population[i][3] = gate0->sz_left + get_mutation(); Bound(population[i][3], 0, IMAGE_WIDTH / 2);
    } else if (GATE_SHAPE == TRAPEZIUM) {
      // also the half-sizes of the right and left part of the gate are optimised:
      population[i][3] = gate0->sz_left + get_mutation(); Bound(population[i][3], 0, IMAGE_WIDTH / 2);
      population[i][4] = gate0->sz_right + get_mutation(); Bound(population[i][4], 0, IMAGE_WIDTH);
    }

    population[j][0] = gen_gate->x + get_mutation(); Bound(population[j][0], 0, IMAGE_WIDTH / 2);
    population[j][1] = gen_gate->y + get_mutation(); Bound(population[j][1], 0, IMAGE_WIDTH / 2);
    population[j][2] = gen_gate->sz + get_mutation(); Bound(population[j][2], 0, IMAGE_WIDTH / 2);
    if (GATE_SHAPE == RECTANGLE || GATE_SHAPE == BOUNDING_BOX) {
      population[j][3] = gen_gate->sz_left + get_mutation(); Bound(population[j][3], 0, IMAGE_WIDTH / 2);
    } else if (GATE_SHAPE == TRAPEZIUM) {
      // also the half-sizes of the right and left part of the gate are optimised:
      population[j][3] = gen_gate->sz_left + get_mutation(); Bound(population[j][3], 0, IMAGE_WIDTH / 2);
      population[j][4] = gen_gate->sz_right + get_mutation(); Bound(population[j][4], 0, IMAGE_WIDTH / 2);
    }
  }

  float total_sum_weights = sum(weights, n_points);

  float fits[N_INDIVIDUALS];
  int32_t genome0[N_GENES];
  genome0[0] = gate0->x;
  genome0[1] = gate0->y;
  genome0[2] = gate0->sz;
  if (GATE_SHAPE == RECTANGLE || GATE_SHAPE == BOUNDING_BOX) {
    genome0[3] = gate0->sz_left;
  } else if (GATE_SHAPE == TRAPEZIUM) {
    genome0[3] = gate0->sz_left;
    genome0[4] = gate0->sz_right;
  }
  int32_t best_genome[N_GENES];
  memcpy(best_genome, genome0, sizeof(genome0));

  uint16_t index;
  for (g = 0; g < n_generations; g++) {
    for (i = 0; i < N_INDIVIDUALS; i++) {
      if (GATE_SHAPE == CIRCLE) {
        if (FF == DISTANCE_FIT) {
          // Optimise mean distance to circle (and possibly stick)
          fits[i] = mean_distance_to_circle(population[i], points, n_points);
        } else {
          // Optimise the number of inliers
          fits[i] = get_outlier_ratio(population[i], total_sum_weights);
        }
      } else if (GATE_SHAPE == SQUARE) {
        // Optimise mean distance to square (and possibly stick)
        fits[i] = mean_distance_to_square(population[i], points, n_points);
      } else if (GATE_SHAPE == RECTANGLE) {
        fits[i] = mean_distance_to_rectangle(population[i], points, n_points);
      } else if (GATE_SHAPE == TRAPEZIUM) {
        // Optimise mean distance to a polygon (and possibly stick)
        fits[i] = mean_distance_to_polygon(population[i], points, n_points);
      } else if (GATE_SHAPE == BOUNDING_BOX) {
        // find bounding box for points
        fits[i] = bounding_box_score(population[i], points, n_points, integral_image);
      }
    }

    // if better than any previous individual, remember it:
    if (get_minimum_f(fits, N_INDIVIDUALS, &index) < fitness) {
      for (ge = 0; ge < N_GENES; ge++) {
        best_genome[ge] = population[index][ge];
      }
      fitness = fits[index];
    }

    // fill the new population with mutated copies of this generation's best:
    // super elitist evolution:
    if (g < n_generations - 1) {
      for (j = 0; j < N_GENES; j++) {
        for (i = 0, k = N_INDIVIDUALS / 2; i < N_INDIVIDUALS / 2; i++, k++) {
          // mutate all genes in the same way:
          population[i][j] = best_genome[j] + get_mutation();
          if (j > 1) {
            Bound(population[i][j], 0, IMAGE_WIDTH / 2);
          } else {
            Bound(population[i][j], 0, IMAGE_WIDTH);
          }

          // mutate all genes in the same way:
          if (g < n_generations / 2) {
            population[k][j] = genome0[j] + get_mutation();
          } else {
            population[k][j] = best_genome[j] + get_mutation();
          }
          if (j > 1) {
            Bound(population[k][j], 0, IMAGE_WIDTH / 2);
          } else {
            Bound(population[k][j], 0, IMAGE_WIDTH);
          }
        }
      }
    }
  }

  // put the final values back in the parameters:
  //if (FF == DISTANCE_FIT) { (*fitness) /= total_sum_weights; }
  gen_gate->x = best_genome[0];
  gen_gate->y = best_genome[1];
  gen_gate->sz = best_genome[2];
  if (GATE_SHAPE == RECTANGLE || GATE_SHAPE == BOUNDING_BOX) {
    // radius has x-direction size, s_left has y-direction size
    gen_gate->sz_left = best_genome[3];
    gen_gate->sz_right = best_genome[3];
  } else if (GATE_SHAPE == TRAPEZIUM) {
    gen_gate->sz_left = best_genome[3];
    gen_gate->sz_right = best_genome[4];
  } else {
    gen_gate->sz_left = gen_gate->sz;
    gen_gate->sz_right = gen_gate->sz;
  }

  return fitness;
}

uint32_t convert_image_to_points(struct image_t *img, struct roi_t *roi,
                                 struct point_t points[], uint8_t weights[])
{
  uint16_t y, x, sp;
  uint32_t p = 0;
  memset(weights, 0, MAX_POINTS);

  // We stop sampling at MAX_POINTS, but do not want our samples to be biased toward a certain
  // part of the image.
  // We have different grid starting points (GRID_STEP*GRID_STEP) so that for every different
  // starting point sp, we will sample different positions in the image, finally covering the
  // whole image.
  for (sp = 0; sp < GRID_STEP * GRID_STEP; sp++) {
    for (y = roi->tl.y + Y0[sp]; y < roi->br.y; y += GRID_STEP) {
      for (x = roi->tl.x + X0[sp]; x < roi->br.x; x += GRID_STEP) {
        if (check_pixel(img, x, y, roi)) {
          // add the points to the array, and use disparity as the weight:
          points[p].x = x;
          points[p].y = y;
          if (img->type == IMAGE_GRAYSCALE) {
            weights[p] = ((uint8_t *)img->buf)[y * img->w + x];
          } else {
            weights[p] = 1;
          }

          // if the maximum number of points is reached, return:
          if (++p == MAX_POINTS) {
            return p;
          }
        }
      }
    }
  }

  return p;
}

float mean_distance_to_circle(int16_t *genome, struct point_t *points, uint32_t n_points)
{
  static struct point_f point;
  static float dx, dy;
  static float dist_center, error;
  static uint32_t p;

  float sum_distance = 0.0f;

  float x = genome[0];
  float y = genome[1];
  float r = genome[2];

  for (p = 0; p < n_points; p++) {
    point.x = points[p].x; point.y = points[p].y;
    dx = point.x - x;
    dy = point.y - y;
    arm_sqrt_f32(dx * dx + dy * dy, &dist_center);
    error = fabs(dist_center - r);

#ifdef STICK
    float error_stick;
    // determine distance to the stick:
    struct point_f stick1;
    stick1.x = x;
    stick1.y = y + r;
    struct point_f stick2;
    stick2.x = x;
    stick2.y = y + 2 * r;
    error_stick = distance_to_vertical_segment(stick1, stick2, point);

    // take the smallest error:
    if (error_stick < error) { error = error_stick; }
#endif

    // apply outlier threshold before applying weights:
    if (error > outlier_threshold) { error = outlier_threshold; }

    if (WEIGHTED) {
      sum_distance += error * weights[p];
    } else {
      sum_distance += error;
    }
  }
  return sum_distance / n_points;
}

float mean_distance_to_square(int16_t *genome, struct point_t *points, uint32_t n_points)
{
  static struct point_f square_top_left;
  static struct point_f square_top_right;
  static struct point_f square_bottom_right;
  static struct point_f square_bottom_left;

  static struct point_f point;
  static float error;
  static uint32_t p;
  static uint16_t index;
  static uint8_t n_sides = 4;

  float sum_distance = 0.0f;

  float x = genome[0];
  float y = genome[1];
  float r = genome[2];

  // determine corner points:
  square_top_left.x = x - r;
  square_top_left.y = y - r; // positive y direction is down
  square_top_right.x = x + r;
  square_top_right.y = y - r;
  square_bottom_left.x = x - r;
  square_bottom_left.y = y + r;
  square_bottom_right.x = x + r;
  square_bottom_right.y = y + r;
  float side_distances[n_sides];

  for (p = 0; p < n_points; p++) {
    // get the current point:
    point.x = points[p].x; point.y = points[p].y;
    // determine the distance to the four sides of the square and select the smallest one:
    side_distances[0] = distance_to_vertical_segment(square_bottom_left, square_top_left, point);
    side_distances[1] = distance_to_vertical_segment(square_bottom_right, square_top_right, point);
    side_distances[2] = distance_to_horizontal_segment(square_top_left, square_top_right, point);
    side_distances[3] = distance_to_horizontal_segment(square_bottom_left, square_bottom_right, point);
    error = get_minimum_f(side_distances, n_sides, &index);

#ifdef STICK
    float error_stick;
    // determine distance to the stick:
    struct point_f stick1;
    stick1.x = x;
    stick1.y = y + r;
    struct point_f stick2;
    stick2.x = x;
    stick2.y = y + 2 * r;
    error_stick = distance_to_vertical_segment(stick1, stick2, point);

    // take the smallest error:
    if (error_stick < error) { error = error_stick; }
#endif

    // apply outlier threshold before applying weights:
    if (error > outlier_threshold) { error = outlier_threshold; }

    if (WEIGHTED) {
      sum_distance += error * weights[p];
    } else {
      sum_distance += error;
    }
  }
  return sum_distance / n_points;
}

float mean_distance_to_rectangle(int16_t *genome, struct point_t *points, uint32_t n_points)
{
  static struct point_f rectangle_top_left;
  static struct point_f rectangle_top_right;
  static struct point_f rectangle_bottom_right;
  static struct point_f rectangle_bottom_left;

  static float error;
  static uint32_t p;
  static uint16_t index;
  static uint8_t n_sides = 4;
  static struct point_f point;

  float x = genome[0];
  float y = genome[1];
  float rx = genome[2];
  float ry = genome[3];

  float sum_distance = 0.0f;

  // determine corner points:
  rectangle_top_left.x = x - rx;
  rectangle_top_left.y = y + ry;
  rectangle_top_right.x = x + rx;
  rectangle_top_right.y = y + ry;
  rectangle_bottom_left.x = x - rx;
  rectangle_bottom_left.y = y - ry;
  rectangle_bottom_right.x = x + rx;
  rectangle_bottom_right.y = y - ry;
  float side_distances[n_sides];

  for (p = 0; p < n_points; p++) {
    // get the current point:
    point.x = points[p].x; point.y = points[p].y;
    // determine the distance to the four sides of the square and select the smallest one:
    side_distances[0] = distance_to_vertical_segment(rectangle_bottom_left, rectangle_top_left, point);
    side_distances[1] = distance_to_vertical_segment(rectangle_bottom_right, rectangle_top_right, point);
    side_distances[2] = distance_to_horizontal_segment(rectangle_top_left, rectangle_top_right, point);
    side_distances[3] = distance_to_horizontal_segment(rectangle_bottom_left, rectangle_bottom_right, point);
    error = get_minimum_f(side_distances, n_sides, &index);

    // apply outlier threshold before applying weights:
    if (error > outlier_threshold) { error = outlier_threshold; }

    if (WEIGHTED) {
      sum_distance += error * weights[p];
    } else {
      sum_distance += error;
    }
  }
  return sum_distance / n_points;
}

float mean_distance_to_polygon(int16_t *genome, struct point_t *points, uint32_t n_points)
{
  static struct point_f square_top_left;
  static struct point_f square_top_right;
  static struct point_f square_bottom_right;
  static struct point_f square_bottom_left;

  static float error;
  static uint32_t p;
  static uint16_t index;
  static uint8_t n_sides = 4;
  static struct point_f point;

  float x = genome[0];
  float y = genome[1];
  float rx = genome[2];
  float r_left = genome[3];
  float r_right = genome[4];

  float sum_distance = 0.0f;

  // determine corner points:
  square_top_left.x = x - rx;
  square_top_left.y = y + r_left;
  square_top_right.x = x + rx;
  square_top_right.y = y + r_right;
  square_bottom_left.x = x - rx;
  square_bottom_left.y = y - r_left;
  square_bottom_right.x = x + rx;
  square_bottom_right.y = y - r_right;
  float side_distances[n_sides];

  for (p = 0; p < n_points; p++) {
    // get the current point:
    point.x = points[p].x; point.y = points[p].y;
    // determine the distance to the four sides of the square and select the smallest one:
    side_distances[0] = distance_to_segment(square_bottom_left, square_top_left, point);
    side_distances[1] = distance_to_segment(square_bottom_right, square_top_right, point);
    side_distances[2] = distance_to_segment(square_top_left, square_top_right, point);
    side_distances[3] = distance_to_segment(square_bottom_left, square_bottom_right, point);
    error = get_minimum_f(side_distances, n_sides, &index);

#ifdef STICK
    float error_stick;
    // determine distance to the stick:
    struct point_f stick1;
    stick1.x = x;
    stick1.y = y - (s_left + s_right) / 2;
    struct point_f stick2;
    stick2.x = x;
    stick2.y = y - (s_left + s_right);
    error_stick = distance_to_vertical_segment(stick2, stick1, point);

    // take the smallest error:
    if (error_stick < error) { error = error_stick; }
#endif

    // apply outlier threshold before applying weights:
    if (error > outlier_threshold) { error = outlier_threshold; }

    if (WEIGHTED) {
      sum_distance += error * weights[p];
    } else {
      sum_distance += error;
    }
  }
  return sum_distance / n_points;
}

float bounding_box_score(int16_t *genome, struct point_t *points, uint32_t n_points, uint32_t *integral_image)
{
  float score = 10000000;

  float x = genome[0];
  float y = genome[1];
  float ro = genome[2];
  float ri = genome[3];

  if (x - ro < 0 || x + ro >= IMAGE_WIDTH || y - ro < 0 || y + ro >= IMAGE_HEIGHT) {
    return score;
  }

  float outer = (float)get_sum_disparities(MAX(x - ro, 0), MAX(y - ro, 0), MIN(x + ro, w), MIN(y + ro, h), integral_image,
                w, h);
  float inner = (float)get_sum_disparities(MAX(x - ri, 0), MAX(y - ri, 0), MIN(x + ri, w), MIN(y + ri, h), integral_image,
                w, h);

  if (inner > ri * ri && outer > inner && ri >= min_gate_size && ro > ri) {
    //score = ro * inner / (ri * ri *outer);
    score = inner * (ro * ro - ri * ri) / ((outer - inner) * ri * ri);
  }

  return score;
}

float get_angle_from_polygon(float s_left, float s_right, struct image_t *color_image)
{
  // The sizes of the sides in sight gives the angle to the gate, as their sizes in the real world
  // are known, and equal to GATE_SIZE. This of course assuming a pinhole camera model.
  // So we have a triangle with three known lengths and can determine the angle to the center of the
  // gate.
  float psi_estimate = 0;

  // since the sides of the square gate are straight up, we only need FOV_H to determine the
  // distance of the camera to the sides.
  float gamma_left = (s_left / color_image->w) * FOVY;
  float d_left = (0.5 * s_left) / tanf(0.5 * gamma_left);
  float gamma_right = (s_right / color_image->w) * FOVY;
  float d_right = (0.5 * s_right) / tanf(0.5 * gamma_right);

  // angles seen from a top view, with the GATE_SIZE gate on top
  float angle_right = acosf((GATE_SIZE * GATE_SIZE + d_right * d_right - d_left * d_left) /
                            (2 * GATE_SIZE * d_right)); // angle at the right pole
  float angle_left = acosf((GATE_SIZE * GATE_SIZE + d_left * d_left - d_right * d_right) /
                           (2 * GATE_SIZE * d_left)); // angle at the left pole

  // determine psi:
  if (angle_right > angle_left) {
    psi_estimate = (angle_right - angle_left) / 2;
  } else {
    psi_estimate = -(angle_left - angle_right) / 2;
  }
  return psi_estimate;
}

float get_outlier_ratio(int16_t *genome, float total_sum_weights)
{
  float x = genome[0];
  float y = genome[1];
  float r = genome[2];

  float outlier_ratio = 0.0f;
  struct point_f point;
  float dx, dy;
  float dist_center, error;
  uint16_t p;
  for (p = 0; p < n_points; p++) {
    point.x = points[p].x; point.y = points[p].y;
    dx = point.x - x;
    dy = point.y - y;
    arm_sqrt_f32(dx * dx + dy * dy, &dist_center);
    error = fabs(dist_center - r);

#ifdef STICK
    float error_stick;
    // determine distance to the stick:
    struct point_f stick1;
    stick1.x = x;
    stick1.y = y + r;
    struct point_f stick2;
    stick2.x = x;
    stick2.y = y + 2 * r;
    error_stick = distance_to_vertical_segment(stick1, stick2, point);

    // take the smallest error:
    if (error_stick < error) { error = error_stick; }
#endif

    // if outlier, augment outlier_ratio:
    if (error > outlier_threshold) {
      if (WEIGHTED) {
        outlier_ratio += weights[p];
      } else {
        outlier_ratio += 1.0f;
      }
    }
  }
  outlier_ratio /= total_sum_weights;
  return outlier_ratio;
}


float distance_to_line(struct point_f Q1, struct point_f Q2, struct point_f P)
{
  // see e.g., http://mathworld.wolfram.com/Point-LineDistance2-Dimensional.html
  float norm_Q2_Q1;
  arm_sqrt_f32((Q1.x - Q2.x) * (Q1.x - Q2.x) + (Q1.y - Q2.y) * (Q1.y - Q2.y), &norm_Q2_Q1);
  float det = (Q2.x - Q1.x) * (P.y - Q1.y) - (Q2.y - Q1.y) * (P.x - Q1.x);
  float dist_line = fabs(det) / norm_Q2_Q1;
  return dist_line;
}

float distance_to_segment(struct point_f Q1, struct point_f Q2, struct point_f P)
{
  float dist_line = distance_to_line(Q1, Q2, P);

  // calculate intersection point:
  float rx = Q2.y - Q1.y; // always negative, -r
  float ry = -(Q2.x - Q1.x);
  float norm_r;
  arm_sqrt_f32(rx * rx + ry * ry, &norm_r);
  rx = (rx / norm_r) * dist_line;
  ry = (ry / norm_r) * dist_line;

  // rx < 0, so:
  // if P.x > Q1.x, it should be P.x + rx
  // else it should be P.x - rx
  float i_x;
  float i_y;
  if (P.x > Q1.x) {
    i_x = P.x + rx;
    i_y = P.y + ry;
  } else {
    i_x = P.x - rx;
    i_y = P.y - ry;
  }
  struct point_f I;
  I.x = i_x;
  I.y = i_y;

  /*
  Slow code:
  float dI = distance_to_line(Q1, Q2, I);
  if (dI > 1e-10)
  {
    I.x = P.x - rx;
    I.y = P.y - ry;
  }
  */

  /*
  // Slow code:
  // check if it is on the segment:
  float d1, d2, d_12;
  arm_sqrt_f32((Q1.x-I.x)*(Q1.x - I.x) + (Q1.y - I.y)*(Q1.y - I.y), &d1);
  arm_sqrt_f32((Q2.x - I.x)*(Q2.x - I.x) + (Q2.y - I.y)*(Q2.y - I.y), &d2);
  arm_sqrt_f32((Q2.x - Q1.x)*(Q2.x - Q1.x) + (Q2.y - Q1.y)*(Q2.y - Q1.y), &d_12);
  if (d1 > d_12 || d2 > d_12)
  {
    // not on segment, determine minimum distance to one of the two extremities:
    arm_sqrt_f32((Q1.x - P.x)*(Q1.x - P.x) + (Q1.y - P.y)*(Q1.y - P.y), &dist_line);
    arm_sqrt_f32((Q2.x - P.x)*(Q2.x - P.x) + (Q2.y - P.y)*(Q2.y - P.y), &d2);
    if (d2 < dist_line) dist_line = d2;
  }
  */

  // leave out superfluous sqrtf - for comparisons it does not matter (monotonously increasing functions)
  // we can still precalculate (Q1.x - I.x) etc. but I don't know if it is actually calculated twice (optimized by compiler?)
  float d1 = (Q1.x - I.x) * (Q1.x - I.x) + (Q1.y - I.y) * (Q1.y - I.y);
  float d2 = (Q2.x - I.x) * (Q2.x - I.x) + (Q2.y - I.y) * (Q2.y - I.y);
  float d_12 = (Q2.x - Q1.x) * (Q2.x - Q1.x) + (Q2.y - Q1.y) * (Q2.y - Q1.y);
  if (d1 > d_12 || d2 > d_12) {
    // not on segment, determine minimum distance to one of the two extremities:
    dist_line = (Q1.x - P.x) * (Q1.x - P.x) + (Q1.y - P.y) * (Q1.y - P.y);
    d2 = (Q2.x - P.x) * (Q2.x - P.x) + (Q2.y - P.y) * (Q2.y - P.y);
    if (d2 < dist_line) { arm_sqrt_f32(d2, &dist_line); }
  }

  return dist_line;
}

float distance_to_vertical_segment(struct point_f Q1, struct point_f Q2, struct point_f P)
{
  // Q1.y should be smaller than Q2.y, y positive down
  // so first top then bottom

  // Calculating the distance to a vertical segment is actually quite simple:
  // If the y coordinate of P is in between Q1.y and Q2.y, the shortest distance is orthogonal to the line
  // If P.y < Q1.y (which is < Q2.y), then the distance to Q1 should be taken
  // If P.y > Q2.y, then the distance to Q2 should be taken:
  float dist_line;

  if (P.y < Q1.y) {
    arm_sqrt_f32((Q1.x - P.x) * (Q1.x - P.x) + (Q1.y - P.y) * (Q1.y - P.y), &dist_line);
  } else if (P.y <= Q2.y) {
    dist_line = fabs(P.x - Q1.x); // straight line to the vertical line segment
  } else {
    arm_sqrt_f32((Q2.x - P.x) * (Q2.x - P.x) + (Q2.y - P.y) * (Q2.y - P.y), &dist_line);
  }

  return dist_line;
}

float distance_to_horizontal_segment(struct point_f Q1, struct point_f Q2, struct point_f P)
{
  // Q1.x should be smaller than Q2.x

  // Calculating the distance to a horizontal segment is actually quite simple:
  // If the x coordinate of P is in between Q1.x and Q2.x, the shortest distance is orthogonal to the line
  // If P.x < Q1.x (which is < Q2.x), then the distance to Q1 should be taken
  // If P.x > Q2.x, then the distance to Q2 should be taken:
  float dist_line;

  if (P.x > Q2.x) {
    arm_sqrt_f32((Q2.x - P.x) * (Q2.x - P.x) + (Q2.y - P.y) * (Q2.y - P.y), &dist_line);
  } else if (P.x >= Q1.x) {
    dist_line = fabs(P.y - Q1.y); // straight line to the horizontal line segment
  } else {
    arm_sqrt_f32((Q1.x - P.x) * (Q1.x - P.x) + (Q1.y - P.y) * (Q1.y - P.y), &dist_line);
  }

  return dist_line;
}

void draw_stick(struct image_i *Im, float x_center, float y_center, float radius, uint8_t *color)
{
  int x, y;
  x = (int) x_center;
  for (y = (int)(y_center + radius); y < (int)(y_center + 2 * radius); y++) {
    if (x >= 0 && x < Im->w && y >= 0 && y < Im->h) {
      Im->image[y * Im->w + x] = color[0];
    }
  }
}
