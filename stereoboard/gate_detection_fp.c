/*
 * gate_detection.c
 *
 *  Created on: Sep 10, 2016
 *      Author: Guido de Croon
 */

#include "gate_detection_fp.h"
#include <math.h>
#include "main_parameters.h"
#include <stdlib.h>
#include "stm32f4xx_conf.h"


// variables that have to be remembered in between function calls:

// since MAX_POINTS_FP means that the algorithm will stop gathering points after MAX_POINTS_FP, we sample according to a "moving" grid
// these starting points are made for a grid with step size 3
#define GRID_STEP_FP 3
int Y0_FP[9] = {0,1,2,0,1,2,0,1,2};
int X0_FP[9] = {0,0,0,1,1,1,2,2,2};

q15_t points_x[MAX_POINTS_FP];
q15_t points_y[MAX_POINTS_FP];
uint32_t n_points_fp;

// Settings for the evolution:
#define N_INDIVIDUALS_FP 10
#define N_GENES_FP 3
uint16_t n_generations_fp = 30; // could be reduced for instance when there are many points
q15_t Population_fp[N_INDIVIDUALS_FP][N_GENES_FP];
// mutations will give a number in the range [-MUTATION_RANGE, MUTATION_RANGE]
#define MUTATION_RANGE 3

// Settings for the fitting:
q15_t weights_fp[MAX_POINTS_FP];
q15_t total_sum_weights;
int min_points_fp = 5;
int WEIGHTED_FP = 1; 
#define STICK_FP  0

#define CIRCLE_FP 0
#define SQUARE_FP 1
int SHAPE_FP = CIRCLE_FP;
// Now a parameter:
// int min_disparity = 2;
int outlier_threshold_fp = 20; 

// whether to draw on the disparity image:
int GATE_GRAPHICS = 1;

/**
 * Function takes a disparity image and fits a circular gate to the close-by points.
 * - initialize_fit_with_pars will use the x_center, y_center, etc. to initialize the population used in the evolutionary algorithm to make the fit.
 * - The results are put back in the parameters. 
 * @author Guido
 */

void gate_detection_fp(struct image_i* disparity_image, q15_t* x_center, q15_t* y_center, q15_t* radius, q15_t* fitness, int initialize_fit_with_pars, int min_sub_disparity)
{
  // 1) convert the disparity map to a vector of points:
	convert_disparitymap_to_points_fp(disparity_image, min_sub_disparity);

  // if there are enough points close by:
	if (n_points_fp > min_points_fp)
	{
		// 2) fit a window to the points

		// determine initial guess:
		if (!initialize_fit_with_pars)
		{
      // determine the mean x and y coordinate:
      arm_mean_q15(points_x, n_points_fp, x_center);
      arm_mean_q15(points_y, n_points_fp, y_center); 
      
      // we use the std dev of the points in x direction as initialization:
      // could be done better:

      /*
      // does not work:      
      q15_t std_x, std_y;
      arm_std_q15(points_x, n_points_fp, &std_x);
      arm_std_q15(points_y, n_points_fp, &std_y);
      (*radius) = (std_x+std_y) / 2;
      */
      (*radius) = 40;

  		// run the fit procedure:
  		fit_window_to_points_fp(x_center, y_center, radius, fitness);
		}
		else
		{
  		// run the fit procedure:
      fit_window_to_points_fp(x_center, y_center, radius, fitness);
		}

    if(GATE_GRAPHICS)
    {
      // draw a circle on the disparity image:
      uint8_t color[1];
      color[0] = 255; // should be 128 for SmartUAV!!!
		  draw_circle_fp(disparity_image, (*x_center), (*y_center), (*radius), color);
      if(STICK_FP)
        draw_stick_fp(disparity_image, (*x_center), (*y_center), (*radius), color);
    }
	}
	else
	{
		(*fitness) = BAD_FIT_FP;
	}

}

void convert_disparitymap_to_points_fp(struct image_i* disparity_image, int min_sub_disparity)
{
  q15_t y, x, sp;
	uint8_t disp;
	uint16_t p = 0;
  
  // We stop sampling at MAX_POINTS_FP, but do not want our samples to be biased toward a certain
  // part of the image. 
  // We have different grid starting points (GRID_STEP_FP*GRID_STEP_FP) so that for every different 
  // starting point sp, we will sample different positions in the image, finally covering the
  // whole image.
  for(sp = 0; sp < GRID_STEP_FP*GRID_STEP_FP; sp++)
  {
	  for (y = Y0_FP[sp]; y < (*disparity_image).h; y+=GRID_STEP_FP)
	  {
		  for (x = X0_FP[sp]; x < (*disparity_image).w; x+=GRID_STEP_FP)
		  {
        // get the disparity from the image:
			  disp = (*disparity_image).image[y*(*disparity_image).w + x];

			  if (disp > min_sub_disparity)
			  {
          // add the points to the array, and use disparity as the weight:
				  points_x[p] = x;
				  points_y[p] = y;
				  weights_fp[p] = (q15_t) disp;

          // count the number of points:
          p++;

          // if the maximum number of points is reached, return:
          if(p == MAX_POINTS_FP)
          {
            n_points_fp = p;
            return;
          }
			  }
        else
        {
          // make the pixel black, so that we can see it on the ground station:
          disparity_image->image[y * disparity_image->w + x] = 0;
        }
		  }
	  }
  }

  // set the global variable n_points_fp to the right value:
  n_points_fp = p;
  // calculate the total sum of weights_fp: (don't seem to be able to find a "sum" function):
  arm_mean_q15(weights_fp, n_points_fp, &total_sum_weights);
  total_sum_weights *= n_points_fp;
}


void fit_window_to_points_fp(q15_t* x0, q15_t* y0, q15_t* size0, q15_t* fitness)
{
  // a) initialize Population_fp, seeding it with the initial guess:
  uint16_t i, g;
	for (i = 0; i < N_INDIVIDUALS_FP; i++)
	{
    Population_fp[i][0] = (*x0) + get_mutation_fp();
    Population_fp[i][1] = (*y0) + get_mutation_fp();
		Population_fp[i][2] = (*size0) + get_mutation_fp();
	}

	// large number, since we will minimize it:
	(*fitness) = 30000;
	q15_t fits[N_INDIVIDUALS_FP];
	q15_t best_genome[N_GENES_FP]; // over the whole evolution
  uint32_t index;
  q15_t min_fit; // for a single generation
  q15_t min_genome[N_GENES_FP]; // for a single generation

  // b) perform the evolution over n_generations_fp generations
	for (g = 0; g < n_generations_fp; g++)
	{
		for (i = 0; i < N_INDIVIDUALS_FP; i++)
		{
			if (SHAPE_FP == CIRCLE_FP)
			{
          // optimize mean distance to circle (and possibly stick) 
				  fits[i] = mean_distance_to_circle_fp(Population_fp[i]);
			}
		}

		// get the best individual and store it in min_genome:
    arm_min_q15(fits, (uint32_t) N_INDIVIDUALS_FP, &min_fit, &index);
    arm_copy_q15(Population_fp[index], min_genome, (uint32_t) N_GENES_FP);

		// if better than any previous individual, remember it:
		if (min_fit < (*fitness))
		{
      arm_copy_q15(min_genome, best_genome, (uint32_t) N_GENES_FP);
			(*fitness) = min_fit;
		}

		// fill the new population with mutated copies of this generation's best:
		if (g < n_generations_fp - 1)
		{
			// super elitist evolution:
			for (i = 0; i < N_INDIVIDUALS_FP; i++)
			{
				Population_fp[i][0] = min_genome[0] + get_mutation_fp();
				Population_fp[i][1] = min_genome[1] + get_mutation_fp();
				Population_fp[i][2] = min_genome[2] + get_mutation_fp();
        // size should not become too small
        Population_fp[i][2] = (Population_fp[i][2] < 10) ? 10 : Population_fp[i][2];
			}
		}
	}

  // put the final values back in the parameters:
  (*fitness) /= (total_sum_weights / FITNESS_RESOLUTION);
	(*x0) = best_genome[0];
	(*y0) = best_genome[1];
	(*size0) = best_genome[2];

  return;
}

q15_t get_mutation_fp()
{
  // hopefully (MUTATION_RANGE*2+1) is optimized at compilation time:
	q15_t rand_num = (q15_t) (rand() % (MUTATION_RANGE*2+1)) - MUTATION_RANGE;
	return rand_num;
}


q15_t mean_distance_to_circle_fp(q15_t* genome)
{
  uint16_t p;
	q31_t mean_distance = 0;
  q15_t dist_center, error, error_stick;

  // x = genome[0], y = genome[1], r = genome[2]
	q15_t xs[MAX_POINTS_FP];
	q15_t ys[MAX_POINTS_FP];
	q15_t r = genome[2];

	struct point_i stick1;
	struct point_i stick2;
  if(STICK_FP)
  {
    stick1.x = genome[0];
    stick1.y = genome[1] + r;
    stick2.x = genome[0];
    stick2.y = genome[1] + 2*r;
  }

  // calculate distances to circle in a vector manner:
  arm_fill_q15(genome[0], xs, n_points_fp);
  arm_fill_q15(genome[1], ys, n_points_fp);
  // xs and ys will contain dxs, dys:
  arm_sub_q15(xs, points_x, xs, n_points_fp);
  arm_sub_q15(ys, points_y, ys, n_points_fp);
  // and now the squares, dx*dx and dy*dy:
  // the arm one did not work... for some reason.
  multiply(xs, xs, xs, n_points_fp);
  multiply(ys, ys, ys, n_points_fp);
  // add them and put it in xs:
  arm_add_q15(xs, ys, xs, n_points_fp);

  for (p = 0; p < n_points_fp; p++)
	{
    // TODO: we could also leave out the sqrt, since for relative fitness it would not matter:
    // then r should become r^2
    // the arm one does not work... for some reason.
    // arm_sqrt_q15(xs[p], &dist_center);
    dist_center = sqrt((int) xs[p]);
    error = abs(dist_center - r);
    
		if (STICK_FP)
		{
			// determine distance to the stick:
			error_stick = distance_to_vertical_segment_fp(stick1, stick2, genome[0], genome[1]);

			// take the smallest error:
			if (error_stick < error) error = error_stick;
		}

		// apply outlier threshold before applying weights_fp:
		if (error > outlier_threshold_fp) error = outlier_threshold_fp;

		if (WEIGHTED_FP)
		{
			mean_distance += error * weights_fp[p];
		}
		else
		{
			mean_distance += error;
		}
    
  }
	mean_distance /= n_points_fp;
	return mean_distance;
}

q15_t mean_distance_to_square_fp(q15_t* genome)
{
  uint16_t p;
  uint32_t index;
	q15_t mean_distance = 0;
  q15_t error, error_stick;

  // x = genome[0], y = genome[1], r = genome[2]
	q15_t x = genome[0];
	q15_t y = genome[1];
	q15_t r = genome[2];

  // determine points for the stick:
	struct point_i stick1;
	struct point_i stick2;
  if(STICK_FP)
  {
    stick1.x = genome[0];
    stick1.y = genome[1] - r;
    stick2.x = genome[0];
    stick2.y = genome[1] - 2*r;
  }

   // determine corner points:
  struct point_i square_top_left;
  struct point_i square_top_right;
  struct point_i square_bottom_right;
  struct point_i square_bottom_left;
  square_top_left.x = x-r;
  square_top_left.y = y+r; // positive y direction is up TODO: it is down!!!
  square_top_right.x = x+r;
  square_top_right.y = y+r; // positive y direction is up
  square_bottom_left.x = x-r;
  square_bottom_left.y = y-r; // positive y direction is up
  square_bottom_right.x = x+r;
  square_bottom_right.y = y-r; // positive y direction is up
  q15_t side_distances[4];

  for (p = 0; p < n_points_fp; p++)
	{
    // determine the distance to the four sides of the square and select the smallest one:
    side_distances[0] = distance_to_vertical_segment_fp(square_top_left, square_bottom_left, x, y);
    side_distances[1] = distance_to_vertical_segment_fp(square_top_right, square_bottom_right, x, y);
    side_distances[2] = distance_to_horizontal_segment_fp(square_top_left, square_top_right, x, y);
    side_distances[3] = distance_to_horizontal_segment_fp(square_bottom_left, square_bottom_right, x, y);
    arm_min_q15(side_distances, (uint32_t) 4, &error, &index);

		if (STICK_FP)
		{
			// determine distance to the stick:
			error_stick = distance_to_vertical_segment_fp(stick1, stick2, genome[0], genome[1]);

			// take the smallest error:
			if (error_stick < error) error = error_stick;
		}

		// apply outlier threshold before applying weights_fp:
		if (error > outlier_threshold_fp) error = outlier_threshold_fp;

		if (WEIGHTED_FP)
		{
			mean_distance += error * weights_fp[p];
		}
		else
		{
			mean_distance += error;
		}
    
  }
	mean_distance /= n_points_fp;
	return mean_distance;
}

q15_t distance_to_vertical_segment_fp(struct point_i Q1, struct point_i Q2, q15_t x, q15_t y)
{
  // Q1.y should be smaller than Q2.y

  // Calculating the distance to a vertical segment is actually quite simple:
  // If the y coordinate of P is in between Q1.y and Q2.y, the shortest distance is orthogonal to the line
  // If P.y < Q1.y (which is < Q2.y), then the distance to Q1 should be taken
  // If P.y > Q2.y, then the distance to Q2 should be taken:
  q15_t dist_line;

  if(y < Q1.y)
  {
    // arm_sqrt_q15((Q1.x - x)*(Q1.x - x) + (Q1.y - y)*(Q1.y - y), &dist_line);
    dist_line = sqrt((int)((Q1.x - x)*(Q1.x - x) + (Q1.y - y)*(Q1.y - y)));
  }
  else if(y <= Q2.y)
  {
    dist_line = abs(x - Q1.x); // straight line to the vertical line segment
  }
  else
  {
    // arm_sqrt_q15((Q2.x - x)*(Q2.x - x) + (Q2.y - y)*(Q2.y - y), &dist_line);    
    dist_line = sqrt((int)((Q2.x - x)*(Q2.x - x) + (Q2.y - y)*(Q2.y - y)));
  }

  return dist_line;
}

q15_t distance_to_horizontal_segment_fp(struct point_i Q1, struct point_i Q2, q15_t x, q15_t y)
{
  // Q1.x should be smaller than Q2.x

  // Calculating the distance to a horizontal segment is actually quite simple:
  // If the x coordinate of P is in between Q1.x and Q2.x, the shortest distance is orthogonal to the line
  // If P.x < Q1.x (which is < Q2.y), then the distance to Q1 should be taken
  // If P.x > Q2.x, then the distance to Q2 should be taken:
  q15_t dist_line;

  if(x > Q2.x)
  {
    // arm_sqrt_q15((Q2.x - x)*(Q2.x - x) + (Q2.y - y)*(Q2.y - y), &dist_line);
    dist_line = sqrt((int)((Q2.x - x)*(Q2.x - x) + (Q2.y - y)*(Q2.y - y)));
  }
  else if(x >= Q1.x)
  {
    dist_line = abs(y - Q1.y); // straight line to the vertical line segment
  }
  else
  {
    // arm_sqrt_q15((Q1.x - x)*(Q1.x - x) + (Q1.y - y)*(Q1.y - y), &dist_line);
    dist_line = sqrt((int)((Q1.x - x)*(Q1.x - x) + (Q1.y - y)*(Q1.y - y)));
  }

  return dist_line;

}

void draw_circle_fp(struct image_i* Im, q15_t x_center, q15_t y_center, q15_t radius, uint8_t* color)
{
  float t_step = 0.05; // should depend on radius, but hey...
	int x, y;
  float t;
	for (t = 0.0f; t < (float)(2 * PI); t += t_step)
	{
		x = (int)x_center + (int)(cosf(t)*radius);
		y = (int)y_center + (int)(sinf(t)*radius);
		if (x >= 0 && x < Im->w && y >= 0 && y < Im->h)
		{
      Im->image[y*Im->w+x] = color[0];
		}
	}
  return;
}

void draw_stick_fp(struct image_i* Im, q15_t x_center, q15_t y_center, q15_t radius, uint8_t* color)
{
  int x, y;
  x = (int) x_center;
  for(y = (int)(y_center + radius); y <  (int)(y_center + 2*radius); y++)
  {
    if (x >= 0 && x < Im->w && y >= 0 && y < Im->h)
		{
      Im->image[y*Im->w+x] = color[0];
		} 
  }
}

void multiply(q15_t* a, q15_t* b, q15_t* result, uint32_t n_elements)
{
  uint32_t i = 0;
  while(i < n_elements)
  {
    // no checking of overflow here...
    result[i] = a[i] * b[i];
    i++;
  }
}
