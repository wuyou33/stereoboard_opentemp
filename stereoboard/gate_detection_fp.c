/*
 * gate_detection.c
 *
 *  Created on: Sep 10, 2016
 *      Author: Guido de Croon
 */

#include "gate_detection.h"
#include <math.h>
// one would expect this to be part of math.h, but...
#define PI 3.14159265359
#include "main_parameters.h"
#include <stdlib.h>

// variables that have to be remembered in between function calls:

// since MAX_POINTS means that the algorithm will stop gathering points after MAX_POINTS, we sample according to a "moving" grid
// these starting points are made for a grid with step size 3
#define GRID_STEP 3
int Y0[9] = {0,1,2,0,1,2,0,1,2};
int X0[9] = {0,0,0,1,1,1,2,2,2};

q15_t points_x[MAX_POINTS];
q15_t points_y[MAX_POINTS];
uint32_t n_points;

// Settings for the evolution:
#define N_INDIVIDUALS 30
#define N_GENES 3
uint16_t n_generations = 10; // could be reduced for instance when there are many points
q15_t Population[N_INDIVIDUALS][N_GENES];
// mutations will give a number in the range [-MUTATION_RANGE, MUTATION_RANGE]
#define MUTATION_RANGE 3

// normally the fitness was between 0 an 1, more concentrated on low values 
// now it will be between 0 and FITNESS_RESOLUTION
#define FITNESS_RESOLUTION 100
// Settings for the fitting:
q15_t weights[MAX_POINTS];
q15_t total_sum_weights;
int min_points = 5;
int WEIGHTED = 1; 
int STICK = 0;
#define CIRCLE 0
#define SQUARE 1
int SHAPE = CIRCLE;
// Now a parameter:
// int min_disparity = 2;
int outlier_threshold = 400; // 20^2

// whether to draw on the disparity image:
int GRAPHICS = 1;

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
	if (n_points > min_points)
	{
		// 2) fit a window to the points

		// determine initial guess:
		if (!initialize_fit_with_pars)
		{
      // determine the mean x and y coordinate:
      arm_mean_q15(points_x, n_points, &x_center);
      arm_mean_q15(points_y, n_points, &y_center); 
      arm_sqrt_q15 (mean_x*mean_x + mean_y*mean_y, radius);

  		// run the fit procedure:
  		fit_window_to_points_fp(x_center, y_center, radius, fitness);
		}
		else
		{
  		// run the fit procedure:
      fit_window_to_points_fp(x_center, y_center, radius, fitness);
		}

    if(GRAPHICS)
    {
      // draw a circle on the disparity image:
      uint8_t color[1];
      color[0] = 128;
		  draw_circle_fp(disparity_image, (*x_center), (*y_center), (*radius), color);
      if(STICK)
        draw_stick_fp(disparity_image, (*x_center), (*y_center), (*radius), color);
    }
	}
	else
	{
		(*fitness) = BAD_FIT;
	}

}

void convert_disparitymap_to_points_fp(struct image_i* disparity_image, int min_sub_disparity)
{
  q15_t y, x, sp;
	uint8_t disp;
	uint16_t p = 0;
  
  // We stop sampling at MAX_POINTS, but do not want our samples to be biased toward a certain
  // part of the image. 
  // We have different grid starting points (GRID_STEP*GRID_STEP) so that for every different 
  // starting point sp, we will sample different positions in the image, finally covering the
  // whole image.
  for(sp = 0; sp < GRID_STEP*GRID_STEP; sp++)
  {
	  for (y = Y0[sp]; y < (*disparity_image).h; y+=GRID_STEP)
	  {
		  for (x = X0[sp]; x < (*disparity_image).w; x+=GRID_STEP)
		  {
        // get the disparity from the image:
			  disp = (*disparity_image).image[y*(*disparity_image).w + x];

			  if (disp > min_sub_disparity)
			  {
          // add the points to the array, and use disparity as the weight:
				  points_x[p] = x;
				  points_y[p] = y;
				  weights[p] = (q15_t) disp;

          // count the number of points:
          p++;

          // if the maximum number of points is reached, return:
          if(p == MAX_POINTS)
          {
            n_points = p;
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

  // set the global variable n_points to the right value:
  n_points = p;
  // calculate the total sum of weights: (don't seem to be able to find a "sum" function):
  arm_mean_q15(weights, n_points, &total_sum_weights);
  total_sum_weights *= n_points;
}


void fit_window_to_points_fp(q15_t* x0, q15_t* y0, q15_t* size0, q15_t* fitness)
{
  // a) initialize Population, seeding it with the initial guess:
  uint16_t i, g, ge;
	for (i = 0; i < N_INDIVIDUALS; i++)
	{
    Population[i][0] = (*x0) + get_mutation_fp();
    Population[i][1] = (*y0) + get_mutation_fp();
		Population[i][2] = (*size0) + get_mutation_fp();
	}

	// large number, since we will minimize it:
	(*fitness) = 1000000;
	q15_t fits[N_INDIVIDUALS];
	q15_t best_genome[N_GENES]; // over the whole evolution
  q15_t index, min_fit; // for a single generation
  q15_t min_genome[N_GENES]; // for a single generation

  // b) perform the evolution over n_generations generations
	for (g = 0; g < n_generations; g++)
	{
		for (i = 0; i < N_INDIVIDUALS; i++)
		{
			if (SHAPE == CIRCLE)
			{
          // optimize mean distance to circle (and possibly stick) 
				  fits[i] = mean_distance_to_circle_fp(Population[i]);
			}
		}

		// get the best individual and store it in min_genome:
    arm_min_q15 (fits, N_INDIVIDUALS, &min_fit, &index);
    arm_copy_q15 (Population[index], min_genome, N_GENES);

		// if better than any previous individual, remember it:
		if (min_fit < (*fitness))
		{
      arm_copy(min_genome, best_genome, N_GENES);
			(*fitness) = min_fit;
		}

		// fill the new population with mutated copies of this generation's best:
		if (g < n_generations - 1)
		{
			// super elitist evolution:
			for (i = 0; i < N_INDIVIDUALS; i++)
			{
				Population[i][0] = min_genome[0] + get_mutation_fp();
				Population[i][1] = min_genome[1] + get_mutation_fp();
				Population[i][2] = min_genome[2] + get_mutation_fp();
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
	q15_t mean_distance = 0;
  q15_t dist_center, error, error_stick;

  // x = genome[0], y = genome[1], r = genome[2]
	q15_t xs[MAX_POINTS];
	q15_t ys[MAX_POINTS];
	q15_t r = genome[2];

	struct point_i stick1;
	struct point_i stick2;
  if(STICK)
  {
    stick1.x = genome[0];
    stick1.y = genome[1] - r;
    stick2.x = genome[0];
    stick2.y = genome[1] - 2*r;
  }

  // calculate distances to circle in a vector manner:
  arm_fill_q15 (genome[0], xs, n_points);
  arm_fill_q15 (genome[1], ys, n_points);
  // xs and ys will contain dxs, dys:
  arm_sub_q15(xs, points_x, xs, n_points);
  arm_sub_q15(ys, points_y, ys, n_points);
  // and now the squares, dx*dx and dy*dy:
  arm_mult_q15(xs, xs, xs, n_points);
  arm_mult_q15(ys, ys, ys, n_points);
  // add them and put it in xs:
  arm_add_q15 (xs, ys, xs, n_points);

  for (p = 0; p < n_points; p++)
	{
    // TODO: we could also leave out the sqrt, since for relative fitness it would not matter:
    // then r should become r^2
    arm_sqrt_q15(dist_center, &dist_center);
    error = abs(dist_center - r);
    
		if (STICK)
		{
			// determine distance to the stick:
			error_stick = distance_to_vertical_segment_fp(stick1, stick2, genome[0], genome[1]);

			// take the smallest error:
			if (error_stick < error) error = error_stick;
		}

		// apply outlier threshold before applying weights:
		if (error > outlier_threshold) error = outlier_threshold;

		if (WEIGHTED)
		{
			mean_distance += error * weights[p];
		}
		else
		{
			mean_distance += error;
		}
    
  }
	mean_distance /= n_points;
	return mean_distance;
}

q15_t distance_to_vertical_segment_fp(struct point_i Q1, struct point_i Q2, q15_t x, q15_t y)
{
  // Calculating the distance to a vertical segment is actually quite simple:
  // If the y coordinate of P is in between Q1.y and Q2.y, the shortest distance is orthogonal to the line
  // If P.y > Q1.y (which is > Q2.y), then the distance to Q1 should be taken
  // If P.y < Q2.y, then the distance to Q2 should be taken:
  q15_t dist_line;

  if(P.y > Q1.y)
  {
    arm_sqrt_q15((Q1.x - x)*(Q1.x - x) + (Q1.y - y)*(Q1.y - y), &dist_line);
  }
  else if(P.y >= Q2.y)
  {
    dist_line = abs(x - Q1.x); // straight line to the vertical line segment
  }
  else
  {
    arm_sqrt_q15((Q2.x - x)*(Q2.x - x) + (Q2.y - y)*(Q2.y - y), &dist_line);
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
  for(y = (int)(y_center - 2*radius); y <  (int)(y_center - radius); y++)
  {
    if (x >= 0 && x < Im->w && y >= 0 && y < Im->h)
		{
      Im->image[y*Im->w+x] = color[0];
		} 
  }
}
