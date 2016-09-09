/*
 * gate_detection.c
 *
 *  Created on: Sep 5, 2016
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

/*
// since MAX_POINTS means that the algorithm will stop gathering points after MAX_POINTS, the order of columns / rows has to be "random":
int ys[96] = {87, 83, 57, 50, 76, 84, 81, 73, 10, 33, 86, 3, 28, 49, 90, 25, 6, 16, 75, 89, 66, 85, 64, 30, 17, 60, 95, 92, 26, 20, 80, 40, 23, 70, 27, 4, 36, 43, 38, 65, 78, 51, 62, 96, 15, 29, 52, 94, 54, 45, 58, 72, 5, 71, 14, 44, 47, 41, 42, 79, 34, 74, 56, 69, 37, 93, 31, 63, 59, 88, 9, 55, 46, 11, 1, 35, 77, 32, 82, 18, 22, 7, 39, 48, 61, 68, 19, 67, 53, 91, 12, 2, 24, 13, 8, 21};
int xs[128] = {49, 7, 4, 82, 53, 39, 118, 68, 55, 67, 106, 77, 116, 117, 127, 99, 103, 61, 74, 22, 48, 113, 31, 89, 2, 108, 112, 85, 6, 110, 62, 16, 95, 120, 30, 83, 21, 125, 71, 56, 14, 8, 90, 91, 40, 66, 51, 52, 17, 114, 88, 105, 33, 18, 5, 102, 38, 122, 107, 41, 44, 124, 79, 59, 29, 100, 27, 26, 69, 24, 94, 46, 76, 10, 75, 63, 81, 47, 54, 96, 87, 119, 123, 73, 128, 1, 45, 9, 28, 58, 42, 72, 36, 86, 97, 12, 121, 92, 64, 11, 126, 13, 50, 98, 32, 80, 43, 34, 15, 57, 84, 78, 20, 19, 109, 35, 70, 3, 111, 65, 23, 37, 93, 101, 115, 60, 104, 25};
*/
struct point_f points[MAX_POINTS];
uint16_t n_points;

// Settings for the evolution:
#define N_INDIVIDUALS 10
#define N_GENES 3
uint16_t n_generations = 30; // could be reduced for instance when there are many points
float Population[N_INDIVIDUALS][N_GENES];

// watch out: inliers fit does not work so well...
#define DISTANCE_FIT 0
#define INLIERS_FIT 1
#define FF DISTANCE_FIT

// Settings for the fitting:
float weights[MAX_POINTS];
int min_points = 5;
int WEIGHTED = 1; 
int STICK = 1;
#define CIRCLE 0
#define SQUARE 1
int SHAPE = CIRCLE;
// Now a parameter:
// int min_disparity = 2;
float outlier_threshold = 20.0f;


// whether to draw on the disparity image:
int GRAPHICS = 1;

/**
 * Function takes a disparity image and fits a circular gate to the close-by points.
 * - initialize_fit_with_pars will use the x_center, y_center, etc. to initialize the population used in the evolutionary algorithm to make the fit.
 * - The results are put back in the parameters. 
 * @author Guido
 */

void gate_detection(struct image_i* disparity_image, float* x_center, float* y_center, float* radius, float* fitness, int initialize_fit_with_pars, int min_sub_disparity)
{
  // 1) convert the disparity map to a vector of points:
	convert_disparitymap_to_points(disparity_image, min_sub_disparity);

  // if there are enough points close by:
	if (n_points > min_points)
	{
		// 2) fit a window to the points

		// determine initial guess:
		if (!initialize_fit_with_pars)
		{
  		float x0, y0, size0;
			// initialize with average position and size
			float mean_x = 0;
			float mean_y = 0;
      uint16_t i;
			for (i = 0; i < n_points; i++)
			{
				mean_x += points[i].x;
				mean_y += points[i].y;
			}
			mean_x /= n_points;
			mean_y /= n_points;
			x0 = mean_x;
			y0 = mean_y;
			size0 = sqrtf(mean_x*mean_x + mean_y*mean_y);

  		// run the fit procedure:
  		fit_window_to_points(&x0, &y0, &size0, fitness);
      (*x_center) = x0;
      (*y_center) = y0;
      (*radius) = size0;
		}
		else
		{
  		// run the fit procedure:
      fit_window_to_points(x_center, y_center, radius, fitness);
		}

    if(GRAPHICS)
    {
      // draw a circle on the disparity image:
      uint8_t color[1];
      color[0] = 255;
		  draw_circle(disparity_image, (*x_center), (*y_center), (*radius), color);
    }
	}
	else
	{
		(*fitness) = BAD_FIT;
	}

}

void fit_window_to_points(float* x0, float* y0, float* size0, float* fitness)
{
  // a) initialize Population, seeding it with the initial guess:
  uint16_t i, g, ge;
	for (i = 0; i < N_INDIVIDUALS; i++)
	{
    Population[i][0] = (*x0) + 5 * get_random_number() - 2.5f;
    Population[i][1] = (*y0) + 5 * get_random_number() - 2.5f;
		Population[i][2] = (*size0) + 5 * get_random_number() - 2.5f;
	}

  float total_sum_weights = get_sum(weights, n_points);

	// large number, since we will minimize it:
	(*fitness) = 1000000;
	float fits[N_INDIVIDUALS];
	float best_genome[N_GENES];
  best_genome[0] = (*x0);
  best_genome[1] = (*y0);
  best_genome[2] = (*size0);
	for (g = 0; g < n_generations; g++)
	{
		for (i = 0; i < N_INDIVIDUALS; i++)
		{
			if (SHAPE == CIRCLE)
			{
        if(FF == DISTANCE_FIT)
        {
          // optimize mean distance to circle (and possibly stick) 
				  fits[i] = mean_distance_to_circle(Population[i]);
        }
        else
        {
          // optimize the number of inliers
          fits[i] = get_outlier_ratio(Population[i], total_sum_weights);
        }
			}
		}

		// get the best individual and store it in min_genome:
		int index;
		float min_fit = get_minimum(fits, N_INDIVIDUALS, &index);
    float min_genome[N_GENES];
    for(ge = 0; ge < N_GENES; ge++)
    {
		  min_genome[ge] = Population[index][ge];
    }

		// if better than any previous individual, remember it:
		if (min_fit < (*fitness))
		{
			for (ge = 0; ge < 3; ge++)
			{
				best_genome[ge] = min_genome[ge];
			}
			(*fitness) = min_fit;
		}

		// fill the new population with mutated copies of this generation's best:
		if (g < n_generations - 1)
		{
			// super elitist evolution:
			for (i = 0; i < N_INDIVIDUALS; i++)
			{
				Population[i][0] = min_genome[0] + 5 * get_random_number() - 2.5f;
				Population[i][1] = min_genome[1] + 5 * get_random_number() - 2.5f;
				Population[i][2] = min_genome[2] + 5 * get_random_number() - 2.5f;
			}
		}

	}

  // put the final values back in the parameters:
  if(FF == DISTANCE_FIT) (*fitness) /= total_sum_weights;
	(*x0) = best_genome[0];
	(*y0) = best_genome[1];
	(*size0) = best_genome[2];

  return;
}

float get_random_number()
{
	int rand_num = rand() % 1000;
	float r = (float)rand_num / 1000.0f;
	return r;
}

float get_minimum(float* nums, int n_elements, int *index)
{
  (*index) = 0;
	float min = nums[0];
  uint16_t i;
	for (i = 1; i < n_elements; i++)
	{
		if (nums[i] < min)
		{
			(*index) = i;
			min = nums[i];
		}
	}
	return min;
}

float get_sum(float* nums, int n_elements)
{
  float sum = nums[0];
  uint16_t i;
	for (i = 1; i < n_elements; i++)
	{
		sum += nums[i];
	}
	return sum;
}

void convert_disparitymap_to_points(struct image_i* disparity_image, int min_sub_disparity)
{
  int y, x, sp;
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
				  points[p].x = (float) x;
				  points[p].y = (float) y;
				  weights[p] = (float) disp;

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

  /*
  // THIS RESULTS IN VISITING CERTAIN ROWS FIRST
	uint16_t x0 = 0;
	uint8_t n_x = 5; // 5 x-coordinates at a time for each y coordinate

  // ys and xs are permutations of the columns and rows, respectively
  // we do batches of xs per y in order to prevent dominance of a single column:
	while (x0 < (*disparity_image).w - n_x)
	{
		for (y = 0; y < (*disparity_image).h; y++)
		{
			for (x = x0; x < x0 + n_x; x++)
			{
				// get the disparity from the image:
				disp = (*disparity_image).image[ys[y] * (*disparity_image).w + xs[x]];

				if (disp > min_disparity * RESOLUTION_FACTOR)
				{
					// add the points to the array, and use disparity as the weight:
					points[p].x = (float)xs[x];
					points[p].y = (float)ys[y];
					weights[p] = (float)disp;

					// count the number of points:
					p++;

					// if the maximum number of points is reached, return:
          // TODO: if still not working, don't return, but make remaining entries in disparity map 0
					if (p == MAX_POINTS)
					{
						n_points = p;
						return;
					}
				}
				else
				{
					// make the pixel black, so that we can see it on the ground station:
					disparity_image->image[ys[y] * disparity_image->w + xs[x]] = 0;
				}
			}
		}
		
		x0 += n_x;
	}
  */

  /* 
  // OLD CODE that leads to biases in where points are sampled:

	// convert the disparity map to points:
  int y, x;
  uint8_t disp;
  uint16_t p = 0;
	for (y = 0; y < (*disparity_image).h; y++)
	{
		for (x = 0; x < (*disparity_image).w; x++)
		{
      // get the disparity from the image:
			disp = (*disparity_image).image[ys[y]*(*disparity_image).w + xs[x]];

			if (disp > min_disparity * RESOLUTION_FACTOR)
			{
        // add the points to the array, and use disparity as the weight:
				points[p].x = (float) xs[x];
				points[p].y = (float) ys[y];
				weights[p] = (float) disp;

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
        disparity_image->image[ys[y] * disparity_image->w + xs[x]] = 0;
      }
		}
	}
  */

  // set the global variable n_points to the right value:
  n_points = p;
}

float mean_distance_to_circle(float* genome)
{
	float x = genome[0];
	float y = genome[1];
	float r = genome[2];

	float mean_distance = 0.0f;
	struct point_f point;
	float dx, dy;
	float dist_center, error, error_stick;
  uint8_t p;
	for (p = 0; p < n_points; p++)
	{
		point = points[p];
		dx = point.x - x;
		dy = point.y - y;
		dist_center = sqrtf(dx*dx + dy*dy);
		error = fabs(dist_center - r);

		if (STICK)
		{
			// determine distance to the stick:
			struct point_f stick1;
      stick1.x = x;
      stick1.y = y - r;
			struct point_f stick2;
      stick2.x = x;
      stick2.y = y - 2*r;
			error_stick = distance_to_segment(stick1, stick2, point);

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

float get_outlier_ratio(float* genome, float total_sum_weights)
{
	float x = genome[0];
	float y = genome[1];
	float r = genome[2];

	float outlier_ratio = 0.0f;
	struct point_f point;
	float dx, dy;
	float dist_center, error, error_stick;
  uint8_t p;
	for (p = 0; p < n_points; p++)
	{
		point = points[p];
		dx = point.x - x;
		dy = point.y - y;
		dist_center = sqrtf(dx*dx + dy*dy);
		error = fabs(dist_center - r);
    
		if (STICK)
		{
			// determine distance to the stick:
			struct point_f stick1;
      stick1.x = x;
      stick1.y = y - r;
			struct point_f stick2;
      stick2.x = x;
      stick2.y = y - 2*r;
			error_stick = distance_to_segment(stick1, stick2, point);

			// take the smallest error:
			if (error_stick < error) error = error_stick;
		}

		// if outlier, augment outlier_ratio:
		if (error > outlier_threshold)
    {
  		if (WEIGHTED)
	  	{
	  		outlier_ratio += weights[p];
		  }
		  else
		  {
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
	float norm_Q2_Q1 = sqrtf((Q1.x - Q2.x)*(Q1.x - Q2.x) + (Q1.y - Q2.y)*(Q1.y - Q2.y));
	float det = (Q2.x - Q1.x)*(P.y - Q1.y) - (Q2.y - Q1.y)*(P.x - Q1.x);
	float dist_line = fabs(det) / norm_Q2_Q1;
	return dist_line;
}

float distance_to_segment(struct point_f Q1, struct point_f Q2, struct point_f P)
{
  float dist_line = distance_to_line(Q1, Q2, P);

	// calculate intersection point:
	float rx = Q2.y - Q1.y; // always negative, -r
	float ry = -(Q2.x - Q1.x);
	float norm_r = sqrtf(rx*rx+ry*ry);
  rx = (rx / norm_r) * dist_line;
  ry = (ry / norm_r) * dist_line;

  // rx < 0, so:
  // if P.x > Q1.x, it should be P.x + rx
  // else it should be P.x - rx
	float i_x;
	float i_y;
  if(P.x > Q1.x)
  {
    i_x = P.x + rx;
    i_y = P.y + ry;
  }
  else
  {
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
	float d1 = sqrtf((Q1.x-I.x)*(Q1.x - I.x) + (Q1.y - I.y)*(Q1.y - I.y));
	float d2 = sqrtf((Q2.x - I.x)*(Q2.x - I.x) + (Q2.y - I.y)*(Q2.y - I.y));
	float d_12 = sqrtf((Q2.x - Q1.x)*(Q2.x - Q1.x) + (Q2.y - Q1.y)*(Q2.y - Q1.y));
	if (d1 > d_12 || d2 > d_12)
	{
		// not on segment, determine minimum distance to one of the two extremities:
		dist_line = sqrtf((Q1.x - P.x)*(Q1.x - P.x) + (Q1.y - P.y)*(Q1.y - P.y));
		d2 = sqrtf((Q2.x - P.x)*(Q2.x - P.x) + (Q2.y - P.y)*(Q2.y - P.y));
		if (d2 < dist_line) dist_line = d2;
	}
  */
		 
  // leave out superfluous sqrtf - for comparisons it does not matter (monotonously increasing functions)
  // we can still precalculate (Q1.x - I.x) etc. but I don't know if it is actually calculated twice (optimized by compiler?)
  float d1 = (Q1.x - I.x)*(Q1.x - I.x) + (Q1.y - I.y)*(Q1.y - I.y);
	float d2 = (Q2.x - I.x)*(Q2.x - I.x) + (Q2.y - I.y)*(Q2.y - I.y);
	float d_12 = (Q2.x - Q1.x)*(Q2.x - Q1.x) + (Q2.y - Q1.y)*(Q2.y - Q1.y);
	if (d1 > d_12 || d2 > d_12)
	{
		// not on segment, determine minimum distance to one of the two extremities:
		dist_line = (Q1.x - P.x)*(Q1.x - P.x) + (Q1.y - P.y)*(Q1.y - P.y);
		d2 = (Q2.x - P.x)*(Q2.x - P.x) + (Q2.y - P.y)*(Q2.y - P.y);
		if (d2 < dist_line) dist_line = sqrtf(d2);
	}

	return dist_line;
}

void draw_circle(struct image_i* Im, float x_center, float y_center, float radius, uint8_t* color)
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
