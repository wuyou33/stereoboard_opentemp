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
#define MAX_POINTS 150
struct point_f points[MAX_POINTS];
uint16_t n_points;

// Settings for the evolution:
#define N_INDIVIDUALS 10
#define N_GENES 3
uint16_t n_generations = 30; // could be reduced for instance when there are many points
float Population[N_INDIVIDUALS][N_GENES];

// Settings for the fitting:
float weights[MAX_POINTS];
int min_points = 5;
int WEIGHTED = 1; 
int STICK = 1;
#define CIRCLE 0
#define SQUARE 1
int SHAPE = CIRCLE;
int min_disparity = 3;
float outlier_threshold = 25.0f;


// whether to draw on the disparity image:
int GRAPHICS = 1;

/**
 * Function takes a disparity image and fits a circular gate to the close-by points.
 * - initialize_fit_with_pars will use the x_center, y_center, etc. to initialize the population used in the evolutionary algorithm to make the fit.
 * - The results are put back in the parameters. 
 * @author Guido
 */

void gate_detection(struct image_i* disparity_image, float* x_center, float* y_center, float* radius, float* fitness, int initialize_fit_with_pars)
{
  // 1) convert the disparity map to a vector of points:
	convert_disparitymap_to_points(disparity_image);

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
			size0 = sqrt(mean_x*mean_x + mean_y*mean_y);

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
				fits[i] = mean_distance_to_circle(Population[i]);
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
	(*fitness) /= get_sum(weights, N_INDIVIDUALS);
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
  float sum = 0.0f;
  uint16_t i;
	for (i = 1; i < n_elements; i++)
	{
		sum += nums[i];
	}
	return sum;
}

void convert_disparitymap_to_points(struct image_i* disparity_image)
{

	// convert the disparity map to points:
  int y, x;
  uint8_t disp;
  uint16_t p = 0;
	for (y = 0; y < (*disparity_image).h; y++)
	{
		for (x = 0; x < (*disparity_image).w; x++)
		{
			// get the disparity from the image:
			disp = (*disparity_image).image[y*(*disparity_image).w + x];

			if (disp > min_disparity * RESOLUTION_FACTOR)
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
	float rx = Q2.y - Q1.y;
	float ry = -(Q2.x - Q1.x);
	float norm_r = sqrtf(rx*rx+ry*ry);
	float i_x = (rx / norm_r) * dist_line + P.x;
	float i_y = (ry / norm_r) * dist_line + P.y;
	struct point_f I;
  I.x = i_x;
  I.y = i_y;
	float dI = distance_to_line(Q1, Q2, I);
	if (dI > 1e-10)
	{
		I.x = P.x - rx;
		I.y = P.y - ry;
	}
		
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
			Im->image[y * (Im->w) + x] = 128;
		}

	}/*
  x_center = 64;
  y_center = 48;

  x = (int) x_center;
  y = (int) y_center;

  //Im->image[(y*128)+x] = 255;
  /*
  if (x >= 2 && x < Im->w-2 && y >= 2 && y < Im->h-2)
  {
	  for(x = (int)x_center-2; x < (int)x_center+2; x++)
	  {
		  for(y = (int)y_center-2; y < (int)y_center+2; y++)
		  {
			  Im->image[y*Im->w+x] = 255; //color[0];
		  }
	  }
  }*/
  return;
}
