/*
 * textons.c
 *
 *  Created on: 16 feb. 2016
 *      Author: Kevin
 */

#include "main_parameters.h"
#include "learning.h"
#include "textons.h"

q7_t histogram[n_textons];
q7_t temporary_cluster[n_textons*150];
q7_t histogram_cluster[n_textons*clust_size];
q7_t distance_cluster[clust_size];
uint16_t nr_in_clust = 0;
uint16_t nr_in_temp_clust = 0;

void learning_collisions_init(void)
{
	initTexton();

	// Initialise histogram_cluster with zeros
	uint16_t i;
	for (i=0; i<n_textons*clust_size; i++)
	{
	  histogram_cluster[i] = 0;
	}
}

void learning_collisions_run( uint8_t *image )
{
	getTextonDistribution(image, histogram);

	uint8_t collision = checkCollision();
	if (collision==1)
	{
	  addTempToCluster(histogram);
	}
	else
	{
	  addHistogramToTemp(histogram);
	}
}

uint8_t checkCollision()
{
  uint8_t collision = 0;

  return collision;
}

void addHistogramToTemp(q7_t *histogram)
{
  if (nr_in_temp_clust<150)
  {
    // Add latest histogram to temp clust
    arm_copy_q7(histogram, &temporary_cluster[nr_in_temp_clust*n_textons], n_textons);

    // Increment nr_in_temp_clust
    nr_in_temp_clust++;
  }
}

void addTempToCluster(q7_t *histogram)
{
  // Find closest pair
  uint32_t dist;
  uint32_t nearest =  1294967295;
  uint32_t furthest =  0;

  uint16_t i, j, k;

  struct nearestPair{
    uint32_t distance;
    uint16_t id_1;
    uint16_t id_2;
    uint16_t nextPair;
    uint16_t prevPair;
    //struct nearestPair *nextPair;
    //struct nearestPair *prevPair;
  };
  //struct nearestPair nearestPairs = malloc(nr_in_temp_clust * sizeof *nearestPairs);
  struct nearestPair nearestPairs[150];
  uint16_t n;
  for(n=0; n<150; n++)
  {
    nearestPairs[n].distance = 1294967295;
    nearestPairs[n].nextPair = n+1;
    nearestPairs[n].prevPair = n-1;
  }
  uint16_t nearest_id = 0;
  uint16_t furthest_id = 149;

  for(i=0; i<clust_size-1; i++)
  {
    uint16_t l;
    for(l=0; l<nr_in_temp_clust; l++)
    {
      if (nearestPairs[l].id_1==i || nearestPairs[l].id_2==i)
      {
        continue;
      }
    }
    for(j=i+1; j<clust_size; j++)
    {
      for(l=0; l<nr_in_temp_clust; l++)
      {
        if (nearestPairs[l].id_1==j || nearestPairs[l].id_2==j)
        {
          continue;
        }
      }
      dist = getEuclDistHist(&histogram_cluster[i*n_textons],&histogram_cluster[j*n_textons]);

      /*if ((i==0) && (j==1))
      {
        nearestPairs[0].distance = dist;
        nearestPairs[0].id_1 = i;
        nearestPairs[0].id_2 = j;
        nearestPairs[0].nextPair = 0;
        nearestPairs[0].prevPair = 0;

        nearest_id = 0;
        furthest_id = 0;

        cnt++;
      }
      else if ((dist<=nearest) && (cnt<nr_in_temp_clust))
      {
        nearest = dist;


        nearestPairs[cnt].distance = dist;
        nearestPairs[cnt].id_1 = i;
        nearestPairs[cnt].id_2 = j;
        nearestPairs[cnt].nextPair = nearest_id;
        nearestPairs[nearest_id].prevPair = cnt;
        nearestPairs[cnt].prevPair = 0;

        nearest_id = cnt;

        cnt++;
      }
      else if ((dist>=furthest) && (cnt<nr_in_temp_clust))
      {
        furthest = dist;

        nearestPairs[cnt].distance = dist;
        nearestPairs[cnt].id_1 = i;
        nearestPairs[cnt].id_2 = j;
        nearestPairs[cnt].nextPair = 0;
        nearestPairs[cnt].prevPair = furthest_id;
        nearestPairs[furthest_id].nextPair = cnt;

        furthest_id = cnt;

        cnt++;
      }
      else if ((dist<furthest) && (dist>nearest) && (cnt<nr_in_temp_clust))
      {
        uint16_t k;
        for(k=0; k<cnt-1; k++)
        {
          if ((dist>=nearestPairs[k].distance) && (dist<nearestPairs[nearestPairs[k].nextPair].distance))
          {
            nearestPairs[cnt].distance = dist;
            nearestPairs[cnt].id_1 = i;
            nearestPairs[cnt].id_2 = j;
            nearestPairs[cnt].nextPair = nearestPairs[k].nextPair;
            nearestPairs[cnt].prevPair = k;
            nearestPairs[nearestPairs[k].nextPair].prevPair = cnt;
            nearestPairs[k].nextPair = cnt;
            break;
          }
        }
        cnt++;
      }*/
      if (dist<=nearest)
      {
        nearest = dist;

        nearestPairs[furthest_id].distance = dist;
        nearestPairs[furthest_id].id_1 = i;
        nearestPairs[furthest_id].id_2 = j;
        nearestPairs[furthest_id].nextPair = nearest_id;
        nearestPairs[nearest_id].prevPair = furthest_id;

        nearest_id = furthest_id;
        furthest_id = nearestPairs[furthest_id].prevPair;
        furthest = nearestPairs[furthest_id].distance;
      }
      else if ((dist<furthest) && (dist>nearest))
      {
        for(k=0; k<(nr_in_temp_clust-1); k++)
        {
          if ((dist>=nearestPairs[k].distance) && (dist<nearestPairs[nearestPairs[k].nextPair].distance))
          {
            uint16_t prev_furthest_id =  nearestPairs[furthest_id].prevPair;
            uint16_t prev_furthest =  nearestPairs[prev_furthest_id].distance;

            nearestPairs[furthest_id].distance = dist;
            nearestPairs[furthest_id].id_1 = i;
            nearestPairs[furthest_id].id_2 = j;
            nearestPairs[furthest_id].nextPair = nearestPairs[k].nextPair;
            nearestPairs[furthest_id].prevPair = k;
            nearestPairs[nearestPairs[k].nextPair].prevPair = furthest_id;
            nearestPairs[k].nextPair = furthest_id;

            furthest_id = prev_furthest_id;
            furthest = prev_furthest;

            break;
          }
        }
      }
    }


    // Merge closest pairs
    for(j=0; j<nr_in_temp_clust; j++)
    {
      for(i=0; i<n_textons; i++)
      {
        histogram_cluster[nearestPairs[j].id_1*n_textons+i] = (histogram_cluster[nearestPairs[j].id_1*n_textons+i] + histogram_cluster[nearestPairs[j].id_2*n_textons+i])/2;
        //TODO: (KL) Replace for combination of arm_add and arm_div
      }
      distance_cluster[nearestPairs[j].id_1] = (distance_cluster[nearestPairs[j].id_1] + distance_cluster[nearestPairs[j].id_2])/2;
    }

    // Add latest histogram
    for(j=0; j<nr_in_temp_clust; j++)
    {
      for(i=0; i<n_textons; i++)
      {
        histogram_cluster[nearestPairs[j].id_2*n_textons+i] = temporary_cluster[j*n_textons+i];
        //TODO: (KL) Replace for arm_copy
      }
      distance_cluster[nearestPairs[j].id_2] = nr_in_temp_clust-j; // Number of frames before a collision
    }
    nr_in_temp_clust = 0;
  }
}

uint32_t getEuclDistHist(q7_t *hist1, q7_t *hist2)
{
  q15_t hist1_15[n_textons];
  q15_t hist2_15[n_textons];
  uint8_t j;
  for(j=0; j<n_textons; j++)
  {
    hist1_15[j]=hist1[j];
    hist2_15[j]=hist2[j];
  }

  q15_t diff[n_textons];

  arm_sub_q15(hist1_15, hist2_15, diff, n_textons);

  /*arm_mult_q31_LSB(diff, diff, diff, n_textons); // Watch out, this function can overflow!!!

  // Calculate sum of array by a iteratively adding half of the array to the other half
  uint8_t temp_patch_size = n_textons;
  while(temp_patch_size>=8 && temp_patch_size%2==0) // Do this until less than 8 or an odd number of items remain
  {
    temp_patch_size /= 2;
    arm_add_q31(diff, &diff[temp_patch_size], diff, temp_patch_size);
  }

  // Calculate remaining of sum piece by piece
  uint32_t sum = 0;
  uint8_t i;
  for(i=0; i<temp_patch_size; i++)
  {
    sum += diff[i];
  }*/
  q63_t sumq;
  arm_dot_prod_q15(diff, diff, n_textons, &sumq);
  uint32_t sum = sumq;

  return sum;
}
