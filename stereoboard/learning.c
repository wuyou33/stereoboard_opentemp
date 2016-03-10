/*
 * textons.c
 *
 *  Created on: 16 feb. 2016
 *      Author: Kevin
 */

#include "main_parameters.h"
#include "learning.h"
#include "textons.h"
#include "tmg3993.h"
#include "commands.h"

q7_t histogram[n_textons];
q7_t temporary_cluster[n_textons*150];
q7_t histogram_cluster[n_textons*clust_size];
q7_t distance_cluster[clust_size];
uint16_t nr_in_clust = 0;
uint16_t nr_in_temp_clust = 0;
uint8_t once = 0;
volatile uint8_t distance = 0;
uint16_t clust_idx = 0;
uint16_t last_clust_idx;

void learning_collisions_init(void)
{
	initTexton();

	init_Angle_Measurement();

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

	TMG3993_Read_Proximity();
	TMG3993_Read_Proximity();
	int16_t collision = Angle_Measurement();

	if (collision==127)
	{
	  if (nr_in_clust>5)
	  {
	    distance = getDistanceKNN(histogram, 5);
	  }
	  //if (nr_in_clust<clust_size)
	  //{
	  //  addHistogramToCluster(histogram);
	  //}
	  //else
	  //{
	  addHistogramToTemp(histogram);
	  //}
	}

	//SendCommandNumber((uint8_t) collision);
	if (collision==127)
	{
	  once = 0;
	}

  if (collision==127)
  {
    if (distance>250)
    {
      distance = 250;
    }
    SendCommandNumber((uint8_t)distance); ////////////////////// REMOVED -127
  }
  else if (collision>=0)
	{
	  SendCommandNumber((uint8_t) 254);
	}
  else if (collision<0)
  {
    SendCommandNumber((uint8_t) 255);
  }
  if ((collision!=127) && (once==0))
  {
    addTempToCluster(histogram);
    once = 1;
  }
}

uint8_t getDistanceKNN(q7_t *histogram, uint8_t K)
{
  uint16_t i;
  uint8_t k;
  uint32_t d;
  uint32_t dist[K];
  uint16_t idx[K];
  uint32_t dist_max = 99999999;
  uint16_t idx_max = 0;
  uint16_t distance = 0;

  // Initialise dist with very large distances
  for (k=0; k<K; k++)
  {
    dist[k] = 99999999;
  }

  // Calculate for current histogram distance to each histogram in cluster
  for (i=0; i<clust_size; i++)
  {
    d = getEuclDistHist(histogram,&histogram_cluster[i*n_textons]);

    // If one of K smallest, store distance
    if (d<dist_max)
    {
      dist[idx_max] = d;
      idx[idx_max] = i;

      // Find largest distance in stored distances
      dist_max = dist[0];
      idx_max = 0;

      for(k=1; k<K; k++)
      {
        if (dist[k]>dist_max)
        {
          dist_max = dist[k];
          idx_max = k;
        }
      }
    }
  }

  // Calculate average distance of k Nearest Neighbours
  for (k=0; k<K; k++)
  {
    distance += distance_cluster[idx[k]];
  }
  distance /= K;

  return (uint8_t)distance;
}

uint8_t checkCollision(void)
{
  uint8_t collision = 0;

  return collision;
}

void addHistogramToTemp(q7_t *histogram)
{
  // Increment nr_in_temp_clust
  if (nr_in_temp_clust<150)
  {
    nr_in_temp_clust++;
  }
  clust_idx++;


  // Add latest histogram to temp clust
  arm_copy_q7(histogram, &temporary_cluster[(clust_idx%150)*n_textons], n_textons);

  // Most recent value at
  last_clust_idx = clust_idx%150;
}

/*void addHistogramToCluster(q7_t *histogram)
{
  // Add latest histogram to temp clust
  arm_copy_q7(histogram, &histogram_cluster[nr_in_clust*n_textons], n_textons);

  // Increment nr_in_temp_clust
  nr_in_clust++;
}*/

void addTempToCluster(q7_t *histogram)
{
  // Find closest pair
  volatile uint32_t dist;
  uint32_t nearest =  1294967295;
  uint32_t furthest =  0;

  uint16_t i, j, k;

  struct nearestPair{
    uint32_t distance;
    uint16_t id_1;
    uint16_t id_2;
    uint16_t nextPair;
    uint16_t prevPair;
  };
  struct nearestPair nearestPairs[150];
  uint16_t n;
  for(n=0; n<nr_in_temp_clust; n++)
  {
    nearestPairs[n].distance = 1294967295;
    nearestPairs[n].nextPair = n+1;
    nearestPairs[n].prevPair = n-1;
    if (n==nr_in_temp_clust-1)
    {
      nearestPairs[n].nextPair = 0;
    }
    if (n==0)
    {
      nearestPairs[n].prevPair = nr_in_temp_clust-1;
    }
  }
  uint16_t nearest_id = 0;
  uint16_t furthest_id = nr_in_temp_clust-1;

  for(i=0; i<(clust_size-1); i++)
  {
    if (i%4==0)
    {
      int16_t collision = Angle_Measurement();
      if (collision<0)
      {
        SendCommandNumber((uint8_t) 255);
      }
    }
    uint8_t cont = 0;
    uint16_t l;
    for(l=0; l<nr_in_temp_clust; l++)
    {
      if ((nearestPairs[l].id_1==i) || (nearestPairs[l].id_2==i))
      {
        cont = 1;
        break;
      }
    }
    if (cont==1)
    {
       continue;
    }
    for(j=i+1; j<clust_size; j++)
    {
      cont = 0;
      for(l=0; l<nr_in_temp_clust; l++)
      {
        if ((nearestPairs[l].id_1==j) || (nearestPairs[l].id_2==j))
        {
          cont = 1;
          break;
        }
      }
      if (cont==1)
      {
         continue;
      }
      dist = getEuclDistHist(&histogram_cluster[i*n_textons],&histogram_cluster[j*n_textons]);

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
      histogram_cluster[nearestPairs[j].id_2*n_textons+i] = temporary_cluster[(last_clust_idx-j)%150*n_textons+i];
      //TODO: (KL) Replace for arm_copy
    }
    distance_cluster[nearestPairs[j].id_2] = j; // Number of frames before a collision

    // Increment nr_in_temp_clust
    nr_in_clust++;
  }
  nr_in_temp_clust = 0;
}


q63_t getEuclDistHist(q7_t *hist1, q7_t *hist2)
{
  q15_t hist1_15[n_textons];
  q15_t hist2_15[n_textons];
  q15_t diff[n_textons];

  arm_q7_to_q15_LSB(hist1, hist1_15, n_textons);
  arm_q7_to_q15_LSB(hist2, hist2_15, n_textons);

  arm_sub_q15(hist1_15, hist2_15, diff, n_textons);

  q63_t sum;
  arm_dot_prod_q15(diff, diff, n_textons, &sum);

  return sum;
}
