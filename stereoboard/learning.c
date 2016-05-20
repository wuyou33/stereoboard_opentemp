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
#include "stm32f4xx_flash.h"
#include "sys_time.h"

q7_t histogram[n_textons];
q7_t temporary_cluster[n_textons*64]; //highest distance = 64, so everything stays positive in signed byte
q7_t histogram_cluster[n_textons*clust_size];
q7_t distance_cluster[clust_size];
uint16_t nr_in_clust = 0;
uint16_t nr_in_temp_clust = 0;
uint8_t once = 0;
volatile uint8_t distance = 0;
uint16_t last_clust_idx;
uint16_t clust_idx = 0;
uint16_t flash_cnt = 0;

uint8_t filled=0;

void learning_collisions_init(void)
{
  // Read histogram_cluster from flash
  readClusterFromFlash11(histogram_cluster, distance_cluster);

	initTexton();

	init_Angle_Measurement();

	// Initialise histogram_cluster with zeros
	/*uint16_t i;
	for (i=0; i<n_textons*clust_size; i++)
	{
	  histogram_cluster[i] = 0;
	}*/
}

void learning_collisions_run( uint8_t *image )
{
	getTextonDistribution(image, histogram);

	TMG3993_Read_Proximity();
	TMG3993_Read_Proximity();
	int16_t collision = Angle_Measurement();

	//if (collision==127)
	//{
	// if (nr_in_clust>10)
	//  {
	    distance = getDistanceKNN(histogram, 3);
	//  }
	//  addHistogramToTemp(histogram);
	//}

	/*if (collision==127)
	{
	  once = 0;
	}
*/
  //if (collision==127)
  //{
    if (distance>250)
    {
      distance = 250;
    }
    SendCommandNumber((uint8_t)distance);
    flash_cnt = 0;
    if (distance<18)
    {
      SendCommandNumber((uint8_t) 255);
    }

  //}
  /*else if (collision>=0)
	{
	  SendCommandNumber((uint8_t) 254);
	  flash_cnt++;
	}
  else if (collision<0)
  {
    SendCommandNumber((uint8_t) 255);
    flash_cnt++;
  }
  if ((collision!=127) && (once==0))
  {
    addTempToCluster();
    once = 1;
  }
  // Store histogram_cluster to flash if proximity is triggered for long period
  if (flash_cnt>150) // Check how long this is
  {
    writeClusterToFlash11(histogram_cluster, distance_cluster);
    // Fast blinking of LED to confirm writing was done
    int x;
    int time_prev;
    time_prev = sys_time_get();
    for(x=0; x<100; x++){
      led_toggle();
      while (sys_time_get()<(time_prev+50))
      {
        if (time_prev>19900)
        {
          time_prev = 0;
        }
      }
      time_prev = sys_time_get();
    }
    // Set flash_cnt back to zero, such that measurements continue
    flash_cnt = 0;
  }
  */
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
  /* This function write, during flying, all histograms to a temporary cluster. They will be further processed after a collision happened. */

  // Increment nr_in_temp_clust
  if (nr_in_temp_clust<64)
  {
    nr_in_temp_clust++;
  }
  clust_idx++;


  // Add latest histogram to temp clust
  arm_copy_q7(histogram, &temporary_cluster[(clust_idx%64)*n_textons], n_textons);

  // Most recent value at
  last_clust_idx = clust_idx%64;
}

void addTempToCluster(void)
{
  /* This function writes everything from the temporary cluster to the real histograms cluster, after a collision occurred. */

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
  struct nearestPair nearestPairs[64];
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
      if (collision<0 && collision!=127)
      {
        SendCommandNumber((uint8_t) 255);

      }
      else if (collision>=0 && collision!=127)
      {
        SendCommandNumber((uint8_t) 254);
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
    /*for(i=0; i<n_textons; i++)
    {
      histogram_cluster[nearestPairs[j].id_1*n_textons+i] = (histogram_cluster[nearestPairs[j].id_1*n_textons+i] + histogram_cluster[nearestPairs[j].id_2*n_textons+i])/2;
      //TODO: (KL) Replace for combination of arm_add and arm_div
    }*/
    arm_add_q7(&histogram_cluster[nearestPairs[j].id_1*n_textons],&histogram_cluster[nearestPairs[j].id_2*n_textons], &histogram_cluster[nearestPairs[j].id_1*n_textons], n_textons); // Merge histograms (Add 2 histograms)
    arm_shift_q7(&histogram_cluster[nearestPairs[j].id_1*n_textons], -1, &histogram_cluster[nearestPairs[j].id_1*n_textons], n_textons); // Merge histograms (Divide by 2, bit shift right)

    distance_cluster[nearestPairs[j].id_1] = (distance_cluster[nearestPairs[j].id_1] + distance_cluster[nearestPairs[j].id_2])/2; // Merge distance labels
  }

  // Add latest histogram
  for(j=0; j<nr_in_temp_clust; j++)
  {
    /*for(i=0; i<n_textons; i++)
    {
      histogram_cluster[nearestPairs[j].id_2*n_textons+i] = temporary_cluster[(last_clust_idx-j)%150*n_textons+i];
      //TODO: (KL) Replace for arm_copy
    }*/
    arm_copy_q7(&temporary_cluster[(last_clust_idx-j)%64*n_textons],&histogram_cluster[nearestPairs[j].id_2*n_textons], n_textons);

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

void writeClusterToFlash11(q7_t *histClust, q7_t *distClust)
{
  uint32_t startAddress = 0x080E0000; // startAddress of sector 11
  uint16_t histSize = n_textons*clust_size;

  //Unlock flash writing
  FLASH_Unlock();

  // Clear pending flags (if any)
  FLASH_ClearFlag(FLASH_FLAG_EOP | FLASH_FLAG_OPERR  |
               FLASH_FLAG_WRPERR | FLASH_FLAG_PGAERR |
               FLASH_FLAG_PGPERR | FLASH_FLAG_PGSERR);

  // Erase flash, 0x080E0000 - 0x0805FFFF (128 KBytes), device operating range: 2.7V to 3.6V
  FLASH_EraseSector(FLASH_Sector_11, VoltageRange_3);


  uint16_t i;
  // Write histograms to flash
  for(i=0; i<(histSize); i++)
  {
    FLASH_ProgramByte(startAddress + i, histClust[i]);
  }
  // Write distances to flash
  for(i=0; i<(clust_size); i++)
  {
    FLASH_ProgramByte(startAddress + histSize + i, distClust[i]);
  }

  // Lock the flash for writing
  FLASH_Lock();
}

void readClusterFromFlash11(q7_t *histClust, q7_t *distClust)
{
  uint32_t startAddress = 0x080E0000; // startAddress of sector 11
  uint16_t histSize = n_textons*clust_size;

  uint16_t i;
  // Read histograms from flash
  for(i=0; i<histSize; i++)
  {
    histClust[i] = *(int8_t *)(startAddress + i);
  }
  // Read distances from flash
  for(i=0; i<clust_size; i++)
  {
    distClust[i] = *(int8_t *)(startAddress + histSize + i);
  }
}
