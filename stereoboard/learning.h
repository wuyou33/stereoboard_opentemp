/*
 * learning.h
 *
 *  Created on: 16 feb. 2016
 *      Author: Kevin
 */

#include <stdint.h>
#include <inttypes.h>
#include <arm_math.h>

void learning_collisions_init(void);
void learning_collisions_run(uint8_t *image);
void addHistogramToTemp(q7_t *histogram);
void addTempToCluster(void);
q63_t getEuclDistHist(q7_t *hist1, q7_t *hist2);
uint8_t checkCollision(void);
uint8_t getDistanceKNN(q7_t *histogram, uint8_t K);
void writeClusterToFlash11(q7_t *histClust, q7_t *distClust);
void readClusterFromFlash11(q7_t *histClust, q7_t *distClust);
