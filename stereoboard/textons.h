/*
 * textons.h
 *
 *  Created on: 16 feb. 2016
 *      Author: Kevin
 */

#include <stdint.h>
#include <inttypes.h>
#include <arm_math.h>

#define n_patches_h 10
#define n_patches_v  7
#define patch_size_h 5
#define patch_size_v 5
#define patch_size ((patch_size_h*patch_size_v)-1)*3 //subtract 1 to end up with multiple of 4
#define n_textons 24 //this is also number of bins in histogram
#define clust_size 500

void initTexton(void);
void getTextonDistribution(uint8_t *image, q7_t *histogram);
q63_t getEuclDistPatch(q15_t patch[], uint8_t texton_id);


