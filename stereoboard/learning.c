/*
 * textons.c
 *
 *  Created on: 16 feb. 2016
 *      Author: Kevin
 */

#include "main_parameters.h"
#include "learning.h"

void learning_collisions_init(void)
{
	initTexton();
}

void learning_collisions_run( uint8_t *image )
{
	getTextonDistribution(image);

}
