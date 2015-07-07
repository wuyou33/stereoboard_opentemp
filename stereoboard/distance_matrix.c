/*
 * distance_matrix.c
 *
 *  Created on: Jul 6, 2015
 *      Author: roland
 */

#include "distance_matrix.h"


void calculateDistanceMatrix(uint8_t* disparity_image,
		int* matrixBuffer,
		uint8_t blackBorderSize, uint8_t pixelsPerLine, uint8_t widthPerBin,
		uint8_t heightPerBin,uint8_t *toSendBuffer, uint32_t disparity_range) {

	int indexBuffer;

	uint8_t y;
	uint8_t valueInImageBuffer=0;
	uint16_t positionInImageBuffer=0;
	uint8_t positionInMatrix=0;
	uint8_t x;
	uint8_t z;
	uint8_t highestValues[MATRIX_WIDTH_BINS*MATRIX_HEIGHT_BINS][5];
	uint16_t sumDisparities[MATRIX_WIDTH_BINS*MATRIX_HEIGHT_BINS][disparity_range];
	for (x = 0; x < MATRIX_WIDTH_BINS*MATRIX_HEIGHT_BINS; x++) {
		for(y=0;y<5;y++){
			highestValues[x][y]=0;
		}
		for(y=0;y<disparity_range;y++){
			sumDisparities[x][y]=0;
		}
	}

	for (x = 0; x < MATRIX_WIDTH_BINS; x++) {
		for (y = 0; y < MATRIX_HEIGHT_BINS; y++) {
			int line;
			for (line = 0; line < heightPerBin; line++) {
				int bufferIndex = 0;
				for (bufferIndex = 0; bufferIndex < widthPerBin;
						bufferIndex++) {
					positionInImageBuffer = pixelsPerLine* (y * heightPerBin) + line * pixelsPerLine+ widthPerBin * x + blackBorderSize+ bufferIndex;
					valueInImageBuffer=disparity_image[positionInImageBuffer];

					positionInMatrix = y * MATRIX_WIDTH_BINS + x;

					sumDisparities[positionInMatrix][valueInImageBuffer]++;

					/*
					if(valueInImageBuffer>matrixBuffer[positionInMatrix])
					{
						matrixBuffer[positionInMatrix]=valueInImageBuffer;
					}
					for(z=0;z <5;z++)
					{
						if(valueInImageBuffer>highestValues[positionInMatrix][z])
						{
							highestValues[positionInMatrix][z]=valueInImageBuffer;
							break;
						}
					}
					*/
				}
			}
		}
	}


	// Average by dividing by the amount of pixels per bin
	int bufferIndex;
	int anyOn=0;

	for (bufferIndex = 0; bufferIndex < MATRIX_WIDTH_BINS * MATRIX_HEIGHT_BINS;
			bufferIndex++) {

		int sum_disparities = 0;
		for ( y = disparity_range-1; y>=0; y--)
		{
			int COUNTER_THRESHOLD = 10;
			sum_disparities += sumDisparities[bufferIndex][y];
			if (sum_disparities > COUNTER_THRESHOLD)
			{
				toSendBuffer[bufferIndex] = y;
				if ( y > CLOSE_BOUNDARY )
				{
					anyOn=1;
				}
				break;

			}
		}
		/*
		toSendBuffer[bufferIndex]=highestValues[bufferIndex][4];
		if(toSendBuffer[bufferIndex]>CLOSE_BOUNDARY)
		{
			anyOn=1;
		}
		*/
	}
	if(anyOn==1){
		led_set();
	}
	else
	{
		led_clear();
	}


}
