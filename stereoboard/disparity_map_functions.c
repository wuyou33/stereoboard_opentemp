
#include "disparity_map_functions.h"

#define MIN(X, Y) (((X) < (Y)) ? (X) : (Y))
#define MAX(X, Y) (((X) > (Y)) ? (X) : (Y))

void get_average_left_right(uint8_t histogramBuffer[], uint8_t* avg_disp_left, uint8_t* avg_disp_right, uint8_t pixelsPerLine)
{
	uint8_t x;
	long sum_left=0;
	long sum_right=0;
	uint8_t half_width=pixelsPerLine/2;
	// get the average of the left half of the image:
	for (x = 0; x < half_width; x++ )
	{
		sum_left += histogramBuffer[x];
	}
	(*avg_disp_left) = sum_left / half_width;
	// get the average of the right half of the image:
	for (x = half_width; x < pixelsPerLine; x++ )
	{
		sum_right += histogramBuffer[x];
	}
	(*avg_disp_right) = sum_right / half_width;
}
void get_average_left_right_features(uint8_t featureBuffer[], int featureCount,uint8_t* avg_disp_left, uint8_t* avg_disp_right, uint8_t pixelsPerLine)
{
	uint8_t x;
	long sum_left=0;
	long sum_right=0;
	int featureIndex;
	int sumCountLeft=0;
	int sumCountRight=0;
	uint8_t half_width=pixelsPerLine/2;
	for(featureIndex=0;featureIndex<featureCount; featureIndex++){
		uint8_t indexy=featureBuffer[featureIndex*3];
		uint8_t indexX=featureBuffer[featureIndex*3+1];
		uint8_t value=featureBuffer[featureIndex*3+2];
		if(indexX < half_width){
			sum_left+=value;
			sumCountLeft++;
		}
		else{
			sum_right+=value;
			sumCountRight++;
		}
	}
	(*avg_disp_left) = sum_left / sumCountLeft;
	(*avg_disp_right) = sum_right / sumCountRight;
}


void histogram_x_direction(uint8_t *disparity_image, uint8_t *histogramBuffer, horizontal_histogram_type histogram_type,uint8_t blackBorderSize, uint8_t pixelsPerLine, uint8_t heightPerLine)
{
  uint8_t valueInImageBuffer = 0;
  uint16_t positionInImageBuffer = 0;
  uint8_t x;
  uint8_t y;
  uint8_t minimumPixelsPerColumn;
  if(histogram_type==AVOID_ME_HISTOGRAM){
	  minimumPixelsPerColumn=5;
  }
  else{
	  minimumPixelsPerColumn=30;
  }
  double sumSurroundingBuffer[pixelsPerLine];
  double maxValue = 0.0;

  // Fill the disparity histogram:
  int histoIndex;
#ifdef HISTOGRAM_WIDTH
  uint8_t dist=HISTOGRAM_WIDTH;
#else
  uint8_t dist=3;
#endif
  for (histoIndex = 0; histoIndex < pixelsPerLine ; histoIndex++) {
	histogramBuffer[histoIndex] = 0;
  }
  for (histoIndex = dist; histoIndex < pixelsPerLine-dist ; histoIndex++) {

	int sumThisPlace =0;
	int numPixelsFound=0;
	for (x = histoIndex-dist; x < histoIndex+dist; x++) {
	   for (y = 10; y < heightPerLine-10; y++) {
         positionInImageBuffer = pixelsPerLine * y + x;
         valueInImageBuffer = disparity_image[positionInImageBuffer];
         if(histogram_type==AVOID_ME_HISTOGRAM){
             if(valueInImageBuffer>0){
    			 numPixelsFound++;
    		  }
    		  sumThisPlace += valueInImageBuffer;
         }
         else{ // Follow me function
             if(valueInImageBuffer>12){
    			 numPixelsFound++;

    		  sumThisPlace += valueInImageBuffer;
             }
            }

       }
    }
	if(numPixelsFound>5){
		sumThisPlace/=numPixelsFound;
		if(sumThisPlace>maxValue){
			maxValue = sumThisPlace;
		}
	}
	else{
		sumThisPlace=0;
	}
	sumSurroundingBuffer[histoIndex] = sumThisPlace;

  }
  for (histoIndex = dist; histoIndex < pixelsPerLine-dist ; histoIndex++) {
	  histogramBuffer[histoIndex] = sumSurroundingBuffer[histoIndex];

	  //If avoiding: set zeros who are dangerous to higher values.
	  if(histogram_type==AVOID_ME_HISTOGRAM){
		  if(histogramBuffer[histoIndex]==0){
			  //Filter surrounding histos
			  if(sumSurroundingBuffer[MAX(0,histoIndex-1)]>0){
				  histogramBuffer[histoIndex]=sumSurroundingBuffer[histoIndex-1];
			  }
			  else if(sumSurroundingBuffer[MIN(pixelsPerLine,histoIndex+1)]>0){
				  histogramBuffer[histoIndex]=sumSurroundingBuffer[histoIndex+1];
			  }
			  else{
				  //else set max
				  histogramBuffer[histoIndex]=120;
			  }

		  }
	  }
  }
}

void histogram_z_direction(uint8_t *disparity_image, uint8_t *histogramBuffer, uint8_t pixelsPerLine, uint8_t heightPerLine)
{
	// compute disparity histogram
	uint8_t y;
	uint8_t x;
	uint8_t max_y=90;
	uint8_t disp_min=1;
	uint8_t disp;

	for (x = 0; x < pixelsPerLine; x++) {
			histogramBuffer[x]=0;
	}
	for (y = 0; y < max_y; y++) {
		for (x = 0; x < pixelsPerLine; x++) {

			disp = disparity_image[x + y * pixelsPerLine];
			if (disp > disp_min)
			{
				histogramBuffer[disp]++;
			}
		}
	}
}

void histogram_z_direction_features(uint8_t disparity_coordinates[], uint8_t histogramBuffer[], uint8_t featureCount, uint8_t maxDisparityToMeasure)
{
	// compute disparity histogram
	uint8_t indexInCoordinateMap;
	uint8_t x;
	uint8_t disp_min=1;
	uint8_t disp;

	for (x = 0; x < maxDisparityToMeasure; x++) {
			histogramBuffer[x]=0;
	}
	for (indexInCoordinateMap = 0; indexInCoordinateMap < featureCount; indexInCoordinateMap++){
		disp = disparity_coordinates[(indexInCoordinateMap)*3+2];
		if (disp > disp_min && disp < maxDisparityToMeasure)
		{
			histogramBuffer[disp]++;
		}
	}
}
