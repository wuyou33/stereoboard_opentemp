/*
 * radar.h
 *
 *  Created on: May 13, 2015
 *      Author: mavlab
 */
/*
#ifndef RADAR_H_
#define RADAR_H_


#include "parameters.h"

// Settings for the matrix
//uint8_t horizontalBins=4;
//uint8_t verticalBins=4;

// Settings of the camera... used by the distance matrix algorithm
uint8_t blackBorderSize=22;
uint8_t pixelsPerLine = 128;
uint8_t pixelsPerColumn = 96;

// Settings for the depth matrix algorithm, calculated based on other settings
uint8_t widthPerBin =(pixelsPerLine-2*blackBorderSize)/horizontalBins;
uint8_t heightPerBin = pixelsPerColumn/verticalBins;

// Initialise matrixbuffer
int matrixBuffer[verticalBins*horizontalBins];
uint8_t toSendBuffer[verticalBins*horizontalBins];




void calculateDistanceMatrix(uint8_t* disparity_image_buffer_8bit,int* matrixBuffer,uint8_t horizontalBins,uint8_t verticalBins,uint8_t blackBorderSize,uint8_t pixelsPerLine,uint8_t widthPerBin,uint8_t heightPerBin)
{

  int indexBuffer;

  uint8_t y;
  uint8_t x;
  for(x=0; x < horizontalBins; x++)
  {
    for(y=0;y<verticalBins;y++)
    {
      int line;
      for (line=0; line < heightPerBin;line++)
      {
        int bufferIndex=0;
        for(bufferIndex=0; bufferIndex < widthPerBin; bufferIndex++)
        {
          matrixBuffer[y*horizontalBins+x]+=disparity_image_buffer_8bit[pixelsPerLine*(y*heightPerBin)+line*pixelsPerLine+widthPerBin*x+blackBorderSize+bufferIndex];
        }
      }
    }
  }

}

void radar_run(void)
{
  // Determine disparities:
  min_y = 0;
  max_y = 95;
  stereo_vision_Kirk(current_image_buffer, disparity_image_buffer_8bit, image_width, image_height, disparity_min, disparity_range, disparity_step, thr1, thr2, min_y, max_y);

  // Initialise matrixbuffer by setting all values back to zero.
  int bufferIndex = 0;
  for(bufferIndex=0; bufferIndex < verticalBins*horizontalBins; bufferIndex+=1)
  {
    matrixBuffer[bufferIndex] = 0;
    toSendBuffer[bufferIndex]=0;
  }

  // Create the distance matrix by summing pixels per bin
  calculateDistanceMatrix(disparity_image_buffer_8bit,matrixBuffer,horizontalBins,verticalBins,blackBorderSize,pixelsPerLine,widthPerBin,heightPerBin);


  // Average by dividing by the amount of pixels per bin

  for(bufferIndex=0;bufferIndex < horizontalBins*verticalBins; bufferIndex++)
  {
    //toSendBuffer[bufferIndex]=matrixBuffer[bufferIndex]/(widthPerBin*heightPerBin);
    toSendBuffer[bufferIndex]=1;//matrixBuffer[bufferIndex]/265;
  }

  // Send the matrix
  SendMatrix(toSendBuffer);
}

#endif /* RADAR_H_ */
