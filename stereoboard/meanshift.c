
#include "meanshift.h"

#include <math.h>
#include "main_parameters.h"

/*
void meanshiftUpdate(uint8_t image[],uint16_t width, uint16_t height, uint16_t* trackPosX, uint16_t* trackPosY){
  uint16_t totalPosX=0;
  uint16_t totalPosY=0;
  uint16_t countedPoints=0;
  uint16_t trackingWidth = 40;
  uint16_t iterations=10;
  while(iterations>0){
    iterations--;
    // Search for non-empty places

    uint16_t startPosX = *trackPosX - trackingWidth/2;
    uint16_t endPosX = *trackPosX + trackingWidth/2;
    if(startPosX < 0){
      startPosX=0;
      endPosX = trackingWidth;
    }
    if(endPosX > width){
      endPosX=width;
      startPosX = width-trackingWidth;
    }


    uint16_t startPosY = *trackPosY - trackingWidth/2;
      uint16_t endPosY = *trackPosY + trackingWidth/2;
      if(startPosY < 0){
        startPosY=0;
        endPosY = trackingWidth;
      }
      if(endPosY > height){
        endPosY=height;
        startPosY = height-trackingWidth;
      }
    uint16_t xpos;
    uint16_t ypos;
    for(xpos=startPosX; xpos < endPosX; xpos++){
      for(ypos=startPosY; ypos < endPosY; ypos++){

    // For each cell in subpart
      // if non empty
        if(image[ypos*width+xpos]>0){
          totalPosX += xpos;
          totalPosY += ypos;
          countedPoints++;
        }
      }
    }

    if(countedPoints>0){
      float meanPosX = totalPosX/countedPoints;
      float meanPosY = totalPosY/countedPoints;





      *trackPosX = meanPosX;
      *trackPosY = meanPosY;
    }
    for(xpos=startPosX; xpos < endPosX; xpos++){
      for(ypos=startPosY; ypos < endPosY; ypos++){

        image[ypos*width+xpos]=128;
      }
    }
  }





}*/

static struct rectangle_i searchWindow = {.x = 50,.y = 50, .w = 30, .h = 30};

void updateSearchWindow(uint16_t x, uint16_t y, uint16_t w, uint16_t h)
{
	searchWindow.x = x;
	searchWindow.y = y;
	searchWindow.w = w;
	searchWindow.h = h;
}

void run_meanshift(struct image_i *disparity_image)
{
	//uint16_t* trackPosX, uint16_t* trackPosY, uint16_t* searchWindowWidth, uint16_t* searchWindowHeight
	float distanceToObject = meanshiftUpdate(disparity_image, &searchWindow);

	// Draw a square around the object we track
	uint16_t startPosX = searchWindow.x - searchWindow.w / 2;
	uint16_t endPosX = searchWindow.x + searchWindow.w / 2;
	if (startPosX < 0) {
	  startPosX = 0;
	  endPosX = searchWindow.w;
	}
	if (endPosX > IMAGE_WIDTH) {
	  endPosX = IMAGE_WIDTH;
	  startPosX = IMAGE_WIDTH - searchWindow.w;
	}

	uint16_t startPosY = searchWindow.y - searchWindow.h / 2;
	uint16_t endPosY = searchWindow.y + searchWindow.h / 2;
	if (startPosY < 0) {
	  startPosY = 0;
	  endPosY = searchWindow.h;
	}
	if (endPosY > IMAGE_HEIGHT) {
	  endPosY = IMAGE_HEIGHT;
	  startPosY = IMAGE_HEIGHT - searchWindow.h;
	}

	uint16_t xpos, ypos;
	for (xpos = startPosX; xpos < endPosX; xpos++) {
	  for (ypos = startPosY; ypos < endPosY; ypos++) {
	    disparity_image->image[ypos * IMAGE_WIDTH + xpos] = 128;
	  }
	}

	uint8_t meanshift_track[5];
	meanshift_track[0] = (uint8_t)searchWindow.x;
	meanshift_track[1] = (uint8_t)searchWindow.y;
	meanshift_track[2] = (uint8_t)distanceToObject;   // Kirk: this doesn't look right!
#ifdef SEND_MEANSHIFT
	SendArray(meanshift_track, 3, 1);
#endif
}

float meanshiftUpdate(struct image_i *disparity_image, struct rectangle_i *searchrectangle)
{
  uint16_t pixelsCounted = 0 ;
  uint16_t sumPixelsInSquare = 0;
  uint16_t iterations = 10;
  float distanceToObject = 0.;

  uint16_t startPosX, endPosX;
  uint16_t startPosY, endPosY;

  uint16_t xpos, ypos;
  float M00, M10, M01, M20, M02;
  float probability;
  uint16_t searchWindowX, searchWindowY;

  while (iterations-- > 0) {
    sumPixelsInSquare = 0;
    pixelsCounted = 0;

    // Set the window we search in
    startPosX = searchrectangle->x - searchrectangle->w / 2;
    endPosX = searchrectangle->x + searchrectangle->w / 2;
    if (startPosX < 0) {
      startPosX = 0;
      endPosX = searchrectangle->w;
    }
    if (endPosX > disparity_image->w) {
      endPosX = disparity_image->w;
      startPosX = disparity_image->w - searchrectangle->w;
    }

    startPosY = searchrectangle->y - searchrectangle->h / 2;
    endPosY = searchrectangle->y + searchrectangle->h / 2;
    if (startPosY < 0) {
      startPosY = 0;
      endPosY = searchrectangle->h;
    }
    if (endPosY > disparity_image->h) {
      endPosY = disparity_image->h;
      startPosY = disparity_image->h - searchrectangle->h;
    }

    // Init variables
    M00 = 0.0;
    M10 = 0.0;
    M01 = 0.0;
    M20 = 0.0;
    M02 = 0.0;
    // Search for nonempty spaces, assign these spaces a probability
    for (searchWindowX = 0; searchWindowX < searchrectangle->w; searchWindowX++) {
      for (searchWindowY = 0 ; searchWindowY < searchrectangle->h; searchWindowY++) {
        xpos = startPosX + searchWindowX;
        ypos = startPosY + searchWindowY;
        probability = 0.;
        if (disparity_image->image[ypos * disparity_image->w + xpos] > 0) {
          probability = disparity_image->image[ypos * disparity_image->w + xpos] / (6.0 * 20.);
          pixelsCounted++;
          sumPixelsInSquare += disparity_image->image[ypos * disparity_image->w + xpos];
        }
        M00 += probability;
        M10 += searchWindowX * probability;
        M01 += searchWindowY * probability;

        M20 += searchWindowX * searchWindowX * probability;
        M02 += searchWindowY * searchWindowY * probability;
      }
    }

    if (M10 > 0 && M01 > 0) {
      distanceToObject = sumPixelsInSquare / pixelsCounted;
      float centerX = M10 / M00;
      float centerY = M01 / M00;
      searchrectangle->x = startPosX + centerX;
      searchrectangle->y = startPosY + centerY;

      float ratio = (M20 / (centerX * centerX)) / (M02 / (centerY * centerY));
      searchrectangle->w = 4.5 * sqrtf(2. * M00) * ratio;
      searchrectangle->h = 4.5 * sqrtf(2. * M00) / ratio;

      if (searchrectangle->w < 10) {
        searchrectangle->w = 10;
      }
      if (searchrectangle->h < 10) {
        searchrectangle->h = 10;
      }

      if (searchrectangle->w > 80) {
        searchrectangle->w = 80;
      }
      if (searchrectangle->h > 80) {
        searchrectangle->h = 80;
      }
    }
  }
  return distanceToObject;
}


//          term_crit = ( cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_COUNT, 10, 1 );
//
//          ret, track_window = cv2.CamShift(img, track_window, term_crit)
//          print "ret: " , ret
//          print "Track window: ", track_window
//          x,y,w,h = track_window
//          pts = cv2.boxPoints(ret)
//          pts = np.int0(pts)
//          img = cv2.polylines(img,[pts],True, 255,2)
