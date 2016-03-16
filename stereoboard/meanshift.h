#ifndef _MEANSHIFT_H_
#define _MEANSHIFT_H_
#include <math.h>
#include "data_types.h"
/*
void meanshiftUpdate(uint8_t image[],int width, int height, int* trackPosX, int* trackPosY){
	int totalPosX=0;
	int totalPosY=0;
	int countedPoints=0;
	int trackingWidth = 40;
	int iterations=10;
	while(iterations>0){
		iterations--;
		// Search for non-empty places

		int startPosX = *trackPosX - trackingWidth/2;
		int endPosX = *trackPosX + trackingWidth/2;
		if(startPosX < 0){
			startPosX=0;
			endPosX = trackingWidth;
		}
		if(endPosX > width){
			endPosX=width;
			startPosX = width-trackingWidth;
		}


		int startPosY = *trackPosY - trackingWidth/2;
			int endPosY = *trackPosY + trackingWidth/2;
			if(startPosY < 0){
				startPosY=0;
				endPosY = trackingWidth;
			}
			if(endPosY > height){
				endPosY=height;
				startPosY = height-trackingWidth;
			}
		int xpos;
		int ypos;
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


void meanshiftUpdate(uint8_t_image disparity_image, int_rectangle* searchrectangle,float* distanceToObject){
	int totalPosX=0;
	int totalPosY=0;
	int countedPoints=0;
	int pixelsCounted = 0 ;
	int sumPixelsInSquare = 0;
	int iterations=10;
	while(iterations>0){
		sumPixelsInSquare = 0;
		pixelsCounted = 0;
		iterations--;

		// Set the window we search in
		int startPosX = searchrectangle->x - searchrectangle->width/2;
		int endPosX = searchrectangle->x + searchrectangle->width/2;
		if(startPosX < 0){
			startPosX=0;
			endPosX = searchrectangle->width;
		}
		if(endPosX > disparity_image.imageWidth){
			endPosX=disparity_image.imageWidth;
			startPosX = disparity_image.imageWidth-searchrectangle->width;
		}


		int startPosY = searchrectangle->y- searchrectangle->height/2;
		int endPosY = searchrectangle->y + searchrectangle->height/2;
		if(startPosY < 0){
			startPosY=0;
			endPosY = searchrectangle->height;
		}
		if(endPosY > disparity_image.imageHeight){
			endPosY=disparity_image.imageHeight;
			startPosY = disparity_image.imageHeight-searchrectangle->height;
		}


		// Init variables
		int xpos;
		int ypos;
		float M00=0.0;
		float M10=0.0;
		float M01=0.0;
		float M20=0.0;
		float M02=0.0;
		int searchWindowX;
		int searchWindowY;
		// Search for nonempty spaces, assign these spaces a probability
		for(searchWindowX = 0; searchWindowX < searchrectangle->width; searchWindowX++){
			for(searchWindowY =0 ; searchWindowY < searchrectangle->height;searchWindowY++){
				xpos = startPosX+searchWindowX;
				ypos = startPosY+searchWindowY;
				float probability=0.0;
				if(disparity_image.image[ypos*disparity_image.imageWidth+xpos]>0){
					probability = disparity_image.image[ypos*disparity_image.imageWidth+xpos]/(6.0*20.0);
					pixelsCounted++;
					sumPixelsInSquare+=disparity_image.image[ypos*disparity_image.imageWidth+xpos];
				}
				M00+=probability;
				M10+=searchWindowX*probability;
				M01+=searchWindowY*probability;

				M20+=searchWindowX*searchWindowX*probability;
				M02+=searchWindowY*searchWindowY*probability;
			}
		}

		if(M10>0 && M01 > 0){
			*distanceToObject = sumPixelsInSquare/pixelsCounted;
			float centerX = M10/M00;
			float centerY = M01/M00;
			searchrectangle->x = startPosX+centerX;
			searchrectangle->y = startPosY+centerY;

			float ratio = (M20/(centerX*centerX))/(M02/(centerY*centerY));
	//		*searchWindowWidth = 4*sqrtf(M00);//sqrtf(2*M00)*ratio;
	//		*searchWindowHeight=4*sqrtf(M00);//sqrtf(2*M00)/ratio;
			searchrectangle->width= 4.5*sqrtf(2.0*M00)*ratio;
			searchrectangle->height=4.5*sqrtf(2.0*M00)/ratio;

			if(searchrectangle->width < 10){
				searchrectangle->width=10;
			}
			if(searchrectangle->height< 10){
				searchrectangle->height=10;
			}

			if(searchrectangle->width> 80){
				searchrectangle->width=80;
					}
					if(searchrectangle->height > 80){
						searchrectangle->height=80;
					}
		}

	}
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


#endif
