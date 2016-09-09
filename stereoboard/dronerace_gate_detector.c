/*
 * dronerace_gate_detector.c
 *
 *  Created on: Sep 1, 2016
 *      Author: rmeertens
 */
#include "dronerace_gate_detector.h"
struct ValueAndPos {
	float value;
	int position;
};

#define INTEGRAL_IMAGE_WIDTH 128
#define INTEGRAL_IMAGE_HEIGHT 48
typedef struct { uint32_t data[INTEGRAL_IMAGE_WIDTH*INTEGRAL_IMAGE_WIDTH]; } integral_image_t;

uint32_t array_at(int x,int y, uint32_t *data,int width, int height){
	return data[y*width+x];
}

/**
 * Given an integral image, returns the average value in a certain area.
 */
float get_average_value_area_something(int min_x, int min_y, int max_x, int max_y,uint32_t *data) {
	int w = max_x - min_x + 1;
	int h = max_y - min_y + 1;
	float n_pix = w * h;
	int sum_disparities = array_at(min_x,min_y,data,INTEGRAL_IMAGE_WIDTH,INTEGRAL_IMAGE_HEIGHT)
			+ array_at(max_x,max_y,data,INTEGRAL_IMAGE_WIDTH,INTEGRAL_IMAGE_HEIGHT)
			- array_at(max_x,min_y,data,INTEGRAL_IMAGE_WIDTH,INTEGRAL_IMAGE_HEIGHT)
			- array_at(min_x,max_y,data,INTEGRAL_IMAGE_WIDTH,INTEGRAL_IMAGE_HEIGHT);
	float dd;
	if(n_pix>0){
		dd = sum_disparities / n_pix;
	}
	else{
		dd=0.0;
	}

	return dd;
}

void roland_integral(uint8_t *activationArray,uint32_t image_width, uint32_t image_height, uint32_t *my_int){
    // do the first line
    uint8_t *fl = activationArray;
    uint32_t *fli = my_int;
    int x=0;
    fli[0]=fl[0];
    for(x=0; x<image_width-1;x++){
        fli[1]=fl[1]+fli[0];
        fli++;
        fl++;
    }

    // do all other lines
    int line=0;
    for(line=1;line<image_height;line++){
        uint32_t *cur_l_int = my_int+image_width*line;
        uint8_t *cur_l_ac = activationArray + image_width*line;
        uint32_t *prev_l_int = my_int+image_width*(line-1);
        cur_l_int[0] = prev_l_int[0]+cur_l_ac[0];
        for(x=0;x<image_width-1;x++){
            cur_l_int[1]=x;
            cur_l_int[1]=cur_l_ac[1]+cur_l_int[0]+prev_l_int[1]-prev_l_int[0];
            cur_l_int++;
            cur_l_ac++;
            prev_l_int++;
        }
    }
}





/** Compare function for two ValueAndPos structs
 *  Looks at the value (not the pos) of the two structs.
 */
int compare_value_and_pos(const void *a, const void *b) {
	struct ValueAndPos *da = (struct ValueAndPos *) a;
	struct ValueAndPos *db = (struct ValueAndPos *) b;
	int val1 = da->value * 1000.0;
	int val2 = db->value * 1000.0;
	return val2 - val1;
}
/**
 * Smooths array in place by taking the average of smooth_dist values.
 * Single pass.
 * First smooth_dist elements are not smoothed
 */
static void smooth_array(float barray[], int length_array, int smooth_dist) {
	int j,x;
	for ( j = length_array - 1; j > smooth_dist; j--) {
		float sum = 0;
		for (x = 0; x < smooth_dist; x++) {
			sum += barray[j - x];
		}
		barray[j] = sum / (float)smooth_dist;
	}
}
int get_command_detection_gate(uint8_t *current_image_buffer){

	int width = 256;
	int height = 48;
	uint8_t activationArray[(width/2)*height]; // width * height/2
	uint8_t *image = current_image_buffer;
	image+=width;
	int line_i;
	int col_i;

	// Color segmentation on the even lines of the image
	int treshold_red_u=0;
	int treshold_red_v=210;
	uint8_t *act=activationArray;
	for(line_i=0;line_i<height;line_i++){
	  for(col_i=0;col_i<width;col_i+=4){
		  // UYVY
		 if(image[2]>=treshold_red_v){
#ifdef VISUALISE_ACTIVATION_DRONERACE
			act[0]=1;
			act[1]=1;
#endif
			act[0]=255;
			act[1]=255;
		  }
		  else{
			act[0]=0;
			act[1]=0;
		  }
		  image+=4;
		  act+=2;
	  }
	  image+=width;
	}

	int n_rows = height;
	int n_cols = width/2;

	// Get the integral image of the activated pixels
	// Roland note: used get_integral_image and roland_integral. Note sure if they give the same results.
	integral_image_t my_int;
	//		get_integral_image(activationArray,INTEGRAL_IMAGE_WIDTH, INTEGRAL_IMAGE_HEIGHT, my_int.data);
	roland_integral(activationArray,INTEGRAL_IMAGE_WIDTH,INTEGRAL_IMAGE_HEIGHT,my_int.data);
	act=activationArray;


	int x;
#ifdef VISUALISE_ACTIVATION_DRONERACE
	for(x=0;x<INTEGRAL_IMAGE_WIDTH*INTEGRAL_IMAGE_HEIGHT;x++){
		if(act[0]){
			act[0]=200;
		}
		act++;
	}
#endif

	//
	//		 Create an activation array.
	//		 Perform a center surround (but not really surround) square feature search over the whole image
	//		 And store the values in Ver
	int border_size = 8;
	int length_activation_array = n_cols - 3 * border_size;
	float horizontal_activation[length_activation_array];


	int border_y = 0; // borders can be used to filter out parts of the image that are irrelevant

	for (x = 0; x < length_activation_array; x++) {
	//			horizontal_activation[x]=get_average_value_area_something(x, 0, x+3*border_size, 48,my_int.data) ;
	//			horizontal_activation[x]=-1.0*get_average_value_area(x, border_y,x+3*border_size,n_rows-border_y, my_int);
		horizontal_activation[x] = -get_average_value_area_something(x, border_y,x+3*border_size,n_rows-border_y, my_int.data)
				+ 2 * get_average_value_area_something(x+border_size, border_y,x+2*border_size,n_rows-border_y, my_int.data);
	}


	// Smooth the array
	smooth_array(horizontal_activation, length_activation_array, 4);

	// find peaks by searching for places that are higher than their neighbours
	struct ValueAndPos values[length_activation_array];
	int amountPeaks = 0;
	int i=1;
	for (i = 1; i < length_activation_array - 1; i++) {
		if (horizontal_activation[i] > horizontal_activation[i - 1] && horizontal_activation[i] > horizontal_activation[i + 1]) {
			values[amountPeaks].position = i;
			values[amountPeaks].value = horizontal_activation[i];
			activationArray[i]=255;
			amountPeaks++;
		}
	}
	activationArray[0]=amountPeaks;

	int distance_pixels;
	int loc_y=n_rows / 2;

	// LOGIC WITH REGARDS TO THE DETECTED GATE
	if (amountPeaks >= 2) {
		// sort the peaks based on the height of the peak.
		qsort(values, amountPeaks, sizeof(struct ValueAndPos),
				compare_value_and_pos);
		float mean_vals = (values[0].value + values[1].value) / 2;
	//
	//    	  		// Check if this is indeed a window
	//    	  		// Correcsponds to the cross in Guido's MATLAB code.
	//    	  		// The treshold for
	//    	  		float tresholdActuallyGate = 0.4;
	//    	  		float some_sim = 1.0 - values[1].value / values[0].value;
	//    	  		if (mean_vals > 1.0
	//    	  				&& fabs(some_sim) < tresholdActuallyGate) {
	//    	  			for (int i = 0; i < 2; i++) {
	//    	  				line(probImage, Point(0, values[i].position),
	//    	  						Point(n_cols, values[i].position),
	//    	  						Scalar(255, 255, 255), 5);
	//    	  			}
	//
			int dist = values[0].position - values[1].position;
			if (dist < 0) {
				dist *= -1;
			}
			distance_pixels = dist;

			loc_y = (values[0].position + values[1].position) / 2;
			float left_height = values[0].value;
			float right_height = values[1].value;

			if (values[0].position > values[1].position) {
				left_height = values[1].value;
				right_height = values[0].value;
			}
	}
	else {
			loc_y = n_rows / 2;
		}
	uint8_t loc_y_u = loc_y;
	uint8_t locs[3];
	locs[0]=loc_y_u;
	locs[1]=amountPeaks;

	led_toggle();
	// TODO: maybe add a delay here!
	// TODO: sparse matching with a mask?
//	int processed_pixels = stereo_vision_sparse_block_two_sided(current_image_buffer,
//								 disparity_image.image, 10,
//								 disparity_min, disparity_range, disparity_step, thr1, thr2,
//								 min_y, max_y); // add a histogram to it, if you uncomment
	if(loc_y==length_activation_array/2){
		return 1; // immer geradeaus
	}
	else if(loc_y < length_activation_array){
		return 0; // left
	}
	else{
		return 2; // right
	}

#ifdef VISUALISE_ACTIVATION_DRONERACE
	SendArray(activationArray,width/2,height);
#endif
}


