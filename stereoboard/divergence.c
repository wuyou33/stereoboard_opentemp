/*
 * divergence.c
 *
 *  Created on: Apr 9, 2015
 *      Author: knmcguire
 */

#include "divergence.h"
#include "optic_flow.h"

#include <stdlib.h>
void divergence_total(uint8_t divergenceArray[],uint8_t current_image_buffer[], struct edgeflow_parameters_t* edgeflow_parameters, struct edgeflow_results_t* edgeflow_results,uint32_t time)
{
	edgeflow_results->edge_hist[edgeflow_results->current_frame_nr].frame_time = time;

	calculate_edge_flow(current_image_buffer, &edgeflow_results->displacement, &edgeflow_results->edge_flow, edgeflow_results->edge_hist, &edgeflow_results->avg_disp,
			edgeflow_results->previous_frame_offset, edgeflow_results->current_frame_nr, edgeflow_results->quality_measures_edgeflow, edgeflow_parameters->window_size, edgeflow_parameters->disparity_range, 0,
			IMAGE_WIDTH, IMAGE_HEIGHT,edgeflow_parameters->RES);


	edgeflow_results->hz_x = divergence_calc_vel(&edgeflow_results->vel_hor, &edgeflow_results->vel_ver, &edgeflow_results->avg_disp,
			&edgeflow_results->avg_dist, &edgeflow_results->prev_avg_dist, &edgeflow_results->covariance, &edgeflow_results->current_frame_nr,
			edgeflow_results->previous_frame_offset, edgeflow_results->edge_hist, &edgeflow_results->edge_flow, &edgeflow_results->hz_x,
			&edgeflow_results->hz_y, &edgeflow_results->prev_vel_hor, &edgeflow_results->prev_vel_ver, &edgeflow_results->prev_edge_flow, edgeflow_parameters->Q, edgeflow_parameters->R,
			edgeflow_parameters->FOVX, edgeflow_parameters->FOVY, edgeflow_parameters->RES, edgeflow_parameters->use_monocam);

	divergence_to_sendarray(divergenceArray, &edgeflow_results->edge_flow,
			edgeflow_results->previous_frame_offset, edgeflow_results->avg_dist, edgeflow_results->hz_x,
			edgeflow_results->vel_hor, edgeflow_results->vel_ver, edgeflow_results->quality_measures_edgeflow);
}
void divergence_init(struct edgeflow_parameters_t* edgeflow_parameters, struct edgeflow_results_t* edgeflow_results,
		const int8_t FOVX, const int8_t FOVY, int8_t image_width, int8_t image_height, int8_t use_monocam)
{
	edgeflow_parameters->FOVX = FOVX;
	edgeflow_parameters->FOVY = FOVY;
	edgeflow_parameters->image_height = IMAGE_HEIGHT;
	edgeflow_parameters->image_width = IMAGE_WIDTH;
	edgeflow_parameters->Q = 50;
	edgeflow_parameters->R = 100;
	edgeflow_parameters->max_horizon = MAX_HORIZON;
	edgeflow_parameters->max_disparity_range = DISP_RANGE_MAX;
	edgeflow_parameters->disparity_range = 20;
	edgeflow_parameters->window_size = 10;
	edgeflow_parameters->edge_flow_kalman = 1;
	edgeflow_parameters->RES = 100;
	edgeflow_parameters->use_monocam = use_monocam;

	edgeflow_results->R_height = 100;
	edgeflow_results->R_hor = 100;
	edgeflow_results->R_ver = 100;
	edgeflow_results->current_frame_nr = 0;
	edgeflow_results->previous_frame_offset[0] = 1;
	edgeflow_results->previous_frame_offset[1] = 1;
	edgeflow_results->edge_flow.horizontal_flow =  edgeflow_results->prev_edge_flow.horizontal_flow = 0;
	edgeflow_results->edge_flow.horizontal_div =  edgeflow_results->prev_edge_flow.horizontal_div = 0;
	edgeflow_results->edge_flow.vertical_flow =  edgeflow_results->prev_edge_flow.vertical_flow = 0;
	edgeflow_results->edge_flow.vertical_div =  edgeflow_results->prev_edge_flow.vertical_div = 0;

	edgeflow_results->covariance.flow_x = 20;
	edgeflow_results->covariance.flow_y = 20;
	edgeflow_results->covariance.div_x = 20;
	edgeflow_results->covariance.div_y = 20;
	edgeflow_results->covariance.height = 20;

	edgeflow_results->avg_dist = 0;
	edgeflow_results->avg_disp = 0;
	edgeflow_results->prev_avg_dist = 0;
	edgeflow_results->vel_hor = 0;
	edgeflow_results->prev_vel_hor = 0;
	edgeflow_results->vel_ver = 0;
	edgeflow_results->prev_vel_ver = 0;
}
void divergence_to_sendarray(uint8_t divergenceArray[24],
		const struct edge_flow_t* edge_flow, uint8_t previous_frame_offset[2],
		int32_t avg_dist, int32_t hz_x, int32_t vel_hor,
		int32_t vel_ver,
		uint8_t quality_measures_edgeflow[])
{
	divergenceArray[0] = boundint8(edge_flow->horizontal_div
			/ previous_frame_offset[0] + 127); // should be in 0.01px/frame
	divergenceArray[1] = boundint8(edge_flow->horizontal_flow
			/ (10 * previous_frame_offset[0]) + 127); // should be in 0.1px/frame
	divergenceArray[2] = boundint8(edge_flow->vertical_div
			/ previous_frame_offset[1] + 127); // should be in 0.01px/frame
	divergenceArray[3] = boundint8(edge_flow->vertical_flow
			/ (10 * previous_frame_offset[1]) + 127); // should be in 0.1px/frame
	// divergenceArray[4] = (uint8_t)avg_disp;
	divergenceArray[4] = boundint8(avg_dist / 10);
	memcpy(divergenceArray + 5, previous_frame_offset, 2); // copy frame offset to output array
	divergenceArray[7] = boundint8(hz_x);
	divergenceArray[8] = boundint8(vel_hor + 127); // in cm/s
	divergenceArray[9] = boundint8(vel_ver + 127); // in cm/s
	memcpy(divergenceArray + 10, quality_measures_edgeflow,
			10 * sizeof(uint8_t)); // copy quality measures to output array
}

int32_t divergence_calc_vel(int32_t* vel_hor, int32_t* vel_ver,
		int32_t * avg_disp, int32_t * avg_dist, int32_t * prev_avg_dist,
		struct covariance_t* covariance, uint8_t * current_frame_nr,
		uint8_t previous_frame_offset[2],
		struct edge_hist_t edge_hist[MAX_HORIZON],
		const struct edge_flow_t* edge_flow, int32_t *hz_x, int32_t *hz_y,
		int32_t * prev_vel_hor, int32_t * prev_vel_ver,
		struct edge_flow_t* prev_edge_flow, const uint32_t Q, const uint32_t R,
		const int8_t FOVX, const int8_t FOVY, const int32_t RES, const int32_t monocam)
{
	// disparity to distance in dm given 6cm dist between cams and Field of View (FOV) of 60deg
	// d =  Npix*cam_separation /(2*disp*tan(FOV/2))
	// d = 0.06*128 / (2*tan(disp*1.042/2))
	// d = 0.06*128 / (2*disp*1.042/2)
	// d = RES*0.06*128 / (disp*RES*1.042)
	// d = RES*0.06*PIX / (disp*FOVX)
	if (avg_disp > 0) {
		*avg_dist = RES * 3 * IMAGE_WIDTH / (*avg_disp * FOVX);
	} else {
		*avg_dist = 100; // 2 * RES * 6 * IMAGE_WIDTH / 104;
	}
	if (monocam) {
		*avg_dist = 1.0;
	}
	//filter height: TODO: use height directly from lisa s
	*avg_dist = simpleKalmanFilter(&(covariance->height), *prev_avg_dist,
			*avg_dist, Q, R, RES);
	//store the time of the frame
	// Calculate velocity TODO:: Find a way to use extract function correctly for this, since the results is different with than without
	*hz_x = 2000
			/ (edge_hist[*current_frame_nr].frame_time
					- edge_hist[(*current_frame_nr - previous_frame_offset[0]
					                                                       + MAX_HORIZON) %
					            MAX_HORIZON].frame_time);
	*hz_y = 2000
			/ (edge_hist[*current_frame_nr].frame_time
					- edge_hist[(*current_frame_nr - previous_frame_offset[1]
					                                                       + MAX_HORIZON) %
					            MAX_HORIZON].frame_time);
	*vel_hor = edge_flow->horizontal_flow * (*avg_dist) * (*hz_x) * FOVX
			/ (RES * RES * IMAGE_WIDTH);
	*vel_ver = edge_flow->vertical_flow * (*avg_dist) * (*hz_y) * FOVY
			/ (RES * RES * IMAGE_HEIGHT);
	*vel_hor = simpleKalmanFilter(&(covariance->flow_x), *prev_vel_hor, *vel_hor,
			Q, R, RES);
	*vel_ver = simpleKalmanFilter(&(covariance->flow_y), *prev_vel_ver, *vel_ver,
			Q, R, RES);
	// Store previous values
	*prev_avg_dist = *avg_dist;
	*prev_vel_hor = *vel_hor;
	*prev_vel_ver = *vel_ver;
	// move the indices for the edge hist structure
	* current_frame_nr = (*current_frame_nr + 1) % MAX_HORIZON;
	return *hz_x;
}
void calculate_edge_flow(uint8_t in[], struct displacement_t *displacement, struct edge_flow_t *edge_flow,
		struct edge_hist_t edge_hist[], int32_t *avg_disp, uint8_t previous_frame_offset[],
		uint8_t current_frame_nr, uint8_t quality_measures[], uint8_t window_size, uint8_t disp_range, uint16_t edge_threshold,
		uint16_t image_width, uint16_t image_height, uint16_t RES)
{
	// check that inputs within allowable ranges
	if (disp_range > DISP_RANGE_MAX) {
		disp_range = DISP_RANGE_MAX;
	}

	// Define arrays and pointers for edge histogram and displacements
	int32_t *edge_histogram_x = edge_hist[current_frame_nr].horizontal;
	int32_t *prev_edge_histogram_x;
	int32_t edge_histogram_x_right[IMAGE_WIDTH];

	int32_t *edge_histogram_y = edge_hist[current_frame_nr].vertical;
	int32_t *prev_edge_histogram_y;

	// Calculate previous frame number
	//previous_frame_offset[0] = previous_frame_offset[1] = 1;

	// TODO confirm below
	if (MAX_HORIZON > 2) {
		uint32_t flow_mag_x, flow_mag_y;
		flow_mag_x = abs(edge_flow->horizontal_flow);
		flow_mag_y = abs(edge_flow->vertical_flow);

		// TODO check which image we should pick
		// TODO I think you should switch when you go over the RES / flow_mag_x/(disparity_range/some_size) boundary
		// TODO I currently use a switching limit of disparity range/4
		/* if (4 * flow_mag_x * (MAX_HORIZON - 1) > RES * disp_range) {
        previous_frame_offset[0] = (RES * disp_range) / (4 * flow_mag_x) + 1;
      } else {
        previous_frame_offset[0] = MAX_HORIZON - 1;
      }

      if (4 * flow_mag_y * (MAX_HORIZON - 1) > RES * disp_range) {
        previous_frame_offset[1] = (RES * disp_range) / (4 * flow_mag_y) + 1;
      } else {
        previous_frame_offset[1] = MAX_HORIZON - 1;
      }*/
		uint32_t min_flow = 3;
		uint32_t max_flow = 18;
		uint8_t previous_frame_offset_x = previous_frame_offset[0];
		uint8_t previous_frame_offset_y = previous_frame_offset[1];

		if (flow_mag_x > max_flow && previous_frame_offset_x > 1) {
			previous_frame_offset[0] = previous_frame_offset_x - 1;
		}

		if (flow_mag_x < min_flow && previous_frame_offset_x < MAX_HORIZON - 1) {
			previous_frame_offset[0] = previous_frame_offset_x + 1;
		}


		if (flow_mag_y > max_flow && previous_frame_offset_y > 1) {
			previous_frame_offset[1] = previous_frame_offset_y - 1;
		}

		if (flow_mag_y < min_flow && previous_frame_offset_y < MAX_HORIZON - 1) {
			previous_frame_offset[1] = previous_frame_offset_y + 1;
		}
	}


	// the previous frame number relative to dynamic parameters
	uint8_t previous_frame_x = (current_frame_nr - previous_frame_offset[0] + MAX_HORIZON) %
			MAX_HORIZON; // wrap index
	uint8_t previous_frame_y = (current_frame_nr - previous_frame_offset[1] + MAX_HORIZON) %
			MAX_HORIZON; // wrap index

	// copy previous edge histogram based on previous frame number
	prev_edge_histogram_x = edge_hist[previous_frame_x].horizontal;
	prev_edge_histogram_y = edge_hist[previous_frame_y].vertical;

	// Calculate Edge Histogram
	calculate_edge_histogram(in, edge_histogram_x, image_width, image_height, 'x', 'l', edge_threshold);
	calculate_edge_histogram(in, edge_histogram_y, image_width, image_height, 'y', 'l', edge_threshold);
	calculate_edge_histogram(in, edge_histogram_x_right, image_width, image_height, 'x', 'r', edge_threshold);

	// Calculate displacement
	uint32_t min_error_hor = calculate_displacement(edge_histogram_x, prev_edge_histogram_x, displacement->horizontal,
			image_width, window_size,
			disp_range);
	uint32_t min_error_ver = calculate_displacement(edge_histogram_y, prev_edge_histogram_y, displacement->vertical,
			image_height, window_size,
			disp_range);
	*avg_disp = calculate_displacement_fullimage(edge_histogram_x, edge_histogram_x_right, image_width, disp_range);

	// Fit a linear line
	uint32_t line_error_fit_hor = line_fit(displacement->horizontal, &edge_flow->horizontal_div,
			&edge_flow->horizontal_flow, image_width,
			window_size + disp_range, RES);
	uint32_t line_error_fit_ver = line_fit(displacement->vertical, &edge_flow->vertical_div, &edge_flow->vertical_flow,
			image_height,
			window_size + disp_range, RES);

	//Calculate and Store quality values
	uint32_t totalIntensity = getTotalIntensityImage(in, image_width, image_height);
	uint32_t mean_hor = getMean(edge_histogram_x, image_width);
	uint32_t mean_ver = getMean(edge_histogram_y, image_height);
	uint32_t median_hor = getMedian(edge_histogram_x, image_width);
	uint32_t median_ver = getMedian(edge_histogram_y, image_height);
	uint32_t amountPeaks_hor = getAmountPeaks(edge_histogram_x, 500 , image_width);
	uint32_t amountPeaks_ver = getAmountPeaks(edge_histogram_y, 500 , image_height);


	quality_measures[0] = boundint8(totalIntensity / 20000);
	quality_measures[1] = boundint8(mean_hor / 20);
	quality_measures[2] = boundint8(mean_ver / 20);
	quality_measures[3] = boundint8(median_hor / 20);
	quality_measures[4] = boundint8(median_ver / 20);
	quality_measures[5] = boundint8(amountPeaks_hor);
	quality_measures[6] = boundint8(amountPeaks_ver);
	quality_measures[7] = boundint8(line_error_fit_hor / 10);
	quality_measures[8] = boundint8(line_error_fit_ver / 10);

	int32_t* pointer = (int32_t*)quality_measures+9;
	pointer[0]=min_error_hor;
	pointer[1]=min_error_ver;
}

// calculate_edge_histogram calculates the image gradient of the images and makes a edge feature histogram
void calculate_edge_histogram(uint8_t *in, int32_t *edge_histogram, uint16_t image_width, uint16_t image_height,
		char direction, char side, uint16_t edge_threshold)
{
	// TODO use arm_conv_q31()
	int32_t sobel_sum = 0;
	int32_t Sobel[3] = { -1, 0, 1};

	uint32_t y = 0, x = 0;
	int32_t c = 0;

	uint32_t idx = 0;

	// set pixel offset based on which image needed from interlaced image
	uint32_t px_offset = 0;
	if (side == 'l') {
		px_offset = 0;
	} else if (side == 'r') {
		px_offset = 1;
	} else
		while (1); // let user know something is wrong

	// compute edge histogram
	if (direction == 'x') {
		// set values that are not visited
		edge_histogram[0] = edge_histogram[image_width - 1] = 0;
		for (x = 1; x < image_width - 1; x++) {
			edge_histogram[x] = 0;
			for (y = 0; y < image_height; y++) {
				sobel_sum = 0;

				for (c = -1; c <= 1; c++) {
					idx = 2 * (image_width * y + (x + c)); // 2 for interlace

					sobel_sum += Sobel[c + 1] * (int32_t)in[idx + px_offset];
				}
				sobel_sum = abs(sobel_sum);
				if (sobel_sum > edge_threshold) {
					edge_histogram[x] += sobel_sum;
				}
			}
		}
	} else if (direction == 'y') {
		// set values that are not visited
		edge_histogram[0] = edge_histogram[image_height - 1] = 0;
		for (y = 1; y < image_height - 1; y++) {
			edge_histogram[y] = 0;
			for (x = 0; x < image_width; x++) {
				sobel_sum = 0;

				for (c = -1; c <= 1; c++) {
					idx = 2 * (image_width * (y + c) + x); // 2 for interlace

					sobel_sum += Sobel[c + 1] * (int32_t)in[idx + px_offset];
				}
				sobel_sum = abs(sobel_sum);
				if (sobel_sum > edge_threshold) {
					edge_histogram[y] += sobel_sum;
				}
			}
		}
	} else
		while (1);  // hang to show user something isn't right
}

// Calculate_displacement calculates the displacement between two histograms
// D should be half the search disparity range
// W is local search window
uint32_t calculate_displacement(int32_t *edge_histogram, int32_t *edge_histogram_prev, int32_t *displacement,
		uint16_t size,
		uint8_t window, uint8_t disp_range)
{
	int32_t c = 0, r = 0;
	uint32_t x = 0;
	uint32_t SAD_temp[2 * DISP_RANGE_MAX + 1]; // size must be at least 2*D + 1

	int32_t W = window;
	int32_t D = disp_range;
	uint32_t min_error = 0;
	uint32_t min_error_tot = 0;

	memset(displacement, 0, size);

	// TODO: replace with arm offset subtract
	for (x = W + D; x < size - W - D; x++) {
		displacement[x] = 0;
		for (c = -D; c <= D; c++) {
			SAD_temp[c + D] = 0;
			for (r = -W; r <= W; r++) {
				SAD_temp[c + D] += abs(edge_histogram[x + r] - edge_histogram_prev[x + r + c]);
			}
		}
		displacement[x] = (int32_t)getMinimum2(SAD_temp, 2 * D + 1, &min_error) - D;
		min_error_tot += min_error;
	}
	return min_error_tot;
}

// Calculate_displacement calculates the displacement between two histograms
// D should be disparity range
// TODO: for height should always look to the positive disparity range, can ignore negative
int32_t calculate_displacement_fullimage(int32_t *edge_histogram, int32_t *edge_histogram_2, uint16_t size,
		uint8_t disp_range)
{
	int32_t c = 0;
	uint32_t x = 0;
	uint32_t SAD_temp[2 * DISP_RANGE_MAX + 1]; // size must be at least 2*D + 1

	int32_t D = disp_range;

	// TODO check if one loop can be replaced by line diff
	for (c = -D; c <= D; c++) {
		SAD_temp[c + D] = 0;
		for (x = D; x < size - D; x++) {
			SAD_temp[c + D] += abs(edge_histogram[x] - edge_histogram_2[x + c]);
		}
	}

	return D - (int32_t)getMinimum(SAD_temp, 2 * D + 1);
}


// Line_fit fits a line using least squares to the histogram disparity map
uint32_t line_fit(int32_t *displacement, int32_t *divergence, int32_t *flow, uint32_t size, uint32_t border,
		uint16_t RES)
{
	int32_t x;

	int32_t count = 0;
	int32_t sumY = 0;
	int32_t sumX = 0;
	int32_t sumX2 = 0;
	int32_t sumXY = 0;
	int32_t xMean = 0;
	int32_t yMean = 0;
	int32_t divergence_int = 0;
	int32_t border_int = (int32_t)border;
	int32_t size_int = (int32_t)size;
	uint32_t total_error = 0;

	*divergence = 0;
	*flow = 0;

	// compute fixed sums
	int32_t xend = size_int - border_int - 1;
	sumX = xend * (xend + 1) / 2 - border_int * (border_int + 1) / 2 + border_int;
	sumX2 = xend * (xend + 1) * (2 * xend + 1) / 6;
	xMean = (size_int - 1) / 2;
	count = size_int - 2 * border_int;

	for (x = border_int; x < size - border_int; x++) {
		sumY += displacement[x];
		sumXY += x * displacement[x];
	}

	yMean = RES * sumY / count;

	divergence_int = (RES * sumXY - sumX * yMean) / (sumX2 - sumX * xMean);    // compute slope of line ax + b
	*divergence = divergence_int;
	*flow = yMean - *divergence * xMean;  // compute b (or y) intercept of line ax + b

	for (x = border_int; x < size - border_int; x++) {
		total_error += abs(RES * displacement[x] - divergence_int * x + yMean);
	}

	return total_error / size;
}

int ipow(int base, int exp)
{
	int result = 1;
	while (exp) {
		if (exp & 1) {
			result *= base;
		}
		exp >>= 1;
		base *= base;
	}

	return result;
}

void line_fit_RANSAC(int32_t *displacement, int32_t *divergence, int32_t *flow, uint16_t size, uint32_t border,
		uint32_t RES)
{
	//Fit a linear line with RANSAC (from Guido's code)
	uint8_t ransac_iter = 20;
	int32_t it;
	uint32_t ind1, ind2, tmp, entry;
	uint32_t total_error, best_ind;
	int32_t dx, dflow, predicted_flow;
	// flow = a * x + b
	int32_t a[ransac_iter];
	int32_t b[ransac_iter];
	uint32_t errors[ransac_iter];

	uint16_t entries = size - 2 * border;

	for (it = 0; it < ransac_iter; it++) {
		ind1 = rand() % entries + border;
		ind2 = rand() % entries + border;

		while (ind1 == ind2) {
			ind2 = rand() % entries + border;
		}

		// TODO: is this really necessary?
		if (ind1 > ind2) {
			tmp = ind2;
			ind2 = ind1;
			ind1 = tmp;
		}

		dx = ind2 - ind1;   // never zero
		dflow = displacement[ind2] - displacement[ind1];

		// Fit line with two points
		a[it] = RES * dflow / dx;
		b[it] = RES * displacement[ind1] - (a[it] * ind1);
		// evaluate fit:
		total_error = 0;
		for (entry = border; entry < size - border; entry++) {
			predicted_flow = a[it] * entry + b[it];
			//total_error += ipow(RES*displacement[entry] - predicted_flow,2);
			total_error += RES * displacement[entry] - predicted_flow;
		}
		errors[it] = total_error;
	}
	// select best fit:
	best_ind = getMinimum(errors, ransac_iter);

	*divergence = a[best_ind];
	*flow = b[best_ind];
}

void totalKalmanFilter(struct covariance_t *coveriance, struct edge_flow_t *prev_edge_flow,
		struct edge_flow_t *edge_flow, uint32_t Q, uint32_t R, uint32_t RES)
{
	edge_flow->horizontal_flow = simpleKalmanFilter(&(coveriance->flow_x), prev_edge_flow->horizontal_flow,
			edge_flow->horizontal_flow, Q, R, RES);
	edge_flow->vertical_flow = simpleKalmanFilter(&(coveriance->flow_y), prev_edge_flow->vertical_flow,
			edge_flow->vertical_flow, Q, R, RES);
	edge_flow->horizontal_div = simpleKalmanFilter(&(coveriance->div_x), prev_edge_flow->horizontal_div,
			edge_flow->horizontal_div, Q, R, RES);
	edge_flow->vertical_div = simpleKalmanFilter(&(coveriance->div_y), prev_edge_flow->vertical_div,
			edge_flow->vertical_div, Q, R, RES);
}

int32_t simpleKalmanFilter(int32_t *cov, int32_t previous_est, int32_t current_meas, int32_t Q, int32_t R, int32_t RES)
{
	int32_t predict_cov = *cov + Q;
	int32_t K = RES * predict_cov / (*cov + R);

	*cov = ((RES - K) * predict_cov) / RES;

	return (previous_est + (K * (current_meas - previous_est)) / RES);
}


//This function is a visualization tool which visualizes the Edge filter in the one image and the histogram disparity with line fit in the second.
void visualize_divergence(uint8_t *in, int32_t *displacement, int32_t slope, int32_t yInt, uint32_t image_width,
		uint32_t image_height)
{
	uint32_t y = 0;
	uint32_t x = 0;
	uint32_t idx = 0;

	for (y = 0; y < image_height; y++) {
		//line_check1=(uint32_t)(Slope*(float)x+(Yint)+(float)image_height/2);
		//line_check2=(uint32_t)(displacement[x]+image_height/2);
		for (x = 0; x < image_width; x++) {
			idx =  2 * (image_width * y + x);

			/*if(y==line_check1)
        out[idx]=255;
      else if(y==line_check2)
        out[idx]=100;
      else
        out[idx]=0;*/

			in[idx] = 100;
			in[idx + 1] = 0; //in[idx+1];
		}
	}
}

// Small supporting functions

uint32_t getMinimum2(uint32_t *a, uint32_t n, uint32_t *min_error)
{
	uint32_t i;
	uint32_t min_ind = 0;
	uint32_t min_err = a[min_ind];
	for (i = 1; i < n; i++) {
		if (a[i] <= min_err) {
			min_ind = i;
			min_err = a[i];
		}
	}
	*min_error = min_err;
	return min_ind;
}

uint32_t getMaximum(uint32_t *a, uint32_t n)
{
	uint32_t c;
	uint32_t max_error = a[0];
	uint32_t max_ind = 0;

	for (c = 1; c < n; c++) {
		if (a[c] > max_error) {
			max_ind = c;
			max_error = a[c];
		}
	}
	return max_ind;
}

uint32_t getMedian(int32_t *daArray, int32_t iSize)
{
	// Allocate an array of the same size and sort it.
	int32_t i, j;

	int dpSorted[iSize];
	for (i = 0; i < iSize; ++i) {
		dpSorted[i] = daArray[i];
	}
	for (i = iSize - 1; i > 0; --i) {
		for (j = 0; j < i; ++j) {
			if (dpSorted[j] > dpSorted[j + 1]) {
				int32_t dTemp = dpSorted[j];
				dpSorted[j] = dpSorted[j + 1];
				dpSorted[j + 1] = dTemp;
			}
		}
	}

	// Middle or average of middle values in the sorted array.
	int32_t dMedian = 0;
	if ((iSize % 2) == 0) {
		dMedian = (dpSorted[iSize / 2] + dpSorted[(iSize / 2) - 1]) / 2.0;
	} else {
		dMedian = dpSorted[iSize / 2];
	}
	return dMedian;
}

uint32_t getMean(int32_t *daArray, int32_t iSize)
{
	int32_t dSum = daArray[0];
	int32_t i;
	for (i = 1; i < iSize; ++i) {
		dSum += daArray[i];
	}
	return dSum / iSize;
}

uint32_t getTotalIntensityImage(uint8_t *in, uint32_t image_height, uint32_t image_width)
{

	uint32_t y = 0, x = 0;
	uint32_t idx = 0;
	uint32_t px_offset = 0;
	uint32_t totalIntensity = 0;
	for (x = 1; x < image_width - 1; x++) {
		for (y = 0; y < image_height; y++) {
			idx = 2 * (image_width * y + (x)); // 2 for interlace


			totalIntensity += (uint32_t)in[idx + px_offset];
		}
	}
	return totalIntensity;
}

uint32_t getAmountPeaks(int32_t *edgehist, uint32_t median, int32_t size)
{
	uint32_t  amountPeaks = 0;
	uint32_t i = 0;

	for (i = 1; i < size + 1;  i ++) {
		if (edgehist[i - 1] < edgehist[i] && edgehist[i] > edgehist[i + 1] && edgehist[i] > median) {
			amountPeaks ++;
		}
	}
	return amountPeaks;
}

uint8_t boundint8(int32_t value)
{
	uint8_t value_uint8;
	if(value > 255)
	{
		value_uint8 = 255;
	}else if (value < 0)
	{
		value_uint8 = 0;
	}else
		value_uint8 =  (uint8_t)value;

	return value_uint8;
}



