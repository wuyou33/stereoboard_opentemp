/*
 * odometry.c
 *
 *  Created on: Dec 21, 2015
 *      Author: Sjoerd
 */


#include "odometry.h"

void odometry_select_features( uint8_t *in, uint8_t *features, uint16_t feature_count, uint16_t feature_number_select, uint32_t image_width, uint32_t image_height ){

	volatile uint8_t border = 15;

	volatile uint8_t left_border = border;
	volatile uint8_t right_border = image_width - border;
	volatile uint8_t top_border = border;
	volatile uint8_t bottom_border = image_height - border;

	volatile uint16_t step_size = feature_count/(feature_number_select+1);
	volatile uint16_t select_count = 0;
	volatile uint16_t feature_index = 0;
	volatile uint16_t feature_index_2 = 0;
	volatile uint8_t finding_feature = 0;

	volatile uint8_t data = 3;
	volatile uint8_t temp;


	volatile int i = 0;
	volatile int j = 0;

	for ( i = 0; i<feature_count && select_count<feature_number_select; i+= step_size)
	{
		feature_index = data*i;

		finding_feature = 1;
		while ( finding_feature==1 )
		{
			if ( (features[feature_index] > top_border) &&  (features[feature_index] < bottom_border) &&
					(features[feature_index+1] > left_border) &&  (features[feature_index+1] < right_border) )
			{
				finding_feature = 0;
				if ( i!= select_count )
				{
					feature_index_2 = data*select_count;

					for ( j = 0; j<data; j++ )
					{
						temp =  features[feature_index+j];
						features[feature_index+j]   =  features[feature_index_2+j];
						features[feature_index_2+j]   =  temp;

					}

					select_count++;
				}
			}
			feature_index++;
		}
	}
}

void odometry_extract_features( uint8_t *in, q15_t *out, uint8_t *features, uint16_t feature_count, uint16_t feature_number_select, uint32_t image_width, uint32_t image_height, uint8_t feature_window_size_half ){

	volatile uint8_t border = 15;

	volatile uint8_t left_border = border;
	volatile uint8_t right_border = image_width - border;
	volatile uint8_t top_border = border;
	volatile uint8_t bottom_border = image_height - border;

	volatile uint16_t step_size = feature_count/(feature_number_select+1);
	volatile uint16_t select_count = 0;
	volatile uint16_t feature_index = 0;
	volatile uint16_t feature_index_2 = 0;
	volatile uint8_t finding_feature = 0;

	volatile uint8_t data = 3; // data about features: [x y d]
	volatile uint8_t temp;
	volatile uint8_t window_size = (feature_window_size_half*2)+1;
	volatile uint16_t image_width_bytes = image_width*2;


	volatile int i = 0;
	volatile int j = 0;
	volatile int row = 0;
	volatile int col = 0;
	volatile int row_center = 0;
	volatile int col_center_right = 0;
	volatile int col_center = 0;
	volatile int index = 0;
	volatile int location;




	for ( i = 0; i<feature_count && select_count<feature_number_select; i+= step_size)
	{
		feature_index = data*i;

		finding_feature = 1;
		while ( finding_feature==1 )
		{
			if ( (features[feature_index] > top_border) &&  (features[feature_index] < bottom_border) &&
					(features[feature_index+1] > left_border) &&  (features[feature_index+1] < right_border) )
			{
				finding_feature = 0;
				if ( i!= select_count )
				{
					feature_index_2 = data*select_count;

					for ( j = 0; j<data; j++ )
					{
						temp =  features[feature_index+j];
						features[feature_index+j]   =  features[feature_index_2+j];
						features[feature_index_2+j]   =  temp;

					}

					select_count++;
				}
			}
			feature_index++;
		}
	}

	for ( i = 0; i< feature_count; i++)
	{
		feature_index = data*i;
		row_center = features[feature_index];
		col_center = features[feature_index+1];
		col_center_right = col_center + (features[feature_index+2] - DISPARITY_OFFSET_HORIZONTAL)/RESOLUTION_FACTOR;

		for ( col = col_center-feature_window_size_half; col < col_center+feature_window_size_half+1; col++ ) // runthrough all images lines corrsponding to feature window
		{
			for ( row = row_center-feature_window_size_half; row < row_center+feature_window_size_half+1; row++ ) // runthrough all images lines corrsponding to feature window
			{
				location = (row*image_width_bytes)+(col*2);
				out[index]  = (q15_t) in[location];
				index++;
			}

		}
		for ( col = col_center_right-feature_window_size_half; col < col_center_right+feature_window_size_half+1; col++ ) // runthrough all images lines corrsponding to feature window
		{
			for ( row = row_center-feature_window_size_half; row < row_center+feature_window_size_half+1; row++ ) // runthrough all images lines corrsponding to feature window
			{
				location = (row*image_width_bytes)+(col*2)+1;
				out[index]  = (q15_t) in[location];
				index++;
			}

		}
	}

}

void precompute_rotation_coefficients( float32_t *rotation_coefficients, float rotation_step_size, int rotation_step_number, int number_of_rotations  )
{
	float32_t a = 0;
	float32_t b = 0;
	float32_t c = 0;
	float32_t start = (float32_t) -rotation_step_size*(rotation_step_number-1)/2;
	float32_t step =  (float32_t) rotation_step_size;
	float32_t end = (float32_t) rotation_step_size*(rotation_step_number-1)/2;
	volatile float32_t A = 0;
	volatile float32_t B = 0;
	volatile float32_t C = 0;
	volatile float32_t D = 0;
	volatile float32_t E = 0;
	volatile float32_t F = 0;
	volatile float32_t G = 0;
	volatile float32_t H = 0;
	volatile float32_t I = 0;
	int i = 0;
	int j = 0;
	int k = 0;
	float32_t focal_length = 120;
	volatile int cnt = 0;


	for ( a = start, i = 0; i<rotation_step_number; i++, a+= step)
	{
		for ( b = start, j = 0; j<rotation_step_number; j++, b+= step)
			{
			for ( c = start, k = 0; k<rotation_step_number; k++, c+= step)
			{
				/* [ A B C 0
				 *   D E F 0
				 *   G H I 0
				 *   0 0 0 1 ]   according to  http://planning.cs.uiuc.edu/node104.html
				 */
				A = arm_cos_f32 (a)*arm_cos_f32(b);
				B = (arm_cos_f32(a)*arm_sin_f32(b)*arm_sin_f32(c)) - (arm_sin_f32(a)*arm_cos_f32(c));
				C = (arm_cos_f32(a)*arm_sin_f32(b)*arm_cos_f32(c)) + (arm_sin_f32(a)*arm_sin_f32(c));
				C = C*focal_length;

				D = arm_sin_f32(a)* arm_cos_f32(b);
				E = (arm_sin_f32(a)*arm_sin_f32(b)*arm_sin_f32(c)) + (arm_cos_f32(a)*arm_cos_f32(c));
				F = (arm_sin_f32(a)*arm_sin_f32(b)*arm_cos_f32(c)) - (arm_cos_f32(a)*arm_sin_f32(c));
				F = F*focal_length;

				G = -arm_sin_f32(b);
				H = arm_cos_f32(b)*arm_sin_f32(c);
				I = arm_cos_f32(b)*arm_cos_f32(c);
				G = G/focal_length;
				H = H/focal_length;


				rotation_coefficients[cnt] = A;
				rotation_coefficients[cnt+number_of_rotations] = B;
				rotation_coefficients[cnt+(number_of_rotations*2)] = C;
				rotation_coefficients[cnt+(number_of_rotations*3)] = D;
				rotation_coefficients[cnt+(number_of_rotations*4)] = E;
				rotation_coefficients[cnt+(number_of_rotations*5)] = F;
				rotation_coefficients[cnt+(number_of_rotations*6)] = G;
				rotation_coefficients[cnt+(number_of_rotations*7)] = H;
				rotation_coefficients[cnt+(number_of_rotations*8)] = 1.0/I;



				cnt++;

			}
		}
	}
}

void odometry_translate_and_match_features( uint8_t *images, q15_t *feature_window_data, uint8_t *feature_image_coordinates, uint16_t features_ROT_number, uint8_t feature_window_size_half, float32_t *rotation_coefficients, int number_of_rotations, uint32_t image_width )
{
	float32_t temp [number_of_rotations];
	float32_t row [number_of_rotations];
	float32_t col [number_of_rotations];
	float32_t disp [number_of_rotations];
	float32_t scale [number_of_rotations];

	int i = 0;
	int j = 0;
	volatile int index = 0;
	volatile int index_window = 0;
	volatile uint8_t data = 3; // data about features: [x y d]
	float32_t row_left [number_of_rotations];
	float32_t col_left [number_of_rotations];
	//float32_t row_right;
	float32_t col_right [number_of_rotations];
	float32_t disparity_offset = (float32_t) DISPARITY_OFFSET_HORIZONTAL/RESOLUTION_FACTOR;
	volatile q7_t row_center [number_of_rotations];
	volatile q7_t col_center [number_of_rotations];
	volatile q7_t col_center_right [number_of_rotations];
	volatile int row_nr;
	volatile int col_nr;
	volatile int location;
	volatile int image_width_bytes = image_width*2;
	volatile uint8_t feature_window_size = (feature_window_size_half*2)+1;

	q15_t left_window [feature_window_size*feature_window_size];
	q15_t right_window [feature_window_size*feature_window_size];



	for ( i = 0; i< features_ROT_number; i++ )
	{
		index = i*data;

		// compute translated x-coordinates
		arm_scale_f32( &rotation_coefficients[number_of_rotations*0], (float32_t) feature_image_coordinates[index+0], row, (uint32_t) number_of_rotations); // x = A*x
		arm_scale_f32( &rotation_coefficients[number_of_rotations*1], (float32_t) feature_image_coordinates[index+1], temp, (uint32_t) number_of_rotations); // temp = B*y

		arm_add_f32( &rotation_coefficients[number_of_rotations*2], temp, temp, (uint32_t) number_of_rotations); // temp = (B*y) + C
		arm_add_f32( row, temp, row, (uint32_t) number_of_rotations); // x = x + temp = (A*x) + (B*y) + C

		// compute translated y-coordinates
		arm_scale_f32( &rotation_coefficients[number_of_rotations*3], (float32_t) feature_image_coordinates[index+0], col, (uint32_t) number_of_rotations); // col = D*x
		arm_scale_f32( &rotation_coefficients[number_of_rotations*4], (float32_t) feature_image_coordinates[index+1], temp, (uint32_t) number_of_rotations); // temp = E*y

		arm_add_f32( &rotation_coefficients[number_of_rotations*5], temp, temp, (uint32_t) number_of_rotations); // temp = (E*y) + F
		arm_add_f32( col, temp, col, (uint32_t) number_of_rotations); // y = y + temp = (D*x) + (E*y) + F

		/*
		// compute translation scale
		arm_scale_f32( &rotation_coefficients[number_of_rotations*6], (float32_t) feature_image_coordinates[index+0], scale, (uint32_t) number_of_rotations); // scale = G*x
		arm_scale_f32( &rotation_coefficients[number_of_rotations*7], (float32_t) feature_image_coordinates[index+1], temp, (uint32_t) number_of_rotations); // temp = H*y

		arm_add_f32( &rotation_coefficients[number_of_rotations*8], temp, temp, (uint32_t) number_of_rotations); // temp = (H*y) + I
		arm_add_f32( scale, temp, scale, (uint32_t) number_of_rotations); // scale = scale + temp = (G*x) + (H*y) + I

		arm_scale_f32( &rotation_coefficients[number_of_rotations*6], (float32_t) 1.0, temp, (uint32_t) number_of_rotations); // temp = H*y
		arm_scale_f32( &rotation_coefficients[number_of_rotations*7], (float32_t) 1.0, temp, (uint32_t) number_of_rotations); // temp = H*y
		arm_scale_f32( &rotation_coefficients[number_of_rotations*8], (float32_t) 1.0, temp, (uint32_t) number_of_rotations); // temp = H*y
		*/

		//apply scaling
		arm_mult_f32( &rotation_coefficients[number_of_rotations*8], row, row, (uint32_t) number_of_rotations);
		arm_mult_f32( &rotation_coefficients[number_of_rotations*8], col, col, (uint32_t) number_of_rotations);
		arm_scale_f32( &rotation_coefficients[number_of_rotations*8], (float32_t) feature_image_coordinates[index+2], disp, (uint32_t) number_of_rotations );

		//apply disparity for coordinates in right image
		arm_offset_f32( disp, -disparity_offset, disp, (uint32_t) number_of_rotations );
		arm_add_f32( disp, col, col, (uint32_t) number_of_rotations );

		arm_float_to_q7( row_left, row_center, number_of_rotations );
		arm_float_to_q7( col_left, col_center, number_of_rotations );
		arm_float_to_q7( col_right, col_center_right, number_of_rotations );




			/*
			index_window = 0;
			// obtain pixel window
			for ( col_nr = col_center-feature_window_size_half; col_nr < col_center+feature_window_size_half+1; col_nr++ ) // runthrough all images lines corrsponding to feature window
			{
				for ( row_nr = row_center-feature_window_size_half; row_nr < row_center+feature_window_size_half+1; row_nr++ ) // runthrough all images lines corrsponding to feature window
				{
					//location = (row_nr*image_width_bytes)+(col_nr*2);
					//left_window[index_window]  = (q15_t) images[location];
					index_window++;
				}

			}

			index_window = 0;
			for ( col_nr = col_center_right-feature_window_size_half; col_nr < col_center_right+feature_window_size_half+1; col_nr++ ) // runthrough all images lines corrsponding to feature window
			{
				for ( row_nr = row_center-feature_window_size_half; row_nr < row_center+feature_window_size_half+1; row_nr++ ) // runthrough all images lines corrsponding to feature window
				{
					//location = (int) (row_nr*image_width_bytes)+(col_nr*2)+1;
					//right_window[index_window]  = (q15_t) images[location];
					index_window++;
				}

			}
			// Calculate matching cost
			*/







	}

}



