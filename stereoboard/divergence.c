/*
 * divergence.c
 *
 *  Created on: Apr 9, 2015
 *      Author: knmcguire
 */

#include <divergence.h>



int calculate_edge_flow(uint8_t* in, struct displacement_t* displacement,struct edge_flow_t* edge_flow, struct edge_hist_t* edge_hist,int front,int rear,int windowsize,int max_distance,int edge_threshold,uint32_t  image_width,uint32_t  image_height)
{

	//Define arrays and pointers for edge histogram and displacements
	int edge_histogram_x[IMAGE_WIDTH], prev_edge_histogram_x[IMAGE_WIDTH];

	int * edge_histogram_x_p=edge_histogram_x;
	int * prev_edge_histogram_x_p=prev_edge_histogram_x;

	int edge_histogram_y[IMAGE_HEIGHT],prev_edge_histogram_y[IMAGE_HEIGHT];
	int * edge_histogram_y_p=edge_histogram_y;
	int * prev_edge_histogram_y_p=prev_edge_histogram_y;

	float slope_x=0.0;
	float trans_x=0.0;
	float slope_y=0.0;
	float trans_y=0.0;


	//Calculate previous frame number
	int previous_frame_number[2];

	if(MAX_HORIZON!=2){
		float edge_flow_x,edge_flow_y;

		if(edge_flow->horizontal_trans<0)
			edge_flow_x=-1*edge_flow->horizontal_trans;
		else
			edge_flow_x=edge_flow->horizontal_trans;

		if(edge_flow->vertical_trans<0)
			edge_flow_y=-1*edge_flow->vertical_trans;
		else
			edge_flow_y=edge_flow->vertical_trans;

		if((int)(1/edge_flow_x>MAX_HORIZON-1))
			previous_frame_number[0]=MAX_HORIZON-1;
		else
			previous_frame_number[0]=(int)(1/edge_flow_x)+1;

		if((int)(1/edge_flow_y>MAX_HORIZON-1))
			previous_frame_number[1]=MAX_HORIZON-1;
		else
			previous_frame_number[1]=(int)(1/edge_flow_y)+1;
	}else{
		previous_frame_number[0]=1;
		previous_frame_number[1]=1;
	}


	// the previous frame number relative to dynamic parameters
	int previous_frame_number_rel[2];
	previous_frame_number_rel[0]=front-previous_frame_number[0];
	previous_frame_number_rel[1]=front-previous_frame_number[1];
	if(previous_frame_number_rel[0]<0)
		previous_frame_number_rel[0]=rear+(MAX_HORIZON-1-abs(previous_frame_number[0]));
	if(previous_frame_number_rel[1]<0)
		previous_frame_number_rel[1]=rear+(MAX_HORIZON-1-abs(previous_frame_number[1]));


	// copy previous edge histogram based on previous frame number

	memcpy(prev_edge_histogram_x_p,edge_hist[previous_frame_number_rel[0]].horizontal,sizeof(int)*image_width);
	memcpy(prev_edge_histogram_y_p,edge_hist[previous_frame_number_rel[1]].vertical,sizeof(int)*image_height);


	//Calculate Edge Histogram
	calculate_edge_histogram(in,edge_histogram_x_p,image_width,image_height,'x');
	calculate_edge_histogram(in,edge_histogram_y_p,image_width,image_height,'y');

	//Calculate displacement
	//TODO add max distance and window size to function (is divined iin function)
	calculate_displacement(edge_histogram_x_p,prev_edge_histogram_x_p,displacement->horizontal,image_width);
	calculate_displacement(edge_histogram_y_p,prev_edge_histogram_y_p,displacement->vertical,image_height);




	//Fit a linear line
	//TODO Fix RANSAC function from producing NAN's
	line_fit(displacement->horizontal, &slope_x,&trans_x,image_width);
	line_fit(displacement->vertical,  &slope_y,&trans_y,image_height);


	//Correct Divergence slope and translation by the amount of frames skipped
	slope_x=slope_x/(float)previous_frame_number[0];
	trans_x=trans_x/(float)previous_frame_number[0];
	slope_y=slope_y/(float)previous_frame_number[1];
	trans_y=trans_y/(float)previous_frame_number[1];


	edge_flow->horizontal_trans=trans_x;
	edge_flow->vertical_trans=trans_y;
	edge_flow->horizontal_slope=slope_x;
	edge_flow->vertical_slope=slope_y;


	//Copy new edge histogram to the structure
	memcpy(edge_hist[front].horizontal,edge_histogram_x_p,sizeof(int)*image_width);
	memcpy(edge_hist[front].vertical,edge_histogram_y_p,sizeof(int)*image_height);

	return previous_frame_number[0];


}



//calculate_edge_histogram calculates the image gradient of the images and makes a edge feature histogram
void calculate_edge_histogram(uint8_t* in,int* edge_histogram,uint32_t image_width, uint32_t image_height,char direction)
{

	int8_t  sobel_left;
	int8_t  sobel_right;
	int8_t  Sobel[3] = {-1, 0, 1};

	uint16_t y,x;
	uint32_t idx;


	uint32_t edge_histogram_temp=0;

	int8_t  c,r;

	if(direction=='x')
	{
		for( x = 0; x < image_width; x++)
		{
			for( y = 0; y < image_height; y++)
			{

				uint32_t idx = image_width*y*2 + (x)*2;
				sobel_left=0;
				sobel_right=0;

				for(c = -1; c <=1; c++)
				{
					uint32_t idx_filter = image_width*(y)*2 + (x+c)*2;

					sobel_left += Sobel[c+1] * (int8_t)(in[idx_filter+1]);
					sobel_right += Sobel[c+1] * (int8_t)(in[idx_filter]);

				}
				sobel_left=abs(sobel_left);
				edge_histogram_temp += (uint32_t)sobel_left;
				//out[idx+1]=(uint8_t)sobel_left;
				//out[idx]=0;

			}

			edge_histogram[x]=(uint32_t)edge_histogram_temp;
			edge_histogram_temp=0;

		}
	}
	else if(direction=='y'){
		for( y = 0; y < image_height; y++)
		{
			for( x = 0; x < image_width; x++)
			{

				uint32_t idx = image_width*y*2 + (x)*2;
				sobel_left=0;
				sobel_right=0;

				for(c = -1; c <=1; c++)
				{
					uint32_t idx_filter = image_width*(y+c)*2 + (x)*2;

					sobel_left += Sobel[c+1] * (int8_t)(in[idx_filter+1]);
					sobel_right += Sobel[c+1] * (int8_t)(in[idx_filter]);

				}
				sobel_left=abs(sobel_left);
				edge_histogram_temp += (uint32_t)sobel_left;
				//out[idx+1]=(uint8_t)sobel_left;
				//out[idx]=0;

			}

			edge_histogram[y]=(uint32_t)edge_histogram_temp;
			edge_histogram_temp=0;

		}

	}else{}

}

//Calculate_displacement calculates the displacement between two histograms
void calculate_displacement(int* edge_histogram,int* edge_histogram_prev,int* displacement,uint32_t size)
{volatile	int  c=0;
volatile int r=0;
volatile int y=0;
volatile int x=0;

volatile int W=10;
volatile int D=10;
volatile int SAD_temp[20];

volatile int min_index=0;

for(x=0; x<size;x++)
{

	if(x>=W+D&&x<=size-W-D)
	{
		for(c=-D;c<W;c++)
		{
			SAD_temp[D+c]=0;

			for(r=-W;r<W;r++)
				SAD_temp[c+D]+=abs(edge_histogram[x+r]-edge_histogram_prev[x+r+c]);


		}

		min_index=getMinimum(SAD_temp,20);

		displacement[x]=(int)(min_index-W);
	}else{
		displacement[x]=0;
	}
}





}

//Line_fit fits a line using least squared to the histogram disparity map (displacement
void line_fit(int* displacement, float* Slope, float* Yint,uint32_t image_width)
{
	int16_t x;

	uint32_t Count=0;
	float SumY=0;
	float  SumX=0;
	float SumX2=0;
	float SumXY=0;
	float XMean=0;
	float YMean=0;
	float Slope_temp,Yint_temp;


	for(x=0;x<image_width;x++){

		if(displacement[x]!=0){

			SumX+=x;
			SumY+=displacement[x];

			SumX2+=x *x;
			SumXY+=x *displacement[x];
			Count++;
		}

	}

	XMean=SumX/(float)Count;
	YMean=SumY/(float)Count;

	Slope_temp=((float)(SumXY-SumX*YMean)/(float)(SumX2-SumX*XMean));
	Yint_temp= (YMean-Slope_temp* (float)XMean);

	if(Count!=0){
		*Slope=Slope_temp;
		*Yint=Yint_temp;
	}else{
		*Slope=0;
		*Yint=0;}



}

void line_fit_RANSAC( int* displacement, float* Slope, float* Yint,int size,int* X)
{
	//Fit a linear line with RANSAC (from Guido's code)
	int ransac_iter=20;
	int it;
	volatile int k;
	volatile int ind1, ind2, tmp, entry, total_error, best_ind;
	volatile int dx, dflow, predicted_flow;
	// flow = a * x + b
	float a[ransac_iter];
	float b[ransac_iter];
	float a_temp,b_temp;
	int errors[ransac_iter];


	volatile int count_disp=0;

	for(k=0;k<size;k++){
		X[k]=k;
		count_disp+=displacement[k];    }

	if(count_disp!=0)
	{
		for(it = 0; it < ransac_iter; it++)
		{

			errors[it] = 0;
			total_error=0;

			ind1 = rand() % size;
			ind2 = rand() % size;

			while(ind1 == ind2)
				ind2 = rand() % size;

			if(X[ind1] > X[ind2])
			{
				tmp = ind2;
				ind2 = ind1;
				ind1 = tmp;
			}
			/*while(displacement[ind1]==0)
				ind1 = rand() % size;
			while(displacement[ind2]==0)
				ind2 = rand() % size;*/




			dx=X[ind2]-X[ind1];
			dflow = displacement[ind2] - displacement[ind1];

			//Fit line with two points

			a[it] = (float)dflow/(float)dx;
			b[it] = (float)displacement[ind1]- (a[it] *(float)(X[ind1]));

			// evaluate fit:
			for (entry = 0; entry < size; entry++)
			{
				predicted_flow = (int32_t)(a[it] * (float)(X[entry] ) + b[it]);
				total_error += (uint32_t) abs(displacement[entry] - predicted_flow);
			}
			errors[it] = total_error;
		}
		// select best fit:
		best_ind = getMinimum2(errors, 20);
		//printf("%d\n",best_ind);
		(*Slope) = a[best_ind];
		(*Yint) = b[best_ind];
	}
	else
	{
		(*Slope) = 0.0;
		(*Yint)= 0.0;
	}



}

float simpleKalmanFilter(float* cov,float previous_est, float current_meas,float Q,float R)
{

	float predict_state=previous_est;
	float predict_cov=*cov+Q;
	float K=predict_cov*(1/(*cov+R));

	float new_est=predict_state+K*(current_meas-previous_est);
	*cov=(1-K)*predict_cov;

	current_meas=new_est;


	return new_est;
}


//This function is a visualization tool which visualizes the Edge filter in the one image and the histogram disparity with line fit in the second.
void visualize_divergence(uint8_t* in,int32_t* displacement,float Slope, float Yint,uint32_t image_width, uint32_t image_height)
{

	volatile int y=0;
	volatile int x=0;
	volatile uint32_t idx=0;

	uint32_t line_check1=0;
	uint32_t line_check2=0;


	for( y = 0; y < image_height; y++)
	{
		////line_check1=(uint32_t)(Slope*(float)x+(Yint)+(float)image_height/2);
		//line_check2=(uint32_t)(displacement[x]+image_height/2);
		for( x = 0; x < image_width; x++)

		{

			idx =  image_width*y*2 + (x)*2;

			/*if(y==line_check1)
				out[idx]=255;
			else if(y==line_check2)
				out[idx]=100;
			else
				out[idx]=0;*/

			in[idx]=100;
			in[idx+1]=0;//in[idx+1];




		}

	}

}


int getMinimum2(int * flow_error, int  max_ind)
{
	uint32_t i;
	uint32_t min_ind = 0;
	uint32_t min_err = flow_error[0];
	for(i = 1; i < max_ind; i++)
	{
		if(flow_error[i] < min_err&&!isnan(flow_error[i]))
		{
			min_ind = i;
			min_err = flow_error[i];
		}
	}
	return min_ind;
}
