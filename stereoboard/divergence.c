/*
 * divergence.c
 *
 *  Created on: Apr 9, 2015
 *      Author: knmcguire
 */

#include <divergence.h>

//main code for the edge flow divergence cod
void calculate_edge_flow_simple(uint8_t* in,uint32_t* edge_histogram,uint32_t* edge_histogram_prev,int32_t* displacement,float* slope,float* yint,uint32_t image_width, uint32_t image_height)
{
	//Initialize arrays and variables


	//calculate edge histograms from previous image and current
	calculate_edge_histogram(in,edge_histogram,image_width,image_height);
	//calculate_edge_histogram(in_prev,edge_histogram_prev_p,image_width,image_height);

	//calculate displacement between previous image and current
	calculate_displacement(edge_histogram,edge_histogram_prev, displacement, image_width, image_height);


	//Fit a linear model to the displacement to aquire slope and displacement
	line_fit(displacement,slope,yint,image_width);
	//line_fit_RANSAC(displacement,slope,yint,image_width);

}




//Calculates the image_difference from one frame to anot
void image_difference(uint8_t* in,uint8_t* in_prev,uint8_t* out,uint32_t image_width, uint32_t image_height)
{
	uint16_t y,x;
	uint32_t idx ;

	for( x = 0; x < image_width; x++)
	{
		for( y = 0; y < image_height; y++)
		{
			idx = image_width*y*2 + (x)*2;

			out[idx+1]=abs(in_prev[idx+1]-in[idx+1]);//abs(in_prev[idx+1]-in[idx-1]);
			out[idx]=0;//abs(in_prev[idx]-in[idx]);
		}
	}

}

//calculate_edge_histogram calculates the image gradient of the images and makes a edge feature histogram
void calculate_edge_histogram(uint8_t* in,uint32_t* edge_histogram,uint32_t image_width, uint32_t image_height)
{

	int8_t  sobel_left;
	int8_t  sobel_right;
	int8_t  Sobel[3] = {-1, 0, 1};

	uint16_t y,x;
	uint32_t idx;


	uint32_t edge_histogram_temp=0;

	int8_t  c,r;

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

//Calculate_displacement calculates the displacement between two histograms
void calculate_displacement(uint32_t* edge_histogram,uint32_t* edge_histogram_prev,int32_t* displacement,uint32_t image_width,uint32_t image_height)
{	 int32_t  c,r;
uint16_t y,x,flow_ind;

int32_t W=10;
int32_t D=10;
uint32_t SAD_temp[20];

uint32_t min_index=0;

for(x=0; x<image_width;x++)
{

	if(x>=W+D&&x<=image_width-W-D)
	{
		for(c=-D;c<W;c++)
		{
			SAD_temp[D+c]=0;

			for(r=-W;r<W;r++)
				SAD_temp[c+D]+=abs(edge_histogram[x+r]-edge_histogram_prev[x+r+c]);


		}

		min_index=getMinimum(SAD_temp,20);

		displacement[x]=(int32_t)(min_index-W);
	}else{
		displacement[x]=0;
	}
}





}

//Line_fit fits a line using least squared to the histogram disparity map (displacement
void line_fit(int32_t* displacement, float* Slope, float* Yint,uint32_t image_width)
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

void line_fit_RANSAC( int32_t* displacement, float* Slope, float* Yint,uint16_t size)
{

    //Fit a linear line with RANSAC (from Guido's code)
    uint8_t ransac_iter=20;
    uint8_t it;
    uint16_t k;
    uint32_t ind1, ind2, tmp, entry, total_error, best_ind;
    int32_t dx, dflow, predicted_flow;
    // flow = a * x + b
    float a[ransac_iter];
    float b[ransac_iter];
    float a_temp,b_temp;
    uint32_t errors[ransac_iter];

    uint16_t X[size];

    for(k=0;k<size;k++)
        X[k]=k;

    for(it = 0; it < ransac_iter; it++)
    {

        errors[it] = 0;
        total_error=0;


        ind1 = rand() % (size-20)+10;
        ind2 = rand() % (size-20)+10;
		/*while(displacement[ind1]!=0||displacement[ind2]!=0){
	        ind1 = rand() % (size-20)+10;
	        ind2 = rand() % (size-20)+10;
		}*/

        while(ind1 == ind2)
            ind2 = rand() % (size-20)+10;

        if(X[ind1] > X[ind2])
        {
            tmp = ind2;
            ind2 = ind1;
            ind1 = tmp;
        }


        dx=X[ind2]-X[ind1];
        dflow = displacement[ind2] - displacement[ind1];

    //Fit line with two points

        a[it] = (float)dflow/(float)dx;
        b[it] = (float)displacement[ind1]- (a[it] *(float)(X[ind1] - size/2));

        // evaluate fit:
        for (entry = 0; entry < size; entry++)
        {
            predicted_flow = (int32_t)(a[it] * (float)(X[entry] - size/2) + b[it]);
            total_error += (uint32_t) abs(displacement[entry] - predicted_flow);
        }
        errors[it] = total_error;

		//}
		//else
			//errors[it]=100000;
    }

    // select best fit:
    best_ind = getMinimum(errors, 20);
    //printf("%d\n",best_ind);
    (*Slope) = a[best_ind] ;
    (*Yint) = b[best_ind];

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



