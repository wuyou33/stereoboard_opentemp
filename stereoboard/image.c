
#include <image.h>

void setLineNumbers(uint8_t** givenImage, uint16_t array_width, uint16_t array_height)
{
	uint8_t* b = *givenImage;
   uint8_t horizontalLine = 0;
   for (horizontalLine = 0; horizontalLine < array_height; horizontalLine++) {
     // Line data
       b[array_width * horizontalLine]=horizontalLine;
       b[array_width * horizontalLine+1]=horizontalLine;
       b[array_width * horizontalLine+2]=horizontalLine;
       b[array_width * horizontalLine+3]=horizontalLine;
       b[array_width * horizontalLine+4]=horizontalLine;
       b[array_width * horizontalLine+5]=horizontalLine;
       b[array_width * horizontalLine+6]=horizontalLine;
   }

}

void setLineNumbersImage(uint8_t* b, uint16_t width, uint16_t height){

   uint8_t j = 0;
   for (j = 0; j < height; j++) {
		b[j * width * BYTES_PER_PIXEL]=j;
		b[j * width * BYTES_PER_PIXEL+1]=j;
		b[j * width * BYTES_PER_PIXEL+2]=j;
		b[j * width * BYTES_PER_PIXEL+3]=j;
		b[j * width * BYTES_PER_PIXEL+4]=j;
		b[j * width * BYTES_PER_PIXEL+5]=j;

   }

}
