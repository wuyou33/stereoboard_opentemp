
#include <image.h>

void setLineNumbers(uint8_t image[], uint16_t width, uint16_t height)
{
  int j = 0;
  for (j = 0; j < height; j++) {
    // Line data
	  image[j * width * BYTES_PER_PIXEL]=j;
	  image[j * width * BYTES_PER_PIXEL+1]=j;
//	  image[j * width * BYTES_PER_PIXEL+2]=j;

  }
}
