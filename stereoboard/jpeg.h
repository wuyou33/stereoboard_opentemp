
#ifndef __MY_JPEG_HEADER__
#define __MY_JPEG_HEADER__

#include <stdint.h>

#define        FOUR_ZERO_ZERO          0
#define        FOUR_TWO_ZERO           1
#define        FOUR_TWO_TWO            2
#define        FOUR_FOUR_FOUR          3
#define        RGB                     4

unsigned char *encode_image(uint8_t *in, uint8_t *out, uint32_t, uint32_t, uint32_t, uint32_t);

#endif
