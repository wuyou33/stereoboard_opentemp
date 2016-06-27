#ifndef BLOB_H_
#define BLOB_H_

#include <sys/time.h>
#include "../common/stereo_utils.h"

/* The different type of images we currently support */
enum image_type {
  IMAGE_YUV422,     ///< UYVY format (uint16 per pixel)
  IMAGE_GRAYSCALE,  ///< Grayscale image with only the Y part (uint8 per pixel)
  IMAGE_JPEG,       ///< An JPEG encoded image (not per pixel encoded)
  IMAGE_GRADIENT,   ///< An image gradient (int16 per pixel)
  IMAGE_LABELS      ///< Labeled blobs (uint16 per pixel)
};

/* Main image structure */
// TODO KIRK: remove timeval!
struct image_t {
  enum image_type type;   ///< The image type
  uint16_t w;             ///< Image width
  uint16_t h;             ///< Image height
  struct timeval ts;      ///< The timestamp of creation

  uint8_t buf_idx;        ///< Buffer index for V4L2 freeing
  uint32_t buf_size;      ///< The buffer size
  void *buf;              ///< Image buffer (depending on the image_type)
};

/* Image point structure */
struct point_t {
  uint16_t x;             ///< The x coordinate of the point
  uint16_t y;             ///< The y coordinate of the point
};

/* Vector structure for point differences */
struct flow_t {
  struct point_t pos;         ///< The original position the flow comes from
  int16_t flow_x;             ///< The x direction flow in subpixels
  int16_t flow_y;             ///< The y direction flow in subpixels
};

struct image_filter_t {
  uint8_t y_min;
  uint8_t y_max;
  uint8_t u_min;
  uint8_t u_max;
  uint8_t v_min;
  uint8_t v_max;
};

struct image_label_t {
  uint16_t id;
  uint8_t filter;

  uint16_t pixel_cnt;
  uint16_t x_min;
  uint16_t y_min;

  struct point_t contour[512];
  uint16_t contour_cnt;

  uint16_t corners[4];
};

#endif
