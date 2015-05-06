#ifndef _MY_IMAGE_HEADER_
#define _MY_IMAGE_HEADER_


struct img_struct {
  int seq;
  double timestamp;
  unsigned char *buf;
  int w;
  int h;
};

#endif
