
#ifndef _MY_DCMI_H_
#define _MY_DCMI_H_

#include <stdint.h>

// Initialize camera subsystems
void camera_clock_init(void);
void camera_reset_init(void);
void camera_dcmi_bus_init(void);

// Handle the camera reset
void camera_unreset(void);
void camera_reset(void);

// Data streams
void camera_dcmi_dma_enable(void);
void camera_dcmi_it_init(void);
void camera_dma_it_init(void);

// To change image size, change the following define:
#define SMALL_IMAGE
//#define LARGE_IMAGE

#ifdef SMALL_IMAGE
#define FULL_IMAGE_SIZE     (128*96*2)
#define IMAGE_WIDTH       128
#define IMAGE_HEIGHT      96
#else
#define FULL_IMAGE_SIZE     (176*144*2)
#define IMAGE_WIDTH       176
#define IMAGE_HEIGHT      144
#endif

extern uint8_t *current_image_buffer;

extern volatile int frame_counter;

void dcmi_isr(void);
void dma2_stream1_isr(void);


#endif
