
#ifndef _MY_DCMI_H_
#define _MY_DCMI_H_

#include <stdint.h>
#include <main_parameters.h>

// Initialize camera subsystems
void camera_clock_init(void);
void camera_reset_init(void);
void camera_dcmi_bus_init(void);
void camera_dcmi_init(void);

// Handle the camera reset
void camera_unreset(void);
void camera_reset(void);

// Data streams
void camera_dcmi_dma_enable(void);
void camera_dcmi_it_init(void);
void camera_dma_it_init(void);

// When the camera is started in snapshot mode it will take 1 image
void camera_snapshot(void);

void camera_crop(uint16_t offset);

extern uint8_t *current_image_buffer;

extern volatile int frame_counter;

void dcmi_isr(void);
void dma2_stream1_isr(void);


#endif
