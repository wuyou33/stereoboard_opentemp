
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

// To change image size, change:
// - the width and height in the send-command in main.c
// - the full image size in dcmi.h
// - the dma copy (double buffering or not) in dcmi.c (op 3 plaatsen)
// - the configuration of the camera in tcm8230.c

#define FULL_IMAGE_SIZE     (128*96*2)
//#define FULL_IMAGE_SIZE     (176*144*2)

extern uint8_t *current_image_buffer;

extern volatile int frame_counter;

void dcmi_isr(void);
void dma2_stream1_isr(void);


#endif
