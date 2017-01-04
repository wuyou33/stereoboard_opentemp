
#ifndef _MY_DCMI_H_
#define _MY_DCMI_H_

#include <stdint.h>

/** dcmi_mode_t
 * DCMI_MODE_1 Default mode. Medium latency, medium memory use.
 * This mode uses a processing buffer which is only filled when the main loop requests a new image.
 * The DCMI is continuously running but the main will wait till a new image is received before running.
 *
 * DCMI_MODE_2 Minimum latency, high memory use. Use when low latency is paramount and processing is slower
 * than full frame rate. Uses triple data buffer, dual DMA buffer and a processing buffer. With this method,
 * the main loop can immediately use the most recent image for processing without waiting for new image.
 *
 * DCMI_MODE_3 Low latency, low processing. DANGER! Image will be overwritten if code runs too slow.
 * Recommend only use when resultant frame rate is full speed. Uses DMA double buffer to ensure that
 * the currently processed image is not being overwritten by the DCMI
 *
 * DCMI_MODE_4 High latency, low memory use. Required if using a large image
 * This mode only uses the DMA buffer and disables the DCMI between reads. When the main requests a new
 * image, the DCMI is restarted. This induces a large delay but has the lowest memory requirements. Use
 * when a large image is required.
 */
#define DCMI_MODE_1 0
#define DCMI_MODE_2 1
#define DCMI_MODE_3 2
#define DCMI_MODE_4 3

// master helper function to initilaize clock, DMA and DCMI
void camera_init(void);

// Initialize camera subsystems
void camera_clock_init(void);
void camera_reset_init(void);
void camera_dcmi_bus_init(void);
void camera_dcmi_dma_init(void);

// Handle the camera reset
void camera_unreset(void);
void camera_reset(void);

// Data streams
void camera_dcmi_dma_enable(void);
void camera_dcmi_it_init(void);
void camera_dma_it_init(void);

// Request and wait for new camera frame
uint8_t* camera_wait_for_frame(void);

void camera_crop(uint16_t offset);

extern uint8_t *current_image_buffer;
extern volatile uint32_t frame_counter;
extern volatile uint32_t frame_processed; // frame number of last frame processed by main

void dcmi_isr(void);
void dma2_stream1_isr(void);

#endif
