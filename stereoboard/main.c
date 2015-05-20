/**
  ******************************************************************************
  * @file    main.c
  * @author  C. De Wagter
  * @version V1.0.0
  * @date    2013
  * @brief   Main program body
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "led.h"
#include "dcmi.h"
#include "cpld.h"
#include "usart.h"
#include "tcm8230.h"
#include "hmc5883.h"
#include "stm32f4xx_conf.h"
#include "jpeg.h"
#include "arm_math.h"
#include "stereo_vision.h"
#include "window_detection.h"
#include "filter_color.h"
#include "utils.h"
#include "usb.h"
#include "sys_time.h"
#include "raw_digital_video_stream.h"

#include BOARD_FILE
#include "main_parameters.h"

#include "commands.h"

// integral_image has size 128 * 96 * 4 = 49152 bytes = C000 in hex
uint32_t *integral_image = ((uint32_t *) 0x10000000); // 0x10000000 - 0x1000 FFFF = CCM data RAM  (64kB)
//uint8_t* jpeg_image_buffer_8bit = ((uint8_t*) 0x1000D000); // 0x10000000 - 0x1000 FFFF = CCM data RAM
//uint8_t* disparity_image_buffer_8bit = ((uint8_t*) 0x10000000);

uint16_t offset_crop = 0;

/** @addtogroup StereoCam
  * @{
  */

/* Private functions ---------------------------------------------------------*/



/**
  * @brief  Main program
  * @param  None
  * @retval None
  */
int main(void)
{
  /*
    At this stage the microcontroller clock setting is already configured,
    this is done through SystemInit() function which is called from startup
    file (startup_stm32f4xx.s) before to branch to application main.
    To reconfigure the default setting of SystemInit() function, refer to
    system_stm32f4xx.c file
  */


  /****************
   * INITIALIZATION
   ****************/

  // Initialize the LED
  led_init();
  led_set();
  // Initialize the serial communication (before the camera so we can print status)
  usart_init();
  // Initialize the CPLD
  camera_cpld_stereo_init();
  //camera_cpld_stereo_left();
  //camera_cpld_stereo_right();
  //camera_cpld_stereo_pixmux();
  camera_cpld_stereo_linemux();
  //camera_cpld_stereo_framemux();
  // Reset the camera's
  camera_reset_init();
  camera_reset();
  // Make a 21MHz clock signal to the camera's
  camera_clock_init();
  // Stop resetting the camera (pin high)
  camera_unreset();
  // Initialize all camera GPIO and I2C pins
  camera_dcmi_bus_init();
  camera_tcm8230_i2c_init();
  // Start listening to DCMI frames
  camera_dcmi_init();
  // Start DCMI interrupts (interrupts on frame ready)
  camera_dcmi_it_init();
  camera_dcmi_dma_enable();
  // Communicate with camera, setup image type and start streaming
  camera_tcm8230_config();
  // Start DMA image transfer interrupts (interrupts on buffer full)
  camera_dma_it_init();
  // Print welcome message
  char comm_buff[128] = " --- Stereo Camera --- \n\r";
  UsartTx((uint8_t *)&comm_buff, strlen(comm_buff));
  // Disparity image buffer:
  uint8_t disparity_image_buffer_8bit[FULL_IMAGE_SIZE / 2];
  uint16_t ind;
  for (ind = 0; ind < FULL_IMAGE_SIZE / 2; ind++) {
    disparity_image_buffer_8bit[ind] = 0;
  }
#if USE_COLOR
  // slight waste of memory, if color is not used:
  uint8_t filtered_image[FULL_IMAGE_SIZE / 2];
  for (ind = 0; ind < FULL_IMAGE_SIZE / 2; ind++) {
    filtered_image[ind] = 0;
  }
#endif
  uint8_t min_y, max_y;
  uint32_t image_width = IMAGE_WIDTH;
  uint32_t image_height = IMAGE_HEIGHT;
  uint32_t start, stop;

  /***********
   * MAIN LOOP
   ***********/
  //DCMI_CaptureCmd(ENABLE);

  volatile int processed = 0;
  while (1) {

#ifdef LARGE_IMAGE
    offset_crop += 80;
    if (offset_crop == 480) {
      offset_crop = 0;
    }
    camera_crop(offset_crop);
#endif
    DCMI_CaptureCmd(ENABLE); // while no new frame, ask DCMI to capture a new frame
    // wait for new frame
    while (frame_counter == processed)
      ;
    processed = frame_counter;
    //led_toggle();




#if SEND_IMAGE
    Send(current_image_buffer, IMAGE_WIDTH, IMAGE_HEIGHT);
#endif



  }
}

#ifdef  USE_FULL_ASSERT

/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */

  /* Infinite loop */
  while (1) {
  }
}
#endif

/**
  * @}
  */

