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
#include "usbd_cdc_core.h"
#include "usbd_usr.h"
#include "usbd_desc.h"
#include "usbd_cdc_vcp.h"

__ALIGN_BEGIN USB_OTG_CORE_HANDLE  USB_OTG_dev __ALIGN_END;


uint8_t *jpeg_image_buffer_8bit = ((uint8_t *) 0x10000000); //[FULL_IMAGE_SIZE]; // actually too large
//uint8_t jpeg_image_buffer_8bit[FULL_IMAGE_SIZE]; // actually too large


/** @addtogroup StereoCam
  * @{
  */

/* Private function prototypes -----------------------------------------------*/
void Delay(__IO uint32_t nCount);
/* Private functions ---------------------------------------------------------*/

void drawline(int16_t l)
{
  uint16_t width = 128; // 176; // 128;
  uint8_t code[4];
  code[0] = 0xff;
  code[1] = 0x00;
  code[2] = 0x00;
  {
    int i;
    uint8_t debug[256];

    for (i = 0; i < 128 * 2; i++) {
      debug[i] = 127;
    }

    if (l >= 0) {
      l /= 2;
      if (l > 100) {
        l = 100;
      }
      for (i = 0; i < l; i++) {
        debug[i] = 230;
      }
    } else {
      l = -l;
      l /= 2;
      if (l > 100) {
        l = 100;
      }
      for (i = 0; i < l; i++) {
        debug[i] = 20;
      }
    }

    code[3] = 0x80;
    while (usart_tx_ringbuffer_push(code, 4) == 0)
      ;
    while (usart_tx_ringbuffer_push(debug, width) == 0)
      ;
    while (usart_tx_ringbuffer_push(debug + width, width) == 0)
      ;
    code[3] = 0xDA;
    while (usart_tx_ringbuffer_push(code, 4) == 0)
      ;
  }

}

void Send(uint8_t *b)
{
  uint8_t code[4];
  code[0] = 0xff;
  code[1] = 0x00;
  code[2] = 0x00;

  uint16_t width = 128; // 176; // 128;
  uint16_t height = 96; // 144; // 96;

#if 1
  {
    drawline(magneticfield[0]);
    drawline(magneticfield[1]);
    drawline(magneticfield[2]);
  }
#endif

  int j = 0;
  for (j = 0; j < height; j++) {
    code[3] = 0x80;
    while (usart_tx_ringbuffer_push(code, 4) == 0)
      ;
    while (usart_tx_ringbuffer_push(b + width * j * 2, width) == 0)
      ;
    while (usart_tx_ringbuffer_push(b + width * j * 2 + width, width) == 0)
      ;

    code[3] = 0xDA;
    while (usart_tx_ringbuffer_push(code, 4) == 0)
      ;
  }

  code[3] = 0xAB;
  while (usart_tx_ringbuffer_push(code, 4) == 0)
    ;

  //Delay(0xffffff);
}

void SendJpeg(uint8_t *b, uint32_t size)
{
  uint8_t *p = (uint8_t *) & size;

  uint8_t code[10];
  code[0] = '#';
  code[1] = '#';
  code[2] = 'I';
  code[3] = 'M';
  code[4] = 'J';
  code[5] = '2';
  code[6] = p[0];
  code[7] = p[1];
  code[8] = p[2];
  code[9] = 0x00;

  //VCP_DataTx(code,10);
  //Delay(0x07FFFF);

  while (usart_tx_ringbuffer_push(code, 10) == 0)
    ;

  int j = 0;
  for (j = 0; j < size; j++) {
    while (usart_tx_ringbuffer_push(b + j, 1) == 0)
      ;
  }

  /*
  for (j=0;j<size; j+= 64)
  {
    int s = size - j;
    if (s>64)
      s=64;
    VCP_DataTx(b+j,s);
    Delay(0x1FFFFF);
  }
  */

  //while(uart_tx_finished() == 0)
  //  ;
}

void SendDisparityMap(uint8_t *b)
{
  uint8_t code[4];
  code[0] = 0xff;
  code[1] = 0x00;
  code[2] = 0x00;

  int j = 0;
  for (j = 0; j < 96; j++) {
    code[3] = 0x80;
    while (usart_tx_ringbuffer_push(code, 4) == 0)
      ;
    while (usart_tx_ringbuffer_push(b + 128 * j, 128) == 0)
      ;

    code[3] = 0xDA;
    while (usart_tx_ringbuffer_push(code, 4) == 0)
      ;
  }

  code[3] = 0xAB;
  while (usart_tx_ringbuffer_push(code, 4) == 0)
    ;
}

void SendCommand(uint8_t b)
{
  uint8_t code[1];
  if (b == 1) {
    code[0] = 0x61;
  } else {
    code[0] = 0x62;
  }

  while (usart_tx_ringbuffer_push(code, 1) == 0)
    ;
}

void SendCommandHeight(uint8_t b)
{
  uint8_t code[1];
  code[0] = b;

  while (usart_tx_ringbuffer_push(code, 1) == 0)
    ;
}


void init_timer2()
{
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);
  TIM_TimeBaseInitTypeDef TIM_InitStruct;
  TIM_InitStruct.TIM_Prescaler = 42000 - 1;                // This will configure the clock to 2 kHz
  TIM_InitStruct.TIM_CounterMode = TIM_CounterMode_Up;     // Count-up timer mode
  TIM_InitStruct.TIM_Period = 20000 - 1;                    // 10 seconds
  TIM_InitStruct.TIM_ClockDivision = TIM_CKD_DIV1;        // Divide clock by 1
  TIM_InitStruct.TIM_RepetitionCounter = 0;                // Set to 0, not used
  TIM_TimeBaseInit(TIM2, &TIM_InitStruct);
  TIM_Cmd(TIM2, ENABLE);
}

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

  // Initialize the LED
  led_init();
  led_set();

  // Initialize the serial communication (before the camera so we can print status)
  usart_init();

  // Initialize USB transfer
  /*
  char message[64] = " --- Stereo Camera System --- \r\n";
  USBD_Init(&USB_OTG_dev,
             USB_OTG_FS_CORE_ID,
            &USR_desc,
            &USBD_CDC_cb,
            &USR_cb);
  VCP_send_str(message);
  */

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

  // Wait for at least 100 clock cycles
  Delay(0x07FFFF);

  // Stop resetting the camera (pin high)
  camera_unreset();

  // Initialize all camera GPIO and I2C pins
  camera_dcmi_bus_init();
  camera_tcm8230_i2c_init();

  // Start listening to DCMI frames
  camera_dcmi_init();
  camera_dcmi_dma_enable();

  // Wait for at least 2000 clock cycles after reset
  Delay(0x07FFFF);

  // Communicate with camera, setup image type and start streaming
  camera_tcm8230_config();
  hmc5883_config();

  // Start DCMI interrupts
  // camera_dcmi_it_init();

  // Start DMA image transfer interrupts
  camera_dma_it_init();

  // Print welcome message
  char comm_buff[128] = " --- Stereo Camera --- \n\r";
  usart_tx_ringbuffer_push((uint8_t *)&comm_buff, strlen(comm_buff));

  // new vars for jpeg compression:
  //uint8_t jpeg_image_buffer_8bit[FULL_IMAGE_SIZE]; // actually too large
  uint8_t disparity_image_buffer_8bit[FULL_IMAGE_SIZE / 2]; // actually too large
  uint32_t image_width = 128;
  uint32_t image_height = 96;
  uint8_t DISPARITY = 0;
  uint8_t SEND_RAW = 1;
  uint8_t TIME_PROCESSING = 0;
  uint32_t start, stop;
  if (DISPARITY) {
    init_timer2();
  }

  // Avoidance parameters;

  uint16_t obst_thr1 = 1700; // number of pixels with high disparity
  uint8_t obst_thr2 = 5; // number of obstacle detections in row
  uint16_t obst_wait = 800; // time to wait before avoidance manoeuver [ms]
  uint16_t obst_thr3 = 1500; // number of pixels with low disparity (phase 3)
  uint8_t obst_thr4 = 2; // number of NO obstacle detections in row (phase 3)
  uint16_t obst_entr = 70; // entropy threshold
  uint8_t obst_thr5 = 3; // number of obstacle detections in row (phase 4)
  uint16_t obst_wait2 = 500; // time to wait before going from phase 4 to 1 [ms]

  uint8_t phase = 1;
  uint8_t obst_dect = 0;
  uint32_t obst_time = 0;
  uint8_t obst_free = 0;
  uint8_t obst_dect2 = 0;
  uint32_t obst_time2 = 0;
  int16_t lp_height_disparity = 0;
  int16_t FACTOR1 = 10;
  int16_t FACTOR2 = 90;
  int16_t TOTAL = FACTOR1 + FACTOR2;

  int processed = 0;
  while (1) {
    // wait for new frame
    while (frame_counter == processed)
      ;

    processed = frame_counter;

    //usart_tx_ringbuffer_pop_to_usart();
    //if (1)
    {
      hmc5883_read();
    }
    //else
    if (!DISPARITY) {
      //while (usart_char_available())
      if (SEND_RAW) {
        //char tmp = '.';
        //tmp = usart_rx_ringbuffer_pop();

        Send(current_image_buffer);
      } else {
        // JPEG encode the image:
        // encode_image (UINT8 *input_ptr,UINT8 *output_ptr, UINT32 quality_factor, UINT32 image_format, UINT32 image_width, UINT32 image_height)
        // quality factor from 1 (high quality) to 8 (low quality)
        uint32_t quality_factor = 1;
        // format (in jpeg.h)
        uint32_t image_format = FOUR_TWO_TWO;
        uint8_t *end = encode_image(current_image_buffer, jpeg_image_buffer_8bit, quality_factor, image_format, image_width,
                                    image_height);

        uint32_t size = end - jpeg_image_buffer_8bit;

        SendJpeg(jpeg_image_buffer_8bit, size);

      }
    } else {
      if (TIME_PROCESSING) {
        start = TIM_GetCounter(TIM2);
      }

      // call Sjoerd's stereo vision algorithm on the current image:
      uint32_t disparity_range = 10; // at a distance of 1m, disparity is 7-8
      uint8_t thr1 = 4;
      uint8_t thr2 = 4;
      stereo_vision(current_image_buffer, disparity_image_buffer_8bit, image_width, image_height, disparity_range, thr1,
                    thr2);

      SendDisparityMap(disparity_image_buffer_8bit);

      // disparity thresholding:
      uint8_t disparity_threshold = 5;
      uint32_t disparities_high = 0;
      disparities_high = evaluate_disparities(disparity_image_buffer_8bit, image_width, image_height, disparity_threshold,
                                              disparities_high);

      disparity_threshold = 5;

      uint8_t altitude_levels = 5;
      uint32_t disparities_altitude[altitude_levels];
      uint32_t bad_pixels[altitude_levels];
      uint8_t ii;
      for (ii = 0; ii < altitude_levels; ii++) {
        disparities_altitude[ii] = 0;
        bad_pixels[ii] = 0;
      }
      uint16_t x_min = 0;
      uint16_t x_max = image_width;
      evaluate_disparities_altitude(disparity_image_buffer_8bit, image_width, image_height, disparity_threshold,
                                    disparities_altitude, altitude_levels, x_min, x_max, bad_pixels);
      /*SendCommandHeight('D');
      SendCommandHeight(' ');
      print_numbers(disparities_altitude, altitude_levels, 1);
      SendCommandHeight('B');
      SendCommandHeight(' ');
      print_numbers(bad_pixels, altitude_levels, 1);*/
      //print_number(disparities_high_altitude[4], 1);


      // entropy of patches:
      uint32_t entropy = get_entropy_patches(0, current_image_buffer, image_width, image_height, 0, 40, 0, 96, 5, 10, 10);

      //print_number(entropy);

      if (TIME_PROCESSING) {
        stop = TIM_GetCounter(TIM2);
        uint32_t processing_time = 0;
        if (start > stop) {
          // an autoreload of the counter has taken place, so we need to split the processing time in two parts:
          processing_time = ((20000 - start) + stop) / 2; // in ms
        } else {
          processing_time = (stop - start) / 2; // in ms
        }

        print_number(processing_time, 1);
      }

      //SendDisparityMap(disparity_image_buffer_8bit);


      // Control logic
      if (phase == 1) { // unobstructed flight
        if (disparities_high > obst_thr1 || entropy < obst_entr) { // if true, obstacle in sight
          obst_dect++;
        } else {
          obst_dect = 0;
        }

        if (obst_dect > obst_thr2) { // if true, obstacle is consistent
          phase = 2;
          obst_dect = 0; // set zero for later
        }
      } else if (phase == 2) { // obstacle detected, wait for action
        if (obst_time == 0) { // when entering phase, set start time
          obst_time = TIM_GetCounter(TIM2);
        }

        if ((TIM_GetCounter(TIM2) - obst_time) > obst_wait * 2) {  // wait (2 clocks per ms)
          phase = 3;
          obst_time = 0; // set zero for later
        }
      } else if (phase == 3) { // avoid
        // Turn command signal for AutoPilot ???
        if (disparities_high < obst_thr3) { // if true, flight direction is safe
          obst_free++;
        } else {
          obst_free = 0;
        }

        if (obst_free > obst_thr4) { // if true, consistently no obstacles
          if (entropy > obst_entr) { // do the entropy check
            phase = 4;
            obst_free = 0; // set zero for later
          }
        }
      } else if (phase == 4) { // fly straight, but be aware of undetected obstacles
        if (obst_time2 == 0) { // when entering phase, set start time
          obst_time2 =  TIM_GetCounter(TIM2);
        }

        if (disparities_high > obst_thr1) { // if true, obstacle in sight
          obst_dect2++;
        } else {
          obst_dect2 = 0;
        }

        if (obst_dect2 > obst_thr5) { // if true, obstacle is consistent
          phase = 3; // go back to phase 3
          obst_time2 = 0; // set zero for later
          obst_dect2 = 0; // set zero for later

        } else if ((TIM_GetCounter(TIM2) - obst_time2) > obst_wait2 * 2) {  // wait (2 clocks per ms)
          phase = 1;
          obst_time2 = 0; // set zero for later
          obst_dect2 = 0;
        }
      }

      // turn command:
//      if ( phase == 3 )
//        SendCommand(1);
//      else
//        SendCommand(0);


      // height control:
      uint32_t threshold_height_control = 200;
      uint32_t diff_threshold = 0;
//      uint8_t execute_height_control = 0;
      int16_t range_control = 40;
      int16_t half_range = range_control / 2;
      int16_t difference_height = half_range;
      int16_t abs_diff = 0;;
      int16_t large_diff = 250;//800;
      int16_t resol = large_diff / half_range;

//      for(ii = 0; ii < altitude_levels; ii++)
//      {
//        if(disparities_altitude[ii] > threshold_height_control)
//        {
//          execute_height_control = 1;
//          break;
//        }
//      }

//      if(execute_height_control)
//      {
//        difference_height =  - (int16_t)disparities_altitude[0] - (int16_t)disparities_altitude[1] + (int16_t)disparities_altitude[3] + (int16_t)disparities_altitude[4];
//
//        abs_diff = (difference_height < 0) ? -difference_height : difference_height;
//
//        //print_number(abs_diff, 0);
//
//        if(abs_diff > diff_threshold)
//        {
//          if(difference_height < 0)
//          {
//            //difference_height += diff_threshold;
//            difference_height /= resol;
//            if(difference_height < -half_range) difference_height = -half_range;
//          }
//          else
//          {
//            //difference_height -= diff_threshold;
//            difference_height /= resol;
//            if(difference_height > half_range) difference_height = half_range;
//          }
//
//          difference_height += half_range; // to make it a purely positive number
//
//        } else
//          difference_height = half_range;
//      }

      lp_height_disparity = (FACTOR1 * disparities_altitude[altitude_levels - 1] + FACTOR2 * lp_height_disparity) / TOTAL;
      int16_t set_point = 200;
      abs_diff = lp_height_disparity - set_point;

//      print_number(lp_height_disparity, 1);

      if (abs_diff < 0) { // too high
        difference_height = abs_diff / resol;
        // quadratic response:
        difference_height = -(difference_height * difference_height) / 20;
        if (difference_height < -half_range) { difference_height = -half_range; }
      } else { // too low
        //difference_height -= diff_threshold;
        difference_height = abs_diff / resol;
        // quadratic response:
        difference_height = (difference_height * difference_height) / 20;
        if (difference_height > half_range) { difference_height = half_range; }
      }

      difference_height += half_range; // to make it a purely positive number

      char first_char = '0' + (char)((uint16_t)difference_height / 10);
      char second_char = '0' + (char)((uint16_t)difference_height % 10);
      //SendCommandHeight(first_char);
      //SendCommandHeight(second_char);

      //SendCommandHeight(' ');
//      SendCommandHeight('B');
//      SendCommandHeight(' ');
//      print_numbers(bad_pixels, altitude_levels, 1);
      //SendCommandHeight('\n');
      //SendCommandHeight('\r');

      // end height control


    }

  }
}

/**
  * @brief  Delay Function.
  * @param  nCount:specifies the Delay time length.
  * @retval None
  */
void Delay(__IO uint32_t nCount)
{
  while (nCount--) {
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

