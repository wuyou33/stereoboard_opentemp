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
#include "stm32f4xx_conf.h"
#include "jpeg.h"
#include "arm_math.h"
#include "stereo_vision.h"
#include "window_detection.h"
#include "filter_color.h"
#include "usbd_cdc_core.h"
#include "usbd_usr.h"
#include "usbd_desc.h"
#include "usbd_cdc_vcp.h"

#include "main_parameters.h"

__ALIGN_BEGIN USB_OTG_CORE_HANDLE  USB_OTG_dev __ALIGN_END;

// integral_image has size 128 * 96 * 4 = 49152 bytes = C000 in hex
uint32_t *integral_image = ((uint32_t *) 0x10000000); // 0x10000000 - 0x1000 FFFF = CCM data RAM
//uint8_t* jpeg_image_buffer_8bit = ((uint8_t*) 0x1000D000); // 0x10000000 - 0x1000 FFFF = CCM data RAM
//uint8_t* disparity_image_buffer_8bit = ((uint8_t*) 0x10000000);

/** @addtogroup StereoCam
  * @{
  */

/* Private function prototypes -----------------------------------------------*/
void Delay(__IO uint32_t nCount);
/* Private functions ---------------------------------------------------------*/


uint16_t map_value_to_range(uint16_t value, uint16_t range, uint16_t min_val, uint16_t max_val)
{
  value = (value < min_val) ? min_val : value;
  value = (value > max_val) ? max_val : value;
  value -= min_val;
  value = (range * value) / (max_val - min_val);
  value = (value >= range) ? range - 1 : value;
  return value;
}


void Send(uint8_t *b)
{
  uint8_t code[4];
  code[0] = 0xff;
  code[1] = 0x00;
  code[2] = 0x00;

  uint16_t width = IMAGE_WIDTH;
  uint16_t height = IMAGE_HEIGHT;

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

  while (usart_tx_ringbuffer_push(code, 10) == 0)
    ;

  int j = 0;
  for (j = 0; j < size; j++) {
    while (usart_tx_ringbuffer_push(b + j, 1) == 0)
      ;
  }
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
  code[0] = 'a' + b;

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

/**************
 * MAIN DEFINES
 **************/
#define STEREO_PIXMUX 0
#define YUV_COLOR 1


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
  camera_cpld_stereo_pixmux();
  //camera_cpld_stereo_linemux();
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
  // Start DMA image transfer interrupts
  camera_dma_it_init();
  // Print welcome message
  char comm_buff[128] = " --- Stereo Camera --- \n\r";
  usart_tx_ringbuffer_push((uint8_t *)&comm_buff, strlen(comm_buff));
  // Disparity image buffer:
  uint8_t disparity_image_buffer_8bit[FULL_IMAGE_SIZE / 2];
  uint16_t ind;
  for (ind = 0; ind < FULL_IMAGE_SIZE / 2; ind++) {
    disparity_image_buffer_8bit[ind] = 0;
  }
  // slight waste of memory, if color is not used:
  uint8_t filtered_image[FULL_IMAGE_SIZE / 2];
  for (ind = 0; ind < FULL_IMAGE_SIZE / 2; ind++) {
    filtered_image[ind] = 0;
  }
  uint8_t min_y, max_y;
  uint32_t image_width = IMAGE_WIDTH;
  uint32_t image_height = IMAGE_HEIGHT;
  uint32_t start, stop;
  init_timer2();

  /*******************
   * MINOR PARAMETERS:
   *******************/

  // Avoidance parameters;
  uint16_t obst_thr1 = 1700; // number of pixels with high disparity [1700] [3000]
  uint8_t obst_thr2 = 5; // number of obstacle detections in row
  uint16_t obst_wait = 0; //2000; // time to wait before avoidance manoeuver [ms]
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

  uint8_t disparity_threshold = 5; // [5] [4]
  uint32_t disparities_high = 0;
  uint32_t entropy;

  // Stereo parameters:
  uint32_t disparity_range = 15; // at a distance of 1m, disparity is 7-8
  uint8_t thr1 = 4;
  uint8_t thr2 = 4;
  uint8_t diff_threshold = 4; // for filtering

  // Color filtering:
  uint8_t min_U = 0;
  uint8_t min_V = 128;
  uint8_t max_U = 128;
  uint8_t max_V = 255;
  uint16_t n_red_pixels = 0;

  // Avoidance parameters;
  /*uint8_t disp_threshold = 5; // !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
  uint8_t n_disp_bins = 2;
  uint32_t disparities[n_disp_bins];
  uint32_t avg_disparities[n_disp_bins];
  uint8_t bin;
  for(bin = 0; bin < n_disp_bins; bin++)
  {
    avg_disparities[bin] = 0;
  }*/

  // window coordinate:
  /*uint16_t coordinate[2]; // instantaneous
  uint16_t window_coordinate[2]; // low-pass filtered
  window_coordinate[0] = image_width / 2;
  window_coordinate[1] = image_height / 2;
  uint16_t avg_response = 100;
  uint16_t window_threshold = 90;
  uint16_t escape_coordinate[2];
  uint16_t min_disp = 0;
  uint8_t n_cells = 5;
  uint16_t min_response = 100;
  uint8_t n_bits = 2;
  uint32_t weight_new = 1;
  uint32_t weight_old = 3;
  uint32_t weight_total = weight_old + weight_new;*/

  // counter for toggling image type: ONLY necessary if we USE_COLOR
  uint16_t counter = 0;
  uint8_t toggled = 0;
  uint8_t toggle_image_type = STEREO_PIXMUX;

  // timer:
  // start = TIM_GetCounter ( TIM2 );

  /***********
   * MAIN LOOP
   ***********/

  int processed = 0;
  while (1) {
    // wait for new frame
    while (frame_counter == processed)
      ;
    if (SEND_IMAGE) {
      Send(current_image_buffer);
    }

    if (!USE_COLOR || toggled != frame_counter - 1) {

      processed = frame_counter;

      if (toggle_image_type == STEREO_PIXMUX) {
        // Determine disparities:
        min_y = 0;
        max_y = 95;
        //stereo_vision(current_image_buffer, disparity_image_buffer_8bit, image_width, image_height, disparity_range, thr1, thr2, min_y, max_y);
        stereo_vision_Kirk(current_image_buffer, disparity_image_buffer_8bit, image_width, image_height, disparity_range, thr1,
                           thr2, min_y, max_y);



        if (SEND_DISPARITY_MAP) {
          SendDisparityMap(disparity_image_buffer_8bit);
        }

        disparities_high = evaluate_disparities(disparity_image_buffer_8bit, image_width, image_height, disparity_threshold,
                                                disparities_high);

        // entropy of patches:
        entropy = get_entropy_patches(0, current_image_buffer, image_width, image_height, 0, 40, 0, 96, 5, 10, 10);

        // min_response = detect_window_sizes(disparity_image_buffer_8bit, image_width, image_height, coordinate, integral_image, MODE_DISPARITY);

        // left / right control in obstacle zone
        // min_y = 24;
        // max_y = 72;
        // filter_disparity_map(disparity_image_buffer_8bit, diff_threshold, image_width, image_height, min_y, max_y);
        // evaluate_central_disparities2(disparity_image_buffer_8bit, image_width, image_height, disparities, n_disp_bins, min_y, max_y, disp_threshold);

        // Detect window with illuminance:
        // transform_illuminance_image(current_image_buffer, disparity_image_buffer_8bit, image_width, image_height, n_bits, BRIGHT_WINDOW);
        // min_response = detect_window_sizes(disparity_image_buffer_8bit, image_width, image_height, coordinate, integral_image, MODE_ILLUMINANCE);
        //min_disp = detect_escape(disparity_image_buffer_8bit, image_width, image_height, escape_coordinate, integral_image, n_cells);

      } else {
        led_switch();

        n_red_pixels = filter_red_color(current_image_buffer, filtered_image, image_width, image_height, min_U, max_U, min_V,
                                        max_V);
        /*
                // Detect window with filtered color:
                min_response = detect_window_sizes(filtered_image, image_width, image_height, coordinate, integral_image, MODE_FILTER);
                //min_disp = detect_escape(filtered_image, image_width, image_height, escape_coordinate, integral_image, n_cells);
                 *
                 */
      }

      /*if(min_response < window_threshold)
      {
        led_set();
        // rely on window detection:
        window_coordinate[0] = (weight_old * window_coordinate[0] + weight_new * coordinate[0]) / weight_total;
        window_coordinate[1] = (weight_old * window_coordinate[1] + weight_new * coordinate[1]) / weight_total;
        //window_coordinate[0] = (weight_old * window_coordinate[0] + weight_new * 64) / weight_total;
        //window_coordinate[1] = (weight_old * window_coordinate[1] + weight_new * 48) / weight_total;
        //window_coordinate[0] = (weight_old * window_coordinate[0] + weight_new * escape_coordinate[0]) / weight_total;
        //window_coordinate[1] = (weight_old * window_coordinate[1] + weight_new * escape_coordinate[1]) / weight_total;
      }
      else
      {
        led_clear();
        // rely on escape route flying:
        //window_coordinate[0] = (weight_old * window_coordinate[0] + weight_new * escape_coordinate[0]) / weight_total;
        //window_coordinate[1] = (weight_old * window_coordinate[1] + weight_new * escape_coordinate[1]) / weight_total;
        window_coordinate[0] = (weight_old * window_coordinate[0] + weight_new * 64) / weight_total;
        window_coordinate[1] = (weight_old * window_coordinate[1] + weight_new * 48) / weight_total;
        //window_coordinate[0] = (weight_old * window_coordinate[0] + weight_new * coordinate[0]) / weight_total;
        //window_coordinate[1] = (weight_old * window_coordinate[1] + weight_new * coordinate[1]) / weight_total;
      }*/

      if (SEND_ILLUMINANCE) {
        uint16_t xx, yy;
        /*if(min_response < window_threshold)
        {
          for(xx=window_coordinate[0]-2;xx<window_coordinate[0]+2;xx++)
          {
            for(yy = window_coordinate[1]-2; yy<window_coordinate[1]+2;yy++)
            {
              disparity_image_buffer_8bit[window_coordinate[0]+window_coordinate[1]*image_width] = 255;
            }
          }
        }*/
        SendDisparityMap(disparity_image_buffer_8bit);
      }
      if (SEND_FILTER) {
        uint16_t xx, yy;
        /*if(min_response < window_threshold)
        {
          for(xx=window_coordinate[0]-2;xx<window_coordinate[0]+2;xx++)
          {
            for(yy = window_coordinate[1]-2; yy<window_coordinate[1]+2;yy++)
            {
              filtered_image[window_coordinate[0]+window_coordinate[1]*image_width] = 2;
            }
          }
        }*/
        SendDisparityMap(filtered_image);
      }

      if (SEND_COMMANDS) {
        // Control logic
        if (phase == 1) { // unobstructed flight
          if (disparities_high > obst_thr1) { //|| entropy < obst_entr) // if true, obstacle in sight
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
        if (phase == 3) {
          SendCommand(1);
        } else {
          SendCommand(0);
        }

        if (phase == 2 || phase == 3) {
          led_set();
        } else {
          led_clear();
        }

      }

      if (USE_COLOR) {
        if (toggle_image_type != YUV_COLOR) {
          camera_cpld_stereo_right();
          toggle_image_type = YUV_COLOR;
        }

//        counter++;
//        if(counter >= MAX_RATIO)
//        {
//          counter = 0;
//        }
//        if(counter < COLOR_RATIO && toggle_image_type != YUV_COLOR)
//        {
//          camera_cpld_stereo_left();
//          toggled = processed; // skip the next frame
//          toggle_image_type = YUV_COLOR;
//        }
//        else if(toggle_image_type != STEREO_PIXMUX)
//        {
//          camera_cpld_stereo_pixmux();
//          toggled = processed; // skip the next frame
//          toggle_image_type = STEREO_PIXMUX;
//        }
      }
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

