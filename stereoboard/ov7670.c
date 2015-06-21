
#include "ov7670.h"
#include "sccb.h"


/**
  * @brief  Configures the ov7670 camera
  */

void camera_ov7670_i2c_init(void)
{
  SCCB_Init();
}



/***************************************************************
 *
 *
 *      Camera register defines
 *
 *
 **************************************************************/

#define OV7670_ADDR      0x42


void camera_ov7670_config(void)
{
// TODO
#ifdef USE_RGB565
#warning USING_RGB
#else
#endif


  // Reset
  SCCB_WriteReg(OV7670_ADDR, REG_COM7, COM7_RESET);
  Delay(0x07FFFF);
  //Delay(0x07FFFF);

  SCCB_WriteReg(OV7670_ADDR, REG_CLKRC, 0x82); // Clock source
  SCCB_WriteReg(OV7670_ADDR, DBLV, 0x40);
  SCCB_WriteReg(OV7670_ADDR, REG_COM7, 0x08); // output format: yuv
  SCCB_WriteReg(OV7670_ADDR, REG_COM10, 0x02);               // 0x02   VSYNC negative (http://nasulica.homelinux.org/?p=959)


  SCCB_WriteReg(OV7670_ADDR, REG_COM11,  0x00); // Night Mode Off
  //SCCB_WriteReg(OV7670_ADDR, REG_COM11,  0x80); // Night Mode Light
  //SCCB_WriteReg(OV7670_ADDR, REG_COM11,  0xC0); // Night Mode Medium
  //SCCB_WriteReg(OV7670_ADDR, REG_COM11,  0xE0); // Night Mode Strong

  //SCCB_WriteReg(OV7670_ADDR, REG_TSLB, 0x04);  // UYVY YUYV ...
  //SCCB_WriteReg(OV7670_ADDR, REG_TSLB, 0x04);
  //SCCB_WriteReg(OV7670_ADDR, REG_COM7, 0x04); /* output format: rgb */

  //SCCB_WriteReg(OV7670_ADDR, REG_RGB444, 0x00); /* disable RGB444 */
  //SCCB_WriteReg(OV7670_ADDR, REG_COM15, 0xD0); /* set RGB565 */

      /* not even sure what all these do, gonna check the oscilloscope and go
       * from there... */
#if 0
  SCCB_WriteReg(OV7670_ADDR, REG_HSTART, 0x16);
  SCCB_WriteReg(OV7670_ADDR, REG_HSTOP, 0x04);
  SCCB_WriteReg(OV7670_ADDR, REG_HREF, 0x24);
  SCCB_WriteReg(OV7670_ADDR, REG_VSTART, 0x02);
  SCCB_WriteReg(OV7670_ADDR, REG_VSTOP, 0x7a);
  SCCB_WriteReg(OV7670_ADDR, REG_VREF, 0x0a);
#endif
  //  SCCB_WriteReg(OV7670_ADDR, REG_MVFP, 0x27); //Vflip?
  //  SCCB_WriteReg(OV7670_ADDR, REG_COM10, 0x02);  // VSYNC NEGATIVE?
  //  SCCB_WriteReg(OV7670_ADDR, REG_COM10, 0x20);  // NO PCLK outside HREF
#if 1
  SCCB_WriteReg(OV7670_ADDR, REG_COM3, 0x04);     // Scale enable
  SCCB_WriteReg(OV7670_ADDR, REG_COM14, 0x1a); // divide by 4
  SCCB_WriteReg(OV7670_ADDR, 0x72, 0x22); // downsample by 4
  SCCB_WriteReg(OV7670_ADDR, 0x73, 0x02); // divide by 4
#endif
#if 0
  SCCB_WriteReg(OV7670_ADDR, REG_COM3, 0x04);     // Scale enable
  SCCB_WriteReg(OV7670_ADDR, REG_COM14, 0x1b); // divide by 8
  SCCB_WriteReg(OV7670_ADDR, 0x72, 0x33); // downsample by 8
  SCCB_WriteReg(OV7670_ADDR, 0x73, 0xf3); // divide by 8
#endif

      // test pattern
  //SCCB_WriteReg(OV7670_ADDR, 0x70, 1 << 7 + 0x4A);
  //SCCB_WriteReg(OV7670_ADDR, 0x70, 0x35 + 0x80);

#if 0
      // COLOR SETTING V1
  SCCB_WriteReg(OV7670_ADDR, 0x4f, 0x80);
  SCCB_WriteReg(OV7670_ADDR, 0x50, 0x80);
  SCCB_WriteReg(OV7670_ADDR, 0x51, 0x00);
  SCCB_WriteReg(OV7670_ADDR, 0x52, 0x22);
  SCCB_WriteReg(OV7670_ADDR, 0x53, 0x5e);
  SCCB_WriteReg(OV7670_ADDR, 0x54, 0x80);
  SCCB_WriteReg(OV7670_ADDR, 0x56, 0x40);
  SCCB_WriteReg(OV7670_ADDR, 0x58, 0x9e);
  SCCB_WriteReg(OV7670_ADDR, 0x59, 0x88);
  SCCB_WriteReg(OV7670_ADDR, 0x5a, 0x88);
  SCCB_WriteReg(OV7670_ADDR, 0x5b, 0x44);
  SCCB_WriteReg(OV7670_ADDR, 0x5c, 0x67);
  SCCB_WriteReg(OV7670_ADDR, 0x5d, 0x49);
  SCCB_WriteReg(OV7670_ADDR, 0x5e, 0x0e);
  SCCB_WriteReg(OV7670_ADDR, 0x69, 0x00);
  SCCB_WriteReg(OV7670_ADDR, 0x6a, 0x40);
  SCCB_WriteReg(OV7670_ADDR, 0x6b, 0x0a);
  SCCB_WriteReg(OV7670_ADDR, 0x6c, 0x0a);
  SCCB_WriteReg(OV7670_ADDR, 0x6d, 0x55);
  SCCB_WriteReg(OV7670_ADDR, 0x6e, 0x11);
  SCCB_WriteReg(OV7670_ADDR, 0x6f, 0x9f);

  SCCB_WriteReg(OV7670_ADDR, 0xb0, 0x84);
#endif



#if 0
  SCCB_WriteReg(OV7670_ADDR, REG_RGB444, 0x00);              // Disable RGB444
  SCCB_WriteReg(OV7670_ADDR, REG_MVFP, 0x27);                // mirror image

  SCCB_WriteReg(OV7670_ADDR, REG_CLKRC, 0x80);               // prescaler x1
  SCCB_WriteReg(OV7670_ADDR, DBLV, 0x0a);                    // bypass PLL

  SCCB_WriteReg(OV7670_ADDR, REG_COM11, 0x0A) ;
  SCCB_WriteReg(OV7670_ADDR, REG_TSLB, 0x0D);                // 0D = UYVY  04 = YUYV
  SCCB_WriteReg(OV7670_ADDR, REG_COM13, 0x88);               // connect to REG_TSLB
#endif


#ifdef USE_RGB565
      SCCB_WriteReg(OV7670_ADDR, REG_COM7, 0x04);           // RGB + color bar disable
      SCCB_WriteReg(OV7670_ADDR, REG_RGB444, 0x00);         // Disable RGB444
      SCCB_WriteReg(OV7670_ADDR, REG_COM15, 0x10);          // Set rgb565 with Full range    0xD0
      SCCB_WriteReg(OV7670_ADDR, REG_COM3, 0x04);
      SCCB_WriteReg(OV7670_ADDR, REG_CLKRC, 0x80);          // prescaler x1
#endif
#if 0
      SCCB_WriteReg(OV7670_ADDR, REG_COM7, 0x00);           // YUV
      SCCB_WriteReg(OV7670_ADDR, REG_COM17, 0x00);          // color bar disable
      SCCB_WriteReg(OV7670_ADDR, REG_COM3, 0x04);
      SCCB_WriteReg(OV7670_ADDR, REG_COM15, 0xC0);          // Set normal rgb with Full range
#endif

#if 0
  SCCB_WriteReg(OV7670_ADDR, 0x70, 0x3A);                   // Scaling Xsc
  SCCB_WriteReg(OV7670_ADDR, 0x71, 0x35);                   // Scaling Ysc
  SCCB_WriteReg(OV7670_ADDR, 0xA2, 0x02);                   // pixel clock delay
#endif

#if 0  // 160*120
#warning SIZEOK
      SCCB_WriteReg(OV7670_ADDR, REG_COM14, 0x1a);          // divide by 4
      SCCB_WriteReg(OV7670_ADDR, 0x72, 0x22);               // downsample by 4
      SCCB_WriteReg(OV7670_ADDR, 0x73, 0xf2);               // divide by 4
      SCCB_WriteReg(OV7670_ADDR, REG_HREF, 0xa4);
      SCCB_WriteReg(OV7670_ADDR, REG_HSTART, 0x16);
      SCCB_WriteReg(OV7670_ADDR, REG_HSTOP, 0x04);
      SCCB_WriteReg(OV7670_ADDR, REG_VREF, 0x0a);
      SCCB_WriteReg(OV7670_ADDR, REG_VSTART, 0x02);
      SCCB_WriteReg(OV7670_ADDR, REG_VSTOP, 0x7a);
#endif
#if 0
      SCCB_WriteReg(OV7670_ADDR, 0x7a, 0x20);
      SCCB_WriteReg(OV7670_ADDR, 0x7b, 0x1c);
      SCCB_WriteReg(OV7670_ADDR, 0x7c, 0x28);
      SCCB_WriteReg(OV7670_ADDR, 0x7d, 0x3c);
      SCCB_WriteReg(OV7670_ADDR, 0x7e, 0x5a);
      SCCB_WriteReg(OV7670_ADDR, 0x7f, 0x68);
      SCCB_WriteReg(OV7670_ADDR, 0x80, 0x76);
      SCCB_WriteReg(OV7670_ADDR, 0x81, 0x80);
      SCCB_WriteReg(OV7670_ADDR, 0x82, 0x88);
      SCCB_WriteReg(OV7670_ADDR, 0x83, 0x8f);
      SCCB_WriteReg(OV7670_ADDR, 0x84, 0x96);
      SCCB_WriteReg(OV7670_ADDR, 0x85, 0xa3);
      SCCB_WriteReg(OV7670_ADDR, 0x86, 0xaf);
      SCCB_WriteReg(OV7670_ADDR, 0x87, 0xc4);
      SCCB_WriteReg(OV7670_ADDR, 0x88, 0xd7);
      SCCB_WriteReg(OV7670_ADDR, 0x89, 0xe8);

      SCCB_WriteReg(OV7670_ADDR, 0x13, 0xe0);
      SCCB_WriteReg(OV7670_ADDR, 0x00, 0x00);
      SCCB_WriteReg(OV7670_ADDR, 0x10, 0x00);
      SCCB_WriteReg(OV7670_ADDR, 0x0d, 0x40);
      SCCB_WriteReg(OV7670_ADDR, 0x14, 0x18);
      SCCB_WriteReg(OV7670_ADDR, 0xa5, 0x05);
      SCCB_WriteReg(OV7670_ADDR, 0xab, 0x07);
      SCCB_WriteReg(OV7670_ADDR, 0x24, 0x95);
      SCCB_WriteReg(OV7670_ADDR, 0x25, 0x33);
      SCCB_WriteReg(OV7670_ADDR, 0x26, 0xe3);
      SCCB_WriteReg(OV7670_ADDR, 0x9f, 0x78);
      SCCB_WriteReg(OV7670_ADDR, 0xa0, 0x68);
      SCCB_WriteReg(OV7670_ADDR, 0xa1, 0x03);
      SCCB_WriteReg(OV7670_ADDR, 0xa6, 0xd8);
      SCCB_WriteReg(OV7670_ADDR, 0xa7, 0xd8);
      SCCB_WriteReg(OV7670_ADDR, 0xa8, 0xf0);
      SCCB_WriteReg(OV7670_ADDR, 0xa9, 0x90);
      SCCB_WriteReg(OV7670_ADDR, 0xaa, 0x94);
      SCCB_WriteReg(OV7670_ADDR, 0x13, 0xe5);

      SCCB_WriteReg(OV7670_ADDR, 0x0e, 0x61);
      SCCB_WriteReg(OV7670_ADDR, 0x0f, 0x4b);
      SCCB_WriteReg(OV7670_ADDR, 0x16, 0x02);

      SCCB_WriteReg(OV7670_ADDR, 0x21, 0x02);
      SCCB_WriteReg(OV7670_ADDR, 0x22, 0x91);
      SCCB_WriteReg(OV7670_ADDR, 0x29, 0x07);
      SCCB_WriteReg(OV7670_ADDR, 0x33, 0x0b);
      SCCB_WriteReg(OV7670_ADDR, 0x35, 0x0b);
      SCCB_WriteReg(OV7670_ADDR, 0x37, 0x1d);
      SCCB_WriteReg(OV7670_ADDR, 0x38, 0x71);
      SCCB_WriteReg(OV7670_ADDR, 0x39, 0x2a);
      SCCB_WriteReg(OV7670_ADDR, 0x3c, 0x78);
      SCCB_WriteReg(OV7670_ADDR, 0x4d, 0x40);
      SCCB_WriteReg(OV7670_ADDR, 0x4e, 0x20);
      SCCB_WriteReg(OV7670_ADDR, 0x69, 0x00);

      SCCB_WriteReg(OV7670_ADDR, 0x74, 0x10);
      SCCB_WriteReg(OV7670_ADDR, 0x8d, 0x4f);
      SCCB_WriteReg(OV7670_ADDR, 0x8e, 0x00);
      SCCB_WriteReg(OV7670_ADDR, 0x8f, 0x00);
      SCCB_WriteReg(OV7670_ADDR, 0x90, 0x00);
      SCCB_WriteReg(OV7670_ADDR, 0x91, 0x00);
      SCCB_WriteReg(OV7670_ADDR, 0x92, 0x00);

      SCCB_WriteReg(OV7670_ADDR, 0x96, 0x00);
      SCCB_WriteReg(OV7670_ADDR, 0x9a, 0x80);
      SCCB_WriteReg(OV7670_ADDR, 0xb0, 0x84);
      SCCB_WriteReg(OV7670_ADDR, 0xb1, 0x0c);
      SCCB_WriteReg(OV7670_ADDR, 0xb2, 0x0e);
      SCCB_WriteReg(OV7670_ADDR, 0xb3, 0x82);
      SCCB_WriteReg(OV7670_ADDR, 0xb8, 0x0a);

      SCCB_WriteReg(OV7670_ADDR, 0x43, 0x0a);
      SCCB_WriteReg(OV7670_ADDR, 0x44, 0xf0);
      SCCB_WriteReg(OV7670_ADDR, 0x45, 0x34);
      SCCB_WriteReg(OV7670_ADDR, 0x46, 0x58);
      SCCB_WriteReg(OV7670_ADDR, 0x47, 0x28);
      SCCB_WriteReg(OV7670_ADDR, 0x48, 0x3a);
      SCCB_WriteReg(OV7670_ADDR, 0x59, 0x88);
      SCCB_WriteReg(OV7670_ADDR, 0x5a, 0x88);
      SCCB_WriteReg(OV7670_ADDR, 0x5b, 0x44);
      SCCB_WriteReg(OV7670_ADDR, 0x5c, 0x67);
      SCCB_WriteReg(OV7670_ADDR, 0x5d, 0x49);
      SCCB_WriteReg(OV7670_ADDR, 0x5e, 0x0e);
      SCCB_WriteReg(OV7670_ADDR, 0x64, 0x04);
      SCCB_WriteReg(OV7670_ADDR, 0x65, 0x20);
      SCCB_WriteReg(OV7670_ADDR, 0x66, 0x05);
      SCCB_WriteReg(OV7670_ADDR, 0x94, 0x04);
      SCCB_WriteReg(OV7670_ADDR, 0x95, 0x08);

      SCCB_WriteReg(OV7670_ADDR, 0x6c, 0x0a);
      SCCB_WriteReg(OV7670_ADDR, 0x6d, 0x55);
      SCCB_WriteReg(OV7670_ADDR, 0x6e, 0x11);
      SCCB_WriteReg(OV7670_ADDR, 0x6f, 0x9f);
      SCCB_WriteReg(OV7670_ADDR, 0x6a, 0x40);
      SCCB_WriteReg(OV7670_ADDR, 0x01, 0x40);
      SCCB_WriteReg(OV7670_ADDR, 0x02, 0x40);
      SCCB_WriteReg(OV7670_ADDR, 0x13, 0xe7);
      SCCB_WriteReg(OV7670_ADDR, 0x15, 0x02);

      SCCB_WriteReg(OV7670_ADDR, 0x4f, 0x80);
      SCCB_WriteReg(OV7670_ADDR, 0x50, 0x80);
      SCCB_WriteReg(OV7670_ADDR, 0x51, 0x00);
      SCCB_WriteReg(OV7670_ADDR, 0x52, 0x22);
      SCCB_WriteReg(OV7670_ADDR, 0x53, 0x5e);
      SCCB_WriteReg(OV7670_ADDR, 0x54, 0x80);
      SCCB_WriteReg(OV7670_ADDR, 0x58, 0x9e);

      SCCB_WriteReg(OV7670_ADDR, 0x41, 0x08);
      SCCB_WriteReg(OV7670_ADDR, 0x3f, 0x00);
      SCCB_WriteReg(OV7670_ADDR, 0x75, 0x05);
      SCCB_WriteReg(OV7670_ADDR, 0x76, 0xe1);
      SCCB_WriteReg(OV7670_ADDR, 0x4c, 0x00);
      SCCB_WriteReg(OV7670_ADDR, 0x77, 0x01);
      SCCB_WriteReg(OV7670_ADDR, 0x3d, 0xc1);
      SCCB_WriteReg(OV7670_ADDR, 0x4b, 0x09);
      SCCB_WriteReg(OV7670_ADDR, 0xc9, 0x60);
      SCCB_WriteReg(OV7670_ADDR, 0x41, 0x38);
      SCCB_WriteReg(OV7670_ADDR, 0x56, 0x40);

      SCCB_WriteReg(OV7670_ADDR, 0x34, 0x11);
      SCCB_WriteReg(OV7670_ADDR, 0x3b, 0x02);
      SCCB_WriteReg(OV7670_ADDR, 0xa4, 0x88);
      SCCB_WriteReg(OV7670_ADDR, 0x96, 0x00);
      SCCB_WriteReg(OV7670_ADDR, 0x97, 0x30);
      SCCB_WriteReg(OV7670_ADDR, 0x98, 0x20);
      SCCB_WriteReg(OV7670_ADDR, 0x99, 0x30);
      SCCB_WriteReg(OV7670_ADDR, 0x9a, 0x84);
      SCCB_WriteReg(OV7670_ADDR, 0x9b, 0x29);
      SCCB_WriteReg(OV7670_ADDR, 0x9c, 0x03);
      SCCB_WriteReg(OV7670_ADDR, 0x9d, 0x4c);
      SCCB_WriteReg(OV7670_ADDR, 0x9e, 0x3f);
      SCCB_WriteReg(OV7670_ADDR, 0x78, 0x04);

      SCCB_WriteReg(OV7670_ADDR, 0x79, 0x01);
      SCCB_WriteReg(OV7670_ADDR, 0xc8, 0xf0);
      SCCB_WriteReg(OV7670_ADDR, 0x79, 0x0f);
      SCCB_WriteReg(OV7670_ADDR, 0xc8, 0x00);
      SCCB_WriteReg(OV7670_ADDR, 0x79, 0x10);
      SCCB_WriteReg(OV7670_ADDR, 0xc8, 0x7e);
      SCCB_WriteReg(OV7670_ADDR, 0x79, 0x0a);
      SCCB_WriteReg(OV7670_ADDR, 0xc8, 0x80);
      SCCB_WriteReg(OV7670_ADDR, 0x79, 0x0b);
      SCCB_WriteReg(OV7670_ADDR, 0xc8, 0x01);
      SCCB_WriteReg(OV7670_ADDR, 0x79, 0x0c);
      SCCB_WriteReg(OV7670_ADDR, 0xc8, 0x0f);
      SCCB_WriteReg(OV7670_ADDR, 0x79, 0x0d);
      SCCB_WriteReg(OV7670_ADDR, 0xc8, 0x20);
      SCCB_WriteReg(OV7670_ADDR, 0x79, 0x09);
      SCCB_WriteReg(OV7670_ADDR, 0xc8, 0x80);
      SCCB_WriteReg(OV7670_ADDR, 0x79, 0x02);
      SCCB_WriteReg(OV7670_ADDR, 0xc8, 0xc0);
      SCCB_WriteReg(OV7670_ADDR, 0x79, 0x03);
      SCCB_WriteReg(OV7670_ADDR, 0xc8, 0x40);
      SCCB_WriteReg(OV7670_ADDR, 0x79, 0x05);
      SCCB_WriteReg(OV7670_ADDR, 0xc8, 0x30);
      SCCB_WriteReg(OV7670_ADDR, 0x79, 0x26);
      SCCB_WriteReg(OV7670_ADDR, 0x09, 0x03);
      SCCB_WriteReg(OV7670_ADDR, 0x3b, 0x42);

      SCCB_WriteReg(OV7670_ADDR, 0xff, 0xff);   /* END MARKER */
#endif


#if (IMAGE_WIDTH == 320) // *240
      SCCB_WriteReg(OV7670_ADDR, REG_COM14, 0x19);
      SCCB_WriteReg(OV7670_ADDR, 0x72, 0x11);
      SCCB_WriteReg(OV7670_ADDR, 0x73, 0xf1);
      SCCB_WriteReg(OV7670_ADDR, REG_HREF, 0x24);
      SCCB_WriteReg(OV7670_ADDR, REG_HSTART, 0x16);
      SCCB_WriteReg(OV7670_ADDR, REG_HSTOP, 0x04);
      SCCB_WriteReg(OV7670_ADDR, REG_VREF, 0x0a);
      SCCB_WriteReg(OV7670_ADDR, REG_VSTART,0x02);
      SCCB_WriteReg(OV7670_ADDR, REG_VSTOP, 0x7a);

      SCCB_WriteReg(OV7670_ADDR, 0x7a, 0x20);
      SCCB_WriteReg(OV7670_ADDR, 0x7b, 0x1c);
      SCCB_WriteReg(OV7670_ADDR, 0x7c, 0x28);
      SCCB_WriteReg(OV7670_ADDR, 0x7d, 0x3c);
      SCCB_WriteReg(OV7670_ADDR, 0x7e, 0x55);
      SCCB_WriteReg(OV7670_ADDR, 0x7f, 0x68);
      SCCB_WriteReg(OV7670_ADDR, 0x80, 0x76);
      SCCB_WriteReg(OV7670_ADDR, 0x81, 0x80);
      SCCB_WriteReg(OV7670_ADDR, 0x82, 0x88);
      SCCB_WriteReg(OV7670_ADDR, 0x83, 0x8f);
      SCCB_WriteReg(OV7670_ADDR, 0x84, 0x96);
      SCCB_WriteReg(OV7670_ADDR, 0x85, 0xa3);
      SCCB_WriteReg(OV7670_ADDR, 0x86, 0xaf);
      SCCB_WriteReg(OV7670_ADDR, 0x87, 0xc4);
      SCCB_WriteReg(OV7670_ADDR, 0x88, 0xd7);
      SCCB_WriteReg(OV7670_ADDR, 0x89, 0xe8);

      SCCB_WriteReg(OV7670_ADDR, 0x13, 0xe0);
      SCCB_WriteReg(OV7670_ADDR, 0x00, 0x00);
      SCCB_WriteReg(OV7670_ADDR, 0x10, 0x00);
      SCCB_WriteReg(OV7670_ADDR, 0x0d, 0x00);
      SCCB_WriteReg(OV7670_ADDR, 0x14, 0x28);
      SCCB_WriteReg(OV7670_ADDR, 0xa5, 0x05);
      SCCB_WriteReg(OV7670_ADDR, 0xab, 0x07);
      SCCB_WriteReg(OV7670_ADDR, 0x24, 0x75);
      SCCB_WriteReg(OV7670_ADDR, 0x25, 0x63);
      SCCB_WriteReg(OV7670_ADDR, 0x26, 0xA5);
      SCCB_WriteReg(OV7670_ADDR, 0x9f, 0x78);
      SCCB_WriteReg(OV7670_ADDR, 0xa0, 0x68);
      SCCB_WriteReg(OV7670_ADDR, 0xa1, 0x03);
      SCCB_WriteReg(OV7670_ADDR, 0xa6, 0xdf);
      SCCB_WriteReg(OV7670_ADDR, 0xa7, 0xdf);
      SCCB_WriteReg(OV7670_ADDR, 0xa8, 0xf0);
      SCCB_WriteReg(OV7670_ADDR, 0xa9, 0x90);
      SCCB_WriteReg(OV7670_ADDR, 0xaa, 0x94);
      SCCB_WriteReg(OV7670_ADDR, 0x13, 0xe5);

      SCCB_WriteReg(OV7670_ADDR, 0x0e, 0x61);
      SCCB_WriteReg(OV7670_ADDR, 0x0f, 0x4b);
      SCCB_WriteReg(OV7670_ADDR, 0x16, 0x02);
      SCCB_WriteReg(OV7670_ADDR, 0x21, 0x02);
      SCCB_WriteReg(OV7670_ADDR, 0x22, 0x91);
      SCCB_WriteReg(OV7670_ADDR, 0x29, 0x07);
      SCCB_WriteReg(OV7670_ADDR, 0x33, 0x0b);
      SCCB_WriteReg(OV7670_ADDR, 0x35, 0x0b);
      SCCB_WriteReg(OV7670_ADDR, 0x37, 0x1d);
      SCCB_WriteReg(OV7670_ADDR, 0x38, 0x71);
      SCCB_WriteReg(OV7670_ADDR, 0x39, 0x2a);
      SCCB_WriteReg(OV7670_ADDR, 0x3c, 0x78);
      SCCB_WriteReg(OV7670_ADDR, 0x4d, 0x40);
      SCCB_WriteReg(OV7670_ADDR, 0x4e, 0x20);
      SCCB_WriteReg(OV7670_ADDR, 0x69, 0x00);
      SCCB_WriteReg(OV7670_ADDR, 0x6b, 0x00);
      SCCB_WriteReg(OV7670_ADDR, 0x74, 0x19);
      SCCB_WriteReg(OV7670_ADDR, 0x8d, 0x4f);
      SCCB_WriteReg(OV7670_ADDR, 0x8e, 0x00);
      SCCB_WriteReg(OV7670_ADDR, 0x8f, 0x00);
      SCCB_WriteReg(OV7670_ADDR, 0x90, 0x00);
      SCCB_WriteReg(OV7670_ADDR, 0x91, 0x00);
      SCCB_WriteReg(OV7670_ADDR, 0x92, 0x00);
      SCCB_WriteReg(OV7670_ADDR, 0x96, 0x00);
      SCCB_WriteReg(OV7670_ADDR, 0x9a, 0x80);
      SCCB_WriteReg(OV7670_ADDR, 0xb0, 0x84);
      SCCB_WriteReg(OV7670_ADDR, 0xb1, 0x0c);
      SCCB_WriteReg(OV7670_ADDR, 0xb2, 0x0e);
      SCCB_WriteReg(OV7670_ADDR, 0xb3, 0x82);
      SCCB_WriteReg(OV7670_ADDR, 0xb8, 0x0a);
      SCCB_WriteReg(OV7670_ADDR, 0x43, 0x14);
      SCCB_WriteReg(OV7670_ADDR, 0x44, 0xf0);
      SCCB_WriteReg(OV7670_ADDR, 0x45, 0x34);
      SCCB_WriteReg(OV7670_ADDR, 0x46, 0x58);
      SCCB_WriteReg(OV7670_ADDR, 0x47, 0x28);
      SCCB_WriteReg(OV7670_ADDR, 0x48, 0x3a);
      SCCB_WriteReg(OV7670_ADDR, 0x59, 0x88);
      SCCB_WriteReg(OV7670_ADDR, 0x5a, 0x88);
      SCCB_WriteReg(OV7670_ADDR, 0x5b, 0x44);
      SCCB_WriteReg(OV7670_ADDR, 0x5c, 0x67);
      SCCB_WriteReg(OV7670_ADDR, 0x5d, 0x49);
      SCCB_WriteReg(OV7670_ADDR, 0x5e, 0x0e);
      SCCB_WriteReg(OV7670_ADDR, 0x64, 0x04);
      SCCB_WriteReg(OV7670_ADDR, 0x65, 0x20);
      SCCB_WriteReg(OV7670_ADDR, 0x66, 0x05);
      SCCB_WriteReg(OV7670_ADDR, 0x94, 0x04);
      SCCB_WriteReg(OV7670_ADDR, 0x95, 0x08);
      SCCB_WriteReg(OV7670_ADDR, 0x6c, 0x0a);
      SCCB_WriteReg(OV7670_ADDR, 0x6d, 0x55);
      SCCB_WriteReg(OV7670_ADDR, 0x6e, 0x11);
      SCCB_WriteReg(OV7670_ADDR, 0x6f, 0x9f);
      SCCB_WriteReg(OV7670_ADDR, 0x6a, 0x40);
      SCCB_WriteReg(OV7670_ADDR, 0x01, 0x40);
      SCCB_WriteReg(OV7670_ADDR, 0x02, 0x40);
      SCCB_WriteReg(OV7670_ADDR, 0x13, 0xe7);
      SCCB_WriteReg(OV7670_ADDR, 0x15, 0x02);
      SCCB_WriteReg(OV7670_ADDR, 0x4f, 0x80);
      SCCB_WriteReg(OV7670_ADDR, 0x50, 0x80);
      SCCB_WriteReg(OV7670_ADDR, 0x51, 0x00);
      SCCB_WriteReg(OV7670_ADDR, 0x52, 0x22);
      SCCB_WriteReg(OV7670_ADDR, 0x53, 0x5e);
      SCCB_WriteReg(OV7670_ADDR, 0x54, 0x80);
      SCCB_WriteReg(OV7670_ADDR, 0x58, 0x9e);
      SCCB_WriteReg(OV7670_ADDR, 0x41, 0x08);
      SCCB_WriteReg(OV7670_ADDR, 0x3f, 0x00);
      SCCB_WriteReg(OV7670_ADDR, 0x75, 0x05);
      SCCB_WriteReg(OV7670_ADDR, 0x76, 0xe1);
      SCCB_WriteReg(OV7670_ADDR, 0x4c, 0x00);
      SCCB_WriteReg(OV7670_ADDR, 0x77, 0x01);
      SCCB_WriteReg(OV7670_ADDR, 0x3d, 0xc2);
      SCCB_WriteReg(OV7670_ADDR, 0x4b, 0x09);
      SCCB_WriteReg(OV7670_ADDR, 0xc9, 0x60);
      SCCB_WriteReg(OV7670_ADDR, 0x41, 0x38);
      SCCB_WriteReg(OV7670_ADDR, 0x56, 0x40);
      SCCB_WriteReg(OV7670_ADDR, 0x34, 0x11);
      SCCB_WriteReg(OV7670_ADDR, 0x3b, 0x02);
      SCCB_WriteReg(OV7670_ADDR, 0xa4, 0x89);
      SCCB_WriteReg(OV7670_ADDR, 0x96, 0x00);
      SCCB_WriteReg(OV7670_ADDR, 0x97, 0x30);
      SCCB_WriteReg(OV7670_ADDR, 0x98, 0x20);
      SCCB_WriteReg(OV7670_ADDR, 0x99, 0x30);
      SCCB_WriteReg(OV7670_ADDR, 0x9a, 0x84);
      SCCB_WriteReg(OV7670_ADDR, 0x9b, 0x29);
      SCCB_WriteReg(OV7670_ADDR, 0x9c, 0x03);
      SCCB_WriteReg(OV7670_ADDR, 0x9d, 0x4c);
      SCCB_WriteReg(OV7670_ADDR, 0x9e, 0x3f);
      SCCB_WriteReg(OV7670_ADDR, 0x78, 0x04);
      SCCB_WriteReg(OV7670_ADDR, 0x79, 0x01);
      SCCB_WriteReg(OV7670_ADDR, 0xc8, 0xf0);
      SCCB_WriteReg(OV7670_ADDR, 0x79, 0x0f);
      SCCB_WriteReg(OV7670_ADDR, 0xc8, 0x00);
      SCCB_WriteReg(OV7670_ADDR, 0x79, 0x10);
      SCCB_WriteReg(OV7670_ADDR, 0xc8, 0x7e);
      SCCB_WriteReg(OV7670_ADDR, 0x79, 0x0a);
      SCCB_WriteReg(OV7670_ADDR, 0xc8, 0x80);
      SCCB_WriteReg(OV7670_ADDR, 0x79, 0x0b);
      SCCB_WriteReg(OV7670_ADDR, 0xc8, 0x01);
      SCCB_WriteReg(OV7670_ADDR, 0x79, 0x0c);
      SCCB_WriteReg(OV7670_ADDR, 0xc8, 0x0f);
      SCCB_WriteReg(OV7670_ADDR, 0x79, 0x0d);
      SCCB_WriteReg(OV7670_ADDR, 0xc8, 0x20);
      SCCB_WriteReg(OV7670_ADDR, 0x79, 0x09);
      SCCB_WriteReg(OV7670_ADDR, 0xc8, 0x80);
      SCCB_WriteReg(OV7670_ADDR, 0x79, 0x02);
      SCCB_WriteReg(OV7670_ADDR, 0xc8, 0xc0);
      SCCB_WriteReg(OV7670_ADDR, 0x79, 0x03);
      SCCB_WriteReg(OV7670_ADDR, 0xc8, 0x40);
      SCCB_WriteReg(OV7670_ADDR, 0x79, 0x05);
      SCCB_WriteReg(OV7670_ADDR, 0xc8, 0x30);
      SCCB_WriteReg(OV7670_ADDR, 0x79, 0x26);
      SCCB_WriteReg(OV7670_ADDR, 0x09, 0x03);
      SCCB_WriteReg(OV7670_ADDR, 0x3b, 0x42);

      SCCB_WriteReg(OV7670_ADDR, 0xff, 0xff);   /* END MARKER */
#endif

#if (IMAGE_WIDTH == 640) //   // 640*480
      SCCB_WriteReg(OV7670_ADDR, REG_CLKRC, 0x01);
      SCCB_WriteReg(OV7670_ADDR, REG_TSLB,  0x04);
      SCCB_WriteReg(OV7670_ADDR, REG_COM7, 0x01);
      SCCB_WriteReg(OV7670_ADDR, DBLV, 0x4a);
      SCCB_WriteReg(OV7670_ADDR, REG_COM3, 0);
      SCCB_WriteReg(OV7670_ADDR, REG_COM14, 0);

      SCCB_WriteReg(OV7670_ADDR, REG_HSTART, 0x13);
      SCCB_WriteReg(OV7670_ADDR, REG_HSTOP, 0x01);
      SCCB_WriteReg(OV7670_ADDR, REG_HREF, 0xb6);
      SCCB_WriteReg(OV7670_ADDR, REG_VSTART, 0x02);
      SCCB_WriteReg(OV7670_ADDR, REG_VSTOP, 0x7a);
      SCCB_WriteReg(OV7670_ADDR, REG_VREF, 0x0a);
      SCCB_WriteReg(OV7670_ADDR, 0x72, 0x11);
      SCCB_WriteReg(OV7670_ADDR, 0x73, 0xf0);

      /* Gamma curve values */
      SCCB_WriteReg(OV7670_ADDR, 0x7a, 0x20);
      SCCB_WriteReg(OV7670_ADDR, 0x7b, 0x10);
      SCCB_WriteReg(OV7670_ADDR, 0x7c, 0x1e);
      SCCB_WriteReg(OV7670_ADDR, 0x7d, 0x35);
      SCCB_WriteReg(OV7670_ADDR, 0x7e, 0x5a);
      SCCB_WriteReg(OV7670_ADDR, 0x7f, 0x69);
      SCCB_WriteReg(OV7670_ADDR, 0x80, 0x76);
      SCCB_WriteReg(OV7670_ADDR, 0x81, 0x80);
      SCCB_WriteReg(OV7670_ADDR, 0x82, 0x88);
      SCCB_WriteReg(OV7670_ADDR, 0x83, 0x8f);
      SCCB_WriteReg(OV7670_ADDR, 0x84, 0x96);
      SCCB_WriteReg(OV7670_ADDR, 0x85, 0xa3);
      SCCB_WriteReg(OV7670_ADDR, 0x86, 0xaf);
      SCCB_WriteReg(OV7670_ADDR, 0x87, 0xc4);
      SCCB_WriteReg(OV7670_ADDR, 0x88, 0xd7);
      SCCB_WriteReg(OV7670_ADDR, 0x89, 0xe8);

      /* AGC and AEC parameters.  Note we start by disabling those features,
      then turn them only after tweaking the values. */
      SCCB_WriteReg(OV7670_ADDR, 0x13, COM8_FASTAEC | COM8_AECSTEP | COM8_BFILT);
      SCCB_WriteReg(OV7670_ADDR, 0x00, 0);
      SCCB_WriteReg(OV7670_ADDR, 0x10, 0);
      SCCB_WriteReg(OV7670_ADDR, 0x0d, 0x40);
      SCCB_WriteReg(OV7670_ADDR, 0x14, 0x18);
      SCCB_WriteReg(OV7670_ADDR, 0xa5, 0x05);
      SCCB_WriteReg(OV7670_ADDR, 0xab, 0x07);
      SCCB_WriteReg(OV7670_ADDR, 0x24, 0x95);
      SCCB_WriteReg(OV7670_ADDR, 0x25, 0x33);
      SCCB_WriteReg(OV7670_ADDR, 0x26, 0xe3);
      SCCB_WriteReg(OV7670_ADDR, 0x9f, 0x78);
      SCCB_WriteReg(OV7670_ADDR, 0xa0, 0x68);
      SCCB_WriteReg(OV7670_ADDR, 0xa1, 0x03);
      SCCB_WriteReg(OV7670_ADDR, 0xa6, 0xd8);
      SCCB_WriteReg(OV7670_ADDR, 0xa7, 0xd8);
      SCCB_WriteReg(OV7670_ADDR, 0xa8, 0xf0);
      SCCB_WriteReg(OV7670_ADDR, 0xa9, 0x90);
      SCCB_WriteReg(OV7670_ADDR, 0xaa, 0x94);
      SCCB_WriteReg(OV7670_ADDR, 0x13, COM8_FASTAEC|COM8_AECSTEP|COM8_BFILT|COM8_AGC|COM8_AEC);

      /* Almost all of these are magic "reserved" values.  */
      SCCB_WriteReg(OV7670_ADDR, 0x0e, 0x61);
      SCCB_WriteReg(OV7670_ADDR, 0x0f, 0x4b);
      SCCB_WriteReg(OV7670_ADDR, 0x16, 0x02);
      SCCB_WriteReg(OV7670_ADDR, 0x1e, 0x27);
      SCCB_WriteReg(OV7670_ADDR, 0x21, 0x02);
      SCCB_WriteReg(OV7670_ADDR, 0x22, 0x91);
      SCCB_WriteReg(OV7670_ADDR, 0x29, 0x07);
      SCCB_WriteReg(OV7670_ADDR, 0x33, 0x0b);
      SCCB_WriteReg(OV7670_ADDR, 0x35, 0x0b);
      SCCB_WriteReg(OV7670_ADDR, 0x37, 0x1d);
      SCCB_WriteReg(OV7670_ADDR, 0x38, 0x71);
      SCCB_WriteReg(OV7670_ADDR, 0x39, 0x2a);
      SCCB_WriteReg(OV7670_ADDR, 0x3c, 0x78);
      SCCB_WriteReg(OV7670_ADDR, 0x4d, 0x40);
      SCCB_WriteReg(OV7670_ADDR, 0x4e, 0x20);
      SCCB_WriteReg(OV7670_ADDR, 0x69, 0);
      SCCB_WriteReg(OV7670_ADDR, 0x6b, 0x0a);
      SCCB_WriteReg(OV7670_ADDR, 0x74, 0x10);
      SCCB_WriteReg(OV7670_ADDR, 0x8d, 0x4f);
      SCCB_WriteReg(OV7670_ADDR, 0x8e, 0);
      SCCB_WriteReg(OV7670_ADDR, 0x8f, 0);
      SCCB_WriteReg(OV7670_ADDR, 0x90, 0);
      SCCB_WriteReg(OV7670_ADDR, 0x91, 0);
      SCCB_WriteReg(OV7670_ADDR, 0x96, 0);
      SCCB_WriteReg(OV7670_ADDR, 0x9a, 0);
      SCCB_WriteReg(OV7670_ADDR, 0xb0, 0x84);
      SCCB_WriteReg(OV7670_ADDR, 0xb1, 0x0c);
      SCCB_WriteReg(OV7670_ADDR, 0xb2, 0x0e);
      SCCB_WriteReg(OV7670_ADDR, 0xb3, 0x82);
      SCCB_WriteReg(OV7670_ADDR, 0xb8, 0x0a);

      /* More reserved magic, some of which tweaks white balance */
      SCCB_WriteReg(OV7670_ADDR, 0x43, 0x0a);
      SCCB_WriteReg(OV7670_ADDR, 0x44, 0xf0);
      SCCB_WriteReg(OV7670_ADDR, 0x45, 0x34);
      SCCB_WriteReg(OV7670_ADDR, 0x46, 0x58);
      SCCB_WriteReg(OV7670_ADDR, 0x47, 0x28);
      SCCB_WriteReg(OV7670_ADDR, 0x48, 0x3a);
      SCCB_WriteReg(OV7670_ADDR, 0x59, 0x88);
      SCCB_WriteReg(OV7670_ADDR, 0x5a, 0x88);
      SCCB_WriteReg(OV7670_ADDR, 0x5b, 0x44);
      SCCB_WriteReg(OV7670_ADDR, 0x5c, 0x67);
      SCCB_WriteReg(OV7670_ADDR, 0x5d, 0x49);
      SCCB_WriteReg(OV7670_ADDR, 0x5e, 0x0e);
      SCCB_WriteReg(OV7670_ADDR, 0x6c, 0x0a);
      SCCB_WriteReg(OV7670_ADDR, 0x6d, 0x55);
      SCCB_WriteReg(OV7670_ADDR, 0x6e, 0x11);
      SCCB_WriteReg(OV7670_ADDR, 0x6f, 0x9f);
      SCCB_WriteReg(OV7670_ADDR, 0x6a, 0x40);
      SCCB_WriteReg(OV7670_ADDR, 0x01, 0x40);
      SCCB_WriteReg(OV7670_ADDR, 0x02, 0x60);
      SCCB_WriteReg(OV7670_ADDR, 0x13, COM8_FASTAEC|COM8_AECSTEP|COM8_BFILT|COM8_AGC|COM8_AEC|COM8_AWB);

      /* Matrix coefficients */
      SCCB_WriteReg(OV7670_ADDR, 0x4f, 0x80);
      SCCB_WriteReg(OV7670_ADDR, 0x50, 0x80);
      SCCB_WriteReg(OV7670_ADDR, 0x51, 0);
      SCCB_WriteReg(OV7670_ADDR, 0x52, 0x22);
      SCCB_WriteReg(OV7670_ADDR, 0x53, 0x5e);
      SCCB_WriteReg(OV7670_ADDR, 0x54, 0x80);
      SCCB_WriteReg(OV7670_ADDR, 0x58, 0x9e);

      SCCB_WriteReg(OV7670_ADDR, 0x41, 0x08);
      SCCB_WriteReg(OV7670_ADDR, 0x3f, 0);
      SCCB_WriteReg(OV7670_ADDR, 0x75, 0x05);
      SCCB_WriteReg(OV7670_ADDR, 0x76, 0xe1);
      SCCB_WriteReg(OV7670_ADDR, 0x4c, 0);
      SCCB_WriteReg(OV7670_ADDR, 0x77, 0x01);
      SCCB_WriteReg(OV7670_ADDR, 0x3d, 0xc3);
      SCCB_WriteReg(OV7670_ADDR, 0x4b, 0x09);
      SCCB_WriteReg(OV7670_ADDR, 0xc9, 0x60);
      SCCB_WriteReg(OV7670_ADDR, 0x41, 0x38);
      SCCB_WriteReg(OV7670_ADDR, 0x56, 0x40);

      SCCB_WriteReg(OV7670_ADDR, 0x34, 0x11);
      SCCB_WriteReg(OV7670_ADDR, 0x3b, COM11_EXP|COM11_HZAUTO);
      SCCB_WriteReg(OV7670_ADDR, 0xa4, 0x88);
      SCCB_WriteReg(OV7670_ADDR, 0x96, 0);
      SCCB_WriteReg(OV7670_ADDR, 0x97, 0x30);
      SCCB_WriteReg(OV7670_ADDR, 0x98, 0x20);
      SCCB_WriteReg(OV7670_ADDR, 0x99, 0x30);
      SCCB_WriteReg(OV7670_ADDR, 0x9a, 0x84);
      SCCB_WriteReg(OV7670_ADDR, 0x9b, 0x29);
      SCCB_WriteReg(OV7670_ADDR, 0x9c, 0x03);
      SCCB_WriteReg(OV7670_ADDR, 0x9d, 0x4c);
      SCCB_WriteReg(OV7670_ADDR, 0x9e, 0x3f);
      SCCB_WriteReg(OV7670_ADDR, 0x78, 0x04);

      /* Extra-weird stuff.  Some sort of multiplexor register */
      SCCB_WriteReg(OV7670_ADDR, 0x79, 0x01);
      SCCB_WriteReg(OV7670_ADDR, 0xc8, 0xf0);
      SCCB_WriteReg(OV7670_ADDR, 0x79, 0x0f);
      SCCB_WriteReg(OV7670_ADDR, 0xc8, 0x00);
      SCCB_WriteReg(OV7670_ADDR, 0x79, 0x10);
      SCCB_WriteReg(OV7670_ADDR, 0xc8, 0x7e);
      SCCB_WriteReg(OV7670_ADDR, 0x79, 0x0a);
      SCCB_WriteReg(OV7670_ADDR, 0xc8, 0x80);
      SCCB_WriteReg(OV7670_ADDR, 0x79, 0x0b);
      SCCB_WriteReg(OV7670_ADDR, 0xc8, 0x01);
      SCCB_WriteReg(OV7670_ADDR, 0x79, 0x0c);
      SCCB_WriteReg(OV7670_ADDR, 0xc8, 0x0f);
      SCCB_WriteReg(OV7670_ADDR, 0x79, 0x0d);
      SCCB_WriteReg(OV7670_ADDR, 0xc8, 0x20);
      SCCB_WriteReg(OV7670_ADDR, 0x79, 0x09);
      SCCB_WriteReg(OV7670_ADDR, 0xc8, 0x80);
      SCCB_WriteReg(OV7670_ADDR, 0x79, 0x02);
      SCCB_WriteReg(OV7670_ADDR, 0xc8, 0xc0);
      SCCB_WriteReg(OV7670_ADDR, 0x79, 0x03);
      SCCB_WriteReg(OV7670_ADDR, 0xc8, 0x40);
      SCCB_WriteReg(OV7670_ADDR, 0x79, 0x05);
      SCCB_WriteReg(OV7670_ADDR, 0xc8, 0x30);
      SCCB_WriteReg(OV7670_ADDR, 0x79, 0x26);

      SCCB_WriteReg(OV7670_ADDR, 0xff, 0xff); /* END MARKER */
#endif


#if 1 // site 2
      // COLOR SETTING
      SCCB_WriteReg(OV7670_ADDR, 0x4f,0x80);
      SCCB_WriteReg(OV7670_ADDR, 0x50,0x80);
      SCCB_WriteReg(OV7670_ADDR, 0x51,0x00);
      SCCB_WriteReg(OV7670_ADDR, 0x52,0x22);
      SCCB_WriteReg(OV7670_ADDR, 0x53,0x5e);
      SCCB_WriteReg(OV7670_ADDR, 0x54,0x80);
      SCCB_WriteReg(OV7670_ADDR, 0x56,0x40);
      SCCB_WriteReg(OV7670_ADDR, 0x58,0x9e);
      SCCB_WriteReg(OV7670_ADDR, 0x59,0x88);
      SCCB_WriteReg(OV7670_ADDR, 0x5a,0x88);
      SCCB_WriteReg(OV7670_ADDR, 0x5b,0x44);
      SCCB_WriteReg(OV7670_ADDR, 0x5c,0x67);
      SCCB_WriteReg(OV7670_ADDR, 0x5d,0x49);
      SCCB_WriteReg(OV7670_ADDR, 0x5e,0x0e);
      SCCB_WriteReg(OV7670_ADDR, 0x69,0x00);
      SCCB_WriteReg(OV7670_ADDR, 0x6a,0x40);
      SCCB_WriteReg(OV7670_ADDR, 0x6b,0x0a);
      SCCB_WriteReg(OV7670_ADDR, 0x6c,0x0a);
      SCCB_WriteReg(OV7670_ADDR, 0x6d,0x55);
      SCCB_WriteReg(OV7670_ADDR, 0x6e,0x11);
      SCCB_WriteReg(OV7670_ADDR, 0x6f,0x9f);

      SCCB_WriteReg(OV7670_ADDR, 0xb0,0x84);
#endif
}


// DEBUG
#include "usart.h"


void camera_ov7670_read(void)
{
  // TODO Replace with other text
  char buff[128] = "Register .. = .. [..] \n\r";

  uint8_t reply, r;

  for (r = REG_PID; r <= (REG_PID + 25); r++) {
    uint8_t res;
    myhex(r, buff + 9);
    res = SCCB_ReadReg(OV7670_ADDR, r, &reply);
    myhex(reply, buff + 14);
    myhex(res, buff + 18);
    UsartTx((uint8_t *)&buff[0], 24);
  }
}

