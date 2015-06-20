
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
#ifdef USE_RGB
#warning USING_RGB
#else
#endif

#ifdef SMALL_IMAGE
#else
#ifdef LARGE_IMAGE
#else
#endif
#endif

  // Reset
  SCCB_WriteReg(OV7670_ADDR, OV7670_COMMON_CTRL7, 0x80);
  Delay(0x07FFFF);

  SCCB_WriteReg(OV7670_ADDR, REG_CLKRC, 0x84);
  // SCCB_WriteReg(OV7670_ADDR, REG_COM11, 0x0A); // Night Mode
  //SCCB_WriteReg(OV7670_ADDR, REG_TSLB, 0x04);  // UYVY YUYV ...
  //SCCB_WriteReg(OV7670_ADDR, REG_TSLB, 0x04);
  //SCCB_WriteReg(OV7670_ADDR, REG_COM7, 0x04); /* output format: rgb */
  SCCB_WriteReg(OV7670_ADDR, REG_COM7, 0x08); /* output format: yuv */

  //SCCB_WriteReg(OV7670_ADDR, REG_RGB444, 0x00); /* disable RGB444 */
  //SCCB_WriteReg(OV7670_ADDR, REG_COM15, 0xD0); /* set RGB565 */

      /* not even sure what all these do, gonna check the oscilloscope and go
       * from there... */
  //SCCB_WriteReg(OV7670_ADDR, REG_HSTART, 0x16);
  //SCCB_WriteReg(OV7670_ADDR, REG_HSTOP, 0x04);
  //SCCB_WriteReg(OV7670_ADDR, REG_HREF, 0x24);
  //SCCB_WriteReg(OV7670_ADDR, REG_VSTART, 0x02);
  //SCCB_WriteReg(OV7670_ADDR, REG_VSTOP, 0x7a);
  //SCCB_WriteReg(OV7670_ADDR, REG_VREF, 0x0a);
//  SCCB_WriteReg(OV7670_ADDR, REG_COM10, 0x02);  // VSYNC NEGATIVE?
//  SCCB_WriteReg(OV7670_ADDR, REG_COM10, 0x20);  // NO PCLK outside HREF
//  SCCB_WriteReg(OV7670_ADDR, REG_COM3, 0x04);     // Scale enable
//  SCCB_WriteReg(OV7670_ADDR, REG_COM14, 0x1a); // divide by 4
      //SCCB_WriteReg(OV7670_ADDR, REG_COM14, 0x1b); // divide by 8
  // SCCB_WriteReg(OV7670_ADDR, REG_MVFP, 0x27); //Vflip?
//  SCCB_WriteReg(OV7670_ADDR, 0x72, 0x22); // downsample by 4
      //SCCB_WriteReg(OV7670_ADDR, 0x72, 0x33); // downsample by 8
//  SCCB_WriteReg(OV7670_ADDR, 0x73, 0x02); // divide by 4
      //SCCB_WriteReg(OV7670_ADDR, 0x73, 0xf3); // divide by 8

      // test pattern
  //SCCB_WriteReg(OV7670_ADDR, 0x70, 1 << 7);
  //SCCB_WriteReg(OV7670_ADDR, 0x70, 0x0);

/*
      // COLOR SETTING
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
*/
}


// DEBUG
#include "usart.h"


void camera_ov7670_read(void)
{
  // TODO Replace with other text
  char buff[128] = "Register .. = .. [..] \n\r";

  uint8_t reply, r;

  for (r = OV7670_PROD_ID; r <= OV7670_PROD_MAN_ID_H; r++) {
    uint8_t res;
    myhex(r, buff + 9);
    res = SCCB_ReadReg(OV7670_ADDR, r, &reply);
    myhex(reply, buff + 14);
    myhex(res, buff + 18);
    UsartTx((uint8_t *)&buff[0], 24);
  }
}

