
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


  SCCB_WriteReg(OV7670_ADDR, OV7670_COMMON_CTRL7, IMG_OV_SIZE_QCIF);

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

