
#include "ov2640.h"

#include "utils.h"
#include "sccb.h"


/***************************************************************
 *
 *
 *      Camera register defines
 *
 *
 **************************************************************/

#define OV2640_ADDR      0x60

static int ov2640_reset(void)
{
  int ret;
  const struct regval_list reset_seq[] = {
    {BANK_SEL, BANK_SEL_SENS},
    {COM7, COM7_SRST},
    ENDMARKER,
  };

  ret = SCCB_WriteArray(OV2640_ADDR, reset_seq);

  Delay(0x07FFFF);
  //msleep(5);
  return ret;
}


void camera_ov2640_config(void)
{
  int ret;
#ifdef USE_RGB565
#error RGB565 not supported in OV2640
#endif

  // Reset
  ov2640_reset();

  // Init
  ret = SCCB_WriteArray(OV2640_ADDR, ov2640_init_regs);
  if (ret < 0) {
    goto err;
  }

  // select preamble
  ret = SCCB_WriteArray(OV2640_ADDR, ov2640_size_change_preamble_regs);
  if (ret < 0) {
    goto err;
  }

  // set size win
  ret = SCCB_WriteArray(OV2640_ADDR, ov2640_qcif_regs);
  if (ret < 0) {
    goto err;
  }

  // cfmt preamble
  ret = SCCB_WriteArray(OV2640_ADDR, ov2640_format_change_preamble_regs);
  if (ret < 0) {
    goto err;
  }

  // set cfmt
  ret = SCCB_WriteArray(OV2640_ADDR, ov2640_yuv422_regs);
  if (ret < 0) {
    goto err;
  }

  return;
err:
  for (;;) {
    //led_toggle();
    Delay(0x07FFFF);
  }

}


// DEBUG
#include "usart.h"


void camera_ov2640_read(void)
{
  // TODO Replace with other text
  char buff[128] = "Register .. = .. [..] \n\r";
  UsartTx((uint8_t *)&buff[0], 24);

  uint8_t reply, r;

  for (r = PID; r <= (PID + 25); r++) {
    uint8_t res;
    myhex(r, buff + 9);
    //led_toggle();
    res = SCCB_ReadReg(OV2640_ADDR, r, &reply);
    myhex(reply, buff + 14);
    myhex(res, buff + 18);
    UsartTx((uint8_t *)&buff[0], 24);
  }
}

