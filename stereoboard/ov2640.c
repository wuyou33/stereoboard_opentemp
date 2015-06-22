
#include "ov2640.h"
#include "sccb.h"


/**
  * @brief  Configures the ov2640 camera
  */

void camera_ov2640_i2c_init(void)
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

#define OV2640_ADDR      0x60

static int ov2640_write_array( const struct regval_list *vals)
{
  int ret;

  while ((vals->reg_num != 0xff) || (vals->value != 0xff)) {
    ret = SCCB_WriteReg(OV2640_ADDR, vals->reg_num, vals->value);
    if (ret != 0)
      return ret;
    vals++;
  }
  return 0;
}

static int ov2640_reset(void)
{
  int ret;
  const struct regval_list reset_seq[] = {
          {BANK_SEL, BANK_SEL_SENS},
          {COM7, COM7_SRST},
          ENDMARKER,
  };

  ret = ov2640_write_array( reset_seq);

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
  ret = ov2640_write_array(ov2640_init_regs);
  if (ret < 0)
    goto err;

  // select preamble
  ret = ov2640_write_array( ov2640_size_change_preamble_regs);
  if (ret < 0)
          goto err;

  // set size win
  ret = ov2640_write_array( ov2640_qcif_regs);
  if (ret < 0)
          goto err;

  // cfmt preamble
  ret = ov2640_write_array( ov2640_format_change_preamble_regs);
  if (ret < 0)
          goto err;

  // set cfmt
  ret = ov2640_write_array( ov2640_yuv422_regs);
  if (ret < 0)
          goto err;

  return;
err:
  for (;;)
  {
    led_toggle();
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
    led_toggle();
    res = SCCB_ReadReg(OV2640_ADDR, r, &reply);
    myhex(reply, buff + 14);
    myhex(res, buff + 18);
    UsartTx((uint8_t *)&buff[0], 24);
  }
}

