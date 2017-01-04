
#include "tcm8230.h"

#include "main_parameters.h"
#include "camera_type.h"
#include "utils.h"
#include "dcmi.h"
#include "stm32f4xx.h"
#include "stm32f4xx_conf.h"
#include "usart.h"

/**
  * @brief  Configures the tcm8230 camera
  * CDS : Correlated Double Sampling
  * AGC : Automatic Gain Control
  * ADC : Analog to Digital Converter
  * TG : Timing pulse Generator
  * SG : Sync pulse Generator
  * AWB : Auto White Balance
  * ALC : Auto Luminance Control
  */

// Camera Settings (z means zoom)
#define IMG_SIZE_VGA      (0<<2)
#define IMG_SIZE_QVGA     (1<<2)
#define IMG_SIZE_QVGAz    (2<<2)
#define IMG_SIZE_QQVGA    (3<<2)
#define IMG_SIZE_QQVGAz   (4<<2)
#define IMG_SIZE_CIF      (5<<2)
#define IMG_SIZE_QCIF     (6<<2)
#define IMG_SIZE_QCIFz    (7<<2)
#define IMG_SIZE_subQCIF  (8<<2)
#define IMG_SIZE_subQCIFz (9<<2)

#define IMG_FORMAT_YUV422 (0<<1)
#define IMG_FORMAT_RGB565 (1<<1)

#define IMG_COLOR_COLOR   (0<<0)
#define IMG_COLOR_BW      (1<<0)

void camera_tcm8230_i2c_init(void)
{
  // Initialize I2C Interface to camera
  GPIO_InitTypeDef GPIO_InitStructure;

  // Enable DCMI GPIOs clocks: SCL = PB10 SDA = PB11
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);

  // I2C2 clock enable
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_I2C2, ENABLE);

  // Connect I2C2 pins to AF
  GPIO_PinAFConfig(GPIOB, GPIO_PinSource10, GPIO_AF_I2C2);
  GPIO_PinAFConfig(GPIOB, GPIO_PinSource11, GPIO_AF_I2C2);

  // Configure I2C2 GPIO electrical characteristics
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10 | GPIO_Pin_11;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_OD;      ///< Open drain output! NO Push to 3.3V! Will break camera
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;    ///< 2.8V external pull-up resistor: NO internal Push NOR Pull
  GPIO_Init(GPIOB, &GPIO_InitStructure);

  // I2C DeInit
  I2C_DeInit(I2C2);

  // Enable the I2C peripheral
  I2C_Cmd(I2C2, ENABLE);

  /* Set the I2C structure parameters */
  I2C_InitTypeDef I2C_InitStruct;
  I2C_InitStruct.I2C_Mode = I2C_Mode_I2C;
  I2C_InitStruct.I2C_DutyCycle = I2C_DutyCycle_2;
  I2C_InitStruct.I2C_OwnAddress1 = 0xFE;
  I2C_InitStruct.I2C_Ack = I2C_Ack_Enable;
  I2C_InitStruct.I2C_AcknowledgedAddress = I2C_AcknowledgedAddress_7bit;
  I2C_InitStruct.I2C_ClockSpeed = 25000;

  // Initialize the I2C peripheral w/ selected parameters
  I2C_Init(I2C2, &I2C_InitStruct);
}



/***************************************************************
 *
 *
 *      Camera register defines
 *
 *
 **************************************************************/

#define TCM8230_ADDR          0b00111100

#define TCM8230_ADDR_WRITE    (TCM8230_ADDR<<1)
#define TCM8230_ADDR_READ     ((TCM8230_ADDR<<1)+1)

#define TCM_WHO_AM_I          0x00    // reads 0x70
#define TCM_FPS_ADDR          0x02
#define TCM_FPS_SLOW          (1<<7) // 15Hz
#define TCM_FPS_FAST          (0<<7) // 30Hz
#define TCM_ACF               (1<<6) // anti flicker control, 0=50Hz, 1=60Hz
#define TCM_CLK_POL           (1<<1) // clock polarity
#define TCM_ACF_DET           (0<<0) // auto flicker frequency detection, 0=auto, 1=off

#define TCM_IMG_ADDR          0x03
#define TCM_IMG_SIZE(X)       (((X)*4)+2)

#define TCM_EXP_ADDR          0x04
#define TCM_EXP_SHORT         0x0f
#define TCM_EXP_LONG          0x1f

// Auto luminance
#define TCM_ALC_ADDR          0x05 // auto luminance control
#define TCM_ALC_AUTO          (0<<7)
#define TCM_ALC_MANUAL        (1<<7)

#define TCM_ALC_ESRSPD_ADDR   0x06  // electronic shutter speed
#define TCM_ALC_ESRSPD        0x30D  // default 0x020D, max 0x1fff

// sensor analog gain
#define TCM_AG_ADDR           0x07
#define TCM_AG                0xC0  // Default 0xc0, max 0xff. 0=+0dB, 60=+6dB, 30=+12dB, 18=+18dB, 0C=+24dB
// sensor digital gain
#define TCM_DG                0x3B  // Default 80, bits 0-5 => 0=1, 3F=1.984375

#define TCM_ALCL_ADDR         0x09  // auto luminance control lower level 10=black level, 88=100% white level, ff=peak white level
#define TCM_ALCL              0x60  // Default 40

#define TCM_ALC_MODECH_ADDR   0x08  // Auto luminance mode and convergence rate
#define TCM_ALC_CH            0b1000  // Convergence range of ALC 0=0level range, f=255 level range, default 8
#define TCM_ALC_MODE_CENTER_WEIGHT (0<<4)
#define TCM_ALC_MODE_AVERAGE       (1<<4)
#define TCM_ALC_MODE_CENTER_ONLY   (2<<4)
#define TCM_ALC_MODE_BACKLIGHT     (3<<4)

// White Balance
#define TCM_AWB_ADDR          0x0A  // auto white balance ON/OFF
#define TCM_AWB_AUTO          0x00  //ON
#define TCM_AWB_MANUAL        0x80  //OFF

#define TCM_MRG_ADDR          0x0B  // manual white balance gain red
#define TCM_MRG               0x3A  // Reduce red gain slightly. Default 0x40, max 0xff. 0=x0, 40=x1, ff=x3.984375
#define TCM_MBG_ADDR          0x0C  // manual white balance gain blue
#define TCM_MBG               0x40  // Default 0x40, max 0xff. 0=x0, 40=x1, ff=x3.984375

#define TCM_GAMMA_ADDR        0x0D  // Gamma correction, gamma = 0.55
#define TCM_GAMMA             0x01  // On or Off. Default Off

#define TCM_CONTRAST_ADDR     0x11  // Default 9A. 0=x0, 80=x1, ff=1.9921875
#define TCM_BRIGHTNESS_ADDR   0x12  // Default 0C. 0=0, 7f=127, ff=-1,80=-128
#ifndef TCM_BRIGHTNESS
#define TCM_BRIGHTNESS        0x3C  // Default 0C, max 0xff
#endif

#define TCM_VHUE_ADDR         0x13  // Default 0A. Bit6 is polarity, other 5 bits 0=0, 3F=1.9921875
#define TCM_UHUE_ADDR         0x14  // Default 0A. Bit6 is polarity, other 5 bits 0=0, 3F=1.9921875

#define TCM_VGAIN_ADDR        0x15  // Default 38. 0=0, 20=1, 3F=1.96875
#define TCM_UGAIN_ADDR        0x16  // Default 38. 0=0, 20=1, 3F=1.96875

#define TCM_SATU_ADDR         0x18 // Saturation, Default 27, 0=0, 20=1, 3f=1.96875

#define TCM_SWC_ADDR          0x1E  // code synchronization
#define TCM_SWC               0x48  // set sync mode to ITU656, default 0x68

// Line speed for flickerless control
#define TCM_ES100S_ADDR       0x1C  // Setting the number of horizontal lines corresponding to 1/100 sec
#define TCM_ES100S            135   // set line speed based on 21MHz clock, CLK/(780*2*50)
#define TCM_ES120S_ADDR       0x1D  // Setting the number of horizontal lines corresponding to 1/120 sec
#define TCM_ES120S            112   // set line speed based on 21MHz clock, CLK/(780*2*60)

#ifdef USE_RGB565
#warning USING_RGB565
#define IMG_FORMAT IMG_FORMAT_RGB565
#else
#define IMG_FORMAT IMG_FORMAT_YUV422
#endif

  // Supported Image sized
#if (IMAGE_WIDTH==128) && (IMAGE_HEIGHT==96)
#define IMAGE_SIZE IMG_SIZE_subQCIF
#elif (IMAGE_WIDTH==160) && (IMAGE_HEIGHT==120)
#define IMAGE_SIZE IMG_SIZE_QQVGA
#elif (IMAGE_WIDTH==176) && (IMAGE_HEIGHT==144)
#define IMAGE_SIZE IMG_SIZE_QCIF
#elif (IMAGE_WIDTH==320) && (IMAGE_HEIGHT==240)
#define IMAGE_SIZE IMG_SIZE_QVGA
#elif (IMAGE_WIDTH==640) && (IMAGE_HEIGHT==180)
#define IMAGE_SIZE IMG_SIZE_VGA
#else
#ifdef USE_TCM8230
#error "TCM8230 camera chip does not support this resolution"
#define IMAGE_SIZE 0
#endif
#endif


uint8_t tcm8230_WriteReg(uint8_t Addr, uint8_t Data);
uint8_t tcm8230_ReadReg(uint8_t Addr, uint8_t *reply);

void camera_tcm8230_config(void)
{
  // set up camera presets
  tcm8230_WriteReg(TCM_IMG_ADDR, IMG_COLOR_COLOR | IMG_FORMAT | IMAGE_SIZE);
  tcm8230_WriteReg(TCM_SWC_ADDR, TCM_SWC);
  tcm8230_WriteReg(TCM_BRIGHTNESS_ADDR, TCM_BRIGHTNESS);  // increase target brightness
  tcm8230_WriteReg(TCM_ALC_MODECH_ADDR, TCM_ALC_MODE_CENTER_WEIGHT | TCM_ALC_CH);

  // set line speed for flickerless control
  tcm8230_WriteReg(TCM_ES100S_ADDR, TCM_ES100S);
  tcm8230_WriteReg(TCM_ES120S_ADDR, TCM_ES120S);

#ifdef SLOW_TCM8230
  tcm8230_WriteReg(TCM_FPS_ADDR, TCM_FPS_SLOW | TCM_ACF | TCM_CLK_POL | TCM_ACF_DET);
  tcm8230_WriteReg(TCM_EXP_ADDR, TCM_EXP_LONG);
#else
  tcm8230_WriteReg(TCM_FPS_ADDR, TCM_FPS_FAST | TCM_ACF | TCM_CLK_POL | TCM_ACF_DET);
#endif

#if USE_COLOR
  // If color is used, auto white balance should be deactivated
  tcm8230_WriteReg(TCM_AWB_ADDR, TCM_AWB_MANUAL);
  tcm8230_WriteReg(TCM_MRG_ADDR, TCM_MRG);
  tcm8230_WriteReg(TCM_MBG_ADDR, TCM_MBG);

  // Set Manual lumination
  tcm8230_WriteReg(TCM_ALC_ADDR, TCM_ALC_MANUAL | ((TCM_ALC_ESRSPD >> 8)&0b00011111)); // in combination with settings high saturation, seems to fix the color changing problem (making it gray when large bodies of hard color)
  tcm8230_WriteReg(TCM_ALC_ESRSPD_ADDR, TCM_ALC_ESRSPD & 0x00ff);
#endif

  // Set the image saturation. default 39, 127 max, 0 min (black/white)
#if (TCM8230_EXTRA_SATURATION == 1) // Medium saturation setting
  tcm8230_WriteReg(TCM_SATU_ADDR, 80);
#elif (TCM8230_EXTRA_SATURATION == 2) // High saturation setting
  tcm8230_WriteReg(TCM_SATU_ADDR, 127);
#endif

  // Extra options:
  // tcm8230_WriteReg(TCM_VHUE_ADDR, 255);
  // tcm8230_WriteReg(TCM_UHUE_ADDR, 255);
}

void camera_tcm8230_read(void)
{
  // TODO Replace with other text
  char buff[128] = "Register .. = .. [..] \n\r";

  uint8_t reply, r;

  for (r = 0; r < 20; r++) {
    uint8_t res;
    myhex(r, buff + 9);
    res = tcm8230_ReadReg(r, &reply);
    myhex(reply, buff + 14);
    myhex(res, buff + 18);
    UsartTx((uint8_t *)&buff[0], 24);
  }
}

#define TIMEOUT_MAX 50000

/**
  * @brief  Writes a byte at a specific Camera register
  * @param  Addr: tcm8230 register address.
  * @param  Data: Data to be written to the specific register
  * @retval 0x00 if write operation is OK.
  *       0xFF if timeout condition occurred (device not connected or bus error).
  */
uint8_t tcm8230_WriteReg(uint8_t Addr, uint8_t Data)
{
  uint32_t timeout = TIMEOUT_MAX;

  /* Generate the Start Condition */
  I2C_GenerateSTART(I2C2, ENABLE);

  /* Test on I2C2 EV5 and clear it */
  timeout = TIMEOUT_MAX; /* Initialize timeout value */
  while (!I2C_CheckEvent(I2C2, I2C_EVENT_MASTER_MODE_SELECT)) {
    /* If the timeout delay is exceeded, exit with error code */
    if ((timeout--) == 0) { return 0xFF; }
  }

  /* Send DCMI selected device slave Address for write */
  I2C_Send7bitAddress(I2C2, TCM8230_ADDR_WRITE, I2C_Direction_Transmitter);

  /* Test on I2C2 EV6 and clear it */
  timeout = TIMEOUT_MAX; /* Initialize timeout value */
  while (!I2C_CheckEvent(I2C2, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED)) {
    /* If the timeout delay is exceeded, exit with error code */
    if ((timeout--) == 0) { return 0xFF; }
  }

  /* Send I2C2 location address LSB */
  I2C_SendData(I2C2, Addr);

  /* Test on I2C2 EV8 and clear it */
  timeout = TIMEOUT_MAX; /* Initialize timeout value */
  while (!I2C_CheckEvent(I2C2, I2C_EVENT_MASTER_BYTE_TRANSMITTED)) {
    /* If the timeout delay is exceeded, exit with error code */
    if ((timeout--) == 0) { return 0xFF; }
  }

  /* Send Data */
  I2C_SendData(I2C2, Data);

  /* Test on I2C2 EV8 and clear it */
  timeout = TIMEOUT_MAX; /* Initialize timeout value */
  while (!I2C_CheckEvent(I2C2, I2C_EVENT_MASTER_BYTE_TRANSMITTED)) {
    /* If the timeout delay is exceeded, exit with error code */
    if ((timeout--) == 0) { return 0xFF; }
  }

  /* Send I2C2 STOP Condition */
  I2C_GenerateSTOP(I2C2, ENABLE);

  /* If operation is OK, return 0 */
  return 0;
}

/**
//  * @brief  Reads a byte from a specific Camera register
//  * @param  Addr: mt9v034 register address.
//  * @retval data read from the specific register or 0xFF if timeout condition
//  *         occured.
//  */
uint8_t tcm8230_ReadReg(uint8_t Addr, uint8_t *reply)
{
  uint32_t timeout = TIMEOUT_MAX;
  uint8_t Data = 0;

  /* Generate the Start Condition */
  I2C_GenerateSTART(I2C2, ENABLE);

  /* Test on I2C2 EV5 and clear it */
  timeout = TIMEOUT_MAX; /* Initialize timeout value */
  while (!I2C_CheckEvent(I2C2, I2C_EVENT_MASTER_MODE_SELECT)) {
    /* If the timeout delay is exeeded, exit with error code */
    if ((timeout--) == 0) { return 0xFE; }
  }

  /* Send DCMI selected device slave Address for write */
  I2C_Send7bitAddress(I2C2, TCM8230_ADDR_WRITE, I2C_Direction_Transmitter);

  /* Test on I2C2 EV6 and clear it */
  timeout = TIMEOUT_MAX; /* Initialize timeout value */
  while (!I2C_CheckEvent(I2C2, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED)) {
    /* If the timeout delay is exeeded, exit with error code */
    if ((timeout--) == 0) { return 0xFD; }
  }

  /* Send I2C2 location address LSB */
  I2C_SendData(I2C2, Addr);

  /* Test on I2C2 EV8 and clear it */
  timeout = TIMEOUT_MAX; /* Initialize timeout value */
  while (!I2C_CheckEvent(I2C2, I2C_EVENT_MASTER_BYTE_TRANSMITTED)) {
    /* If the timeout delay is exceeded, exit with error code */
    if ((timeout--) == 0) { return 0xFC; }
  }

  /* Clear AF flag if arisen */
  I2C2->SR1 |= (uint16_t)0x0400;

  /* Generate the Start Condition */
  I2C_GenerateSTART(I2C2, ENABLE);

  /* Test on I2C2 EV6 and clear it */
  timeout = TIMEOUT_MAX; /* Initialize timeout value */
  while (!I2C_CheckEvent(I2C2, I2C_EVENT_MASTER_MODE_SELECT)) {
    /* If the timeout delay is exceeded, exit with error code */
    if ((timeout--) == 0) { return 0xFB; }
  }

  /* Send DCMI selected device slave Address for write */
  I2C_Send7bitAddress(I2C2, TCM8230_ADDR_READ, I2C_Direction_Receiver);

  /* Test on I2C2 EV6 and clear it */
  timeout = TIMEOUT_MAX; /* Initialize timeout value */
  while (!I2C_CheckEvent(I2C2, I2C_EVENT_MASTER_RECEIVER_MODE_SELECTED)) {
    /* If the timeout delay is exceeded, exit with error code */
    if ((timeout--) == 0) { return 0xFA; }
  }

  /* Prepare an NACK for the next data received */
  I2C_AcknowledgeConfig(I2C2, DISABLE);

  /* Test on I2C2 EV7 and clear it */
  timeout = TIMEOUT_MAX; /* Initialize timeout value */
  while (!I2C_CheckEvent(I2C2, I2C_EVENT_MASTER_BYTE_RECEIVED)) {
    /* If the timeout delay is exceeded, exit with error code */
    if ((timeout--) == 0) { return 0xF9; }
  }

  /* Prepare Stop after receiving data */
  I2C_GenerateSTOP(I2C2, ENABLE);

  /* Receive the Data */
  Data = I2C_ReceiveData(I2C2);

  *reply = Data;

  /* return the read data */
  return 0;
}
