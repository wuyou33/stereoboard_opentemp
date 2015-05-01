
#include "tcm8230.h"
#include "dcmi.h"
#include "stm32f4xx_i2c.h"


/**
  * @brief  Configures the tcm8230 camera
  */

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
  I2C_InitStruct.I2C_ClockSpeed = 50000;

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

#define TCM8230_ADDR        0b00111100

#define TCM8230_ADDR_WRITE      (TCM8230_ADDR<<1)
#define TCM8230_ADDR_READ     ((TCM8230_ADDR<<1)+1)


#define   TCM_WHO_AM_I      0x00    // reads 0x70
#define   TCM_FPS         0x02
#define   TCM_FPS_SLOW      0x82
#define   TCM_FPS_FAST      0x02

#define   TCM_IMG         0x03
#define   TCM_IMG_SIZE(X)     (((X)*4)+2)

#define   TCM_SWC         0x1E
#define   TCM_SWC_VAL       0x78

#define   TCM_EXP         0x04
#define   EXP_DEFAULT       0x0f
#define   EXP_SHORT       0x00
#define   EXP_LONG        0x10

uint8_t tcm8230_WriteReg(uint8_t Addr, uint8_t Data);
uint8_t tcm8230_ReadReg(uint8_t Addr, uint8_t *reply);

#include "usart.h"

void camera_tcm8230_config(void)
{
  tcm8230_WriteReg(TCM_FPS, TCM_FPS_FAST);
#ifdef SMALL_IMAGE
  tcm8230_WriteReg(TCM_IMG, IMG_COLOR_COLOR | IMG_FORMAT_YUV422 | IMG_SIZE_subQCIF);
#else
  tcm8230_WriteReg(TCM_IMG, IMG_COLOR_COLOR | IMG_FORMAT_YUV422 | IMG_SIZE_QCIF);
#endif
  tcm8230_WriteReg(TCM_SWC, TCM_SWC_VAL);
  tcm8230_WriteReg(TCM_EXP, EXP_DEFAULT | EXP_SHORT);
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
    usart_tx_ringbuffer_push((uint8_t *)&buff[0], 24);
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
