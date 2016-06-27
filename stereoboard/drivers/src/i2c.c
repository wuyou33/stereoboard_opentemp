/*
 * i2c.c
 *
 *  Created on: Jul 6, 2015
 *      Author: roland
 */

#include "i2c.h"
#include "stm32f4xx.h"

uint8_t I2CRead(uint8_t Addr, uint8_t Register, uint8_t *reply)
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
  I2C_Send7bitAddress(I2C2, (Addr << 1), I2C_Direction_Transmitter);

  /* Test on I2C2 EV6 and clear it */
  timeout = TIMEOUT_MAX; /* Initialize timeout value */
  while (!I2C_CheckEvent(I2C2, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED)) {
    /* If the timeout delay is exeeded, exit with error code */
    if ((timeout--) == 0) { return 0xFD; }
  }

  /* Send I2C2 location address LSB */
  I2C_SendData(I2C2, Register);

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
  I2C_Send7bitAddress(I2C2, (Addr << 1) + 1, I2C_Direction_Receiver);

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


/**
  * @brief  Writes a byte at a proximity sensor register
  * @param  Addr: tcm8230 register address.
  * @param  Data: Data to be written to the specific register
  * @retval 0x00 if write operation is OK.
  *       0xFF if timeout condition occurred (device not connected or bus error).
  */
uint8_t I2CWrite(uint8_t Addr, uint8_t Register, uint8_t Data)
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
  I2C_Send7bitAddress(I2C2, (Addr << 1), I2C_Direction_Transmitter);

  /* Test on I2C2 EV6 and clear it */
  timeout = TIMEOUT_MAX; /* Initialize timeout value */
  while (!I2C_CheckEvent(I2C2, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED)) {
    /* If the timeout delay is exceeded, exit with error code */
    if ((timeout--) == 0) { return 0xFF; }
  }

  /* Send I2C2 location address LSB */
  I2C_SendData(I2C2, Register);

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

uint8_t I2CRead16(uint8_t Addr, uint16_t Register, uint8_t *reply)
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
  I2C_Send7bitAddress(I2C2, (Addr << 1), I2C_Direction_Transmitter);

  /* Test on I2C2 EV6 and clear it */
  timeout = TIMEOUT_MAX; /* Initialize timeout value */
  while (!I2C_CheckEvent(I2C2, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED)) {
    /* If the timeout delay is exeeded, exit with error code */
    if ((timeout--) == 0) { return 0xFD; }
  }

  /* Send I2C2 location address LSB */
  I2C_SendData(I2C2, (Register >> 8) & 0xFF);

  /* Test on I2C2 EV8 and clear it */
  timeout = TIMEOUT_MAX; /* Initialize timeout value */
  while (!I2C_CheckEvent(I2C2, I2C_EVENT_MASTER_BYTE_TRANSMITTED)) {
    /* If the timeout delay is exceeded, exit with error code */
    if ((timeout--) == 0) { return 0xFC; }
  }

  /* Send I2C2 location address LSB */
  I2C_SendData(I2C2, Register & 0xFF);

  /* Test on I2C2 EV8 and clear it */
  timeout = TIMEOUT_MAX; /* Initialize timeout value */
  while (!I2C_CheckEvent(I2C2, I2C_EVENT_MASTER_BYTE_TRANSMITTED)) {
    /* If the timeout delay is exceeded, exit with error code */
    if ((timeout--) == 0) { return 0xFC; }
  }

  /* Clear AF flag if arisen */
  I2C2->SR1 |= (uint16_t)0x0400;

  /* Prepare Stop after receiving data */
  I2C_GenerateSTOP(I2C2, ENABLE);

  /* Generate the Start Condition */
  I2C_GenerateSTART(I2C2, ENABLE);

  /* Test on I2C2 EV6 and clear it */
  timeout = TIMEOUT_MAX; /* Initialize timeout value */
  while (!I2C_CheckEvent(I2C2, I2C_EVENT_MASTER_MODE_SELECT)) {
    /* If the timeout delay is exceeded, exit with error code */
    if ((timeout--) == 0) { return 0xFB; }
  }

  /* Send DCMI selected device slave Address for write */
  I2C_Send7bitAddress(I2C2, (Addr << 1) + 1, I2C_Direction_Receiver);

  /* Test on I2C2 EV6 and clear it */
  timeout = TIMEOUT_MAX; /* Initialize timeout value */
  while (!I2C_CheckEvent(I2C2, I2C_EVENT_MASTER_RECEIVER_MODE_SELECTED)) {
    /* If the timeout delay is exceeded, exit with error code */
    if ((timeout--) == 0) { return 0xFA; }
  }

  /* Prepare an NACK for the next data received */
  //I2C_AcknowledgeConfig(I2C2, DISABLE);

  /* Test on I2C2 EV7 and clear it */
 // timeout = TIMEOUT_MAX; /* Initialize timeout value */
 // while (!I2C_CheckEvent(I2C2, I2C_EVENT_MASTER_BYTE_RECEIVED)) {
 //   /* If the timeout delay is exceeded, exit with error code */
 //   if ((timeout--) == 0) { return 0xF9; }
 // }

  /* Prepare Stop after receiving data */
  I2C_GenerateSTOP(I2C2, ENABLE);

  /* Receive the Data */
  Data = I2C_ReceiveData(I2C2);

  *reply = Data;

  /* return the read data */
  return 0;
}

/**
  * @brief  Writes a byte at a proximity sensor register
  * @param  Addr: tcm8230 register address.
  * @param  Data: Data to be written to the specific register
  * @retval 0x00 if write operation is OK.
  *       0xFF if timeout condition occurred (device not connected or bus error).
  */
uint8_t I2CWrite16(uint8_t Addr, uint16_t Register, uint16_t Data)
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
  I2C_Send7bitAddress(I2C2, (Addr << 1), I2C_Direction_Transmitter);

  /* Test on I2C2 EV6 and clear it */
  timeout = TIMEOUT_MAX; /* Initialize timeout value */
  while (!I2C_CheckEvent(I2C2, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED)) {
    /* If the timeout delay is exceeded, exit with error code */
    if ((timeout--) == 0) { return 0xFF; }
  }

  /* Send I2C2 location address MSB */
  I2C_SendData(I2C2, (Register >> 8) & 0xFF);

  /* Test on I2C2 EV8 and clear it */
 // timeout = TIMEOUT_MAX; /* Initialize timeout value */
 // while (!I2C_CheckEvent(I2C2, I2C_EVENT_MASTER_BYTE_TRANSMITTED)) {
  //  /* If the timeout delay is exceeded, exit with error code */
  //  if ((timeout--) == 0) { return 0xFF; }
 // }*/

  /* Send I2C2 location address LSB */
  I2C_SendData(I2C2, Register & 0xFF);

  /* Test on I2C2 EV8 and clear it */
  timeout = TIMEOUT_MAX; /* Initialize timeout value */
  while (!I2C_CheckEvent(I2C2, I2C_EVENT_MASTER_BYTE_TRANSMITTED)) {
    /* If the timeout delay is exceeded, exit with error code */
    if ((timeout--) == 0) { return 0xFF; }
  }

//  /* Send Data MSB */
//  I2C_SendData(I2C2, (Data >> 8) & 0xFF);
//
//  /* Test on I2C2 EV8 and clear it */
//  timeout = TIMEOUT_MAX; /* Initialize timeout value */
//  while (!I2C_CheckEvent(I2C2, I2C_EVENT_MASTER_BYTE_TRANSMITTED)) {
//    /* If the timeout delay is exceeded, exit with error code */
//    if ((timeout--) == 0) { return 0xFF; }
//  }

  /* Send Data LSB */
  I2C_SendData(I2C2, Data & 0xFF);

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


