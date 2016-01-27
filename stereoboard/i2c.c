/*
 * i2c.c
 *
 *  Created on: Jul 6, 2015
 *      Author: roland
 */

#include "i2c.h"
#include "stm32f4xx.h"

//#define PROXIMITY_SENSOR_ADDRESS        0x39
//
//#define PROXIMITY_SENSOR_ADDR_WRITE      (PROXIMITY_SENSOR_ADDRESS<<1)
//#define PROXIMITY_SENSOR_ADDR_READ     ((PROXIMITY_SENSOR_ADDRESS<<1)+1)
//#define ADDRESS_TMG3992       (0x39)
//
//#define REGISTER_ENABLE      	 	(0x80)
//#define REGISTER_WAITTIME     	 	(0x83)
//#define REGISTER_PPULSE       		(0x8E)
//#define REGISTER_CONTROL      		(0x8F)
//#define REGISTER_CONFIG1      		(0x8D)
//#define REGISTER_CONFIG2      		(0x90)
//#define REGISTER_ID           		(0x92)
//#define REGISTER_PDATA        		(0x9C)
//
//#define REGISTER_RDATAL       		(0x96)
//#define REGISTER_GDATAL       		(0x98)
//#define REGISTER_BDATAL       		(0x9A)
//#define REGISTER_CDATAL       		(0x94)
//#define REGISTER_CDATAH       		(0x95)
//
//
///*****************************/
///* REGISTER_CONFIG1 Settings */
///*****************************/
//// Wait Long
//#define COMMAND_WAITLONGENABLE      (0x62)
//#define COMMAND_WAITLONGDISABLE     (0x60)
//
///*****************************/
///* REGISTER_ENABLE Settings */
///*****************************/
//// Power On
//#define COMMAND_POWER_ON        	(0x01<<0)
//// Enable Ambiant Light Sensor
//#define COMMAND_ALS_ENABLE      	(0x01<<1)
//// Enable Proximity Sensor
//#define COMMAND_PROXIMITY_ENABLE 	(0x01<<2)
//// Enable wait
//#define COMMAND_WAIT_ENABLE 		(0x01<<3)
//
///*****************************/
///* REGISTER_PPULSE Settings */
///*****************************/
//// Proximity Pulse Length
//#define COMMAND_4us        		 	(0x00)
//#define COMMAND_8us        			(0x40)
//#define COMMAND_16us        		(0x80)
//#define COMMAND_32us	      		(0xC0)
//// Proximity Pulse Count
//#define COMMAND_8p         			(0x07)
//#define COMMAND_16p         		(0x0F)
//#define COMMAND_32p        			(0x1F)
//#define COMMAND_64p       			(0x3F)
//
//
///*****************************/
///* REGISTER_WAITTIME Settings */
///*****************************/
//// Wait Time
//#define COMMAND_WTIME_1	     		(0xFF) // 2.78 ms
//#define COMMAND_WTIME_220	     	(0xDC) // 100  ms
//#define COMMAND_WTIME_256     		(0x00) // 712  ms
//
///*****************************/
///* REGISTER_CONFIG2 Settings */
///*****************************/
//// LED Boost
//#define COMMAND_BOOST_100      		(0x01)
//#define COMMAND_BOOST_150      		(0x11)
//#define COMMAND_BOOST_200      		(0x21)
//#define COMMAND_BOOST_300      		(0x31)
//
///*****************************/
///* REGISTER_CONTROL Settings */
///*****************************/
//// LED Drive Strength Proximity
//#define COMMAND_LDRIVE_100 			(0x00)
//#define COMMAND_LDRIVE_50 			(0x40)
//#define COMMAND_LDRIVE_25 			(0x80)
//#define COMMAND_LDRIVE_12 			(0xC0)
//// Proximity Gain Control
//#define COMMAND_PGAIN_1				(0x80)
//#define COMMAND_PGAIN_2				(0x80)
//#define COMMAND_PGAIN_4				(0x80)
//#define COMMAND_PGAIN_8				(0x80)
//// ALS and Color Gain Control
//#define COMMAND_AGAIN_1 		    (0x00)
//#define COMMAND_AGAIN_4 		    (0x01)
//#define COMMAND_AGAIN_16 		    (0x02)
//#define COMMAND_AGAIN_64 		    (0x03)

// #define TIMEOUT_MAX 50000
/**
//  * @brief  Reads a byte from the proximity sensor
//  * @param  Addr: mt9v034 register address.
//  * @retval data read from the specific register or 0xFF if timeout condition
//  *         occured.
//  */
//uint8_t readRegisterProximitySensor(uint8_t Addr, uint8_t *reply)
//{
//  uint32_t timeout = TIMEOUT_MAX;
//  uint8_t Data = 0;
//
//  /* Generate the Start Condition */
//  I2C_GenerateSTART(I2C2, ENABLE);
//
//  /* Test on I2C2 EV5 and clear it */
//  timeout = TIMEOUT_MAX; /* Initialize timeout value */
//  while (!I2C_CheckEvent(I2C2, I2C_EVENT_MASTER_MODE_SELECT)) {
//    /* If the timeout delay is exeeded, exit with error code */
//    if ((timeout--) == 0) { return 0xFE; }
//  }
//
//  /* Send DCMI selected device slave Address for write */
//  I2C_Send7bitAddress(I2C2, PROXIMITY_SENSOR_ADDR_WRITE, I2C_Direction_Transmitter);
//
//  /* Test on I2C2 EV6 and clear it */
//  timeout = TIMEOUT_MAX; /* Initialize timeout value */
//  while (!I2C_CheckEvent(I2C2, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED)) {
//    /* If the timeout delay is exeeded, exit with error code */
//    if ((timeout--) == 0) { return 0xFD; }
//  }
//
//  /* Send I2C2 location address LSB */
//  I2C_SendData(I2C2, Addr);
//
//  /* Test on I2C2 EV8 and clear it */
//  timeout = TIMEOUT_MAX; /* Initialize timeout value */
//  while (!I2C_CheckEvent(I2C2, I2C_EVENT_MASTER_BYTE_TRANSMITTED)) {
//    /* If the timeout delay is exceeded, exit with error code */
//    if ((timeout--) == 0) { return 0xFC; }
//  }
//
//  /* Clear AF flag if arisen */
//  I2C2->SR1 |= (uint16_t)0x0400;
//
//  /* Generate the Start Condition */
//  I2C_GenerateSTART(I2C2, ENABLE);
//
//  /* Test on I2C2 EV6 and clear it */
//  timeout = TIMEOUT_MAX; /* Initialize timeout value */
//  while (!I2C_CheckEvent(I2C2, I2C_EVENT_MASTER_MODE_SELECT)) {
//    /* If the timeout delay is exceeded, exit with error code */
//    if ((timeout--) == 0) { return 0xFB; }
//  }
//
//  /* Send DCMI selected device slave Address for write */
//  I2C_Send7bitAddress(I2C2, PROXIMITY_SENSOR_ADDR_READ, I2C_Direction_Receiver);
//
//  /* Test on I2C2 EV6 and clear it */
//  timeout = TIMEOUT_MAX; /* Initialize timeout value */
//  while (!I2C_CheckEvent(I2C2, I2C_EVENT_MASTER_RECEIVER_MODE_SELECTED)) {
//    /* If the timeout delay is exceeded, exit with error code */
//    if ((timeout--) == 0) { return 0xFA; }
//  }
//
//  /* Prepare an NACK for the next data received */
//  I2C_AcknowledgeConfig(I2C2, DISABLE);
//
//  /* Test on I2C2 EV7 and clear it */
//  timeout = TIMEOUT_MAX; /* Initialize timeout value */
//  while (!I2C_CheckEvent(I2C2, I2C_EVENT_MASTER_BYTE_RECEIVED)) {
//    /* If the timeout delay is exceeded, exit with error code */
//    if ((timeout--) == 0) { return 0xF9; }
//  }
//
//  /* Prepare Stop after receiving data */
//  I2C_GenerateSTOP(I2C2, ENABLE);
//
//  /* Receive the Data */
//  Data = I2C_ReceiveData(I2C2);
//
//  *reply = Data;
//
//  /* return the read data */
//  return 0;
//}
//
//
///**
//  * @brief  Writes a byte at a proximity sensor register
//  * @param  Addr: tcm8230 register address.
//  * @param  Data: Data to be written to the specific register
//  * @retval 0x00 if write operation is OK.
//  *       0xFF if timeout condition occurred (device not connected or bus error).
//  */
//uint8_t proximity_sensor_WriteReg(uint8_t Addr, uint8_t Data)
//{
//  uint32_t timeout = TIMEOUT_MAX;
//
//  /* Generate the Start Condition */
//  I2C_GenerateSTART(I2C2, ENABLE);
//
//  /* Test on I2C2 EV5 and clear it */
//  timeout = TIMEOUT_MAX; /* Initialize timeout value */
//  while (!I2C_CheckEvent(I2C2, I2C_EVENT_MASTER_MODE_SELECT)) {
//    /* If the timeout delay is exceeded, exit with error code */
//    if ((timeout--) == 0) { return 0xFF; }
//  }
//
//  /* Send DCMI selected device slave Address for write */
//  I2C_Send7bitAddress(I2C2, PROXIMITY_SENSOR_ADDR_WRITE, I2C_Direction_Transmitter);
//
//  /* Test on I2C2 EV6 and clear it */
//  timeout = TIMEOUT_MAX; /* Initialize timeout value */
//  while (!I2C_CheckEvent(I2C2, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED)) {
//    /* If the timeout delay is exceeded, exit with error code */
//    if ((timeout--) == 0) { return 0xFF; }
//  }
//
//  /* Send I2C2 location address LSB */
//  I2C_SendData(I2C2, Addr);
//
//  /* Test on I2C2 EV8 and clear it */
//  timeout = TIMEOUT_MAX; /* Initialize timeout value */
//  while (!I2C_CheckEvent(I2C2, I2C_EVENT_MASTER_BYTE_TRANSMITTED)) {
//    /* If the timeout delay is exceeded, exit with error code */
//    if ((timeout--) == 0) { return 0xFF; }
//  }
//
//  /* Send Data */
//  I2C_SendData(I2C2, Data);
//
//  /* Test on I2C2 EV8 and clear it */
//  timeout = TIMEOUT_MAX; /* Initialize timeout value */
//  while (!I2C_CheckEvent(I2C2, I2C_EVENT_MASTER_BYTE_TRANSMITTED)) {
//    /* If the timeout delay is exceeded, exit with error code */
//    if ((timeout--) == 0) { return 0xFF; }
//  }
//
//  /* Send I2C2 STOP Condition */
//  I2C_GenerateSTOP(I2C2, ENABLE);
//
//  /* If operation is OK, return 0 */
//  return 0;
//}
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
  I2C_Send7bitAddress(I2C2, (Addr<<1), I2C_Direction_Transmitter);

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
  I2C_Send7bitAddress(I2C2, (Addr<<1)+1, I2C_Direction_Receiver);

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
  I2C_Send7bitAddress(I2C2, (Addr<<1), I2C_Direction_Transmitter);

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

