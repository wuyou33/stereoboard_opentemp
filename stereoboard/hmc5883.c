#include "hmc5883.h"

#include "stm32f4xx_i2c.h"



/***************************************************************
 *
 *
 *      Camera register defines
 *
 *
 **************************************************************/

#define HMC5883_ADDR_WRITE      0x3C
#define HMC5883_ADDR_READ     0x3D

/*
  Address Location  Name  Access
  00  Configuration Register A   Read/Write
  01  Configuration Register B   Read/Write
  02  Mode Register  Read/Write
  03  Data Output X MSB Register  Read
  04  Data Output X LSB Register  Read
  05  Data Output Y MSB Register  Read
  06  Data Output Y LSB Register   Read
  07  Data Output Z MSB Register  Read
  08  Data Output Z LSB Register  Read
  09  Status Register  Read
  10  Identification Register A  Read
  11  Identification Register B  Read
  12  Identification Register C  Read
*/

#define   HMC_REG_A       0x00
#define   HMC_REG_B       0x01
#define   HMC_MODE        0x02
#define   HMC_X           0x03
#define   HMC_Y           0x05
#define   HMC_Z           0x07
#define   HMC_STATUS      0x09

uint8_t hmc5883_WriteReg(uint8_t Addr, uint8_t Data);
uint8_t hmc5883_ReadReg(uint8_t Addr, uint8_t *a, uint8_t *b, uint8_t *c, uint8_t *d, uint8_t *e, uint8_t *f);
uint8_t hmc5883_ReadRegSingle(uint8_t Addr, uint8_t *reply);

#include "usart.h"


int cnt = 0;

void mag_send(void);
void mag_send(void)
{
  char c[2];
  float values[4];

  c[0] = '@';
  c[1] = '%';

  values[0] = cnt++;
  values[1] = magneticfield[0];
  values[2] = magneticfield[1];
  values[3] = magneticfield[2];

  while (UsartTx((uint8_t *)&c[0], 2) == 0);
  while (UsartTx((uint8_t *)&values[0], 16) == 0);

}


void draw_mag_as_line(int m)
{
  int16_t l = magneticfield[m];
  uint16_t width = 128; // 176; // 128;
  uint8_t code[4];
  code[0] = 0xff;
  code[1] = 0x00;
  code[2] = 0x00;
  {
    int i;
    uint8_t debug[256];

    for (i = 0; i < 128 * 2; i++) {
      debug[i] = 127;
    }

    if (l >= 0) {
      l /= 2;
      if (l > 100) {
        l = 100;
      }
      for (i = 0; i < l; i++) {
        debug[i] = 230;
      }
    } else {
      l = -l;
      l /= 2;
      if (l > 100) {
        l = 100;
      }
      for (i = 0; i < l; i++) {
        debug[i] = 20;
      }
    }

    code[3] = 0x80;
    while (UsartTx(code, 4) == 0)
      ;
    while (UsartTx(debug, width) == 0)
      ;
    while (UsartTx(debug + width, width) == 0)
      ;
    code[3] = 0xDA;
    while (UsartTx(code, 4) == 0)
      ;
  }

}

#define SELFTEST_NONE 0
#define SELFTEST_POS  1
#define SELFTEST_NEG  2

int hmc5883_config(void)
{
  // Filter by 4, output 30Hz, normal mode
  hmc5883_WriteReg(HMC_REG_A, (2 << 5) | (5 << 2) | (SELFTEST_NONE << 0));
  // Normal Gain
  hmc5883_WriteReg(HMC_REG_B, (5 << 5));
  // Measure continuous
  hmc5883_WriteReg(HMC_MODE, 0x00);

  // Check Identification: 0x48 0x34 0x33
  {
    uint8_t a;
    uint8_t res = hmc5883_ReadRegSingle(10, &a);
    if (a != 0x48) {
      return -1;
    }
    res = hmc5883_ReadRegSingle(11, &a);
    if (a != 0x34) {
      return -1;
    }
  }
  return 0;
}

int16_t magneticfield[3] = {64, 128, 270};

void hmc5883_read(void)
{
  // TODO Replace with other text
  //char buff[128] = "MAG .... .... .... \n\r";

  uint8_t status, i;
  uint8_t r = hmc5883_ReadRegSingle(HMC_STATUS, &status);

  if (!(status & 0x01)) {
    return;
  }

  i = 0;
  //for (r=3;r<9;r+=2)
  {
    uint8_t *p = (uint8_t *) &magneticfield[0];
    uint8_t res;
    uint8_t a, b, c, d, e, f;
    //myhex(r,buff+9);
    res = hmc5883_ReadReg(HMC_X, &a, &b, &c, &d, &e, &f);
    /*
    myhex(a,buff+4);
    myhex(b,buff+6);
    myhex(c,buff+9);
    myhex(d,buff+11);
    myhex(e,buff+14);
    myhex(f,buff+16);
    */
    p[1] = a;
    p[0] = b;
    p[3] = c;
    p[2] = d;
    p[5] = e;
    p[4] = f;
    //print_number(val,0);
    //UsartTx((uint8_t*)&buff[0], 22);
    //magneticfield[i++] = val;

#ifdef DEBUG_MAG
    mag_send();
#endif
  }
}

#define TIMEOUT_MAX 50000

/**
  * @brief  Writes a byte at a specific Magnetometer register
  * @param  Addr: hmc5883 register address.
  * @param  Data: Data to be written to the specific register
  * @retval 0x00 if write operation is OK.
  *       0xFF if timeout condition occurred (device not connected or bus error).
  */
uint8_t hmc5883_WriteReg(uint8_t Addr, uint8_t Data)
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
  I2C_Send7bitAddress(I2C2, HMC5883_ADDR_WRITE, I2C_Direction_Transmitter);

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
uint8_t hmc5883_ReadReg(uint8_t Addr, uint8_t *a, uint8_t *b, uint8_t *c, uint8_t *d, uint8_t *e, uint8_t *f)
{
  uint32_t timeout = TIMEOUT_MAX;
  uint8_t Data1 = 0;
  uint8_t Data2 = 0;

  /* Generate the Start Condition */
  I2C_GenerateSTART(I2C2, ENABLE);

  /* Test on I2C2 EV5 and clear it */
  timeout = TIMEOUT_MAX; /* Initialize timeout value */
  while (!I2C_CheckEvent(I2C2, I2C_EVENT_MASTER_MODE_SELECT)) {
    /* If the timeout delay is exeeded, exit with error code */
    if ((timeout--) == 0) { return 0xFE; }
  }

  /* Send DCMI selected device slave Address for write */
  I2C_Send7bitAddress(I2C2, HMC5883_ADDR_WRITE, I2C_Direction_Transmitter);

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
  I2C_Send7bitAddress(I2C2, HMC5883_ADDR_READ, I2C_Direction_Receiver);

  /* Test on I2C2 EV6 and clear it */
  timeout = TIMEOUT_MAX; /* Initialize timeout value */
  while (!I2C_CheckEvent(I2C2, I2C_EVENT_MASTER_RECEIVER_MODE_SELECTED)) {
    /* If the timeout delay is exceeded, exit with error code */
    if ((timeout--) == 0) { return 0xFA; }
  }

  /* Prepare an ACK for the next data received */
  I2C_AcknowledgeConfig(I2C2, ENABLE);

  /* Test on I2C2 EV7 and clear it */
  timeout = TIMEOUT_MAX; /* Initialize timeout value */
  while (!I2C_CheckEvent(I2C2, I2C_EVENT_MASTER_BYTE_RECEIVED)) {
    /* If the timeout delay is exceeded, exit with error code */
    if ((timeout--) == 0) { return 0xF9; }
  }

  /* Prepare an ACK for the next data received */
  I2C_AcknowledgeConfig(I2C2, ENABLE);

  /* Receive the Data */
  *a = I2C_ReceiveData(I2C2);

  /* Test on I2C2 EV7 and clear it */
  timeout = TIMEOUT_MAX; /* Initialize timeout value */
  while (!I2C_CheckEvent(I2C2, I2C_EVENT_MASTER_BYTE_RECEIVED)) {
    /* If the timeout delay is exceeded, exit with error code */
    if ((timeout--) == 0) { return 0xF9; }
  }

  /* Prepare an ACK for the next data received */
  I2C_AcknowledgeConfig(I2C2, ENABLE);

  /* Receive the Data */
  *b = I2C_ReceiveData(I2C2);

  /* Test on I2C2 EV7 and clear it */
  timeout = TIMEOUT_MAX; /* Initialize timeout value */
  while (!I2C_CheckEvent(I2C2, I2C_EVENT_MASTER_BYTE_RECEIVED)) {
    /* If the timeout delay is exceeded, exit with error code */
    if ((timeout--) == 0) { return 0xF9; }
  }

  /* Prepare an ACK for the next data received */
  I2C_AcknowledgeConfig(I2C2, ENABLE);

  /* Receive the Data */
  *c = I2C_ReceiveData(I2C2);

  /* Test on I2C2 EV7 and clear it */
  timeout = TIMEOUT_MAX; /* Initialize timeout value */
  while (!I2C_CheckEvent(I2C2, I2C_EVENT_MASTER_BYTE_RECEIVED)) {
    /* If the timeout delay is exceeded, exit with error code */
    if ((timeout--) == 0) { return 0xF9; }
  }

  /* Prepare an ACK for the next data received */
  I2C_AcknowledgeConfig(I2C2, ENABLE);

  /* Receive the Data */
  *d = I2C_ReceiveData(I2C2);

  /* Test on I2C2 EV7 and clear it */
  timeout = TIMEOUT_MAX; /* Initialize timeout value */
  while (!I2C_CheckEvent(I2C2, I2C_EVENT_MASTER_BYTE_RECEIVED)) {
    /* If the timeout delay is exceeded, exit with error code */
    if ((timeout--) == 0) { return 0xF9; }
  }

  /* Prepare an ACK for the next data received */
  I2C_AcknowledgeConfig(I2C2, DISABLE);

  /* Receive the Data */
  *e = I2C_ReceiveData(I2C2);

  /* Test on I2C2 EV7 and clear it */
  timeout = TIMEOUT_MAX; /* Initialize timeout value */
  while (!I2C_CheckEvent(I2C2, I2C_EVENT_MASTER_BYTE_RECEIVED)) {
    /* If the timeout delay is exceeded, exit with error code */
    if ((timeout--) == 0) { return 0xF9; }
  }

  /* Prepare Stop after receiving data */
  I2C_GenerateSTOP(I2C2, ENABLE);

  /* Receive the Data */

  *f = I2C_ReceiveData(I2C2);

  /* return the read data */
  return 0;
}


/**
//  * @brief  Reads a byte from a specific Camera register
//  * @param  Addr: mt9v034 register address.
//  * @retval data read from the specific register or 0xFF if timeout condition
//  *         occured.
//  */
uint8_t hmc5883_ReadRegSingle(uint8_t Addr, uint8_t *reply)
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
  I2C_Send7bitAddress(I2C2, HMC5883_ADDR_WRITE, I2C_Direction_Transmitter);

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
  I2C_Send7bitAddress(I2C2, HMC5883_ADDR_READ, I2C_Direction_Receiver);

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
