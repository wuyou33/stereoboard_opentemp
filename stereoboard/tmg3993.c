/*
 *  TCM8230 Driver in I2C
 *	Kevin Lamers
 *  Copyright (c) See main file for copyright and liability
 */

#include "tmg3993.h"
#include "i2c.h"

#define prox_len 50
#define angle_len 50

int16_t prox[prox_len];
int16_t prox_east[prox_len];
int16_t prox_west[prox_len];
int16_t angle[angle_len];
int16_t prox_filt = 0;
int16_t prox_east_filt;
int16_t prox_west_filt;
int16_t angle_filt;

void TMG3993_Init(void)
{
  I2CWrite(ADDRESS_TMG3993, REGISTER_ENABLE,COMMAND_POWER_ON | COMMAND_PROXIMITY_ENABLE | COMMAND_WAIT_ENABLE);
  I2CWrite(ADDRESS_TMG3993, REGISTER_CONTROL,COMMAND_LDRIVE_100 | COMMAND_PGAIN_2);
  I2CWrite(ADDRESS_TMG3993, REGISTER_PPULSE,COMMAND_32us | COMMAND_64p);
  I2CWrite(ADDRESS_TMG3993, REGISTER_CONFIG2,COMMAND_BOOST_300);
  I2CWrite(ADDRESS_TMG3993, REGISTER_CONFIG1,COMMAND_WAITLONGDISABLE);
  //I2CWrite(ADDRESS_TMG3993, REGISTER_WAITTIME,COMMAND_WTIME_220);
  I2CWrite(ADDRESS_TMG3993, REGISTER_WAITTIME,COMMAND_WTIME_1);
}

int16_t TMG3993_Read_Proximity(void)
{
  uint8_t response;
  I2CRead(ADDRESS_TMG3993, REGISTER_PDATA, &response);
  prox[prox_len-1] = response;

  // Moving average on proximity data
  int i;
  for (i=0;i<prox_len-1;i++){
    prox[i] = prox[i+1];
  }
  uint16_t prox_sum = 0;
  for (i=0;i<prox_len;i++){
    prox_sum = prox_sum + prox[i];
  }
  prox_filt = prox_sum/(prox_len);

  return prox_filt;
}

int16_t TMG3993_Read_Proximity_East(void)
{

  I2CWrite(ADDRESS_TMG3993, REGISTER_CONFIG3,COMMAND_Only_East);

  uint8_t status;
  I2CRead(ADDRESS_TMG3993, REGISTER_STATUS, &status);
  while((status && COMMAND_Proximity_Valid) == 0) {
    I2CRead(ADDRESS_TMG3993, REGISTER_STATUS, &status);
  }

  uint8_t response;
  I2CRead(ADDRESS_TMG3993, REGISTER_PDATA, &response);
  //prox_east[prox_len-1] = response*100+300+60; //compensate for offset between east and west sensor
  prox_east[prox_len-1] = response*100 + prox_filt*4 + 260;

  I2CWrite(ADDRESS_TMG3993, REGISTER_CONFIG3,COMMAND_All_Sensors);


  // Moving average on proximity data
  int i;
  for (i=0;i<prox_len-1;i++){
    prox_east[i] = prox_east[i+1];
  }
  uint32_t prox_east_sum = 0;
  for (i=0;i<prox_len;i++){
    prox_east_sum = prox_east_sum + prox_east[i];
  }
  prox_east_filt = prox_east_sum/(prox_len);


 // prox_east_filt = prox_east_filt + prox_filt*305/10000 + 1;//+4

  return prox_east_filt; //response + prox[prox_len-1]*305/10000 + 4;
}

int16_t TMG3993_Read_Proximity_West(void)
{
  //I2CWrite(ADDRESS_TMG3993, REGISTER_WAITTIME,COMMAND_WTIME_1);
  I2CWrite(ADDRESS_TMG3993, REGISTER_CONFIG3,COMMAND_Only_West);

  uint8_t status;
  I2CRead(ADDRESS_TMG3993, REGISTER_STATUS, &status);
  while((status && COMMAND_Proximity_Valid) == 0) {
    I2CRead(ADDRESS_TMG3993, REGISTER_STATUS, &status);
  }

  uint8_t response;
  I2CRead(ADDRESS_TMG3993, REGISTER_PDATA, &response);
  prox_west[prox_len-1] = response*100;

  I2CWrite(ADDRESS_TMG3993, REGISTER_CONFIG3,COMMAND_All_Sensors);
  //I2CWrite(ADDRESS_TMG3993, REGISTER_WAITTIME,COMMAND_WTIME_220);

  // Moving average on proximity data
  int i;
  for (i=0;i<prox_len-1;i++){
    prox_west[i] = prox_west[i+1];
  }
  uint32_t prox_west_sum = 0;
  for (i=0;i<prox_len;i++){
    prox_west_sum = prox_west_sum + prox_west[i];
  }
  prox_west_filt = prox_west_sum/(prox_len);

  return prox_west_filt;
}

int16_t TMG3993_Read_Angle(void)
{
  //I2CWrite(ADDRESS_TMG3993, REGISTER_WAITTIME,COMMAND_WTIME_1);

	// Calculate angle between sensor and obstacle
	angle[angle_len-1] = (int)((prox_east_filt-prox_west_filt)/(prox_filt*0.001778*100));

  // Moving average on proximity data
	int i;
  for (i=0;i<angle_len-1;i++){
    angle[i] = angle[i+1];
  }
  int32_t  angle_sum = 0;
  for (i=0;i<angle_len;i++){
    angle_sum = angle_sum + angle[i];
 }
  angle_filt = angle_sum/(angle_len);

  //I2CWrite(ADDRESS_TMG3993, REGISTER_WAITTIME,COMMAND_WTIME_220);
  return angle_filt;//angle[prox_len-1];//angle_filt;
}



