/*
 *  TCM8230 Driver in I2C
 *  Kevin Lamers
 *  Copyright (c) See main file for copyright and liability
 */

#include "tmg3993.h"
#include "i2c.h"

#define prox_len 100 //50
#define angle_len 100 //50

int16_t prox[prox_len];
int16_t prox_east[prox_len];
int16_t prox_west[prox_len];
int16_t angle[angle_len];
int16_t prox_filt = 0;
int16_t prox_east_filt;
int16_t prox_west_filt;
int16_t angle_filt;

int32_t prox_idx = 0;
int32_t prox_east_idx = 0;
int32_t prox_west_idx = 0;
int32_t angle_idx = 0;

int32_t prox_sum = 0;
int32_t prox_east_sum = 0;
int32_t prox_west_sum = 0;
int32_t angle_sum = 0;

void TMG3993_Init(void)
{
  I2CWrite(ADDRESS_TMG3993, REGISTER_ENABLE, COMMAND_POWER_ON | COMMAND_PROXIMITY_ENABLE | COMMAND_WAIT_ENABLE);
  I2CWrite(ADDRESS_TMG3993, REGISTER_CONTROL, COMMAND_LDRIVE_100 | COMMAND_PGAIN_2);
  I2CWrite(ADDRESS_TMG3993, REGISTER_PPULSE, COMMAND_32us | COMMAND_64p);
  I2CWrite(ADDRESS_TMG3993, REGISTER_CONFIG2, COMMAND_BOOST_300);
  I2CWrite(ADDRESS_TMG3993, REGISTER_CONFIG1, COMMAND_WAITLONGDISABLE);
  //I2CWrite(ADDRESS_TMG3993, REGISTER_WAITTIME,COMMAND_WTIME_220);
  I2CWrite(ADDRESS_TMG3993, REGISTER_WAITTIME, COMMAND_WTIME_1);
}

int16_t TMG3993_Read_Proximity(void)
{
  uint8_t response;
  I2CRead(ADDRESS_TMG3993, REGISTER_PDATA, &response);

  // Moving average on proximity data
  prox_sum = prox_sum - prox[prox_idx % prox_len];
  prox[prox_idx % prox_len] = response;
  prox_idx++;
  prox_sum = prox_sum + response;
  prox_filt = prox_sum / prox_len;

  return prox_filt;
}

int16_t TMG3993_Read_Proximity_East(void)
{
  //I2CWrite(ADDRESS_TMG3993, REGISTER_WAITTIME,COMMAND_WTIME_1);
  I2CWrite(ADDRESS_TMG3993, REGISTER_CONFIG3, COMMAND_Only_East);

  uint8_t status;
  I2CRead(ADDRESS_TMG3993, REGISTER_STATUS, &status);
  while ((status && COMMAND_Proximity_Valid) == 0) {
    I2CRead(ADDRESS_TMG3993, REGISTER_STATUS, &status);
  }

  uint8_t response;
  I2CRead(ADDRESS_TMG3993, REGISTER_PDATA, &response);

  I2CWrite(ADDRESS_TMG3993, REGISTER_CONFIG3, COMMAND_All_Sensors);
  //I2CWrite(ADDRESS_TMG3993, REGISTER_WAITTIME,COMMAND_WTIME_220);

  // Moving average on proximity data
  response = response + 1 + (prox_filt / 25);
  prox_east_sum = prox_east_sum - prox_east[prox_east_idx % prox_len];
  prox_east[prox_east_idx % prox_len] = response;
  prox_east_idx++;
  prox_east_sum = prox_east_sum + response;
  prox_east_filt = prox_east_sum / prox_len;

  return prox_east_filt;
}

int16_t TMG3993_Read_Proximity_West(void)
{
  //I2CWrite(ADDRESS_TMG3993, REGISTER_WAITTIME,COMMAND_WTIME_1);
  I2CWrite(ADDRESS_TMG3993, REGISTER_CONFIG3, COMMAND_Only_West);

  uint8_t status;
  I2CRead(ADDRESS_TMG3993, REGISTER_STATUS, &status);
  while ((status && COMMAND_Proximity_Valid) == 0) {
    I2CRead(ADDRESS_TMG3993, REGISTER_STATUS, &status);
  }

  uint8_t response;
  I2CRead(ADDRESS_TMG3993, REGISTER_PDATA, &response);

  I2CWrite(ADDRESS_TMG3993, REGISTER_CONFIG3, COMMAND_All_Sensors);
  //I2CWrite(ADDRESS_TMG3993, REGISTER_WAITTIME,COMMAND_WTIME_220);

  // Moving average on proximity data
  prox_west_sum = prox_west_sum - prox_west[prox_west_idx % prox_len];
  prox_west[prox_west_idx % prox_len] = response;
  prox_west_idx++;
  prox_west_sum = prox_west_sum + response;
  prox_west_filt = prox_west_sum / prox_len;

  return prox_west_filt;
}

int16_t TMG3993_Read_Angle(void)
{
  //I2CWrite(ADDRESS_TMG3993, REGISTER_WAITTIME,COMMAND_WTIME_1);

  // Calculate angle between sensor and obstacle
  //angle[angle_len-1] = (int)((prox_east_filt-prox_west_filt)/(prox_filt*0.001778*100));
  int16_t response = (int)((prox_east_filt - prox_west_filt) / (prox_filt * 0.001778 * 100));

  // Moving average on proximity data
  angle_sum = angle_sum - angle[angle_idx % angle_len];
  angle[angle_idx % angle_len] = response;
  angle_idx++;
  angle_sum = angle_sum + response;
  angle_filt = angle_sum / angle_len;


  //I2CWrite(ADDRESS_TMG3993, REGISTER_WAITTIME,COMMAND_WTIME_220);
  return angle_filt;//angle[prox_len-1];//angle_filt;
}



