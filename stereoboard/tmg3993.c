/*
 *  TCM8230 Driver in I2C
 *  Kevin Lamers
 *  Copyright (c) See main file for copyright and liability
 */

#include "tmg3993.h"
#include "i2c.h"

#define prox_len 25 //50
#define angle_len 10 //50
#define offset_len 25

int16_t prox[prox_len];
int16_t prox_east[prox_len];
int16_t prox_west[prox_len];
int32_t prox_north[prox_len];
int16_t angle[angle_len];
int16_t prox_filt = 0;
int16_t prox_east_filt;
int16_t prox_west_filt;
int16_t prox_north_filt;
int16_t angle_filt;

int32_t prox_idx = 0;
int32_t prox_east_idx = 0;
int32_t prox_west_idx = 0;
int32_t prox_north_idx = 0;
int32_t angle_idx = 0;

int32_t prox_sum = 0;
int32_t prox_east_sum = 0;
int32_t prox_west_sum = 0;
int32_t prox_north_sum = 0;
int32_t angle_sum = 0;

int32_t offset_sum = 0;
int32_t offset_filt;
int16_t offset[offset_len];
int32_t offset_idx;

//
int16_t prx;
int16_t ang;
int16_t prx_east;
int16_t prx_west;
int16_t prx_north;
int16_t last_angle = 0;
int16_t offset_EW;
//offset_EW = TMG3993_Offset();
uint8_t first = 1;
int32_t prx_offset=0;
int16_t offset_EW_corr=0;
int16_t difference_offset;

void TMG3993_Init(void)
{
  I2CWrite(ADDRESS_TMG3993, REGISTER_ENABLE,COMMAND_POWER_ON | COMMAND_PROXIMITY_ENABLE | COMMAND_WAIT_ENABLE);
  //I2CWrite(ADDRESS_TMG3993, REGISTER_ENABLE,COMMAND_POWER_ON | COMMAND_GESTURE_ENABLE | COMMAND_PROXIMITY_ENABLE);
  I2CWrite(ADDRESS_TMG3993, REGISTER_CONTROL,COMMAND_LDRIVE_100 | COMMAND_PGAIN_2);
  I2CWrite(ADDRESS_TMG3993, REGISTER_PPULSE,COMMAND_32us | COMMAND_64p);
  I2CWrite(ADDRESS_TMG3993, REGISTER_CONFIG2,COMMAND_BOOST_300);
  I2CWrite(ADDRESS_TMG3993, REGISTER_CONFIG1,COMMAND_WAITLONGDISABLE);
  I2CWrite(ADDRESS_TMG3993, REGISTER_WAITTIME,COMMAND_WTIME_10);  // 30 Hz
  I2CWrite(ADDRESS_TMG3993, REGISTER_CONFIGA3,COMMAND_2X_100);
  I2CWrite(ADDRESS_TMG3993, REGISTER_GPULSE,COMMAND_32us | COMMAND_64p);
}

int16_t TMG3993_Read_Proximity(void)
{
  uint8_t status;
  I2CRead(ADDRESS_TMG3993, REGISTER_STATUS, &status);
  while(!(status & MASK_Proximity_Valid)) {
    I2CRead(ADDRESS_TMG3993, REGISTER_STATUS, &status);
  }

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
  while(!(status & MASK_Proximity_Valid)) {
	  I2CRead(ADDRESS_TMG3993, REGISTER_STATUS, &status);
  }

  uint8_t response;
  I2CRead(ADDRESS_TMG3993, REGISTER_PDATA, &response);

  I2CWrite(ADDRESS_TMG3993, REGISTER_CONFIG3, COMMAND_All_Sensors);
  //I2CWrite(ADDRESS_TMG3993, REGISTER_WAITTIME,COMMAND_WTIME_220);

  // Moving average on proximity data

  //response = response + 2 + (prox_filt/25);
  prox_east_sum = prox_east_sum - prox_east[prox_east_idx%prox_len];
  prox_east[prox_east_idx%prox_len] = response;
  prox_east_idx++;
  prox_east_sum = prox_east_sum + response;
  //prox_east_filt = prox_east_sum/prox_len;
  prox_east_filt = prox_east_sum;

  return prox_east_filt;
}

int16_t TMG3993_Read_Proximity_West(void)
{
  //I2CWrite(ADDRESS_TMG3993, REGISTER_WAITTIME,COMMAND_WTIME_1);
  I2CWrite(ADDRESS_TMG3993, REGISTER_CONFIG3, COMMAND_Only_West);

  uint8_t status;
  I2CRead(ADDRESS_TMG3993, REGISTER_STATUS, &status);
  while(!(status & MASK_Proximity_Valid)) {
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

  //prox_west_filt = prox_west_sum/prox_len;
  prox_west_filt = prox_west_sum;

  return prox_west_filt;
}

int16_t TMG3993_Read_Angle(void)
{
  //I2CWrite(ADDRESS_TMG3993, REGISTER_WAITTIME,COMMAND_WTIME_1);

	// Calculate angle between sensor and obstacle
	//angle[angle_len-1] = (int)((prox_east_filt-prox_west_filt)/(prox_filt*0.001778*100));
  //int16_t response = (int)((prox_east_filt-prox_west_filt)/(prox_filt*0.001778*100));
  int16_t response = (int)(((prox_east_sum-prox_west_sum)));//-(offset_filt))/(prox_filt*0.001778*prox_len))*2;

  // Moving average on proximity data
  angle_sum = angle_sum - angle[angle_idx % angle_len];
  angle[angle_idx % angle_len] = response;
  angle_idx++;
  angle_sum = angle_sum + response;
  angle_filt = angle_sum / angle_len;


  //I2CWrite(ADDRESS_TMG3993, REGISTER_WAITTIME,COMMAND_WTIME_220);
  return angle_filt;//angle[prox_len-1];//angle_filt;
}

int16_t TMG3993_Offset(void)
{
  // Moving average on offset
  /*offset_sum = offset_sum - offset[angle_idx%offset_len];
  offset[offset_idx%offset_len] = (prox_east_sum - prox_west_sum)/prox_len;
  offset_sum = offset_sum + offset[offset_idx%offset_len];
  offset_idx++;
  offset_filt = offset_sum/offset_len;*/

  /*int i;
  for (i=0;i<prox_len;i++){
    TMG3993_Read_Proximity_East();
    TMG3993_Read_Proximity_West();
  }
  offset_filt = prox_east_sum - prox_west_sum;*/

  uint8_t status;
  uint8_t response;
  int32_t sum_east = 0;
  int32_t sum_west = 0;
  int i;

  I2CWrite(ADDRESS_TMG3993, REGISTER_CONFIG3,COMMAND_Only_East);

  for (i=0;i<offset_len;i++){
    I2CRead(ADDRESS_TMG3993, REGISTER_STATUS, &status);
    while(!(status & MASK_Proximity_Valid)) {
      I2CRead(ADDRESS_TMG3993, REGISTER_STATUS, &status);
    }
    I2CRead(ADDRESS_TMG3993, REGISTER_PDATA, &response);
    sum_east += response;
  }

  I2CWrite(ADDRESS_TMG3993, REGISTER_CONFIG3,COMMAND_Only_West);

  for (i=0;i<offset_len;i++){
    I2CRead(ADDRESS_TMG3993, REGISTER_STATUS, &status);
    while(!(status & MASK_Proximity_Valid)) {
      I2CRead(ADDRESS_TMG3993, REGISTER_STATUS, &status);
    }
    I2CRead(ADDRESS_TMG3993, REGISTER_PDATA, &response);
    sum_west += response;
  }

  I2CWrite(ADDRESS_TMG3993, REGISTER_CONFIG3,COMMAND_All_Sensors);

  offset_filt = sum_east - sum_west;

  return offset_filt;
}

int16_t TMG3993_Read_FIFO_East(void)
{
  uint8_t response;
  I2CRead(ADDRESS_TMG3993, REGISTER_GFIFO_E, &response);
  /*while (response == 0){
    I2CRead(ADDRESS_TMG3993, REGISTER_GFIFO_E, &response);
  }*/

  return response;
}

int16_t TMG3993_Read_FIFO_West(void)
{
  uint8_t gflvl;
  I2CRead(ADDRESS_TMG3993, REGISTER_GFLVL, &gflvl);
  while(gflvl == 0) {
    I2CRead(ADDRESS_TMG3993, REGISTER_GFLVL, &gflvl);
  }

  uint8_t response;
  I2CRead(ADDRESS_TMG3993, REGISTER_GFIFO_W, &response);
  /*while (response == 0){
    I2CRead(ADDRESS_TMG3993, REGISTER_GFIFO_W, &response);
  }*/

  return response;
}

int16_t TMG3993_Read_FIFO_North(void)
{


  uint8_t response;
  I2CRead(ADDRESS_TMG3993, REGISTER_GFIFO_N, &response);

  // Moving average on proximity data
  prox_north_sum = prox_north_sum - prox_north[prox_north_idx%prox_len];
  prox_north[prox_north_idx%prox_len] = response;
  prox_north_idx++;
  prox_north_sum = prox_north_sum + response;
  prox_north_filt = prox_north_sum/prox_len;


  return prox_north_filt;
}

int16_t TMG3993_FIFO_Difference(void)
{
  int16_t west = 0;
  int16_t east = 0;
  int i;

  // Enable gestures FIFO buffer
  I2CWrite(ADDRESS_TMG3993, REGISTER_ENABLE,COMMAND_POWER_ON | COMMAND_GESTURE_ENABLE | COMMAND_PROXIMITY_ENABLE);

  for (i=0;i<25;i++){
    west += TMG3993_Read_FIFO_West();
    east += TMG3993_Read_FIFO_East();
  }

  // Disable gestures FIFO buffer
  I2CWrite(ADDRESS_TMG3993, REGISTER_ENABLE,COMMAND_POWER_ON | COMMAND_PROXIMITY_ENABLE);

  // Make sure gestures are really disabled, because apparently without this loop it goes wrong!
  uint8_t enable;
  I2CRead(ADDRESS_TMG3993, REGISTER_ENABLE, &enable);
  while (enable != (COMMAND_POWER_ON | COMMAND_PROXIMITY_ENABLE)){
    I2CWrite(ADDRESS_TMG3993, REGISTER_ENABLE,COMMAND_POWER_ON | COMMAND_PROXIMITY_ENABLE);
    I2CRead(ADDRESS_TMG3993, REGISTER_GFIFO_W, &enable);
  }

  return (east - west)/25;
}

int16_t Angle_Measurement(void)
{
  prx = TMG3993_Read_Proximity();

  if ((prx>(prx_offset+6)) && (last_angle==127)){
    //led_set();
    last_angle = TMG3993_FIFO_Difference()-7;
  }
  if (prx<=(prx_offset+6)){
    //led_clear();
    last_angle = 127;
  }
  return last_angle;
}

void init_Angle_Measurement(void)
{
  TMG3993_Init();

  //led_set();
  int i;
  // Waste some time, apparently sensor needs some time to start up
  for (i=0;i<300;i++){
    prx_offset += TMG3993_Read_Proximity();
  }

  // Correct for proximity value at infinity
  prx_offset = 0;
  for (i=0;i<100;i++){
    prx_offset += TMG3993_Read_Proximity();
  }
  prx_offset /= 100;

  // Correct for offset in East-West difference at infinity
  difference_offset = 0;
  for (i=0;i<3;i++){
    difference_offset += TMG3993_FIFO_Difference();
  }
  difference_offset /= 3;

  first = 0;
  //led_clear();
}
