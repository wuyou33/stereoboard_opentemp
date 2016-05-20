/*
 * VL6180.c
 *
 *  Created on: 3 mrt. 2016
 *      Author: Kevin
 */

#include "VL6180.h"
#include "sys_time.h"

void VL6180xInit(void){
  //Required by datasheet
  //http://www.st.com/st-web-ui/static/active/en/resource/technical/document/application_note/DM00122600.pdf
  I2CWrite16(0x29, 0x0207, 0x01);
  I2CWrite16(0x29, 0x0208, 0x01);
  I2CWrite16(0x29, 0x0096, 0x00);
  I2CWrite16(0x29, 0x0097, 0xfd);
  I2CWrite16(0x29, 0x00e3, 0x00);
  I2CWrite16(0x29, 0x00e4, 0x04);
  I2CWrite16(0x29, 0x00e5, 0x02);
  I2CWrite16(0x29, 0x00e6, 0x01);
  I2CWrite16(0x29, 0x00e7, 0x03);
  I2CWrite16(0x29, 0x00f5, 0x02);
  I2CWrite16(0x29, 0x00d9, 0x05);
  I2CWrite16(0x29, 0x00db, 0xce);
  I2CWrite16(0x29, 0x00dc, 0x03);
  I2CWrite16(0x29, 0x00dd, 0xf8);
  I2CWrite16(0x29, 0x009f, 0x00);
  I2CWrite16(0x29, 0x00a3, 0x3c);
  I2CWrite16(0x29, 0x00b7, 0x00);
  I2CWrite16(0x29, 0x00bb, 0x3c);
  I2CWrite16(0x29, 0x00b2, 0x09);
  I2CWrite16(0x29, 0x00ca, 0x09);
  I2CWrite16(0x29, 0x0198, 0x01);
  I2CWrite16(0x29, 0x01b0, 0x17);
  I2CWrite16(0x29, 0x01ad, 0x00);
  I2CWrite16(0x29, 0x00ff, 0x05);
  I2CWrite16(0x29, 0x0100, 0x05);
  I2CWrite16(0x29, 0x0199, 0x05);
  I2CWrite16(0x29, 0x01a6, 0x1b);
  I2CWrite16(0x29, 0x01ac, 0x3e);
  I2CWrite16(0x29, 0x01a7, 0x1f);
  I2CWrite16(0x29, 0x0030, 0x00);
}

void VL6180xDefautSettings(void){
  //Recommended settings from datasheet
  //http://www.st.com/st-web-ui/static/active/en/resource/technical/document/application_note/DM00122600.pdf

  I2CWrite16(0x29, 0x0011, 0x10);
  I2CWrite16(0x29, VL6180X_READOUT_AVERAGING_SAMPLE_PERIOD, 0x30); //Set Avg sample period
  I2CWrite16(0x29, VL6180X_SYSALS_ANALOGUE_GAIN, 0x46); // Set the ALS gain
  I2CWrite16(0x29, VL6180X_SYSRANGE_VHV_REPEAT_RATE, 0xFF); // Set auto calibration period (Max = 255)/(OFF = 0)
  I2CWrite16(0x29, VL6180X_SYSALS_INTEGRATION_PERIOD, 0x63); // Set ALS integration time to 100ms
  I2CWrite16(0x29, VL6180X_SYSRANGE_VHV_RECALIBRATE, 0x01); // perform a single temperature calibration
  //Optional settings from datasheet
  //http://www.st.com/st-web-ui/static/active/en/resource/technical/document/application_note/DM00122600.pdf
  I2CWrite16(0x29, VL6180X_SYSRANGE_INTERMEASUREMENT_PERIOD, 0x09); // Set default ranging inter-measurement period to 100ms
  I2CWrite16(0x29, VL6180X_SYSALS_INTERMEASUREMENT_PERIOD, 0x0A); // Set default ALS inter-measurement period to 100ms
  I2CWrite16(0x29, VL6180X_SYSTEM_INTERRUPT_CONFIG_GPIO, 0x24); // Configures interrupt on ‘New Sample Ready threshold event’
  //Additional settings defaults from community
  /*VL6180x_setRegister(VL6180X_SYSRANGE_MAX_CONVERGENCE_TIME, 0x32);
  VL6180x_setRegister(VL6180X_SYSRANGE_RANGE_CHECK_ENABLES, 0x10 | 0x01);
  VL6180x_setRegister16bit(VL6180X_SYSRANGE_EARLY_CONVERGENCE_ESTIMATE, 0x7B );
  VL6180x_setRegister16bit(VL6180X_SYSALS_INTEGRATION_PERIOD, 0x64);

  VL6180x_setRegister(VL6180X_READOUT_AVERAGING_SAMPLE_PERIOD,0x30);
  VL6180x_setRegister(VL6180X_SYSALS_ANALOGUE_GAIN,0x40);
  VL6180x_setRegister(VL6180X_FIRMWARE_RESULT_SCALER,0x01);*/

  I2CWrite16(0x29, VL6180X_SYSRANGE_START, 0x03); //Start Continuous Range mode
}

uint8_t getDistance()
{
  /*I2CWrite16(0x29, VL6180X_SYSRANGE_START, 0x01); //Start Single shot mode
  uint32_t sys_time_prev = sys_time_get();
  while (sys_time_get()-sys_time_prev<21){
    uint8_t peer = 1;
  }*/
  uint8_t id;
  I2CRead16(0x29, VL6180X_IDENTIFICATION_MODEL_ID, &id);

  uint8_t distance;
  I2CRead16(0x29, VL6180X_RESULT_RANGE_VAL, &distance);
  I2CWrite16(0x29, VL6180X_SYSTEM_INTERRUPT_CLEAR, 0x07);
  return distance;
}

