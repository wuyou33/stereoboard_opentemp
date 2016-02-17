/**
 *  \file tmg3993.h TMG3993 Driver in I2C
 *
 *  Copyright (c) See main file for copyright and liability
 */
#ifndef __MY_TMG3993_HEADER__
#define __MY_TMG3993_HEADER__

#include "i2c.h"
#include <stdint.h>
#include <inttypes.h>

// Functions
void TMG3993_Init(void);
int16_t TMG3993_Read_Proximity(void);
int16_t TMG3993_Read_Proxity_East(void);
int16_t TMG3993_Read_FIFO_East(void);
int16_t TMG3993_Read_FIFO_West(void);
int16_t TMG3993_Read_FIFO_North(void);
int16_t TMG3993_Read_Proxity_West(void);
int16_t TMG3993_Read_Angle(void);
int16_t TMG3993_Offset(void);
int16_t TMG3993_FIFO_Difference(void);

// Settings
#define ADDRESS_TMG3993       		(0x39)

#define REGISTER_ENABLE        	 	(0x80)
#define REGISTER_WAITTIME     	 	(0x83)
#define REGISTER_PPULSE       		(0x8E)
#define REGISTER_GPULSE           (0xA6)
#define REGISTER_CONTROL      		(0x8F)
#define REGISTER_CONFIG1      		(0x8D)
#define REGISTER_CONFIG2      		(0x90)
#define REGISTER_CONFIG3      		(0x9F)
#define REGISTER_CONFIGA3         (0xA3)
#define REGISTER_ID           		(0x92)
#define REGISTER_PDATA        		(0x9C)
#define REGISTER_STATUS        		(0x93)
#define REGISTER_GFLVL            (0xAE)
#define REGISTER_GFIFO_E          (0xFF)
#define REGISTER_GFIFO_W          (0xFE)
#define REGISTER_GFIFO_S          (0xFD)
#define REGISTER_GFIFO_N          (0xFC)

#define REGISTER_RDATAL       		(0x96)
#define REGISTER_GDATAL       		(0x98)
#define REGISTER_BDATAL       		(0x9A)
#define REGISTER_CDATAL       		(0x94)
#define REGISTER_CDATAH       		(0x95)


/*****************************/
/* REGISTER_CONFIG1 Settings */
/*****************************/
// Wait Long
#define COMMAND_WAITLONGENABLE    (0x62)
#define COMMAND_WAITLONGDISABLE   (0x60)

/*****************************/
/* REGISTER_ENABLE Settings */
/*****************************/
// Power On
#define COMMAND_POWER_ON        	(0x01<<0)
// Enable Ambiant Light Sensor
#define COMMAND_ALS_ENABLE      	(0x01<<1)
// Enable Proximity Sensor
#define COMMAND_PROXIMITY_ENABLE 	(0x01<<2)
// Enable Gesture
#define COMMAND_GESTURE_ENABLE    (0x01<<6)
// Enable wait
#define COMMAND_WAIT_ENABLE 		  (0x01<<3)

/*****************************/
/* REGISTER_PPULSE Settings */
/*****************************/
// Proximity Pulse Length
#define COMMAND_4us        		 	  (0x00)
#define COMMAND_8us        			  (0x40)
#define COMMAND_16us        		  (0x80)
#define COMMAND_32us	      		  (0xC0)
// Proximity Pulse Count
#define COMMAND_8p         			  (0x07)
#define COMMAND_16p         		  (0x0F)
#define COMMAND_32p        			  (0x1F)
#define COMMAND_64p       			  (0x3F)


/*****************************/
/* REGISTER_WAITTIME Settings */
/*****************************/
// Wait Time
#define COMMAND_WTIME_1	     		  (0xFF) // 2.78 ms
#define COMMAND_WTIME_220	     	  (0xDC) // 100  ms
#define COMMAND_WTIME_256     	  (0x00) // 712  ms

/*****************************/
/* REGISTER_CONFIG2 Settings */
/*****************************/
// LED Boost
#define COMMAND_BOOST_100      		(0x01)
#define COMMAND_BOOST_150      		(0x11)
#define COMMAND_BOOST_200      		(0x21)
#define COMMAND_BOOST_300      		(0x31)

/*****************************/
/* REGISTER_CONFIGA3 Settings */
/*****************************/
// LED Boost
#define COMMAND_8X_100            (0xE0)
#define COMMAND_2X_100            (0xA0)

/*****************************/
/* REGISTER_CONTROL Settings */
/*****************************/
// LED Drive Strength Proximity
#define COMMAND_LDRIVE_100 			  (0x00)
#define COMMAND_LDRIVE_50 			  (0x40)
#define COMMAND_LDRIVE_25 			  (0x80)
#define COMMAND_LDRIVE_12 			  (0xC0)
// Proximity Gain Control
#define COMMAND_PGAIN_1				    (0x00)
#define COMMAND_PGAIN_2				    (0x04)
#define COMMAND_PGAIN_4				    (0x08)
#define COMMAND_PGAIN_8				    (0x0C)
// ALS and Color Gain Control
#define COMMAND_AGAIN_1 		      (0x00)
#define COMMAND_AGAIN_4 		      (0x01)
#define COMMAND_AGAIN_16 		      (0x02)
#define COMMAND_AGAIN_64 		      (0x03)

/*****************************/
/* REGISTER_CONFIG3 Settings */
/*****************************/
// Proximity Mask
#define COMMAND_Only_East      		(0x2E)
#define COMMAND_Only_West      		(0x45)
#define COMMAND_All_Sensors			  (0x00)

/*****************************/
/* REGISTER_STATUS Settings */
/*****************************/
// Proximity Valid
#define MASK_Proximity_Valid      (0x02)

#endif
