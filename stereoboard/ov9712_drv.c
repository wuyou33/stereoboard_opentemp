/*
 * ov534-ov9xxx gspca driver
 *
 * Copyright (C) 2009-2011 Jean-Francois Moine http://moinejf.free.fr
 * Copyright (C) 2008 Antonio Ospite <ospite@studenti.unina.it>
 * Copyright (C) 2008 Jim Paris <jim@jtan.com>
 *
 * Based on a prototype written by Mark Ferrell <majortrips@gmail.com>
 * USB protocol reverse engineered by Jim Paris <jim@jtan.com>
 * https://jim.sh/svn/jim/devl/playstation/ps3/eye/test/
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA 02111-1307 USA
 */


#include "ov9712_reg.h"
#include "ov9712.h"
#include "sccb.h"


/*


static void setbrightness( int32_t brightness)
{
        uint8_t val;
        int8_t sval;

        val = brightness;
        if (val < 8)
                val = 15 - val;         // f .. 8
        else
                val = val - 8;          // 0 .. 7
        SCCB_WriteReg(OV9712_ADDR, 0x55,     // brtn - brightness adjustment
                        0x0f | (val << 4));
}

static void setcontrast( int32_t val)
{
        SCCB_WriteReg(OV9712_ADDR, 0x56,     // cnst1 - contrast 1 ctrl coeff
                        val << 4);
}

static void setautogain( int32_t autogain)
{
        uint8_t val;

// fixme: should adjust agc/awb/aec by different controls
        val = SCCB_ReadReg(OV9712_ADDR, 0x13);               // com8
        SCCB_WriteReg(OV9712_ADDR, 0xff, 0x00);
        if (autogain)
                val |= 0x05;            // agc & aec
        else
                val &= 0xfa;
        SCCB_WriteReg(OV9712_ADDR, 0x13, val);
}

static void setexposure( int32_t exposure)
{
        static const uint8_t expo[4] = {0x00, 0x25, 0x38, 0x5e};
        uint8_t val;

        SCCB_WriteReg(OV9712_ADDR, 0x10, expo[exposure]);    // aec[9:2]

        val = SCCB_ReadReg(OV9712_ADDR, 0x13);               // com8
        SCCB_WriteReg(OV9712_ADDR, 0xff, 0x00);
        SCCB_WriteReg(OV9712_ADDR, 0x13, val);

        val = SCCB_ReadReg(OV9712_ADDR, 0xa1);               // aech
        SCCB_WriteReg(OV9712_ADDR, 0xff, 0x00);
        SCCB_WriteReg(OV9712_ADDR, 0xa1, val & 0xe0);        // aec[15:10] = 0
}

static void setsharpness( int32_t val)
{
        if (val < 0) {                          // auto
                val = SCCB_ReadReg(OV9712_ADDR, 0x42);       // com17
                SCCB_WriteReg(OV9712_ADDR, 0xff, 0x00);
                SCCB_WriteReg(OV9712_ADDR, 0x42, val | 0x40);
                                // Edge enhancement strength auto adjust
                return;
        }
        if (val != 0)
                val = 1 << (val - 1);
        SCCB_WriteReg(OV9712_ADDR, 0x3f,     // edge - edge enhance. factor
                        val);
        val = SCCB_ReadReg(OV9712_ADDR, 0x42);               // com17
        SCCB_WriteReg(OV9712_ADDR, 0xff, 0x00);
        SCCB_WriteReg(OV9712_ADDR, 0x42, val & 0xbf);
}

static void setsatur( int32_t val)
{
        uint8_t val1, val2, val3;
        static const uint8_t matrix[5][2] = {
                {0x14, 0x38},
                {0x1e, 0x54},
                {0x28, 0x70},
                {0x32, 0x8c},
                {0x48, 0x90}
        };

        val1 = matrix[val][0];
        val2 = matrix[val][1];
        val3 = val1 + val2;
        SCCB_WriteReg(OV9712_ADDR, 0x4f, val3);      // matrix coeff
        SCCB_WriteReg(OV9712_ADDR, 0x50, val3);
        SCCB_WriteReg(OV9712_ADDR, 0x51, 0x00);
        SCCB_WriteReg(OV9712_ADDR, 0x52, val1);
        SCCB_WriteReg(OV9712_ADDR, 0x53, val2);
        SCCB_WriteReg(OV9712_ADDR, 0x54, val3);
        SCCB_WriteReg(OV9712_ADDR, 0x58, 0x1a);      // mtxs - coeff signs

        val1 = SCCB_ReadReg(OV9712_ADDR, 0x41);      // com16
        SCCB_WriteReg(OV9712_ADDR, 0xff, 0x00);
        SCCB_WriteReg(OV9712_ADDR, 0x41, val1);
}

static void setlightfreq( int32_t freq)
{
        uint8_t val;

        val = SCCB_ReadReg(OV9712_ADDR, 0x13);               // com8
        SCCB_WriteReg(OV9712_ADDR, 0xff, 0x00);
        if (freq == 0) {
                SCCB_WriteReg(OV9712_ADDR, 0x13, val & 0xdf);
                return;
        }
        SCCB_WriteReg(OV9712_ADDR, 0x13, val | 0x20);

        val = SCCB_ReadReg(OV9712_ADDR, 0x42);               // com17
        SCCB_WriteReg(OV9712_ADDR, 0xff, 0x00);
        if (freq == 1)
                val |= 0x01;
        else
                val &= 0xfe;
        SCCB_WriteReg(OV9712_ADDR, 0x42, val);
}

*/


void camera_ov9712_config(void)
{
  uint16_t sensor_id;

  Delay(0x07FFFF);


  camera_ov9712_read();
  for (;;);

  // reset sensor
  SCCB_WriteReg(OV9712_ADDR, 0x12, 0x80);

  Delay(0x07FFFF);


  SCCB_WriteArray(OV9712_ADDR, ov971x_init);

}

// DEBUG
#include "usart.h"



void camera_ov9712_read(void)
{
  // TODO Replace with other text
  char buff[128] = "Register .. = .. [..] \n\r";
  UsartTx((uint8_t *)&buff[0], 24);

  uint8_t reply, r;

  for (r = PID; r <= (PID + 25); r++) {
    uint8_t res;
    myhex(r, buff + 9);
    led_toggle();
    res = SCCB_ReadReg(OV9712_ADDR, r, &reply);
    myhex(reply, buff + 14);
    myhex(res, buff + 18);
    UsartTx((uint8_t *)&buff[0], 24);
  }
}




