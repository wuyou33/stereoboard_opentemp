/*
 * sccb.h: Serial Camera Control Bus (Omnivision)
 *
 *  Created on: Jun 11, 2015
 *      Author: mavlab
 */

#ifndef SCCB_H_
#define SCCB_H_

#include <stdint.h>

struct regval_list {
  uint8_t reg_num;
  uint8_t value;
};

void SCCB_Init(void);
uint8_t SCCB_WriteReg(uint8_t ChipID, uint8_t Addr, uint8_t Data);
uint8_t SCCB_ReadReg(uint8_t ChipID, uint8_t Addr, uint8_t *reply);
uint8_t SCCB_WriteArray(uint8_t ChipID,  const struct regval_list *vals);




#endif /* SCCB_H_ */
