/*
 * sccb.h: Serial Camera Control Bus (Omnivision)
 *
 *  Created on: Jun 11, 2015
 *      Author: mavlab
 */

#ifndef SCCB_H_
#define SCCB_H_

#include <stdint.h>

void SCCB_Init(void);
uint8_t SCCB_WriteReg(uint8_t ChipID, uint8_t Addr, uint8_t Data);
uint8_t SCCB_ReadReg(uint8_t ChipID, uint8_t Addr, uint8_t *reply);



#endif /* SCCB_H_ */
