/*
 * i2c.h
 *
 *  Created on: Jul 6, 2015
 *      Author: roland
 */

#ifndef I2C_H_
#define I2C_H_

#include <stdint.h>
#include <inttypes.h>

#define TIMEOUT_MAX 50000

// Functions
uint8_t I2CRead(uint8_t Addr, uint8_t Register, uint8_t *reply);
uint8_t I2CWrite(uint8_t Addr, uint8_t Register, uint8_t Data);
uint8_t I2CRead16(uint8_t Addr, uint16_t Register, uint8_t *reply);
uint8_t I2CWrite16(uint8_t Addr, uint16_t Register, uint16_t Data);


#endif /* I2C_H_ */
