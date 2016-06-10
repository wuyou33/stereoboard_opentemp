/*
 * commands.h
 *
 *  Created on: May 13, 2015
 *      Author: mavlab
 */

#ifndef COMMANDS_H_
#define COMMANDS_H_

#include "usart.h"

void SendStartComm(void);
void SendCommand(uint8_t);
void SendCommandNumber(uint8_t);

#endif /* COMMANDS_H_ */
