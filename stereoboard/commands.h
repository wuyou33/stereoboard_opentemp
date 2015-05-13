/*
 * commands.h
 *
 *  Created on: May 13, 2015
 *      Author: mavlab
 */

#ifndef COMMANDS_H_
#define COMMANDS_H_

#include "usart.h"

void SendStartComm()
{
  uint8_t code[1];
  code[0] = 0xff;

  while (UsartTx(code, 1) == 0)
    ;
}

void SendCommand(uint8_t b)
{
  uint8_t code[1];
  code[0] = 'a' + b;

  while (UsartTx(code, 1) == 0)
    ;
}

void SendCommandNumber(uint8_t b)
{
  uint8_t code[1];
  code[0] = b;

  while (UsartTx(code, 1) == 0)
    ;
}



#endif /* COMMANDS_H_ */
