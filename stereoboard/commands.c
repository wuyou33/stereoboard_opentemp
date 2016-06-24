/*
 * commands.c
 *
 *  Created on: 10 mrt. 2016
 *      Author: Kevin
 */

#include "commands.h"

void SendStartComm(void)
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


