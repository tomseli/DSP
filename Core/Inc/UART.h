/*
 * UART.h
 *
 *  Created on: Nov 18, 2022
 *      Author: Tom
 */

#ifndef UART_UART_H_
#define UART_UART_H_

#include "stdio.h"
#include "string.h"

void UART_char(unsigned char c);
void UART_int(unsigned int);
void UART_string(char*);

#endif /* UART_UART_H_ */
