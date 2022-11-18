/*
 * UART.c
 *
 *  Created on: Nov 18, 2022
 *      Author: Tom
 */

#include "main.h"
#include "string.h"

extern UART_HandleTypeDef huart2;

/*
 * @brief print any char to UART
 * @param char to print
 */
void UART_char(unsigned char c)
{
	 HAL_UART_Transmit(&huart2, &c, 1, 10);
}


/*
 * @brief print any int to UART
 * @param int to print
 */
void UART_int(unsigned int value)
{
	char text[8];

	HAL_UART_Transmit(&huart2, (uint8_t*) text, sprintf(text, "%d", value), 100);
}


/*
 * @brief print any string to UART
 * @param char* to string
 */
void UART_string(char* s)
{
	int i = 0;

	for(; s[i]; i++)
		UART_char(s[i]);
}
