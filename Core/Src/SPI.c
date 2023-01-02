/*
 * SPI.c
 *
 *  Created on: 13 dec. 2022
 *      Author: Tom
 */
#include "main.h"

extern SPI_HandleTypeDef hspi2;
unsigned char byte = 10;

// HAL_SPI_Transmit_IT(SPI_HandleTypeDef * hspi, uint8_t * pData, uint16_t Size);
void SPI_send_byte()
{
	if(byte > 15)
		byte = 0;
	else
		byte++;
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, 1);
	HAL_SPI_Transmit_IT(&hspi2, &byte, 1);
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, 0);
}
