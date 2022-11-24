/*
 * convert.c
 *
 *  Created on: 23 nov. 2022
 *      Author: tomse
 */
#include "main.h"

#define BUFFERSIZE 4096
#define OFFSET 0

extern TIM_HandleTypeDef htim3;
extern ADC_HandleTypeDef hadc1;

uint32_t teller;
uint32_t pos;
int adc_values[BUFFERSIZE];

/**
 *
 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef* htim)
{
//	volatile int i;
	if(htim == &htim3) // ADC
	{
//		HAL_GPIO_TogglePin(GPIOD, GPIO_PIN_12); //timer freq debug, groen lampie, pin pd12
		pos++;

		adc_values[pos % BUFFERSIZE] = HAL_ADC_GetValue(&hadc1);

		Put_DA(1, adc_values[(pos - OFFSET + BUFFERSIZE) % BUFFERSIZE]);

		HAL_ADC_Start(&hadc1);
	}
}
