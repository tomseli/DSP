/*
 * convert.c
 *
 *  Created on: 23 nov. 2022
 *      Author: tomse
 */
#include "main.h"
//#include "header.h"
#include <math.h>

extern TIM_HandleTypeDef htim3;
extern ADC_HandleTypeDef hadc1;

uint32_t teller;
uint32_t pos;
int adc_values[BUFFERSIZE];
int dsp_buffer[BUFFERSIZE];
int dsp_output;
float MA_kernel[MA_KERNELSIZE];

extern float sinus[BUFFERSIZE];
extern float cosinus[BUFFERSIZE];

extern char flag;

//temp global
float magnitude[BUFFERSIZE/2+1];

void ADC_init()
{
	for(int i = 0; i < MA_KERNELSIZE; i++ )
	{
		MA_kernel[i] = 1/MA_KERNELSIZE;
	}
}

/**
 *
 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef* htim)
{
	HAL_GPIO_TogglePin(GPIOD, GPIO_PIN_12); //timer freq debug, groen lampie, pin pd12
//	volatile int i;
	if(htim == &htim3) // ADC
	{
		pos++;
		adc_values[pos % BUFFERSIZE] = HAL_ADC_GetValue(&hadc1);
		Put_DA((unsigned char) 1, magnitude[pos%(BUFFERSIZE/2)]);
		if(pos%(BUFFERSIZE/2)==0)
		{
//			DFT();
			flag = 1;
			for(int i = 0; i < 20; i++)
				Put_DA((unsigned char) 1, 4000);
		}

		dsp_output = 0;
		HAL_ADC_Start(&hadc1);
	}
}

unsigned short MovingAverage()
{
	unsigned short output = 0;
	float weight = 1.0/MA_KERNELSIZE;

	for(int i = 1-MA_KERNELSIZE; i < 1; i++)
	{
		output += adc_values[(pos+i+BUFFERSIZE)%(BUFFERSIZE)] * weight;
	}
	return output;
}

unsigned short DFT()
{
	unsigned short i, j;
	float input_signal[BUFFERSIZE];
	float ImX[BUFFERSIZE/2+1];
	float ReX[BUFFERSIZE/2+1];

	HAL_GPIO_TogglePin(GPIOD, GPIO_PIN_13);
	// maak een nieuwe buffer
	for(i = 0;i < BUFFERSIZE; i++)
	{
		input_signal[i] = (adc_values[i]-2048.0)/4096.0;
	}

	for(i = 0; i < BUFFERSIZE/2+1; i++)
	{
		ReX[i] = 0;
		ImX[i] = 0;
		for(j = 0; j < BUFFERSIZE/2+1; j++)
		{
			ImX[i] += sinus[(i*j)%BUFFERSIZE] * input_signal[j];
			ReX[i] += cosinus[(i*j)%BUFFERSIZE] * input_signal[j];
		}
		ImX[i] /= (BUFFERSIZE/2);
		ReX[i] /= (BUFFERSIZE/2);
		magnitude[i] = sqrtf(powf(ImX[i], 2) + powf(ReX[i], 2))*4000;
	}
//	HAL_GPIO_TogglePin(GPIOD, GPIO_PIN_13);
	return 0;
}

void CreateWaves()
{
	for(int i = 0; i < BUFFERSIZE; i++)
	{
		sinus[i] = sin(2*PI/BUFFERSIZE*i);
		cosinus[i] = cos(2*PI/BUFFERSIZE*i);
	}
}
