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
	// timer freq debug, groen lampie, pin pd12
	HAL_GPIO_TogglePin(GPIOD, GPIO_PIN_12);

	// ADC
	if(htim == &htim3)
	{
		pos++;

		// vul buffer met waardes uit de adc
		adc_values[pos % BUFFERSIZE] = HAL_ADC_GetValue(&hadc1);

		// voor debug: analoog signaal voor op de oscilloscoop, output L
		Put_DA((unsigned char) 1, magnitude[pos%(BUFFERSIZE/2)]);
		// duur draadje
		Put_DA((unsigned char) 0, adc_values[pos%BUFFERSIZE]);

		// elke keer dat de adc vol zit met nieuwe waarde flaggetje aan voor dft in main
		if(pos%(BUFFERSIZE/2)==0)
		{
			flag = 1;

			// sync puls voor debug op de oscilloscoop.
			for(int i = 0; i < 20; i++)
				Put_DA((unsigned char) 1, 4000);
		}

		// geef de adc opnieuw een schop om te gaan meten.
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

	// zet pd13, orangje lampje om runtime bij te houden
	HAL_GPIO_TogglePin(GPIOD, GPIO_PIN_13);

	// maak een nieuwe buffer omdat de globale buffer blijft updaten
	for(i = 0;i < BUFFERSIZE; i++)
	{
		input_signal[i] = (adc_values[i]-2048.0)/4096.0;
	}

	// de DFT zelf
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

	return 1;
}

void CreateWaves()
{
	//creeer (co)sinus voor een LUT
	for(int i = 0; i < BUFFERSIZE; i++)
	{
		sinus[i] = sin(2*PI/BUFFERSIZE*i);
		cosinus[i] = cos(2*PI/BUFFERSIZE*i);
	}
}
