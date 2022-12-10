/*
 * convert.c
 *
 *  Created on: 23 nov. 2022
 *      Author: tomse
 */
#include "main.h"
//#include "header.h"
#include <math.h>
#include <complex.h>

extern TIM_HandleTypeDef htim3;
extern ADC_HandleTypeDef hadc1;

uint16_t pos = 0;

int dsp_buffer[BUFFERSIZE];
int dsp_output;
float MAF_kernel[MAF_KERNELSIZE];

extern float sinus[BUFFERSIZE];
extern float cosinus[BUFFERSIZE];

extern char flag;

//temp global
unsigned int magnitude[BUFFERSIZE/2+1];
float windowed_sinc[BUFFERSIZE];

// FFT


void ADC_init()
{
	for(int i = 0; i < MAF_KERNELSIZE; i++ )
	{
		MAF_kernel[i] = 1/MAF_KERNELSIZE;
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

		// duur draadje
//		Put_DA((unsigned char) 0, adc_values[pos%BUFFERSIZE]);

		Put_DA((unsigned char) 1, convolution(adc_values, pos));

#ifdef DFT_ENABLE
		// voor debug: analoog signaal voor op de oscilloscoop, output L
		Put_DA((unsigned char) 1, magnitude[pos%(BUFFERSIZE/2)]);

		// elke keer dat de adc vol zit met nieuwe waarde flaggetje aan voor dft in main
		if(pos%(BUFFERSIZE/2)==0)
		{
			flag = 1;

			// sync puls voor debug op de oscilloscoop.
			for(int i = 0; i < 20; i++)
				Put_DA((unsigned char) 1, 4000);
		}
#endif

		// geef de adc opnieuw een schop om te gaan meten.
		HAL_ADC_Start(&hadc1);
	}
}


unsigned short convolution(int* ptr, uint16_t positie)
{
	register unsigned short result = 0;
	register unsigned short i;
	register float temp;


//	for(i = 0; i < BUFFERSIZE; i++)
//	{
//		result += ptr[((positie - i) % BUFFERSIZE)] * windowed_sinc[i];
//	}
	for(i = 0; i < BUFFERHALF; i++)
	{
		temp = windowed_sinc[i];
		result += ptr[(positie - i) % BUFFERSIZE] * temp;
		result += ptr[(positie + i) % BUFFERSIZE] * temp;
	}
	result += ptr[BUFFERHALF] * windowed_sinc[BUFFERHALF];

	return result;
}


/**
 * @brief Returned een gemiddelde waarde van de afgelopen MAF_KERNELSIZE waardes
 * @param int* ptr naar de buffer
 * @param uint32_t huidige positie in de buffer
 */
unsigned short MovingAverage(int* ptr, uint32_t positie)
{
	unsigned short output = 0;
	float weight = 1.0/MAF_KERNELSIZE;

	for(int i = 1-MAF_KERNELSIZE; i < 1; i++)
	{
		output += ptr[(positie-i+BUFFERSIZE)%(BUFFERSIZE)] * weight;
	}
	return output;
}


/**
 * @brief Returned een vorige waarde in de buffer afhankelijk van OFFSET
 * @param int* ptr naar de buffer
 * @param uint32_t huidige positie in de buffer
 */
unsigned short Delay(int* ptr, uint32_t positie)
{
	unsigned short output = 0;

	output = ptr[(positie - OFFSET + BUFFERSIZE)% BUFFERSIZE];

	return output;
}

unsigned short DFT()
{
	unsigned short i, j; // register mogelijk
	float input_signal[BUFFERSIZE];
	float ImX; // register mogelijk
	float ReX; // register mogelijk

	// zet pd13, orangje lampje om runtime bij te houden
	HAL_GPIO_TogglePin(GPIOD, GPIO_PIN_13);

	// maak een nieuwe buffer omdat de globale buffer blijft updaten
	for(i = 0;i < BUFFERSIZE; i++)
	{
		input_signal[i] = (adc_values[i]-2048)/2048.0;
	}

	// de DFT zelf
	for(i = 0; i < BUFFERSIZE/2+1; i++) // frequenties
	{
		ReX = 0;
		ImX = 0;

		for(j = 0; j < BUFFERSIZE; j++) // buffer loopje
		{
			ImX += sinus[(j*i)%BUFFERSIZE] * input_signal[j];
			ReX += cosinus[(j*i)%BUFFERSIZE] * input_signal[j];
		}
		ImX /= (BUFFERSIZE);
		ReX /= (BUFFERSIZE);
		magnitude[i] = (ImX*ImX + ReX*ReX)*8000.0;
	}

	return 1;
}


void CreateWaves()
{
	float sum;
	float temp;
	//creeer (co)sinus voor een LUT
	for(int i = 0; i < BUFFERSIZE; i++)
	{
		temp = (i - BUFFERSIZE/2);
		sinus[i] = sin(2*PI/BUFFERSIZE*i);
		cosinus[i] = cos(2*PI/BUFFERSIZE*i);
		window[i] = 1 + cos(2*PI/BUFFERSIZE*(i+(BUFFERSIZE/2)));

		// bereken windowed sinc fout
		if(temp == 0)
			windowed_sinc[i] = 2*PI*FREQUENCYCUTOFF;
		else
			windowed_sinc[i] = sin(2.0 * PI * FREQUENCYCUTOFF *temp)/temp;

		sum += windowed_sinc[i];
	}

	for(int i = 0; i < BUFFERSIZE; i++)
	{
		windowed_sinc[i] = (1/sum) * windowed_sinc[i];

	}
}
