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
int magnitude[BUFFERSIZE/2+1];
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

//		Put_DA((unsigned char) 1, convolution(adc_values, pos));

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

/**
 * bereken de FFT op een N lengte array
 */
void FFT(float* real, float* imag, int N)
{
	// eindig recursive zodra N = 1;
	if(N == 1)
		return;

	// variables
	int N2 = N/2;
	float real_even[N2], imag_even[N2], real_odd[N2], imag_odd[N2];
	int i, k;

	// splits de input in even en oneven delen (gebasseerd op indexes)
	for(i = 0; i < N2; i++)
	{
		real_even[i] = real[2 * i];
		imag_even[i] = imag[2 * i];
		real_odd[i] = real[2 * i + 1];
		imag_odd[i] = imag[2 * i + 1];
	}

	// recursieve FFT
	FFT(real_even, imag_even, N/2);
	FFT(real_odd, imag_odd, N/2);

	for(k = 0; k < N2; k++)
	{
		// bereken de W^k, gebasseerd op e^(j*2*pi*/N)
		float w_real = cos(2 * PI * k / N);
		float w_imag = sin(2 * PI * k / N);

		// butterfly hocus pocus
		float temp_real = w_real*real_odd[k] - w_imag*imag_odd[k];
		float temp_imag = w_real*imag_odd[k] + w_imag*real_odd[k];

		real[k] = real_even[k] + temp_real;
		imag[k] = imag_even[k] + temp_imag;

		real[k + N2] = real_even[k] - temp_real;
		imag[k + N2] = imag_even[k] - temp_imag;
	}
}


/**
 * Hulp functie om de FFT aan te roepen. Ontkoppeld inputs en outputs
 */
void callFFT(int* input, int* output, int N)
{
	float imag[N];
	float real[N];

	for(int i = 0; i < N; i++)
	{
		imag[i] = 0; // imag moet leeg zijn, is dit de snelste manier?
		real[i] = (float) input[i]; // ontkoppel adc_output van real
	}

	FFT(real, imag, N);

	for(int i = 0; i < N/2; i++)
	{
		output[i] = (int) sqrt(real[i]*real[i] + imag[i]+imag[i]);
	}

	// zet pd13, orangje lampje om runtime bij te houden
	HAL_GPIO_TogglePin(GPIOD, GPIO_PIN_13);
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
