/*
 * Header.h
 *
 *  Created on: 23 nov. 2022
 *      Author: tomse
 */

#ifndef INC_HEADER_H_
#define INC_HEADER_H_

#define ldac_Pin GPIO_PIN_4
#define ldac_GPIO_Port GPIOB
#define SS1_Pin GPIO_PIN_7
#define SS1_GPIO_Port GPIOB

// convert.c
#define TIMERFREQ 5210

#define BUFFERSIZE 1024 // max so far 70
#define BUFFERHALF 41  // vul in: (buffersize-1)/2
#define MAF_KERNELSIZE 10
#define OFFSET 0
#define PI 3.14159265359
#define FREQUENCYCUTOFF 0.01 // fraction of samplerate

extern ADC_HandleTypeDef hadc1;
extern TIM_HandleTypeDef htim3;

extern float sinus[BUFFERSIZE];
extern float cosinus[BUFFERSIZE];
extern float window[BUFFERSIZE];
extern char flag;

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef*);
extern unsigned short MovingAverage(int*, uint32_t);
extern unsigned short delay(int*, uint32_t);
extern unsigned short DFT();
extern void CreateWaves();
extern unsigned short convolution(int*, uint16_t);

extern void FFT(float*, float*, int);
extern void callFFT(int*, int*, int);

extern int adc_values[BUFFERSIZE];
extern int magnitude[BUFFERSIZE/2+1];
#define DFT_ENABLE

// timer.c
extern void Set_sample_frequency(int);

// SPI.c
extern SPI_HandleTypeDef hspi2;

extern void SPI_send_byte();

// main.c
extern void Put_DA(unsigned char, unsigned short);
extern uint32_t DWT_Delay_Init(void);
void DWT_Delay_us(volatile uint32_t);
extern void Put__DA(unsigned short ,unsigned short);

#endif /* INC_HEADER_H_ */
