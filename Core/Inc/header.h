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
#define TIMERFREQ 5120

#define BUFFERSIZE 512
#define MA_KERNELSIZE 10
#define OFFSET 0
#define PI 3.141593

extern ADC_HandleTypeDef hadc1;
extern TIM_HandleTypeDef htim3;

extern float sinus[BUFFERSIZE];
extern float cosinus[BUFFERSIZE];
extern char flag;

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef*);
extern unsigned short MovingAverage();
extern unsigned short DFT();
extern void CreateWaves();

// timer.c
extern void Set_sample_frequency(int);

// main.c
extern void Put_DA(unsigned char, unsigned short);
extern uint32_t DWT_Delay_Init(void);
void DWT_Delay_us(volatile uint32_t);
extern void Put__DA(unsigned short ,unsigned short);

#endif /* INC_HEADER_H_ */
