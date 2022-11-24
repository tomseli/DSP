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
#define TIMERFREQ 25000

extern ADC_HandleTypeDef hadc1;
extern TIM_HandleTypeDef htim3;

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef*);

// timer.c
void Set_sample_frequency(int);

// main.c
extern void Put_DA(unsigned char, unsigned short);
extern uint32_t DWT_Delay_Init(void);
__STATIC_INLINE void DWT_Delay_us(volatile uint32_t);
extern void Put__DA(unsigned short ,unsigned short);

#endif /* INC_HEADER_H_ */
