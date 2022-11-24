/*
 * timer.c
 *
 *  Created on: Nov 23, 2022
 *      Author: tomse
 */
#include "main.h"

void Set_sample_frequency(int frequency)
{
	TIM3->ARR = 96000000 / (2 * frequency);
	TIM3->CNT = 0; // count to 0 to avoid overflow
}

