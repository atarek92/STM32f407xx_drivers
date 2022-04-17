/*
 * timer_test.c
 *
 *  Created on: Apr 17, 2022
 *      Author: atarek
 */


#include <stdint.h>
#include <STM32f407xx_TIMER.h>

#include "string.h"



int main (void)
{
	TIM_ADVANCED_Handle_t pTIMHandle;

	pTIMHandle.pTIMx=TIM1;
	pTIMHandle.TIM_Config.TIM_Prescaler = 5000;
	pTIMHandle.TIM_Config.TIM_AutoRelaod = 50000;
    TIM_ADVANCED_init(&pTIMHandle);

	while (1)
	{
		int x =0;
		while (!Check_TIM_ADVANCED_Interrupt_flag(&pTIMHandle));

		x = 0;
	}
	return 0;
}
