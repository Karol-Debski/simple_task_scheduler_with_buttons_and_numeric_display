/*
 * systick.c
 *
 *  Created on: Sep 4, 2024
 *      Author: karol
 */

#include "systick.h"


static uint32_t reloadValue=0;

static void setConfigOfSysTick()
{
	//set reload value register
	MULTI_BIT_SET_VAL(*SYST_RVR, reloadValue, 0, 24);
	//clear current reload value register
	MULTI_BIT_SET_VAL(*SYST_CVR, 0, 0, 24);
}

void reasumeSysTick(uint32_t lastConterValue)
{
	//load lastConterValue in RVR
	*SYST_RVR=lastConterValue;
	//load lastConterValue in CVR and run counter
	BIT_SET_VAL(*SYST_CSR, 1, 0);
	//load reloadValue for next counting and future
	*SYST_RVR=reloadValue;
}

void initSysTick(uint32_t tickHz)
{
	reloadValue = SYSTICK_TIM_CLK/tickHz-1;

	//enable to use processor clock as source clock
	BIT_SET_VAL(*SYST_CSR, 1, 2);
	//disable exception requests
	BIT_SET_VAL(*SYST_CSR, 0, 1);

	setConfigOfSysTick();
}

void startCounterSysTick()
{
	setConfigOfSysTick();
	//enable counter
	BIT_SET_VAL(*SYST_CSR, 1, 0);
}

void stopCounterSysTick()
{
	BIT_SET_VAL(*SYST_CSR, 0, 0);
}

uint8_t isSysTickFinishedCounting()
{
	if(BIT_READ(*SYST_CSR, 16) == 1)
	{
		return 1U;
	}
	else
	{
		return 0U;
	}
}

uint32_t getSysTickCounterValue()
{
	return MULTI_BIT_READ(*SYST_CVR, 0, 24);
}
