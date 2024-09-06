/*
 * systick.h
 *
 *  Created on: Sep 5, 2024
 *      Author: karol
 */

#ifndef SYSTICK_H_
#define SYSTICK_H_

#include "stm32f407xx.h"

#define SYST_CSR 	((__vo uint32_t*)0xE000E010)
#define SYST_RVR 	((__vo uint32_t*)0xE000E014)
#define SYST_CVR 	((__vo uint32_t*)0xE000E018)
#define SYST_CALIB	((__vo uint32_t*)0xE000E01C)

#define HSI_CLOCK			16000000U
#define SYSTICK_TIM_CLK		HSI_CLOCK



void initSysTick(uint32_t tickHz);

void startCounterSysTick();

void stopCounterSysTick();

void reasumeSysTick(uint32_t lastConterValue);

uint8_t isSysTickFinishedCounting();

uint32_t getSysTickCounterValue();


#endif /* SYSTICK_H_ */
