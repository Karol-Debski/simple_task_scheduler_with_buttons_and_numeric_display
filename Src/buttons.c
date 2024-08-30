/*
 * buttons.c
 *
 *  Created on: Aug 31, 2024
 *      Author: karol
 */
#include "stm32f407xx.h"

void initButtons()
{
	GPIO_Handle_t gpioButtonDHandler={0};
	gpioButtonDHandler=(GPIO_Handle_t){
		.pGPIOx=GPIOE,
		.GPIO_PinConfig=(GPIO_PinConfig_t){
			.GPIO_PinNumber=GPIO_PIN_NUM_7,
			.GPIO_PinMode=GPIO_MODE_IT_FT,
			.GPIO_PinPullUpPullDownMode=GPIO_NO_PULL_DONW_AND_UP,
		}
	};

	GPIO_Init(&gpioButtonDHandler);

	gpioButtonDHandler.GPIO_PinConfig.GPIO_PinNumber=GPIO_PIN_NUM_8;
	GPIO_Init(&gpioButtonDHandler);

	gpioButtonDHandler.GPIO_PinConfig.GPIO_PinNumber=GPIO_PIN_NUM_9;
	GPIO_Init(&gpioButtonDHandler);

	GPIO_IRQPriorityConfig(IRQ_NUM_EXTI9_5, 1);
	GPIO_IRQInterruptConfig(IRQ_NUM_EXTI9_5, GPIO_IRQ_ENABLE);
}

